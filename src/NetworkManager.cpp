// inet_ntoa 경고 해결
#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include "../include/NetworkManager.h"
#include "../include/Logger.h"

#include <iostream>
#include <sstream>
#include <random>
#include <chrono>
#include <cstring>

namespace DPApp {
void NetworkMessage::calculateChecksum() {
    header.checksum = NetworkUtils::calculateCRC32(data);
}

bool NetworkMessage::verifyChecksum() const {
    return header.checksum == NetworkUtils::calculateCRC32(data);
}

// NetworkServer 구현
NetworkServer::NetworkServer() 
    : server_socket_(-1), port_(0), running_(false), shutdown_requested_(false), next_sequence_(0) {
    
    cfg_ = DPApp::RuntimeConfig::loadFromEnv();
    worker_pool_ = std::make_unique<DPApp::ThreadPool>(static_cast<size_t>(cfg_.server_io_threads));

#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
}

NetworkServer::~NetworkServer() {
    stop();
    
#ifdef _WIN32
    WSACleanup();
#endif
}

bool NetworkServer::start(uint16_t port) {
    if (running_) return false;
    
    port_ = port;
    
    if (!initializeSocket()) {
        return false;
    }
    
    running_ = true;
    shutdown_requested_ = false;
    
    server_thread_ = std::thread(&NetworkServer::serverThread, this);
    heartbeat_thread_ = std::thread(&NetworkServer::heartbeatThread, this);
    
    std::cout << "Server started on port " << port << std::endl;
    return true;
}

void NetworkServer::stop() {
    if (!running_) return;
    
    shutdown_requested_ = true;
    running_ = false;
    
    // 모든 클라이언트 연결 해제
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        for (auto& [client_id, client] : clients_) {
            client->is_active = false;
#ifdef _WIN32
            closesocket(client->socket_fd);
#else
            close(client->socket_fd);
#endif
        }
        clients_.clear();
    }
    
    cleanupSocket();
    
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
    
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }
    
    std::cout << "Server stopped" << std::endl;
}

bool NetworkServer::initializeSocket() {
    server_socket_ = static_cast<int>(socket(AF_INET, SOCK_STREAM, 0));
    if (server_socket_ < 0) {
        ELOG << "Failed to create socket";
        return false;
    }

    /// 소켓 옵션 설정
    int opt = 1;
    if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR,
        reinterpret_cast<const char*>(&opt), sizeof(opt)) < 0) {
        ELOG << "Failed to set SO_REUSEADDR";
        return false;
    }

#ifdef SO_REUSEPORT  /// Linux에서만 사용 가능
    setsockopt(server_socket_, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port_);

    if (bind(server_socket_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
        ELOG << "Failed to bind socket";
        return false;
    }

    if (listen(server_socket_, cfg_.server_backlog) < 0) {
        ELOG << "Failed to listen on socket";
        return false;
    }

    return true;
}

void NetworkServer::cleanupSocket() {
    if (server_socket_ >= 0) {
#ifdef _WIN32
        closesocket(server_socket_);
#else
        close(server_socket_);
#endif
        server_socket_ = -1;
    }
}

void NetworkServer::serverThread() {
    while (running_ && !shutdown_requested_) {
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);

        int client_socket = static_cast<int>(accept(server_socket_,
            reinterpret_cast<sockaddr*>(&client_addr),
            &client_len));

        if (client_socket < 0) {
            if (!shutdown_requested_) {
                std::cerr << "Failed to accept client connection" << std::endl;
            }
            continue;
        }

        std::string client_address = inet_ntoa(client_addr.sin_addr);
        uint16_t client_port = ntohs(client_addr.sin_port);

        auto client = std::make_shared<ClientConnection>(client_socket, client_address, client_port);
        client->slave_id = "pending_" + NetworkUtils::generateSlaveId();

        worker_pool_->enqueue([this, client]() {
            this->clientHandlerThread(client);
            });

        std::cout << "Client connected: from " << client_address << ":" << client_port << std::endl;
    }
}

void NetworkServer::clientHandlerThread(std::shared_ptr<ClientConnection> client) {
    std::vector<uint8_t> buffer(65536);
    bool handshake_completed = false;

    while (client->is_active && running_) {
        int bytes_received = recv(client->socket_fd,
            reinterpret_cast<char*>(buffer.data()),
            static_cast<int>(buffer.size()), 0);

        if (bytes_received <= 0) {
            break;  // 클라이언트 연결 종료
        }

        try {
            buffer.resize(bytes_received);
            NetworkMessage message = NetworkMessage::deserialize(buffer);

            client->last_heartbeat = std::chrono::steady_clock::now();

            // 핸드셰이크 메시지 처리
            if (message.header.type == MessageType::HANDSHAKE && !handshake_completed) {
                // 클라이언트가 보낸 ID를 사용
                std::string client_id(message.data.begin(), message.data.end());

                // 기존 pending ID를 실제 ID로 교체
                {
                    std::lock_guard<std::mutex> lock(clients_mutex_);
                    clients_.erase(client->slave_id); // pending ID 제거
                    client->slave_id = client_id;
                    clients_[client_id] = client;
                }

                handshake_completed = true;
                std::cout << "Handshake completed with client: " << client_id << std::endl;
                /// Notify upper layers/tests that the client is now connected with its real ID.
                if (connection_callback_) connection_callback_(client_id, true);
                continue;
            }

            if (!handshake_completed) {
                std::cerr << "Received message before handshake completion" << std::endl;
                continue;
            }

            // 클라이언트가 등록되어 있는지 확인
            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                if (clients_.find(client->slave_id) == clients_.end()) {
                    clients_[client->slave_id] = client;
                }
            }

            if (message_callback_) {
                message_callback_(message, client->slave_id);
            }

            buffer.resize(65536);  // 버퍼 크기 복원

        }
        catch (const std::exception& e) {
            std::cerr << "Error processing message from " << client->slave_id
                << ": " << e.what() << std::endl;
        }
    }

    // 클라이언트 정리
    client->is_active = false;
#ifdef _WIN32
    closesocket(client->socket_fd);
#else
    close(client->socket_fd);
#endif

    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        clients_.erase(client->slave_id);
    }

    if (connection_callback_) {
        connection_callback_(client->slave_id, false);
    }

    std::cout << "Client disconnected: " << client->slave_id << std::endl;
}

bool NetworkServer::sendMessage(const std::string& client_id, const NetworkMessage& message) {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    
    auto it = clients_.find(client_id);
    if (it == clients_.end() || !it->second->is_active) {
        return false;
    }
    
    std::vector<uint8_t> data = message.serialize();
    int bytes_sent = send(it->second->socket_fd, 
                         reinterpret_cast<const char*>(data.data()), 
                         static_cast<int>(data.size()), 0);
    
    return bytes_sent == static_cast<int>(data.size());
}

void NetworkServer::heartbeatThread() {
    while (running_ && !shutdown_requested_) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        auto now = std::chrono::steady_clock::now();
        std::vector<std::string> disconnected_clients;
        
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            for (auto& [client_id, client] : clients_) {
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    now - client->last_heartbeat).count();
                
                if (elapsed > cfg_.heartbeat_timeout_seconds) {  // 30초 타임아웃
                    disconnected_clients.push_back(client_id);
                }
            }
        }
        
        // 타임아웃된 클라이언트 제거
        for (const auto& client_id : disconnected_clients) {
            disconnectClient(client_id);
        }
    }
}

void NetworkServer::disconnectClient(const std::string& client_id) {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    auto it = clients_.find(client_id);
    if (it != clients_.end()) {
        it->second->is_active = false;
#ifdef _WIN32
        closesocket(it->second->socket_fd);
#else
        close(it->second->socket_fd);
#endif
        clients_.erase(it);

        if (connection_callback_) {
            connection_callback_(client_id, false);
        }

        std::cout << "Client forcibly disconnected: " << client_id << std::endl;
    }
}

std::vector<std::string> NetworkServer::getConnectedClients() const {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    std::vector<std::string> client_ids;
    for (const auto& pair : clients_) {
        if (pair.second->is_active) {
            client_ids.push_back(pair.first);
        }
    }

    return client_ids;
}

bool NetworkServer::isClientConnected(const std::string& client_id) const {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    auto it = clients_.find(client_id);
    return it != clients_.end() && it->second->is_active;
}

size_t NetworkServer::getClientCount() const {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    size_t count = 0;
    for (const auto& pair : clients_) {
        if (pair.second->is_active) {
            count++;
        }
    }

    return count;
}

bool NetworkServer::broadcastMessage(const NetworkMessage& message) {
    std::lock_guard<std::mutex> lock(clients_mutex_);

    bool success = true;
    for (const auto& pair : clients_) {
        if (pair.second->is_active) {
            std::vector<uint8_t> data = message.serialize();
            int bytes_sent = send(pair.second->socket_fd,
                reinterpret_cast<const char*>(data.data()),
                static_cast<int>(data.size()), 0);

            if (bytes_sent != static_cast<int>(data.size())) {
                success = false;
            }
        }
    }

    return success;
}

} // namespace DPApp