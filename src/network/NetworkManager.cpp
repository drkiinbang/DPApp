#include "NetworkManager.h"
#include <iostream>
#include <sstream>
#include <random>
#include <chrono>
#include <cstring>

namespace DPApp {

// NetworkMessage 구현
NetworkMessage::NetworkMessage(MessageType type, const std::vector<uint8_t>& payload) 
    : data(payload) {
    header.type = type;
    header.data_length = static_cast<uint32_t>(payload.size());
    calculateChecksum();
}

std::vector<uint8_t> NetworkMessage::serialize() const {
    std::vector<uint8_t> buffer(sizeof(MessageHeader) + data.size());
    
    // 헤더 복사
    std::memcpy(buffer.data(), &header, sizeof(MessageHeader));
    
    // 데이터 복사
    if (!data.empty()) {
        std::memcpy(buffer.data() + sizeof(MessageHeader), data.data(), data.size());
    }
    
    return buffer;
}

NetworkMessage NetworkMessage::deserialize(const std::vector<uint8_t>& buffer) {
    if (buffer.size() < sizeof(MessageHeader)) {
        throw std::runtime_error("Buffer too small for message header");
    }
    
    NetworkMessage message;
    
    // 헤더 복사
    std::memcpy(&message.header, buffer.data(), sizeof(MessageHeader));
    
    // 매직 넘버 확인
    if (message.header.magic != MessageHeader::MAGIC_NUMBER) {
        throw std::runtime_error("Invalid magic number");
    }
    
    // 데이터 복사
    if (message.header.data_length > 0) {
        if (buffer.size() < sizeof(MessageHeader) + message.header.data_length) {
            throw std::runtime_error("Buffer too small for message data");
        }
        
        message.data.resize(message.header.data_length);
        std::memcpy(message.data.data(), 
                   buffer.data() + sizeof(MessageHeader), 
                   message.header.data_length);
    }
    
    // 체크섬 검증
    if (!message.verifyChecksum()) {
        throw std::runtime_error("Checksum verification failed");
    }
    
    return message;
}

void NetworkMessage::calculateChecksum() {
    header.checksum = NetworkUtils::calculateCRC32(data);
}

bool NetworkMessage::verifyChecksum() const {
    return header.checksum == NetworkUtils::calculateCRC32(data);
}

// NetworkServer 구현
NetworkServer::NetworkServer() 
    : server_socket_(-1), port_(0), running_(false), shutdown_requested_(false), next_sequence_(0) {
    
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
    server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket_ < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return false;
    }
    
    // SO_REUSEADDR 설정
    int opt = 1;
    if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, 
                   reinterpret_cast<const char*>(&opt), sizeof(opt)) < 0) {
        std::cerr << "Failed to set socket options" << std::endl;
        return false;
    }
    
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port_);
    
    if (bind(server_socket_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
        std::cerr << "Failed to bind socket" << std::endl;
        return false;
    }
    
    if (listen(server_socket_, 10) < 0) {
        std::cerr << "Failed to listen on socket" << std::endl;
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
        
        int client_socket = accept(server_socket_, 
                                  reinterpret_cast<sockaddr*>(&client_addr), 
                                  &client_len);
        
        if (client_socket < 0) {
            if (!shutdown_requested_) {
                std::cerr << "Failed to accept client connection" << std::endl;
            }
            continue;
        }
        
        std::string client_address = inet_ntoa(client_addr.sin_addr);
        uint16_t client_port = ntohs(client_addr.sin_port);
        
        auto client = std::make_shared<ClientConnection>(client_socket, client_address, client_port);
        client->slave_id = NetworkUtils::generateSlaveId();
        
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            clients_[client->slave_id] = client;
        }
        
        std::thread client_thread(&NetworkServer::clientHandlerThread, this, client);
        client_thread.detach();
        
        if (connection_callback_) {
            connection_callback_(client->slave_id, true);
        }
        
        std::cout << "Client connected: " << client->slave_id 
                  << " from " << client_address << ":" << client_port << std::endl;
    }
}

void NetworkServer::clientHandlerThread(std::shared_ptr<ClientConnection> client) {
    std::vector<uint8_t> buffer(65536);
    
    while (client->is_active && running_) {
        int bytes_received = recv(client->socket_fd, 
                                 reinterpret_cast<char*>(buffer.data()), 
                                 buffer.size(), 0);
        
        if (bytes_received <= 0) {
            break;  // 클라이언트 연결 종료
        }
        
        try {
            buffer.resize(bytes_received);
            NetworkMessage message = NetworkMessage::deserialize(buffer);
            
            client->last_heartbeat = std::chrono::steady_clock::now();
            
            if (message_callback_) {
                message_callback_(message, client->slave_id);
            }
            
            buffer.resize(65536);  // 버퍼 크기 복원
            
        } catch (const std::exception& e) {
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
                         data.size(), 0);
    
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
                
                if (elapsed > 30) {  // 30초 타임아웃
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

// NetworkUtils 구현
namespace NetworkUtils {

std::string generateSlaveId() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, 15);
    
    std::stringstream ss;
    ss << "slave_";
    for (int i = 0; i < 8; ++i) {
        ss << std::hex << dis(gen);
    }
    
    return ss.str();
}

uint32_t calculateCRC32(const std::vector<uint8_t>& data) {
    // 간단한 CRC32 구현
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return ~crc;
}

} // namespace NetworkUtils

} // namespace DPApp