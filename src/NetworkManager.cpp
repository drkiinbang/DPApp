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

    const int heartbeat_interval = 10;

    //========================================
    // NetworkMessage Implementation
    //========================================
    NetworkMessage::NetworkMessage(MessageType type, const std::vector<uint8_t>& payload)
        : data(payload) {
        header.type = type;
        header.data_length = static_cast<uint32_t>(payload.size());
        calculateChecksum();
    }

    std::vector<uint8_t> NetworkMessage::serialize() const {
        std::vector<uint8_t> buffer(sizeof(MessageHeader) + data.size());
        std::memcpy(buffer.data(), &header, sizeof(MessageHeader));
        if (!data.empty()) {
            std::memcpy(buffer.data() + sizeof(MessageHeader), data.data(), data.size());
        }
        return buffer;
    }

    NetworkMessage NetworkMessage::deserialize(const std::vector<uint8_t>& header_buffer, const std::vector<uint8_t>& data_buffer) {
        if (header_buffer.size() < sizeof(MessageHeader)) {
            throw std::runtime_error("Buffer too small for message header");
        }

        NetworkMessage message;
        std::memcpy(&message.header, header_buffer.data(), sizeof(MessageHeader));

        if (message.header.magic != MessageHeader::MAGIC_NUMBER) {
            throw std::runtime_error("Invalid magic number");
        }

        message.data = data_buffer;

        if (message.header.data_length != message.data.size()) {
            throw std::runtime_error("Data length mismatch in header vs actual data");
        }

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


    //========================================
    // NetworkUtils Implementation
    //========================================
    namespace NetworkUtils {

        bool recvAll(int socket_fd, std::vector<uint8_t>& buffer, size_t bytes_to_receive) {
            if (bytes_to_receive == 0) {
                buffer.clear();
                return true;
            }
            buffer.resize(bytes_to_receive);
            size_t total_bytes_received = 0;
            while (total_bytes_received < bytes_to_receive) {
                int bytes_received = recv(socket_fd, reinterpret_cast<char*>(buffer.data()) + total_bytes_received, static_cast<int>(bytes_to_receive - total_bytes_received), 0);
                if (bytes_received == 0) {
                    ILOG << "Disconnected: bytes_received == 0";
                    return false;
                }
                else if (bytes_received < 0) {
#ifdef _WIN32
                    int error = WSAGetLastError();
                    if (error == WSAEWOULDBLOCK || error == WSAEINTR) {
                        ILOG << "Retry (in case nonblocking socket or interrupt)";
                        continue;
                    }
#else
                    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                        ILOG << "errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR";
                        continue;
                    }
#endif
                    ILOG << "Disconnected: bytes_received < 0";
                    return false;
                }
                total_bytes_received += bytes_received;
            }
            return true;
        }

        bool sendAll(int socket_fd, const std::vector<uint8_t>& buffer) {
            if (buffer.empty()) return true;
            size_t total_bytes_sent = 0;
            while (total_bytes_sent < buffer.size()) {
                int bytes_sent = send(socket_fd, reinterpret_cast<const char*>(buffer.data()) + total_bytes_sent, static_cast<int>(buffer.size() - total_bytes_sent), 0);
                if (bytes_sent < 0) return false;
                total_bytes_sent += bytes_sent;
            }
            return true;
        }

        std::string generateSlaveId() {
            static std::random_device rd;
            static std::mt19937 gen(rd());
            static std::uniform_int_distribution<> dis(0, 15);
            std::stringstream ss;
            ss << "slave_";
            for (int i = 0; i < 8; ++i) ss << std::hex << dis(gen);
            return ss.str();
        }

        uint32_t calculateCRC32(const std::vector<uint8_t>& data) {
            uint32_t crc = 0xFFFFFFFF;
            for (uint8_t byte : data) {
                crc ^= byte;
                for (int i = 0; i < 8; ++i) {
                    crc = (crc & 1) ? ((crc >> 1) ^ 0xEDB88320) : (crc >> 1);
                }
            }
            return ~crc;
        }

        std::vector<uint8_t> serializeTask(const ProcessingTask& task) {
            std::vector<uint8_t> data;
            data.reserve(sizeof(uint32_t) * 2 + sizeof(uint8_t) + sizeof(uint64_t) + task.parameters.size());
            const uint8_t* ptr;
            ptr = reinterpret_cast<const uint8_t*>(&task.task_id);
            data.insert(data.end(), ptr, ptr + sizeof(uint32_t));
            ptr = reinterpret_cast<const uint8_t*>(&task.chunk_id);
            data.insert(data.end(), ptr, ptr + sizeof(uint32_t));
            data.push_back(static_cast<uint8_t>(task.task_type));
            uint64_t param_len = task.parameters.size();
            ptr = reinterpret_cast<const uint8_t*>(&param_len);
            data.insert(data.end(), ptr, ptr + sizeof(uint64_t));
            data.insert(data.end(), task.parameters.begin(), task.parameters.end());
            return data;
        }

        ProcessingTask deserializeTask(const std::vector<uint8_t>& data) {
            if (data.size() < sizeof(uint32_t) * 2 + sizeof(uint8_t) + sizeof(uint64_t)) throw std::runtime_error("Invalid task data size");
            ProcessingTask task;
            size_t offset = 0;
            std::memcpy(&task.task_id, data.data() + offset, sizeof(uint32_t)); offset += sizeof(uint32_t);
            std::memcpy(&task.chunk_id, data.data() + offset, sizeof(uint32_t)); offset += sizeof(uint32_t);
            task.task_type = static_cast<TaskType>(data[offset]); offset += sizeof(uint8_t);
            uint64_t param_len;
            std::memcpy(&param_len, data.data() + offset, sizeof(uint64_t)); offset += sizeof(uint64_t);
            if (offset + param_len > data.size()) throw std::runtime_error("Invalid task data format");
            task.parameters.assign(data.begin() + offset, data.begin() + offset + param_len);
            return task;
        }

        std::vector<uint8_t> serializeResult(const ProcessingResult& result) {
            std::vector<uint8_t> data;
            uint64_t error_msg_len = result.error_message.length();
            uint64_t points_len = result.processed_points.size();
            uint64_t result_data_len = result.result_data.size();
            size_t total_size = sizeof(uint32_t) * 2 + sizeof(bool) + sizeof(uint64_t) * 3 + error_msg_len + (points_len * sizeof(Point3D)) + result_data_len;
            data.reserve(total_size);
            const uint8_t* ptr;
            ptr = reinterpret_cast<const uint8_t*>(&result.task_id); data.insert(data.end(), ptr, ptr + sizeof(uint32_t));
            ptr = reinterpret_cast<const uint8_t*>(&result.chunk_id); data.insert(data.end(), ptr, ptr + sizeof(uint32_t));
            ptr = reinterpret_cast<const uint8_t*>(&result.success); data.insert(data.end(), ptr, ptr + sizeof(bool));
            ptr = reinterpret_cast<const uint8_t*>(&error_msg_len); data.insert(data.end(), ptr, ptr + sizeof(uint64_t));
            ptr = reinterpret_cast<const uint8_t*>(&points_len); data.insert(data.end(), ptr, ptr + sizeof(uint64_t));
            ptr = reinterpret_cast<const uint8_t*>(&result_data_len); data.insert(data.end(), ptr, ptr + sizeof(uint64_t));
            data.insert(data.end(), result.error_message.begin(), result.error_message.end());
            for (const auto& point : result.processed_points) {
                ptr = reinterpret_cast<const uint8_t*>(&point);
                data.insert(data.end(), ptr, ptr + sizeof(Point3D));
            }
            data.insert(data.end(), result.result_data.begin(), result.result_data.end());
            return data;
        }

        ProcessingResult deserializeResult(const std::vector<uint8_t>& data) {
            size_t header_size = sizeof(uint32_t) * 2 + sizeof(bool) + sizeof(uint64_t) * 3;
            if (data.size() < header_size) throw std::runtime_error("Invalid result data size");
            ProcessingResult result;
            size_t offset = 0;
            std::memcpy(&result.task_id, data.data() + offset, sizeof(uint32_t)); offset += sizeof(uint32_t);
            std::memcpy(&result.chunk_id, data.data() + offset, sizeof(uint32_t)); offset += sizeof(uint32_t);
            std::memcpy(&result.success, data.data() + offset, sizeof(bool)); offset += sizeof(bool);
            uint64_t error_msg_len, points_len, result_data_len;
            std::memcpy(&error_msg_len, data.data() + offset, sizeof(uint64_t)); offset += sizeof(uint64_t);
            std::memcpy(&points_len, data.data() + offset, sizeof(uint64_t)); offset += sizeof(uint64_t);
            std::memcpy(&result_data_len, data.data() + offset, sizeof(uint64_t)); offset += sizeof(uint64_t);
            size_t expected_size = header_size + error_msg_len + (points_len * sizeof(Point3D)) + result_data_len;
            if (data.size() != expected_size) throw std::runtime_error("Invalid result data format");
            if (error_msg_len > 0) { result.error_message.assign(reinterpret_cast<const char*>(data.data() + offset), error_msg_len); offset += error_msg_len; }
            result.processed_points.resize(points_len);
            std::memcpy(result.processed_points.data(), data.data() + offset, points_len * sizeof(Point3D)); offset += points_len * sizeof(Point3D);
            if (result_data_len > 0) result.result_data.assign(data.begin() + offset, data.end());
            return result;
        }

        std::vector<uint8_t> serializeChunk(const PointCloudChunk& chunk) {
            std::vector<uint8_t> data;
            uint64_t points_count = chunk.points.size();
            uint64_t filename_len = chunk.header.filename.length();
            size_t total_size = sizeof(uint32_t) + sizeof(uint64_t) + sizeof(double) * 6 + sizeof(uint64_t) + filename_len + (points_count * sizeof(Point3D));
            data.reserve(total_size);
            const uint8_t* ptr;
            ptr = reinterpret_cast<const uint8_t*>(&chunk.chunk_id); data.insert(data.end(), ptr, ptr + sizeof(uint32_t));
            ptr = reinterpret_cast<const uint8_t*>(&points_count); data.insert(data.end(), ptr, ptr + sizeof(uint64_t));
            ptr = reinterpret_cast<const uint8_t*>(&chunk.min_x); data.insert(data.end(), ptr, ptr + sizeof(double) * 6);
            ptr = reinterpret_cast<const uint8_t*>(&filename_len); data.insert(data.end(), ptr, ptr + sizeof(uint64_t));
            data.insert(data.end(), chunk.header.filename.begin(), chunk.header.filename.end());
            for (const auto& point : chunk.points) {
                ptr = reinterpret_cast<const uint8_t*>(&point);
                data.insert(data.end(), ptr, ptr + sizeof(Point3D));
            }
            return data;
        }

        PointCloudChunk deserializeChunk(const std::vector<uint8_t>& data) {
            size_t header_size = sizeof(uint32_t) + sizeof(uint64_t) + sizeof(double) * 6 + sizeof(uint64_t);
            if (data.size() < header_size) throw std::runtime_error("Invalid chunk data size");
            PointCloudChunk chunk;
            size_t offset = 0;
            std::memcpy(&chunk.chunk_id, data.data() + offset, sizeof(uint32_t)); offset += sizeof(uint32_t);
            uint64_t points_count;
            std::memcpy(&points_count, data.data() + offset, sizeof(uint64_t)); offset += sizeof(uint64_t);
            std::memcpy(&chunk.min_x, data.data() + offset, sizeof(double) * 6); offset += sizeof(double) * 6;
            uint64_t filename_len;
            std::memcpy(&filename_len, data.data() + offset, sizeof(uint64_t)); offset += sizeof(uint64_t);
            size_t expected_size = header_size + filename_len + (points_count * sizeof(Point3D));
            if (data.size() != expected_size) throw std::runtime_error("Invalid chunk data format");
            if (filename_len > 0) { chunk.header.filename.assign(reinterpret_cast<const char*>(data.data() + offset), filename_len); offset += filename_len; }
            chunk.points.resize(points_count);
            std::memcpy(chunk.points.data(), data.data() + offset, points_count * sizeof(Point3D));
            return chunk;
        }

    } // namespace NetworkUtils

    //========================================
    // NetworkServer Implementation
    //========================================
    NetworkServer::NetworkServer() : server_socket_(-1), port_(0), running_(false), shutdown_requested_(false) {
        cfg_ = DPApp::RuntimeConfig::loadFromEnv();
        worker_pool_ = std::make_unique<DPApp::ThreadPool>(static_cast<size_t>(cfg_.server_io_threads));
#ifdef _WIN32
        WSADATA wsaData; WSAStartup(MAKEWORD(2, 2), &wsaData);
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
        if (!initializeSocket()) return false;
        running_ = true;
        shutdown_requested_ = false;
        server_thread_ = std::thread(&NetworkServer::serverThread, this);
        heartbeat_thread_ = std::thread(&NetworkServer::heartbeatThread, this);
        ILOG << "Server started on port " << port;
        return true;
    }

    void NetworkServer::stop() {
        if (!running_) return;
        shutdown_requested_ = true;
        running_ = false;
        cleanupSocket();
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            for (auto& [id, client] : clients_) {
                client->is_active = false;
#ifdef _WIN32
                shutdown(client->socket_fd, SD_BOTH);
                closesocket(client->socket_fd);
#else
                shutdown(client->socket_fd, SHUT_RDWR);
                close(client->socket_fd);
#endif
            }
            clients_.clear();
        }
        if (server_thread_.joinable()) server_thread_.join();
        if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
        ILOG << "Server stopped";
    }

    bool NetworkServer::initializeSocket() {
        server_socket_ = static_cast<int>(socket(AF_INET, SOCK_STREAM, 0));
        if (server_socket_ < 0) { ELOG << "Failed to create socket"; return false; }
        int opt = 1;
        setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt));
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(port_);
        if (bind(server_socket_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) { ELOG << "Failed to bind socket"; cleanupSocket(); return false; }
        if (listen(server_socket_, cfg_.server_backlog) < 0) { ELOG << "Failed to listen"; cleanupSocket(); return false; }
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
        while (running_) {
            sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);
            int client_socket = static_cast<int>(accept(server_socket_, reinterpret_cast<sockaddr*>(&client_addr), &client_len));
            if (client_socket < 0) {
                if (running_) ELOG << "Failed to accept client connection";
                continue;
            }
            auto client = std::make_shared<ClientConnection>(client_socket, sockaddrToString(client_addr), ntohs(client_addr.sin_port));
            client->slave_id = "pending_" + NetworkUtils::generateSlaveId();
            worker_pool_->enqueue([this, client]() { this->clientHandlerThread(client); });
        }
    }

    void NetworkServer::clientHandlerThread(std::shared_ptr<ClientConnection> client) {
        std::string current_client_id = client->slave_id;
        ILOG << "Client handler started [ID: " << current_client_id << ", Address: " << client->address << ":" << client->port << "]";
        while (client->is_active && running_) {
            try {
                std::vector<uint8_t> header_buffer;
                if (!NetworkUtils::recvAll(client->socket_fd, header_buffer, sizeof(MessageHeader))) { ILOG << "Client " << current_client_id << " disconnected (header read failed)."; break; }
                MessageHeader header;
                if (header_buffer.size() >= sizeof(MessageHeader)) {
                    std::memcpy(&header, header_buffer.data(), sizeof(MessageHeader));
                }
                else {
                    throw std::runtime_error("Header buffer size insufficient");
                }
                if (header.magic != MessageHeader::MAGIC_NUMBER) { WLOG << "Invalid magic number from " << current_client_id; break; }
                std::vector<uint8_t> data_buffer;
                if (header.data_length > 0 && !NetworkUtils::recvAll(client->socket_fd, data_buffer, header.data_length)) { ILOG << "Client " << current_client_id << " disconnected (payload read failed)."; break; }
                NetworkMessage message = NetworkMessage::deserialize(header_buffer, data_buffer);
                client->last_heartbeat = std::chrono::steady_clock::now();
                if (message.header.type == MessageType::HANDSHAKE || message.header.type == MessageType::SLAVE_REGISTER) {
                    std::string new_slave_id(message.data.begin(), message.data.end());
                    if (current_client_id != new_slave_id) {
                        std::lock_guard<std::mutex> lock(clients_mutex_);
                        clients_.erase(current_client_id);
                        client->slave_id = new_slave_id;
                        clients_[new_slave_id] = client;
                        ILOG << "Client ID updated: " << current_client_id << " -> " << new_slave_id;
                        current_client_id = new_slave_id;
                        if (connection_callback_) connection_callback_(current_client_id, true);
                    }
                }
                if (message_callback_) message_callback_(message, current_client_id);
            }
            catch (const std::exception& e) { ELOG << "Exception in client handler for " << current_client_id << ": " << e.what(); break; }
        }
        client->is_active = false;
        disconnectClient(current_client_id);
        ILOG << "Client handler ended for " << current_client_id;
    }

    bool NetworkServer::sendMessage(const std::string& client_id, const NetworkMessage& message) {
        std::shared_ptr<ClientConnection> client;
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            auto it = clients_.find(client_id);
            if (it == clients_.end() || !it->second->is_active) return false;
            client = it->second;
        }
        return NetworkUtils::sendAll(client->socket_fd, message.serialize());
    }

    void NetworkServer::heartbeatThread() {
        while (running_) {
            std::this_thread::sleep_for(std::chrono::seconds(cfg_.heartbeat_timeout_seconds / 2));
            if (!running_) break;
            auto now = std::chrono::steady_clock::now();
            std::vector<std::string> disconnected_clients;
            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                for (auto& [id, client] : clients_) {
                    if (std::chrono::duration_cast<std::chrono::seconds>(now - client->last_heartbeat).count() > cfg_.heartbeat_timeout_seconds) {
                        disconnected_clients.push_back(id);
                    }
                }
            }
            for (const auto& id : disconnected_clients) {
                WLOG << "Client " << id << " timed out. Disconnecting.";
                disconnectClient(id);
            }
        }
    }

    void NetworkServer::disconnectClient(const std::string& client_id) {
        std::shared_ptr<ClientConnection> client;
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            auto it = clients_.find(client_id);
            if (it != clients_.end()) {
                client = it->second;
                clients_.erase(it);
            }
        }
        if (client) {
            client->is_active = false;
#ifdef _WIN32
            shutdown(client->socket_fd, SD_BOTH); closesocket(client->socket_fd);
#else
            shutdown(client->socket_fd, SHUT_RDWR); close(client->socket_fd);
#endif
            if (connection_callback_) connection_callback_(client_id, false);
            ILOG << "Client disconnected: " << client_id;
        }
    }

    std::vector<std::string> NetworkServer::getConnectedClients() const {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        std::vector<std::string> client_ids;
        for (const auto& [id, conn] : clients_) if (conn->is_active) client_ids.push_back(id);
        return client_ids;
    }

    bool NetworkServer::isClientConnected(const std::string& client_id) const {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        auto it = clients_.find(client_id);
        return it != clients_.end() && it->second->is_active;
    }

    size_t NetworkServer::getClientCount() const {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        return clients_.size();
    }

    bool NetworkServer::broadcastMessage(const NetworkMessage& message) {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        bool all_success = true;
        for (const auto& [id, conn] : clients_) {
            if (conn->is_active && !sendMessage(id, message)) all_success = false;
        }
        return all_success;
    }

    std::string NetworkServer::sockaddrToString(const sockaddr_in& addr) {
        char buffer[INET_ADDRSTRLEN];
        if (inet_ntop(AF_INET, &addr.sin_addr, buffer, INET_ADDRSTRLEN)) return std::string(buffer);
        return "?.?.?.?";
    }

    //========================================
    // NetworkClient Implementation
    //========================================
    NetworkClient::NetworkClient() : client_socket_(-1), connected_(false), shutdown_requested_(false) {
#ifdef _WIN32
        WSADATA wsaData; WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
        slave_id_ = NetworkUtils::generateSlaveId();
    }

    NetworkClient::~NetworkClient() {
        disconnect();
#ifdef _WIN32
        WSACleanup();
#endif
    }

    bool NetworkClient::connect(const std::string& address, uint16_t port) {
        if (connected_) return false;
        server_address_ = address;
        server_port_ = port;
        if (!initializeSocket()) return false;
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        inet_pton(AF_INET, address.c_str(), &server_addr.sin_addr);
        if (::connect(client_socket_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
            ELOG << "Failed to connect to server: " << address << ":" << port;
            cleanupSocket();
            return false;
        }
        connected_ = true;
        shutdown_requested_ = false;
        client_thread_ = std::thread(&NetworkClient::clientThread, this);
        NetworkMessage reg_msg(MessageType::SLAVE_REGISTER, std::vector<uint8_t>(slave_id_.begin(), slave_id_.end()));
        if (!sendMessage(reg_msg)) { disconnect(); return false; }
        heartbeat_thread_ = std::thread(&NetworkClient::heartbeatThread, this);
        if (connection_callback_) connection_callback_(slave_id_, true);
        ILOG << "Connected to server: " << address << ":" << port << " as " << slave_id_;
        return true;
    }

    void NetworkClient::disconnect() {
        if (!connected_) return;
        shutdown_requested_ = true;
        connected_ = false;
        cleanupSocket();
        if (client_thread_.joinable()) client_thread_.join();
        if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
        if (connection_callback_) connection_callback_(slave_id_, false);
        ILOG << "Disconnected from server";
    }

    bool NetworkClient::initializeSocket() {
        client_socket_ = static_cast<int>(socket(AF_INET, SOCK_STREAM, 0));
        return client_socket_ >= 0;
    }

    void NetworkClient::cleanupSocket() {
        if (client_socket_ >= 0) {
#ifdef _WIN32
            shutdown(client_socket_, SD_BOTH); closesocket(client_socket_);
#else
            shutdown(client_socket_, SHUT_RDWR); close(client_socket_);
#endif
            client_socket_ = -1;
        }
    }

    bool NetworkClient::sendMessage(const NetworkMessage& message) {
        if (!connected_) return false;
        return NetworkUtils::sendAll(client_socket_, message.serialize());
    }

    void NetworkClient::clientThread() {
        while (connected_ && !shutdown_requested_) {
            try {
                std::vector<uint8_t> header_buffer;
                if (!NetworkUtils::recvAll(client_socket_, header_buffer, sizeof(MessageHeader))) {
                    if (connected_) ILOG << "Server disconnected (header read failed).";
                    break;
                }
                MessageHeader header;
                if (header_buffer.size() >= sizeof(MessageHeader)) {
                    std::memcpy(&header, header_buffer.data(), sizeof(MessageHeader));
                }
                else {
                    throw std::runtime_error("Header buffer size insufficient");
                }
                if (header.magic != MessageHeader::MAGIC_NUMBER) { WLOG << "Invalid magic number from server."; break; }
                std::vector<uint8_t> data_buffer;
                if (header.data_length > 0 && !NetworkUtils::recvAll(client_socket_, data_buffer, header.data_length)) {
                    if (connected_) ILOG << "Server disconnected (payload read failed).";
                    break;
                }
                NetworkMessage message = NetworkMessage::deserialize(header_buffer, data_buffer);
                if (message_callback_) message_callback_(message, "server");
            }
            catch (const std::exception& e) {
                if (connected_) ELOG << "Exception in client thread: " << e.what();
                break;
            }
        }
        if (connected_) {
            connected_ = false;
            if (connection_callback_) connection_callback_(slave_id_, false);
        }
    }

    void NetworkClient::heartbeatThread() {
        while (connected_ && !shutdown_requested_) {
            std::this_thread::sleep_for(std::chrono::seconds(heartbeat_interval));
            if (connected_ && !shutdown_requested_) {
                NetworkMessage heartbeat(MessageType::HEARTBEAT, std::vector<uint8_t>(slave_id_.begin(), slave_id_.end()));
                if (!sendMessage(heartbeat)) {
                    WLOG << "Failed to send heartbeat";
                }
            }
        }
    }

} // namespace DPApp