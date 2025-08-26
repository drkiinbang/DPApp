#include "../include/NetworkManager.h"
#include <iostream>
#include <chrono>

namespace DPApp {

// NetworkClient 구현
NetworkClient::NetworkClient() 
    : client_socket_(-1), server_port_(0), connected_(false), 
      shutdown_requested_(false), next_sequence_(0) {
    
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
    
    slave_id_ = NetworkUtils::generateSlaveId();
}

NetworkClient::~NetworkClient() {
    disconnect();
    
#ifdef _WIN32
    WSACleanup();
#endif
}

bool NetworkClient::connect(const std::string& server_address, uint16_t port) {
    if (connected_) return false;
    
    server_address_ = server_address;
    server_port_ = port;
    
    if (!initializeSocket()) {
        return false;
    }
    
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    
    if (inet_pton(AF_INET, server_address.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid server address: " << server_address << std::endl;
        cleanupSocket();
        return false;
    }
    
    if (::connect(client_socket_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
        std::cerr << "Failed to connect to server: " << server_address << ":" << port << std::endl;
        cleanupSocket();
        return false;
    }
    
    connected_ = true;
    shutdown_requested_ = false;
    
    client_thread_ = std::thread(&NetworkClient::clientThread, this);
    heartbeat_thread_ = std::thread(&NetworkClient::heartbeatThread, this);
    
    // 슬레이브 등록 메시지 전송
    NetworkMessage register_msg(MessageType::SLAVE_REGISTER, 
                               std::vector<uint8_t>(slave_id_.begin(), slave_id_.end()));
    sendMessage(register_msg);
    
    if (connection_callback_) {
        connection_callback_(slave_id_, true);
    }
    
    std::cout << "Connected to server: " << server_address << ":" << port 
              << " as " << slave_id_ << std::endl;
    
    return true;
}

void NetworkClient::disconnect() {
    if (!connected_) return;
    
    // 슬레이브 등록 해제 메시지 전송
    NetworkMessage unregister_msg(MessageType::SLAVE_UNREGISTER, 
                                 std::vector<uint8_t>(slave_id_.begin(), slave_id_.end()));
    sendMessage(unregister_msg);
    
    shutdown_requested_ = true;
    connected_ = false;
    
    cleanupSocket();
    
    if (client_thread_.joinable()) {
        client_thread_.join();
    }
    
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }
    
    if (connection_callback_) {
        connection_callback_(slave_id_, false);
    }
    
    std::cout << "Disconnected from server" << std::endl;
}

bool NetworkClient::initializeSocket() {
    client_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket_ < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return false;
    }
    
    return true;
}

void NetworkClient::cleanupSocket() {
    if (client_socket_ >= 0) {
#ifdef _WIN32
        closesocket(client_socket_);
#else
        close(client_socket_);
#endif
        client_socket_ = -1;
    }
}

void NetworkClient::clientThread() {
    std::vector<uint8_t> buffer(65536);
    
    while (connected_ && !shutdown_requested_) {
        int bytes_received = recv(client_socket_, 
                                 reinterpret_cast<char*>(buffer.data()), 
                                 buffer.size(), 0);
        
        if (bytes_received <= 0) {
            if (!shutdown_requested_) {
                std::cerr << "Connection lost to server" << std::endl;
                connected_ = false;
            }
            break;
        }
        
        try {
            buffer.resize(bytes_received);
            NetworkMessage message = NetworkMessage::deserialize(buffer);
            
            if (message_callback_) {
                message_callback_(message, "server");
            }
            
            buffer.resize(65536);  // 버퍼 크기 복원
            
        } catch (const std::exception& e) {
            std::cerr << "Error processing message from server: " << e.what() << std::endl;
        }
    }
}

void NetworkClient::heartbeatThread() {
    while (connected_ && !shutdown_requested_) {
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        if (connected_) {
            NetworkMessage heartbeat(MessageType::HEARTBEAT, 
                                   std::vector<uint8_t>(slave_id_.begin(), slave_id_.end()));
            sendMessage(heartbeat);
        }
    }
}

bool NetworkClient::sendMessage(const NetworkMessage& message) {
    if (!connected_) return false;
    
    std::vector<uint8_t> data = message.serialize();
    int bytes_sent = send(client_socket_, 
                         reinterpret_cast<const char*>(data.data()), 
                         data.size(), 0);
    
    return bytes_sent == static_cast<int>(data.size());
}

} // namespace DPApp