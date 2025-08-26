#include "../include/NetworkManager.h"
#include <iostream>
#include <chrono>

namespace DPApp {

    NetworkClient::NetworkClient()
        : client_socket_(-1), server_port_(0), connected_(false),
        shutdown_requested_(false), next_sequence_(0) {

#ifdef _WIN32
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

        // 초기 ID를 생성하지만, 서버와의 handshake를 통해 동기화
        slave_id_ = NetworkUtils::generateSlaveId();
    }

    NetworkClient::~NetworkClient() {
        disconnect();

#ifdef _WIN32
        WSACleanup();
#endif
    }

    bool NetworkClient::connect(const std::string& server_address, uint16_t port) {
        if (connected_) {
            std::cout << "Already connected, returning false" << std::endl;
            return false;
        }

        std::cout << "Attempting to connect to " << server_address << ":" << port << std::endl;

        server_address_ = server_address;
        server_port_ = port;

        if (!initializeSocket()) {
            std::cout << "Failed to initialize socket" << std::endl;
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

        std::cout << "Calling connect()..." << std::endl;
        if (::connect(client_socket_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
            std::cerr << "Failed to connect to server: " << server_address << ":" << port << std::endl;
            cleanupSocket();
            return false;
        }

        std::cout << "Socket connected successfully" << std::endl;

        connected_ = true;
        shutdown_requested_ = false;

        std::cout << "Starting client thread..." << std::endl;
        client_thread_ = std::thread(&NetworkClient::clientThread, this);

        // 연결 안정화를 위한 대기
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "Sending handshake message..." << std::endl;
        // 핸드셰이크 메시지 전송
        NetworkMessage handshake_msg(MessageType::HANDSHAKE,
            std::vector<uint8_t>(slave_id_.begin(), slave_id_.end()));
        if (!sendMessage(handshake_msg)) {
            std::cerr << "Failed to send handshake message" << std::endl;
            disconnect();
            return false;
        }

        std::cout << "Sending registration message..." << std::endl;
        // 슬레이브 등록 메시지 전송
        NetworkMessage register_msg(MessageType::SLAVE_REGISTER,
            std::vector<uint8_t>(slave_id_.begin(), slave_id_.end()));
        if (!sendMessage(register_msg)) {
            std::cerr << "Failed to send registration message" << std::endl;
            disconnect();
            return false;
        }

        std::cout << "Starting heartbeat thread..." << std::endl;
        // 하트비트 스레드는 등록 후에 시작
        heartbeat_thread_ = std::thread(&NetworkClient::heartbeatThread, this);

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
        if (client_socket_ != -1) {
            NetworkMessage unregister_msg(MessageType::SLAVE_UNREGISTER,
                std::vector<uint8_t>(slave_id_.begin(), slave_id_.end()));
            sendMessage(unregister_msg);

            // 메시지 전송 완료를 위한 짧은 대기
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

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
        client_socket_ = static_cast<int>(socket(AF_INET, SOCK_STREAM, 0));
        if (client_socket_ < 0) {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }

        // 연결 타임아웃 설정 (선택적)
#ifdef _WIN32
        DWORD timeout = 5000; // 5초
        setsockopt(client_socket_, SOL_SOCKET, SO_RCVTIMEO,
            reinterpret_cast<const char*>(&timeout), sizeof(timeout));
        setsockopt(client_socket_, SOL_SOCKET, SO_SNDTIMEO,
            reinterpret_cast<const char*>(&timeout), sizeof(timeout));
#else
        struct timeval timeout;
        timeout.tv_sec = 5;
        timeout.tv_usec = 0;
        setsockopt(client_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        setsockopt(client_socket_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
#endif

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

        std::cout << "Client thread started for " << slave_id_ << std::endl;

        while (connected_ && !shutdown_requested_) {
            try {
                std::cout << "Waiting for data from server..." << std::endl;

                int bytes_received = recv(client_socket_,
                    reinterpret_cast<char*>(buffer.data()),
                    static_cast<int>(buffer.size()), 0);

                std::cout << "recv() returned: " << bytes_received << std::endl;

                if (bytes_received < 0) {
                    int error = errno;
                    std::cerr << "recv() error: " << error << " - ";
#ifdef _WIN32
                    error = WSAGetLastError();
                    if (error == WSAETIMEDOUT) {
                        std::cout << "Socket timeout, continuing..." << std::endl;
                        continue;
                    }
                    else if (error == WSAECONNRESET) {
                        std::cerr << "Connection reset by server" << std::endl;
                    }
                    else {
                        std::cerr << "WSA Error: " << error << std::endl;
                    }
#else
                    if (error == EAGAIN || error == EWOULDBLOCK) {
                        std::cout << "Socket timeout, continuing..." << std::endl;
                        continue;
                    }
                    else {
                        std::cerr << "Socket error: " << strerror(error) << std::endl;
                    }
#endif
                    break;
                }
                else if (bytes_received == 0) {
                    std::cout << "Server closed connection gracefully" << std::endl;
                    break;
                }

                std::cout << "Received " << bytes_received << " bytes from server" << std::endl;

                buffer.resize(bytes_received);
                NetworkMessage message = NetworkMessage::deserialize(buffer);

                std::cout << "Deserialized message type: " << static_cast<int>(message.header.type) << std::endl;

                if (message_callback_) {
                    message_callback_(message, "server");
                }

                buffer.resize(65536);  // 버퍼 크기 복원

            }
            catch (const std::exception& e) {
                std::cerr << "Exception in client thread: " << e.what() << std::endl;
                break;
            }
        }

        if (!shutdown_requested_) {
            std::cerr << "Connection lost to server" << std::endl;
            connected_ = false;

            // 연결 끊김 콜백 호출
            if (connection_callback_) {
                connection_callback_(slave_id_, false);
            }
        }

        std::cout << "Client thread ending for " << slave_id_ << std::endl;
    }

    void NetworkClient::heartbeatThread() {
        // 초기 등록 완료를 위한 대기
        std::this_thread::sleep_for(std::chrono::seconds(1));

        while (connected_ && !shutdown_requested_) {
            std::this_thread::sleep_for(std::chrono::seconds(10));

            if (connected_ && !shutdown_requested_) {
                NetworkMessage heartbeat(MessageType::HEARTBEAT,
                    std::vector<uint8_t>(slave_id_.begin(), slave_id_.end()));
                if (!sendMessage(heartbeat)) {
                    std::cerr << "Failed to send heartbeat" << std::endl;
                    break;
                }
            }
        }
    }

    bool NetworkClient::sendMessage(const NetworkMessage& message) {
        if (!connected_ || client_socket_ == -1) return false;

        try {
            std::vector<uint8_t> data = message.serialize();
            int bytes_sent = send(client_socket_,
                reinterpret_cast<const char*>(data.data()),
                static_cast<int>(data.size()), 0);

            return bytes_sent == static_cast<int>(data.size());
        }
        catch (const std::exception& e) {
            std::cerr << "Error sending message: " << e.what() << std::endl;
            return false;
        }
    }

} // namespace DPApp