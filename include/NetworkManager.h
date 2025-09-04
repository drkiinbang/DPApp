#pragma once

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <map>
#include <iostream>
#include <random>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

#include "PointCloudTypes.h"
#include "RuntimeConfig.h"
#include "ThreadPool.h"

namespace DPApp {

// 메시지 타입 정의
enum class MessageType : uint8_t {
    HANDSHAKE = 0x01,
    TASK_ASSIGNMENT = 0x02,
    TASK_RESULT = 0x03,
    HEARTBEAT = 0x04,
    SLAVE_REGISTER = 0x05,
    SLAVE_UNREGISTER = 0x06,
    ERROR_MESSAGE = 0x07,
    SHUTDOWN = 0x08
};

// 네트워크 메시지 헤더
struct MessageHeader {
    uint32_t magic;         // 매직 넘버 (0xDEADBEEF)
    MessageType type;       // 메시지 타입
    uint32_t data_length;   // 데이터 길이
    uint32_t sequence;      // 시퀀스 번호
    uint32_t checksum;      // 체크섬
    
    static const uint32_t MAGIC_NUMBER = 0xDEADBEEF;
    
    MessageHeader() : magic(MAGIC_NUMBER), type(MessageType::HANDSHAKE), 
                     data_length(0), sequence(0), checksum(0) {}
};

// 네트워크 메시지
struct NetworkMessage {
    MessageHeader header;
    std::vector<uint8_t> data;
    
    NetworkMessage() = default;

    NetworkMessage(MessageType type, const std::vector<uint8_t>& payload)
        : data(payload) {
        header.type = type;
        header.data_length = static_cast<uint32_t>(payload.size());
        calculateChecksum();
    }
    
    /// Serialization
    std::vector<uint8_t> serialize() const {
        std::vector<uint8_t> buffer(sizeof(MessageHeader) + data.size());

        // 헤더 복사
        std::memcpy(buffer.data(), &header, sizeof(MessageHeader));

        // 데이터 복사
        if (!data.empty()) {
            std::memcpy(buffer.data() + sizeof(MessageHeader), data.data(), data.size());
        }

        return buffer;
    }

    /// Deserialization
    static NetworkMessage deserialize(const std::vector<uint8_t>& buffer) {
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
    
    // 체크섬 계산
    void calculateChecksum();
    bool verifyChecksum() const;
};

// 클라이언트 연결 정보
struct ClientConnection {
    int socket_fd;
    std::string address;
    uint16_t port;
    std::string slave_id;
    std::atomic<bool> is_active;
    std::chrono::steady_clock::time_point last_heartbeat;
    
    ClientConnection() : socket_fd(-1), port(0), is_active(false) {}
    ClientConnection(int fd, const std::string& addr, uint16_t p) 
        : socket_fd(fd), address(addr), port(p), is_active(true),
          last_heartbeat(std::chrono::steady_clock::now()) {}
};

// 네트워크 이벤트 콜백
using MessageCallback = std::function<void(const NetworkMessage&, const std::string& client_id)>;
using ConnectionCallback = std::function<void(const std::string& client_id, bool connected)>;

// Master용 네트워크 서버
class NetworkServer {
public:
    NetworkServer();
    ~NetworkServer();
    
    // 서버 시작/중지
    bool start(uint16_t port);
    void stop();
    
    // 콜백 설정
    void setMessageCallback(MessageCallback callback) { message_callback_ = callback; }
    void setConnectionCallback(ConnectionCallback callback) { connection_callback_ = callback; }
    
    // 메시지 전송
    bool sendMessage(const std::string& client_id, const NetworkMessage& message);
    bool broadcastMessage(const NetworkMessage& message);
    
    // 클라이언트 관리
    std::vector<std::string> getConnectedClients() const;
    bool isClientConnected(const std::string& client_id) const;
    void disconnectClient(const std::string& client_id);
    
    // 상태 조회
    bool isRunning() const { return running_; }
    size_t getClientCount() const;

private:
    void serverThread();
    void clientHandlerThread(std::shared_ptr<ClientConnection> client);
    void heartbeatThread();
    
    bool initializeSocket();
    void cleanupSocket();
    
    int server_socket_;
    uint16_t port_;
    std::atomic<bool> running_;
    std::atomic<bool> shutdown_requested_;
    
    std::thread server_thread_;
    std::thread heartbeat_thread_;
    
    std::map<std::string, std::shared_ptr<ClientConnection>> clients_;
    mutable std::mutex clients_mutex_;
    
    MessageCallback message_callback_;
    ConnectionCallback connection_callback_;
    
    uint32_t next_sequence_;
    std::mutex sequence_mutex_;

    DPApp::RuntimeConfig cfg_;
    std::unique_ptr<DPApp::ThreadPool> worker_pool_;
};

/// Utility functions
namespace NetworkUtils {
    inline std::string generateSlaveId() {
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

    inline uint32_t calculateCRC32(const std::vector<uint8_t>& data) {
        // 간단한 CRC32 구현
        uint32_t crc = 0xFFFFFFFF;

        for (uint8_t byte : data) {
            crc ^= byte;
            for (int i = 0; i < 8; ++i) {
                if (crc & 1) {
                    crc = (crc >> 1) ^ 0xEDB88320;
                }
                else {
                    crc >>= 1;
                }
            }
        }

        return ~crc;
    }

    inline std::vector<uint8_t> serializeTask(const ProcessingTask& task) {
        std::vector<uint8_t> data;

        /// 헤더: task_id, chunk_id, task_type 길이, 파라미터 길이
        const int marginSize = 64;
        data.reserve(marginSize + sizeof(uint32_t) * 2 + sizeof(uint8_t) + sizeof(size_t) + task.parameters.size());

        /// task_id (4 bytes)
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&task.task_id);
        data.insert(data.end(), ptr, ptr + sizeof(uint32_t));

        /// chunk_id (4 bytes)
        ptr = reinterpret_cast<const uint8_t*>(&task.chunk_id);
        data.insert(data.end(), ptr, ptr + sizeof(uint32_t));

        /// task_type (1 byte)
        data.push_back(static_cast<uint8_t>(task.task_type));

        /// 파라미터 길이 (8 bytes on x64)
        size_t param_len = task.parameters.size();
        ptr = reinterpret_cast<const uint8_t*>(&param_len);
        data.insert(data.end(), ptr, ptr + sizeof(size_t));

        /// 파라미터 데이터 (variable size)
        data.insert(data.end(), task.parameters.begin(), task.parameters.end());

        return data;
    }

    inline ProcessingTask deserializeTask(const std::vector<uint8_t>& data) {
        if (data.size() < sizeof(uint32_t) * 2 + sizeof(uint8_t) + sizeof(size_t)) {
            throw std::runtime_error("Invalid task data size");
        }

        ProcessingTask task;
        size_t offset = 0;

        // task_id
        std::memcpy(&task.task_id, data.data() + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);

        // chunk_id
        std::memcpy(&task.chunk_id, data.data() + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);

        // task_type
        uint8_t type_val = data[offset];
        task.task_type = static_cast<TaskType>(type_val);
        offset += sizeof(uint8_t);

        // 파라미터 길이
        size_t param_len;
        std::memcpy(&param_len, data.data() + offset, sizeof(size_t));
        offset += sizeof(size_t);

        // 데이터 크기 검증
        if (offset + param_len > data.size()) {
            throw std::runtime_error("Invalid task data format");
        }

        // 파라미터
        task.parameters.assign(data.begin() + offset, data.begin() + offset + param_len);

        return task;
    }

    inline std::vector<uint8_t> serializeResult(const ProcessingResult& result) {
        std::vector<uint8_t> data;

        // 헤더 크기 계산
        size_t header_size = sizeof(uint32_t) * 2 +  // task_id, chunk_id
            sizeof(bool) +            // success
            sizeof(size_t) * 3;       // error_message_len, points_len, result_data_len

        data.resize(header_size);
        size_t offset = 0;

        // task_id
        std::memcpy(data.data() + offset, &result.task_id, sizeof(uint32_t));
        offset += sizeof(uint32_t);

        // chunk_id
        std::memcpy(data.data() + offset, &result.chunk_id, sizeof(uint32_t));
        offset += sizeof(uint32_t);

        // success
        std::memcpy(data.data() + offset, &result.success, sizeof(bool));
        offset += sizeof(bool);

        // 길이 정보들
        size_t error_msg_len = result.error_message.length();
        size_t points_len = result.processed_points.size();
        size_t result_data_len = result.result_data.size();

        std::memcpy(data.data() + offset, &error_msg_len, sizeof(size_t));
        offset += sizeof(size_t);

        std::memcpy(data.data() + offset, &points_len, sizeof(size_t));
        offset += sizeof(size_t);

        std::memcpy(data.data() + offset, &result_data_len, sizeof(size_t));
        offset += sizeof(size_t);

        // error_message
        data.insert(data.end(), result.error_message.begin(), result.error_message.end());

        // processed_points
        for (const auto& point : result.processed_points) {
            const uint8_t* point_bytes = reinterpret_cast<const uint8_t*>(&point);
            data.insert(data.end(), point_bytes, point_bytes + sizeof(Point3D));
        }

        // result_data
        data.insert(data.end(), result.result_data.begin(), result.result_data.end());

        return data;
    }

    inline ProcessingResult deserializeResult(const std::vector<uint8_t>& data) {
        size_t header_size = sizeof(uint32_t) * 2 + sizeof(bool) + sizeof(size_t) * 3;

        if (data.size() < header_size) {
            throw std::runtime_error("Invalid result data size");
        }

        ProcessingResult result;
        size_t offset = 0;

        // task_id
        std::memcpy(&result.task_id, data.data() + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);

        // chunk_id
        std::memcpy(&result.chunk_id, data.data() + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);

        // success
        std::memcpy(&result.success, data.data() + offset, sizeof(bool));
        offset += sizeof(bool);

        // 길이 정보들
        size_t error_msg_len, points_len, result_data_len;

        std::memcpy(&error_msg_len, data.data() + offset, sizeof(size_t));
        offset += sizeof(size_t);

        std::memcpy(&points_len, data.data() + offset, sizeof(size_t));
        offset += sizeof(size_t);

        std::memcpy(&result_data_len, data.data() + offset, sizeof(size_t));
        offset += sizeof(size_t);

        // 데이터 크기 검증
        size_t expected_size = header_size + error_msg_len +
            (points_len * sizeof(Point3D)) + result_data_len;

        if (data.size() != expected_size) {
            throw std::runtime_error("Invalid result data format");
        }

        // error_message
        if (error_msg_len > 0) {
            result.error_message = std::string(reinterpret_cast<const char*>(data.data() + offset), error_msg_len);
            offset += error_msg_len;
        }

        // processed_points
        result.processed_points.resize(points_len);
        for (size_t i = 0; i < points_len; ++i) {
            std::memcpy(&result.processed_points[i], data.data() + offset, sizeof(Point3D));
            offset += sizeof(Point3D);
        }

        // result_data
        if (result_data_len > 0) {
            result.result_data.assign(data.begin() + offset, data.begin() + offset + result_data_len);
        }

        return result;
    }

    inline std::vector<uint8_t> serializeChunk(const PointCloudChunk& chunk) {
        std::vector<uint8_t> data;

        // 헤더 크기 계산
        size_t header_size = sizeof(uint32_t) +       // chunk_id
            sizeof(size_t) +          // points count
            sizeof(double) * 6 +      // bounds
            sizeof(size_t);           // filename length

        data.resize(header_size);
        size_t offset = 0;

        // chunk_id
        std::memcpy(data.data() + offset, &chunk.chunk_id, sizeof(uint32_t));
        offset += sizeof(uint32_t);

        // points count
        size_t points_count = chunk.points.size();
        std::memcpy(data.data() + offset, &points_count, sizeof(size_t));
        offset += sizeof(size_t);

        // bounds
        std::memcpy(data.data() + offset, &chunk.min_x, sizeof(double));
        offset += sizeof(double);
        std::memcpy(data.data() + offset, &chunk.min_y, sizeof(double));
        offset += sizeof(double);
        std::memcpy(data.data() + offset, &chunk.min_z, sizeof(double));
        offset += sizeof(double);
        std::memcpy(data.data() + offset, &chunk.max_x, sizeof(double));
        offset += sizeof(double);
        std::memcpy(data.data() + offset, &chunk.max_y, sizeof(double));
        offset += sizeof(double);
        std::memcpy(data.data() + offset, &chunk.max_z, sizeof(double));
        offset += sizeof(double);

        // header filename length
        size_t filename_len = chunk.header.filename.length();
        std::memcpy(data.data() + offset, &filename_len, sizeof(size_t));
        offset += sizeof(size_t);

        // header filename
        data.insert(data.end(), chunk.header.filename.begin(), chunk.header.filename.end());

        // points data
        for (const auto& point : chunk.points) {
            const uint8_t* point_bytes = reinterpret_cast<const uint8_t*>(&point);
            data.insert(data.end(), point_bytes, point_bytes + sizeof(Point3D));
        }

        return data;
    }

    inline PointCloudChunk deserializeChunk(const std::vector<uint8_t>& data) {
        size_t header_size = sizeof(uint32_t) + sizeof(size_t) + sizeof(double) * 6 + sizeof(size_t);

        if (data.size() < header_size) {
            throw std::runtime_error("Invalid chunk data size");
        }

        PointCloudChunk chunk;
        size_t offset = 0;

        // chunk_id
        std::memcpy(&chunk.chunk_id, data.data() + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);

        // points count
        size_t points_count;
        std::memcpy(&points_count, data.data() + offset, sizeof(size_t));
        offset += sizeof(size_t);

        // bounds
        std::memcpy(&chunk.min_x, data.data() + offset, sizeof(double));
        offset += sizeof(double);
        std::memcpy(&chunk.min_y, data.data() + offset, sizeof(double));
        offset += sizeof(double);
        std::memcpy(&chunk.min_z, data.data() + offset, sizeof(double));
        offset += sizeof(double);
        std::memcpy(&chunk.max_x, data.data() + offset, sizeof(double));
        offset += sizeof(double);
        std::memcpy(&chunk.max_y, data.data() + offset, sizeof(double));
        offset += sizeof(double);
        std::memcpy(&chunk.max_z, data.data() + offset, sizeof(double));
        offset += sizeof(double);

        // filename length
        size_t filename_len;
        std::memcpy(&filename_len, data.data() + offset, sizeof(size_t));
        offset += sizeof(size_t);

        // 데이터 크기 검증
        size_t expected_size = header_size + filename_len + (points_count * sizeof(Point3D));
        if (data.size() != expected_size) {
            throw std::runtime_error("Invalid chunk data format");
        }

        // filename
        if (filename_len > 0) {
            chunk.header.filename = std::string(reinterpret_cast<const char*>(data.data() + offset), filename_len);
            offset += filename_len;
        }

        // points
        chunk.points.resize(points_count);
        for (size_t i = 0; i < points_count; ++i) {
            std::memcpy(&chunk.points[i], data.data() + offset, sizeof(Point3D));
            offset += sizeof(Point3D);
        }

        return chunk;
    }
} /// namespace NetworkUtils

// Slave용 네트워크 클라이언트
class NetworkClient {
public:
    NetworkClient()
        : client_socket_(-1), server_port_(0), connected_(false),
        shutdown_requested_(false), next_sequence_(0) {

#ifdef _WIN32
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

        // 초기 ID를 생성하지만, 서버와의 handshake를 통해 동기화
        slave_id_ = NetworkUtils::generateSlaveId();
    }

    ~NetworkClient() {
        disconnect();

#ifdef _WIN32
        WSACleanup();
#endif
    }
    
    // 서버 연결/해제
    bool connect(const std::string& server_address, uint16_t port) {
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

    void disconnect() {
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
    
    /// Set callback
    void setMessageCallback(MessageCallback callback) { message_callback_ = callback; }
    void setConnectionCallback(ConnectionCallback callback) { connection_callback_ = callback; }
    
    /// Send a message
    bool sendMessage(const NetworkMessage& message) {
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
    
    /// Check connection status
    bool isConnected() const { return connected_; }
    std::string getSlaveId() const { return slave_id_; }

private:
    void clientThread() {
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

    void heartbeatThread() {
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
    
    bool initializeSocket() {
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

    void cleanupSocket() {
        if (client_socket_ >= 0) {
#ifdef _WIN32
            closesocket(client_socket_);
#else
            close(client_socket_);
#endif
            client_socket_ = -1;
        }
    }
    
private:
    int client_socket_;
    std::string server_address_;
    uint16_t server_port_;
    std::string slave_id_;
    
    std::atomic<bool> connected_;
    std::atomic<bool> shutdown_requested_;
    
    std::thread client_thread_;
    std::thread heartbeat_thread_;
    
    MessageCallback message_callback_;
    ConnectionCallback connection_callback_;
    
    uint32_t next_sequence_;
    std::mutex sequence_mutex_;
};

} // namespace DPApp