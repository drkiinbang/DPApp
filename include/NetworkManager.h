#pragma once

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <map>

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
    NetworkMessage(MessageType type, const std::vector<uint8_t>& payload);
    
    // 직렬화/역직렬화
    std::vector<uint8_t> serialize() const;
    static NetworkMessage deserialize(const std::vector<uint8_t>& buffer);
    
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
};

// Slave용 네트워크 클라이언트
class NetworkClient {
public:
    NetworkClient();
    ~NetworkClient();
    
    // 서버 연결/해제
    bool connect(const std::string& server_address, uint16_t port);
    void disconnect();
    
    // 콜백 설정
    void setMessageCallback(MessageCallback callback) { message_callback_ = callback; }
    void setConnectionCallback(ConnectionCallback callback) { connection_callback_ = callback; }
    
    // 메시지 전송
    bool sendMessage(const NetworkMessage& message);
    
    // 상태 조회
    bool isConnected() const { return connected_; }
    std::string getSlaveId() const { return slave_id_; }

private:
    void clientThread();
    void heartbeatThread();
    
    bool initializeSocket();
    void cleanupSocket();
    
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

// 유틸리티 함수들
namespace NetworkUtils {
    std::string generateSlaveId();
    std::vector<uint8_t> serializeTask(const ProcessingTask& task);
    ProcessingTask deserializeTask(const std::vector<uint8_t>& data);
    std::vector<uint8_t> serializeResult(const ProcessingResult& result);
    ProcessingResult deserializeResult(const std::vector<uint8_t>& data);
    std::vector<uint8_t> serializeChunk(const PointCloudChunk& chunk);
    PointCloudChunk deserializeChunk(const std::vector<uint8_t>& data);
    uint32_t calculateCRC32(const std::vector<uint8_t>& data);
}

} // namespace DPApp