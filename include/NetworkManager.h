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
#ifndef INET_ADDRSTRLEN
#define INET_ADDRSTRLEN 16
#endif
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#endif

#include "PointCloudTypes.h"
#include "RuntimeConfig.h"
#include "ThreadPool.h"

#include "bim/MeshChunk.h"

namespace DPApp {

    /// 메시지 타입
    enum class MessageType : uint8_t {
        HANDSHAKE = 0x01,
        TASK_ASSIGNMENT = 0x02,
        TASK_RESULT = 0x03,
        HEARTBEAT = 0x04,
        SLAVE_REGISTER = 0x05,
        SLAVE_UNREGISTER = 0x06,
        ERROR_MESSAGE = 0x07,
        SHUTDOWN = 0x08,
        PAUSE = 0x09,      /// Slave 처리 일시정지
        RESUME = 0x0A      /// Slave 처리 재개
    };

    // 네트워크 메시지 헤더
    struct MessageHeader {
        uint32_t magic;
        MessageType type;
        uint32_t data_length;
        uint32_t sequence;
        uint32_t checksum;

        static const uint32_t MAGIC_NUMBER = 0xDEADBEEF;

        MessageHeader() : magic(MAGIC_NUMBER), type(MessageType::HANDSHAKE),
            data_length(0), sequence(0), checksum(0) {
        }
    };

    // 네트워크 메시지
    struct NetworkMessage {
        MessageHeader header;
        std::vector<uint8_t> data;

        NetworkMessage() = default;
        NetworkMessage(MessageType type, std::vector<uint8_t> payload);

        std::vector<uint8_t> serialize() const;
        static NetworkMessage deserialize(const std::vector<uint8_t>& header_buffer, std::vector<uint8_t> data_buffer);

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

        /// 이 클라이언트 소켓으로의 전송을 직렬화한다 (Slave 쪽의 동일한 문제는
        /// NetworkClient::send_mutex_ 참고: NetworkServer::heartbeatThread()와 메인
        /// 태스크 분배 경로가 동시에 같은 클라이언트로 전송할 수 있음).
        std::mutex send_mutex;

        ClientConnection(int fd, const std::string& addr, uint16_t p)
            : socket_fd(fd), address(addr), port(p), is_active(true),
            last_heartbeat(std::chrono::steady_clock::now()) {
        }
    };

    // 네트워크 이벤트 콜백
    using MessageCallback = std::function<void(const NetworkMessage&, const std::string& client_id)>;
    using ConnectionCallback = std::function<void(const std::string& client_id, bool connected)>;

    // Master용 네트워크 서버
    class NetworkServer {
    public:
        NetworkServer();
        ~NetworkServer();

        bool start(uint16_t port);
        void stop();

        void setMessageCallback(MessageCallback callback) { message_callback_ = callback; }
        void setConnectionCallback(ConnectionCallback callback) { connection_callback_ = callback; }

        bool sendMessage(const std::string& client_id, const NetworkMessage& message);
        bool broadcastMessage(const NetworkMessage& message);

        std::vector<std::string> getConnectedClients() const;
        bool isClientConnected(const std::string& client_id) const;
        void disconnectClient(const std::string& client_id);

        bool isRunning() const { return running_; }
        size_t getClientCount() const;

    private:
        std::string sockaddrToString(const sockaddr_in& addr);
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

        std::unique_ptr<DPApp::ThreadPool> worker_pool_;
        DPApp::RuntimeConfig cfg_;
    };

    /// 유틸리티 함수
    namespace NetworkUtils {
        bool recvAll(int socket_fd, std::vector<uint8_t>& buffer, size_t bytes_to_receive);
        bool sendAll(int socket_fd, const std::vector<uint8_t>& buffer);
        std::string generateSlaveId();
        uint32_t calculateCRC32(const std::vector<uint8_t>& data);

        std::vector<uint8_t> serializeTask(const ProcessingTask& task);
        ProcessingTask deserializeTask(const std::vector<uint8_t>& data);
        std::vector<uint8_t> serializeResult(const ProcessingResult& result);
        ProcessingResult deserializeResult(const std::vector<uint8_t>& data);
        std::vector<uint8_t> serializeChunk(const PointCloudChunk& chunk);
        PointCloudChunk deserializeChunk(const std::vector<uint8_t>& data);

        /// BIM/PC 청크 직렬화/역직렬화
        std::vector<uint8_t> serializeMeshChunk(const chunkbim::MeshChunk& meshChunk);
        chunkbim::MeshChunk deserializeMeshChunk(const std::vector<uint8_t>& data);
        std::vector<uint8_t> serializeBimPcChunk(const BimPcChunk& chunk);
        BimPcChunk deserializeBimPcChunk(const std::vector<uint8_t>& data);

        /// BIM/PC 처리 결과 직렬화/역직렬화
        std::vector<uint8_t> serializeBimPcResult(const BimPcResult& result);
        BimPcResult deserializeBimPcResult(const std::vector<uint8_t>& data);
    }

    // Slave용 네트워크 클라이언트
    class NetworkClient {
    public:
        NetworkClient();
        ~NetworkClient();

        bool connect(const std::string& server_address, uint16_t port);
        void disconnect();

        void setMessageCallback(MessageCallback callback) { message_callback_ = callback; }
        void setConnectionCallback(ConnectionCallback callback) { connection_callback_ = callback; }

        bool sendMessage(const NetworkMessage& message);

        bool isConnected() const { return connected_; }
        std::string getSlaveId() const { return slave_id_; }

    private:
        void clientThread(); // .cpp 파일에서 구현될 함수
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

        /// sendMessage()를 직렬화하여, 동시에 호출하는 쪽들(-t > 1일 때의 여러 처리
        /// 스레드, 그리고 -t 값과 무관하게 항상 실행되는 독립적인 하트비트 스레드)이
        /// 같은 소켓에 대한 send() 호출을 뒤섞지 못하게 한다. sendAll()은 메시지 하나당
        /// 여러 번의 send() 호출을 낼 수 있으므로, 이 락이 없으면 큰 결과와 하트비트
        /// (또는 결과 2개)가 동시에 전송될 때 메시지 중간에 뒤섞여, Master가 메시지
        /// 프레임으로 파싱하는 바이트 스트림이 깨질 수 있다.
        std::mutex send_mutex_;

        MessageCallback message_callback_;
        ConnectionCallback connection_callback_;
    };

} // namespace DPApp