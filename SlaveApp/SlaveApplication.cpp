#ifdef _WIN32
#define NOMINMAX
#include <io.h>
#endif

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <signal.h>
#include <queue>
#include <iomanip>

#ifndef _WIN32
#include <unistd.h>
#endif

#include "../include/PointCloudTypes.h"
#include "../include/NetworkManager.h"
#include "../include/TaskManager.h"
#include "../include/TestTask.h"  // 테스트 Task 처리용
#include "../include/bimtree/IcpProcessor.hpp"
#include "../include/Logger.h"
#include "../include/RuntimeConfig.h"

using namespace DPApp;

// 즉시 중지를 위한 전역 취소 플래그
namespace DPApp {
    std::atomic<bool> g_cancel_requested{ false };
}

class SlaveApplication {
public:
    SlaveApplication() : running_(false), server_port_(8080), processing_threads_(1) {}

    ~SlaveApplication() {
        stop();
    }

    bool initialize(int argc, char* argv[]) {
        // 로거 초기화
        std::time_t t = std::time(nullptr);
        std::tm tm{};
#ifdef _WIN32
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        char logFilename[512];
        std::strftime(logFilename, sizeof(logFilename), "SlaveApp_%Y-%m-%d_%H-%M-%S.log", &tm);
        DPApp::Logger::initialize(logFilename);
        DPApp::Logger::setMinLevel(DPApp::LogLevel::INFO);

        // 환경변수로부터 설정 로딩
        cfg_ = DPApp::RuntimeConfig::loadFromEnv();

        // 명령줄 인자 파싱
        parseCommandLine(argc, argv);

        // 네트워크 클라이언트 설정
        client_ = std::make_unique<NetworkClient>();
        client_->setMessageCallback([this](const NetworkMessage& msg, const std::string& sender_id) {
            handleMessage(msg, sender_id);
            });

        client_->setConnectionCallback([this](const std::string& client_id, bool connected) {
            handleConnection(client_id, connected);
            });

        // 태스크 처리기 설정
        task_processor_ = std::make_unique<TaskProcessor>();

        return true;
    }

    bool start() {
        if (running_) return false;

        DPApp::g_cancel_requested.store(false, std::memory_order_relaxed);

        ILOG << "Starting DPApp Slave Worker...";
        ILOG << "Server: " << server_address_ << ":" << server_port_;
        ILOG << "Processing threads: " << processing_threads_;
        ILOG << "Slave ID: " << client_->getSlaveId();

        // 서버에 연결. 한 번 시도해서 실패하면 바로 포기하는 대신 재시도한다.
        // 예전에는 Master가 막 시작된 직후의 일시적인 연결 거부(또는 잠깐의 네트워크
        // 문제)만으로도 Slave 프로세스가 즉시 죽어버렸다; 이제는 연결이 끊겼을 때와
        // 마찬가지로 재시도한다.
        const int max_attempts = cfg_.slave_max_reconnect_attempts; // <=0이면 무한 재시도
        int attempt = 0;
        bool connected = false;
        while (!connected) {
            ++attempt;
            connected = client_->connect(server_address_, server_port_);
            if (connected) break;

            if (max_attempts > 0 && attempt >= max_attempts) {
                ELOG << "Failed to connect to master server after " << attempt << " attempt(s), giving up";
                return false;
            }

            WLOG << "Failed to connect to master server (attempt " << attempt
                << (max_attempts > 0 ? "/" + std::to_string(max_attempts) : "")
                << "), retrying in " << cfg_.slave_reconnect_interval_seconds << "s...";
            for (int waited = 0; waited < cfg_.slave_reconnect_interval_seconds; ++waited) {
                if (DPApp::g_cancel_requested.load(std::memory_order_relaxed)) {
                    ELOG << "Connection retry cancelled";
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        running_ = true;

        // 처리 스레드 시작
        for (size_t i = 0; i < processing_threads_; ++i) {
            processing_thread_pool_.emplace_back(&SlaveApplication::processingLoop, this, i);
        }

        // 상태 보고 스레드 시작
        status_thread_ = std::thread(&SlaveApplication::statusLoop, this);

        ILOG << "Slave worker started successfully";

        return true;
    }

    void stop() {
        if (!running_ && !shutdown_requested_) return;

        ILOG << "Stopping slave worker...";

        // running 플래그를 먼저 false로 설정
        running_ = false;
        shutdown_requested_ = true;

        // 대기 중인 모든 스레드에게 종료를 알림
        task_queue_cv_.notify_all();

        /// [수정 이력] 블로킹된 send/recv 호출을 풀어주려면 네트워크를 먼저 끊어야 한다
        if (client_) {
            ILOG << "Disconnecting network to unblock threads...";
            client_->disconnect();
        }

        // 처리 스레드 종료 (타임아웃 적용)
        ILOG << "Waiting for processing threads...";
        auto thread_shutdown_start = std::chrono::steady_clock::now();
        const auto max_wait_time = std::chrono::seconds(5);  // 최대 5초 대기

        for (auto& thread : processing_thread_pool_) {
            if (thread.joinable()) {
                auto elapsed = std::chrono::steady_clock::now() - thread_shutdown_start;
                if (elapsed < max_wait_time) {
                    // 남은 시간만큼 대기
                    auto remaining = max_wait_time - elapsed;
                    if (std::chrono::milliseconds(100) < remaining) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }

                    try {
                        thread.join();
                        ILOG << "Processing thread joined successfully";
                    }
                    catch (const std::exception& e) {
                        WLOG << "Error joining processing thread: " << e.what();
                        thread.detach();
                    }
                }
                else {
                    WLOG << "Processing thread join timeout, detaching...";
                    thread.detach();
                }
            }
        }
        processing_thread_pool_.clear();

        // status 스레드 종료 (타임아웃 적용)
        ILOG << "Waiting for status thread...";
        if (status_thread_.joinable()) {
            // 최대 3초 대기 후 detach
            bool joined = false;
            for (int i = 0; i < 30 && !joined; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                // status_thread가 running_ 체크하므로 곧 끝날 것이므로 타임아웃 불필요
            }

            try {
                // 타임아웃 이후에도 joinable하면 detach
                if (status_thread_.joinable()) {
                    WLOG << "Status thread join timeout, detaching...";
                    status_thread_.detach();
                }
            }
            catch (const std::exception& e) {
                WLOG << "Error with status thread: " << e.what();
            }
        }

        // 입력 스레드를 위해 stdin 닫기
        ILOG << "Closing stdin for input thread...";
#ifdef _WIN32
        try {
            // Windows에서는 stdin의 대기 중인 I/O만 취소 (완전 종료는 하지 않음)
            HANDLE hStdin = GetStdHandle(STD_INPUT_HANDLE);
            if (hStdin != INVALID_HANDLE_VALUE && hStdin != NULL) {
                CancelIoEx(hStdin, NULL);  // 대기 중인 I/O 취소
            }
            // _close()는 호출하지 않음 - assertion 발생 방지
        }
        catch (...) {
            // 이미 닫혔거나 오류 발생 -- 무시
        }
#else
        try {
            if (fcntl(STDIN_FILENO, F_GETFD) != -1) {
                close(STDIN_FILENO);
            }
        }
        catch (...) {
            // 이미 닫혔거나 오류 발생 -- 무시
        }
#endif

        // 입력 스레드 종료 (타임아웃 적용)
        ILOG << "Waiting for input thread...";
        if (input_thread_.joinable()) {
            // 최대 2초 대기
            bool joined = false;
            auto input_start = std::chrono::steady_clock::now();
            while (!joined && std::chrono::steady_clock::now() - input_start < std::chrono::seconds(2)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            try {
                if (input_thread_.joinable()) {
                    WLOG << "Input thread join timeout, detaching...";
                    input_thread_.detach();
                }
            }
            catch (const std::exception& e) {
                WLOG << "Error with input thread: " << e.what();
            }
        }

        ILOG << "Slave worker stopped completely";
    }

    void handleNetworkDisconnect() {
        if (running_) {
            ILOG << "Network disconnected, initiating graceful shutdown...";
            running_ = false;
            task_queue_cv_.notify_all();
        }
    }

    void run() {
        if (!start()) {
            return;
        }

        printHelp();

        // 입력 처리를 위한 별도 스레드
        input_thread_ = std::thread([this]() {
            std::string command;
            while (running_ && !shutdown_requested_ && !force_exit_) {
                if (std::getline(std::cin, command)) {
                    if (!processCommand(command)) {
                        running_ = false;  // quit 명령 처리
                        break;
                    }
                }

                // 입력이 없어도 종료 조건은 계속 확인
                if (shutdown_requested_ || force_exit_) {
                    break;
                }
            }
            });

        // 메인 루프에서 주기적으로 상태 확인
        while (running_ && !shutdown_requested_ && !force_exit_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // 연결 상태 확인
            if (!client_->isConnected()) {
                WLOG << "Lost connection to master server";
                break;
            }
        }

        // 종료 사유 로그
        if (shutdown_requested_) {
            ILOG << "Shutdown requested by master, stopping...";
        }
        else if (force_exit_) {
            ILOG << "Force exit requested, stopping immediately...";
        }
        else if (!running_) {
            ILOG << "Normal shutdown requested, stopping...";
        }

        stop();

        if (force_exit_) {
            std::exit(0);
        }
    }

private:
    std::atomic<bool> shutdown_requested_{ false };
    std::atomic<bool> force_exit_{ false };

    struct TaskQueueItem {
        ProcessingTask task;
        PointCloudChunk chunk;
        BimPcChunk bimpc_chunk;
        TestChunk test_chunk;
        icp::IcpChunk icp_chunk;       /// <-- 추가
        bool is_bimpc;
        bool is_test;
        bool is_icp;                   /// <-- 추가

        TaskQueueItem() : is_bimpc(false), is_test(false), is_icp(false) {}
        TaskQueueItem(const ProcessingTask& t, const PointCloudChunk& c)
            : task(t), chunk(c), is_bimpc(false), is_test(false), is_icp(false) {
        }
        TaskQueueItem(const ProcessingTask& t, const BimPcChunk& bc)
            : task(t), bimpc_chunk(bc), is_bimpc(true), is_test(false), is_icp(false) {
        }
        TaskQueueItem(const ProcessingTask& t, const TestChunk& tc)
            : task(t), test_chunk(tc), is_bimpc(false), is_test(true), is_icp(false) {
        }
        /// ICP 청크를 위해 추가된 생성자
        TaskQueueItem(const ProcessingTask& t, const icp::IcpChunk& ic)
            : task(t), icp_chunk(ic), is_bimpc(false), is_test(false), is_icp(true) {
        }
    };

    void parseCommandLine(int argc, char* argv[]) {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];

            if (arg == "-s" || arg == "--server") {
                if (i + 1 < argc) {
                    server_address_ = argv[++i];
                }
            }
            else if (arg == "-p" || arg == "--port") {
                if (i + 1 < argc) {
                    server_port_ = static_cast<uint16_t>(std::stoi(argv[++i]));
                }
            }
            else if (arg == "-t" || arg == "--threads") {
                /// [수정 이력] 예전에는 processing_threads_가 1로 하드코딩된 `const size_t`여서,
                /// config/slave.ini의 `processing_threads` 설정을 어떻게 바꿔도 아무 효과가
                /// 없었다. 지금은 일반 멤버 변수로 바뀌어 CLI에서 재정의할 수 있다.
                if (i + 1 < argc) {
                    int threads = std::stoi(argv[++i]);
                    if (threads > 0) {
                        processing_threads_ = static_cast<size_t>(threads);
                    }
                }
            }
            else if (arg == "-h" || arg == "--help") {
                printUsage();
                exit(0);
            }
        }
    }

    void printUsage() {
        ILOG << "Usage: slave [options]";
        ILOG << "Options:";
        ILOG << "  -s, --server <address>   Master server address (default: localhost)";
        ILOG << "  -p, --port <port>        Master server port (default: 8080)";
        ILOG << "  -t, --threads <count>    Number of processing threads (default: 1)";
        ILOG << "  -h, --help               Show this help message";
    }

    void printHelp() {
        ILOG << "";
        ILOG << "=== DPApp Slave Console Commands ===";
        ILOG << "status        - Show worker status";
        ILOG << "stats         - Show processing statistics";
        ILOG << "tasks         - Show supported task types";
        ILOG << "queue         - Show task queue status";
        ILOG << "quit          - Graceful stop (finish current task)";
        ILOG << "quit-now      - Immediate stop (cancel current task)";
        ILOG << "help          - Show this help";
        ILOG << "====================================";
        ILOG << "";
    }

    bool processCommand(const std::string& command) {
        if (command.empty()) return true;

        std::istringstream iss(command);
        std::string cmd;
        iss >> cmd;

        if (cmd == "status") {
            printWorkerStatus();
        }
        else if (cmd == "stats") {
            printProcessingStats();
        }
        else if (cmd == "tasks") {
            printSupportedTasks();
        }
        else if (cmd == "queue") {
            printQueueStatus();
        }
        else if (cmd == "help") {
            printHelp();
        }
        // 정상(graceful) 종료
        else if (cmd == "quit" || cmd == "exit") {
            ILOG << "[GRACEFUL] finish current task then stop.";
            // 대기 중인 스레드들을 깨움
            task_queue_cv_.notify_all();
            // run() 루프를 빠져나가 stop()을 호출하게 함
            return false;
        }
        // 즉시 종료: 진행 중인 태스크도 취소를 시도
        else if (cmd == "quit-now" || cmd == "exit-now" || cmd == "quit!") {
            ILOG << "[IMMEDIATE] cancel in-flight task and stop NOW.";
            DPApp::g_cancel_requested.store(true, std::memory_order_relaxed);
            task_queue_cv_.notify_all();
            return false;
        }
        else {
            WLOG << "Unknown command: " << cmd;
            WLOG << "Type 'help' for available commands";
        }
        return true;
    }

    void handleMessage(const NetworkMessage& message, const std::string& sender_id) {
        switch (message.header.type) {
        case MessageType::TASK_ASSIGNMENT:
            if (!shutdown_requested_ && !force_exit_) {
                handleTaskAssignment(message);
            }
            break;

        case MessageType::HEARTBEAT:
            if (!shutdown_requested_ && !force_exit_) {
                sendHeartbeatResponse();
            }
            break;

        case MessageType::SHUTDOWN:
            ILOG << "Received SHUTDOWN command from master - initiating immediate shutdown";

            // 즉시 종료 플래그 설정
            shutdown_requested_ = true;
            force_exit_ = true;
            running_ = false;

            // 대기 중인 모든 스레드를 깨움
            task_queue_cv_.notify_all();
#ifdef _WIN32
            _close(_fileno(stdin));
#else
            close(STDIN_FILENO);
#endif
            // 블로킹을 풀기 위해 네트워크를 즉시 끊음
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                if (client_) {
                    client_->disconnect();
                }
                }).detach();

            break;

        default:
            WLOG << "Unknown message type from master: " << static_cast<int>(message.header.type);
            break;
        }
    }

    void handleConnection(const std::string& client_id, bool connected) {
        if (connected) {
            ILOG << "Connected to master server as: " << client_id;
        }
        else {
            ILOG << "Disconnected from master server";
            // 계속 실행 중이어야 하는 상태이고, 이 연결 끊김이 우리가 스스로 시작한 게
            // 아닐 때만 재연결을 시도한다
            if (running_ && !shutdown_requested_) {
                WLOG << "Connection lost, attempting to reconnect...";
                // 재연결 로직은 여기에 들어갈 예정
            }
            else {
                // 우리가 직접 종료를 시작한 것이면 재연결을 시도하지 않음
                ILOG << "Shutdown in progress, not attempting reconnection";
            }
        }
    }

    void handleTaskAssignment(const NetworkMessage& message) {
        try {
            const uint8_t* data = message.data.data();
            size_t offset = 0;

            /// 1. Task 데이터 크기 읽기
            uint32_t task_data_size = 0;
            std::memcpy(&task_data_size, data + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);

            /// 2. Task 데이터 역직렬화
            std::vector<uint8_t> task_buffer(data + offset, data + offset + task_data_size);
            offset += task_data_size;
            ProcessingTask task = NetworkUtils::deserializeTask(task_buffer);

            /// 3. is_bimpc 플래그 읽기
            uint8_t is_bimpc_flag = data[offset];
            offset += 1;

            /// 4. 청크 데이터 크기 읽기
            uint32_t chunk_data_size = 0;
            std::memcpy(&chunk_data_size, data + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);

            ILOG << "Received task " << task.task_id
                << " (type: " << taskStr(task.task_type)
                << ", is_bimpc: " << (is_bimpc_flag ? "Yes" : "No")
                << ", chunk_data_size: " << chunk_data_size << " bytes)";

            /// ICP Task 타입인지 확인
            bool is_icp_task = (task.task_type == TaskType::ICP_FINE_ALIGNMENT);

            /// 5. 테스트 Task 타입인지 확인
            bool is_test_task = (task.task_type == TaskType::TEST_ECHO ||
                task.task_type == TaskType::TEST_COMPUTE ||
                task.task_type == TaskType::TEST_DELAY ||
                task.task_type == TaskType::TEST_FAIL);
            
            /// ICP Task 처리
            if (is_icp_task && chunk_data_size > 0) {
                /// IcpChunk 처리
                std::vector<uint8_t> chunk_buffer(data + offset, data + offset + chunk_data_size);
                icp::IcpChunk icp_chunk = icp::deserializeIcpChunk(chunk_buffer);

                ILOG << "Task " << task.task_id << " is ICP task ("
                    << "source: " << icp_chunk.sourcePoints.size() << " pts, "
                    << "target: " << icp_chunk.targetPoints.size() << " pts)";

                {
                    std::lock_guard<std::mutex> lock(task_queue_mutex_);
                    task_queue_.push(TaskQueueItem(task, icp_chunk));
                }
                task_queue_cv_.notify_one();
            }
            else if (is_test_task && chunk_data_size > 0) {
                /// TestChunk 처리
                std::vector<uint8_t> chunk_buffer(data + offset, data + offset + chunk_data_size);
                TestChunk test_chunk = TestChunk::deserialize(chunk_buffer);

                ILOG << "Task " << task.task_id << " is TEST task ("
                    << taskStr(task.task_type) << ", "
                    << test_chunk.input_numbers.size() << " numbers, "
                    << "base_time: " << test_chunk.base_processing_time_ms << "ms)";

                {
                    std::lock_guard<std::mutex> lock(task_queue_mutex_);
                    task_queue_.push(TaskQueueItem(task, test_chunk));
                }
                task_queue_cv_.notify_one();
            }
            else if (is_bimpc_flag && chunk_data_size > 0) {
                /// BimPcChunk 처리
                std::vector<uint8_t> chunk_buffer(data + offset, data + offset + chunk_data_size);
                BimPcChunk bimpc_chunk = NetworkUtils::deserializeBimPcChunk(chunk_buffer);

                ILOG << "Task " << task.task_id << " has BimPcChunk ("
                    << bimpc_chunk.points.size() << " points, "
                    << bimpc_chunk.bim.faces.size() << " faces)";

                {
                    std::lock_guard<std::mutex> lock(task_queue_mutex_);
                    task_queue_.push(TaskQueueItem(task, bimpc_chunk));
                }
                task_queue_cv_.notify_one();
            }
            else if (chunk_data_size > 0) {
                /// PointCloudChunk 처리
                std::vector<uint8_t> chunk_buffer(data + offset, data + offset + chunk_data_size);
                PointCloudChunk chunk = NetworkUtils::deserializeChunk(chunk_buffer);

                ILOG << "Task " << task.task_id << " has PointCloudChunk ("
                    << chunk.points.size() << " points)";

                {
                    std::lock_guard<std::mutex> lock(task_queue_mutex_);
                    task_queue_.push(TaskQueueItem(task, chunk));
                }
                task_queue_cv_.notify_one();
            }
            else {
                /// 청크 데이터가 없는 경우 처리 (예: 테스트 Task)
                if (is_test_task) {
                    ILOG << "Task " << task.task_id << " is TEST task with default data";
                    TestChunk test_chunk = TestChunk::generate(task.chunk_id, 100, 10000);
                    {
                        std::lock_guard<std::mutex> lock(task_queue_mutex_);
                        task_queue_.push(TaskQueueItem(task, test_chunk));
                    }
                }
                else {
                    ILOG << "Task " << task.task_id << " has no chunk data";
                    {
                        std::lock_guard<std::mutex> lock(task_queue_mutex_);
                        task_queue_.push(TaskQueueItem(task, PointCloudChunk()));
                    }
                }
                task_queue_cv_.notify_one();
            }
        }
        catch (const std::exception& e) {
            ELOG << "Error handling task assignment: " << e.what();
        }
    }

    void sendHeartbeatResponse() {
        NetworkMessage response(MessageType::HEARTBEAT,
            std::vector<uint8_t>(client_->getSlaveId().begin(),
                client_->getSlaveId().end()));
        if (!client_->sendMessage(response)) {
            WLOG << "Failed to send heartbeat response";
        }
    }

    void processingLoop(size_t thread_id) {
        ILOG << "Processing thread " << thread_id << " started";

        while (running_) {
            TaskQueueItem item;

            /// 큐에서 태스크 가져오기
            {
                std::unique_lock<std::mutex> lock(task_queue_mutex_);
                task_queue_cv_.wait(lock, [this] { return !task_queue_.empty() || !running_; });

                if (!running_) break;

                if (!task_queue_.empty()) {
                    item = task_queue_.front();
                    task_queue_.pop();
                }
                else {
                    continue;
                }
            }

            ILOG << "Thread " << thread_id << " processing task " << item.task.task_id
                << " (is_bimpc: " << (item.is_bimpc ? "Yes" : "No")
                << ", is_test: " << (item.is_test ? "Yes" : "No") << ")";

            auto start_time = std::chrono::steady_clock::now();

            /// ICP Task 처리
            if (item.is_icp) {
                icp::IcpResult icp_result;
                try {
                    icp_result = processIcpTask(item.task, item.icp_chunk);
                }
                catch (const std::exception& e) {
                    icp_result.chunk_id = item.icp_chunk.chunk_id;
                    icp_result.success = false;
                    icp_result.errorMessage = "ICP processing exception: " + std::string(e.what());
                }

                auto end_time = std::chrono::steady_clock::now();
                auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    end_time - start_time).count();

                ILOG << "Thread " << thread_id << " completed ICP task " << item.task.task_id
                    << " in " << processing_time << "ms (success: "
                    << (icp_result.success ? "Yes" : "No")
                    << ", RMSE: " << icp_result.finalRMSE << ")";

                sendIcpResult(icp_result, item.task.task_id);
                updateStats(icp_result.success, processing_time / 1000.0);
            }
            /// 테스트 Task 처리
            else if (item.is_test) {
                TestResult test_result;
                try {
                    test_result = processTestTask(item.task, item.test_chunk);
                }
                catch (const std::exception& e) {
                    test_result.task_id = item.task.task_id;
                    test_result.chunk_id = item.task.chunk_id;
                    test_result.success = false;
                    test_result.error_message = "Test processing exception: " + std::string(e.what());
                }

                auto end_time = std::chrono::steady_clock::now();
                auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    end_time - start_time).count();

                ILOG << "Thread " << thread_id << " completed TEST task " << item.task.task_id
                    << " in " << processing_time << "ms (success: "
                    << (test_result.success ? "Yes" : "No") << ")";

                if (!test_result.success) {
                    WLOG << "Test Task " << item.task.task_id << " failed: " << test_result.error_message;
                }

                /// 테스트 결과를 Master로 전송
                sendTestResult(test_result);
                updateStats(test_result.success, processing_time / 1000.0);
            }
            /// BimPcChunk와 PointCloudChunk 사이의 분기 처리
            else if (item.is_bimpc) {
                /// BimPcChunk 처리
                BimPcResult result;
                try {
                    result = task_processor_->processTask(item.task, item.bimpc_chunk);
                }
                catch (const std::exception& e) {
                    result.task_id = item.task.task_id;
                    result.chunk_id = item.task.chunk_id;
                    result.success = false;
                    result.error_message = "Processing exception: " + std::string(e.what());
                }

                auto end_time = std::chrono::steady_clock::now();
                auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    end_time - start_time).count();

                ILOG << "Thread " << thread_id << " completed BimPc task " << item.task.task_id
                    << " in " << processing_time << "ms (success: "
                    << (result.success ? "Yes" : "No") << ")";

                if (!result.success) {
                    WLOG << "Task " << item.task.task_id << " failed: " << result.error_message;
                }

                /// BimPc 결과를 Master로 전송
                sendBimPcResult(result);
                updateStats(result.success, processing_time / 1000.0);
            }
            else {
                /// PointCloudChunk 처리
                ProcessingResult result;
                try {
                    result = task_processor_->processTask(item.task, item.chunk);
                }
                catch (const std::exception& e) {
                    result.task_id = item.task.task_id;
                    result.chunk_id = item.task.chunk_id;
                    result.success = false;
                    result.error_message = "Processing exception: " + std::string(e.what());
                }

                auto end_time = std::chrono::steady_clock::now();
                auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    end_time - start_time).count();

                ILOG << "Thread " << thread_id << " completed task " << item.task.task_id
                    << " in " << processing_time << "ms (success: "
                    << (result.success ? "Yes" : "No") << ")";

                if (!result.success) {
                    WLOG << "Task " << item.task.task_id << " failed: " << result.error_message;
                }

                /// 결과를 Master로 전송
                sendTaskResult(result);
                updateStats(result.success, processing_time / 1000.0);
            }
        }

        ILOG << "Processing thread " << thread_id << " stopped";
    }

    void sendTaskResult(const ProcessingResult& result) {
        try {
            std::vector<uint8_t> result_data = NetworkUtils::serializeResult(result);

            /// is_bimpc 플래그 추가 (1바이트)
            std::vector<uint8_t> message_data;
            message_data.push_back(0);  /// is_bimpc = false
            message_data.insert(message_data.end(), result_data.begin(), result_data.end());

            NetworkMessage message(MessageType::TASK_RESULT, message_data);

            if (client_->sendMessage(message)) {
                ILOG << "Task result " << result.task_id << " sent to master";
            }
            else {
                ELOG << "Failed to send task result " << result.task_id << " to master";
            }
        }
        catch (const std::exception& e) {
            ELOG << "Error sending task result: " << e.what();
        }
    }

    void sendBimPcResult(const BimPcResult& result) {
        try {
            std::vector<uint8_t> result_data = NetworkUtils::serializeBimPcResult(result);

            /// is_bimpc 플래그 추가 (1바이트)
            std::vector<uint8_t> message_data;
            message_data.push_back(1);  /// is_bimpc = true
            message_data.insert(message_data.end(), result_data.begin(), result_data.end());

            NetworkMessage message(MessageType::TASK_RESULT, message_data);

            if (client_->sendMessage(message)) {
                ILOG << "BimPcResult " << result.task_id << " sent to master";
            }
            else {
                ELOG << "Failed to send BimPcResult " << result.task_id << " to master";
            }
        }
        catch (const std::exception& e) {
            ELOG << "Error sending BimPcResult: " << e.what();
        }
    }

    /// =========================================
    /// 테스트 Task 처리 함수
    /// =========================================

    TestResult processTestTask(const ProcessingTask& task, TestChunk& chunk) {
        TestResult result;
        result.task_id = task.task_id;
        result.chunk_id = chunk.chunk_id;

        ILOG << "[TEST] Processing task " << task.task_id
            << " (type: " << taskStr(task.task_type) << ")";

        switch (task.task_type) {
        case TaskType::TEST_ECHO:
            result = TestProcessors::processEcho(task.task_id, chunk);
            break;

        case TaskType::TEST_COMPUTE:
            result = TestProcessors::processCompute(task.task_id, chunk);
            break;

        case TaskType::TEST_DELAY:
            result = TestProcessors::processDelay(task.task_id, chunk);
            break;

        case TaskType::TEST_FAIL:
            result = TestProcessors::processFail(task.task_id, chunk);
            break;

        default:
            result.success = false;
            result.error_message = "Unknown test task type: " + std::string(taskStr(task.task_type));
            break;
        }

        return result;
    }

    /// ICP 태스크 처리
    icp::IcpResult processIcpTask(const ProcessingTask& task, const icp::IcpChunk& chunk) {
        ILOG << "[ICP] Processing task " << task.task_id
            << " (chunk: " << chunk.chunk_id
            << ", source: " << chunk.sourcePoints.size()
            << ", target: " << chunk.targetPoints.size() << " pts)";

        icp::IcpResult result;
        result.chunk_id = chunk.chunk_id;

        try {
            /// ICP 처리 실행
            result = DPApp::IcpProcessors::processFineAlignment(chunk, g_cancel_requested);

            if (!result.success) {
                WLOG << "[ICP] Task " << task.task_id << " failed: " << result.errorMessage;
            }
        }
        catch (const std::exception& e) {
            result.success = false;
            result.errorMessage = std::string("Exception: ") + e.what();
            ELOG << "[ICP] Task " << task.task_id << " exception: " << e.what();
        }

        return result;
    }

    /// ICP 결과를 master로 전송
    void sendIcpResult(const icp::IcpResult& result, uint32_t task_id) {
        try {
            /// ICP 결과 직렬화
            std::vector<uint8_t> result_data = icp::serializeIcpResult(result);

            /// 메시지 포맷: [result_type: 1B][task_id: 4B][result_data]
            /// result_type: 0=normal, 1=bimpc, 2=test, 3=icp
            std::vector<uint8_t> message_data;
            message_data.reserve(1 + sizeof(uint32_t) + result_data.size());

            message_data.push_back(3);  /// result_type = 3 (ICP)

            /// task_id 추가
            size_t pos = message_data.size();
            message_data.resize(pos + sizeof(uint32_t));
            std::memcpy(message_data.data() + pos, &task_id, sizeof(uint32_t));

            /// 결과 데이터 추가
            message_data.insert(message_data.end(), result_data.begin(), result_data.end());

            NetworkMessage message(MessageType::TASK_RESULT, message_data);

            if (client_->sendMessage(message)) {
                ILOG << "IcpResult sent to master (task: " << task_id
                    << ", chunk: " << result.chunk_id
                    << ", success: " << (result.success ? "Yes" : "No")
                    << ", RMSE: " << result.finalRMSE
                    << ", time: " << result.processingTimeMs << "ms)";
            }
            else {
                ELOG << "Failed to send IcpResult (task: " << task_id << ")";
            }
        }
        catch (const std::exception& e) {
            ELOG << "Error sending IcpResult: " << e.what();
        }
    }

    void sendTestResult(const TestResult& result) {
        try {
            /// TestResult를 직렬화
            std::vector<uint8_t> result_data = result.serialize();

            /// is_test flag (2) 추가하여 Master가 테스트 결과임을 인식
            std::vector<uint8_t> message_data;
            message_data.push_back(2);  /// is_test = true (0: normal, 1: bimpc, 2: test)
            message_data.insert(message_data.end(), result_data.begin(), result_data.end());

            NetworkMessage message(MessageType::TASK_RESULT, message_data);

            if (client_->sendMessage(message)) {
                ILOG << "TestResult " << result.task_id << " sent to master"
                    << " (success: " << (result.success ? "Yes" : "No")
                    << ", time: " << result.processing_time_ms << "ms)";
            }
            else {
                ELOG << "Failed to send TestResult " << result.task_id << " to master";
            }
        }
        catch (const std::exception& e) {
            ELOG << "Error sending TestResult: " << e.what();
        }
    }

    void statusLoop() {
        while (running_) {
            // 빠른 종료를 위해 더 자주 확인
            for (int i = 0; i < 60 && running_; ++i) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            if (running_) {
                ILOG << "=== Worker Status Update ===";
                printProcessingStats();
                printQueueStatus();
                ILOG << "============================";
            }
        }
    }

    void updateStats(bool success, double processing_time) {
        std::lock_guard<std::mutex> lock(stats_mutex_);

        if (success) {
            completed_tasks_++;
        }
        else {
            failed_tasks_++;
        }

        total_processing_time_ = total_processing_time_ + processing_time;

        if (completed_tasks_ + failed_tasks_ > 0) {
            avg_processing_time_ = total_processing_time_ / (completed_tasks_ + failed_tasks_);
        }
    }

    void printWorkerStatus() {
        ILOG << "=== Worker Status ===";
        ILOG << "Connected: " << (client_->isConnected() ? "Yes" : "No");
        ILOG << "Slave ID: " << client_->getSlaveId();
        ILOG << "Server: " << server_address_ << ":" << server_port_;
        ILOG << "Running: " << (running_ ? "Yes" : "No");
        ILOG << "====================";
    }

    void printProcessingStats() {
        std::lock_guard<std::mutex> lock(stats_mutex_);

        ILOG << "=== Processing Statistics ===";
        ILOG << "Completed tasks: " << completed_tasks_;
        ILOG << "Failed tasks: " << failed_tasks_;

        uint32_t total_tasks = completed_tasks_ + failed_tasks_;
        if (total_tasks > 0) {
            double success_rate = static_cast<double>(completed_tasks_) / total_tasks * 100;
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << success_rate << "%";
            ILOG << "Success rate: " << oss.str();
        }

        std::ostringstream avg_oss;
        avg_oss << std::fixed << std::setprecision(3) << avg_processing_time_ << "s";
        ILOG << "Average processing time: " << avg_oss.str();

        std::ostringstream total_oss;
        total_oss << std::fixed << std::setprecision(1) << total_processing_time_ << "s";
        ILOG << "Total processing time: " << total_oss.str();

        ILOG << "=============================";
    }

    void printSupportedTasks() {
        auto task_types = task_processor_->getSupportedTaskTypes();

        ILOG << "=== Supported Task Types ===";
        for (const auto& task_type : task_types) {
            ILOG << "- " << taskStr(task_type);
        }
        ILOG << "============================";
    }

    void printQueueStatus() {
        std::lock_guard<std::mutex> lock(task_queue_mutex_);

        ILOG << "=== Task Queue Status ===";
        ILOG << "Queue size: " << task_queue_.size();
        ILOG << "=========================";
    }

private:
    DPApp::RuntimeConfig cfg_;

    std::unique_ptr<NetworkClient> client_;
    std::unique_ptr<TaskProcessor> task_processor_;

    std::atomic<bool> running_;
    std::string server_address_ = "localhost";
    uint16_t server_port_;
    size_t processing_threads_ = 1;

    // 태스크 큐
    std::queue<TaskQueueItem> task_queue_;
    std::mutex task_queue_mutex_;
    std::condition_variable task_queue_cv_;

    // 처리 스레드
    std::vector<std::thread> processing_thread_pool_;
    std::thread status_thread_;

    // 입력 스레드
    std::thread input_thread_;

    // 통계
    std::mutex stats_mutex_;
    std::atomic<uint32_t> completed_tasks_{ 0 };
    std::atomic<uint32_t> failed_tasks_{ 0 };
    double total_processing_time_{ 0.0 };
    double avg_processing_time_{ 0.0 };
};

// 전역 변수 (시그널 핸들러용)
static SlaveApplication* g_app = nullptr;

void signalHandler(int signal) {
    ILOG << "";
    ILOG << "Received signal " << signal << ", shutting down...";
    if (g_app) {
        g_app->stop();
    }
    //exit(0);
}

int main(int argc, char* argv[]) {
    // 시그널 핸들러 설정
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    SlaveApplication app;
    g_app = &app;

    if (!app.initialize(argc, argv)) {
        std::cerr << "Failed to initialize slave application" << std::endl;
        return 1;
    }

    try {
        app.run();
    }
    catch (const std::exception& e) {
        std::cerr << "Application error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Application exited normally" << std::endl;
    return 0;
}