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
#include "../include/Logger.h"
#include "../include/RuntimeConfig.h"

using namespace DPApp;

// Global cancellation flag for immediate stop
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
        // Initialize logger
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

        // Load configuration from environment
        cfg_ = DPApp::RuntimeConfig::loadFromEnv();

        // Parse command line arguments
        parseCommandLine(argc, argv);

        // Setup network client
        client_ = std::make_unique<NetworkClient>();
        client_->setMessageCallback([this](const NetworkMessage& msg, const std::string& sender_id) {
            handleMessage(msg, sender_id);
            });

        client_->setConnectionCallback([this](const std::string& client_id, bool connected) {
            handleConnection(client_id, connected);
            });

        // Setup task processor
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

        // Connect to server
        if (!client_->connect(server_address_, server_port_)) {
            ELOG << "Failed to connect to master server";
            return false;
        }

        running_ = true;

        // Start processing threads
        for (size_t i = 0; i < processing_threads_; ++i) {
            processing_thread_pool_.emplace_back(&SlaveApplication::processingLoop, this, i);
        }

        // Start status reporting thread
        status_thread_ = std::thread(&SlaveApplication::statusLoop, this);

        ILOG << "Slave worker started successfully";

        return true;
    }

    void stop() {
        if (!running_ && !shutdown_requested_) return;

        ILOG << "Stopping slave worker...";

        // Set running flag to false first
        running_ = false;
        shutdown_requested_ = true;

        // Notify all waiting threads about shutdown
        task_queue_cv_.notify_all();

        /// [Fix] The network must be disconnected first to unblock blocked send/recv calls
        if (client_) {
            ILOG << "Disconnecting network to unblock threads...";
            client_->disconnect();
        }

        // Processing threads shutdown (with timeout)
        ILOG << "Waiting for processing threads...";
        auto thread_shutdown_start = std::chrono::steady_clock::now();
        const auto max_wait_time = std::chrono::seconds(5);  // Max 5 seconds wait

        for (auto& thread : processing_thread_pool_) {
            if (thread.joinable()) {
                auto elapsed = std::chrono::steady_clock::now() - thread_shutdown_start;
                if (elapsed < max_wait_time) {
                    // Wait for remaining time
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

        // Status thread shutdown (with timeout)
        ILOG << "Waiting for status thread...";
        if (status_thread_.joinable()) {
            // 최대 3초 대기 후 detach
            bool joined = false;
            for (int i = 0; i < 30 && !joined; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                // status_thread는 running_ 체크하므로 곧 종료될 것
                // 하지만 sleep 중일 수 있으므로 타임아웃 필요
            }

            try {
                // 타임아웃 후에도 joinable이면 detach
                if (status_thread_.joinable()) {
                    WLOG << "Status thread join timeout, detaching...";
                    status_thread_.detach();
                }
            }
            catch (const std::exception& e) {
                WLOG << "Error with status thread: " << e.what();
            }
        }

        // Close stdin for input thread
        ILOG << "Closing stdin for input thread...";
#ifdef _WIN32
        try {
            // Windows에서 stdin의 대기 중인 I/O만 취소 (파일 닫지 않음)
            HANDLE hStdin = GetStdHandle(STD_INPUT_HANDLE);
            if (hStdin != INVALID_HANDLE_VALUE && hStdin != NULL) {
                CancelIoEx(hStdin, NULL);  // 대기 중인 I/O 취소
            }
            // _close()는 호출하지 않음 - assertion 발생 원인
        }
        catch (...) {
            // Already closed or error, ignore
        }
#else
        try {
            if (fcntl(STDIN_FILENO, F_GETFD) != -1) {
                close(STDIN_FILENO);
            }
        }
        catch (...) {
            // Already closed or error, ignore
        }
#endif

        // Input thread shutdown (with timeout)
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

        // Separate thread for input processing
        input_thread_ = std::thread([this]() {
            std::string command;
            while (running_ && !shutdown_requested_ && !force_exit_) {
                if (std::getline(std::cin, command)) {
                    if (!processCommand(command)) {
                        running_ = false;  // quit command processing
                        break;
                    }
                }

                // Check exit conditions even without input
                if (shutdown_requested_ || force_exit_) {
                    break;
                }
            }
            });

        // Main loop periodically checks status
        while (running_ && !shutdown_requested_ && !force_exit_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // Check connection status
            if (!client_->isConnected()) {
                WLOG << "Lost connection to master server";
                break;
            }
        }

        // Log shutdown reason
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
        TestChunk test_chunk;  // 테스트용 청크
        bool is_bimpc;
        bool is_test;  // 테스트 Task 여부

        TaskQueueItem() : is_bimpc(false), is_test(false) {}
        TaskQueueItem(const ProcessingTask& t, const PointCloudChunk& c)
            : task(t), chunk(c), is_bimpc(false), is_test(false) {
        }
        TaskQueueItem(const ProcessingTask& t, const BimPcChunk& bc)
            : task(t), bimpc_chunk(bc), is_bimpc(true), is_test(false) {
        }
        TaskQueueItem(const ProcessingTask& t, const TestChunk& tc)
            : task(t), test_chunk(tc), is_bimpc(false), is_test(true) {
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
        // Graceful quit
        else if (cmd == "quit" || cmd == "exit") {
            ILOG << "[GRACEFUL] finish current task then stop.";
            // Wake up waiting threads
            task_queue_cv_.notify_all();
            // Exit run() loop to call stop()
            return false;
        }
        // Immediate quit: attempt to cancel in-flight tasks too
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

            // Set immediate shutdown flags
            shutdown_requested_ = true;
            force_exit_ = true;
            running_ = false;

            // Wake up all waiting threads
            task_queue_cv_.notify_all();
#ifdef _WIN32
            _close(_fileno(stdin));
#else
            close(STDIN_FILENO);
#endif
            // Immediately disconnect network to unblock
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
            // Only attempt reconnection if we're still supposed to be running
            // and the disconnection wasn't initiated by us
            if (running_ && !shutdown_requested_) {
                WLOG << "Connection lost, attempting to reconnect...";
                // Reconnection logic would go here
            }
            else {
                // If we initiated the shutdown, don't try to reconnect
                ILOG << "Shutdown in progress, not attempting reconnection";
            }
        }
    }

    void handleTaskAssignment(const NetworkMessage& message) {
        try {
            const uint8_t* data = message.data.data();
            size_t offset = 0;

            /// 1. Read Task data size
            uint32_t task_data_size = 0;
            std::memcpy(&task_data_size, data + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);

            /// 2. Deserialize Task data
            std::vector<uint8_t> task_buffer(data + offset, data + offset + task_data_size);
            offset += task_data_size;
            ProcessingTask task = NetworkUtils::deserializeTask(task_buffer);

            /// 3. Read is_bimpc flag
            uint8_t is_bimpc_flag = data[offset];
            offset += 1;

            /// 4. Read Chunk data size
            uint32_t chunk_data_size = 0;
            std::memcpy(&chunk_data_size, data + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);

            ILOG << "Received task " << task.task_id
                << " (type: " << taskStr(task.task_type)
                << ", is_bimpc: " << (is_bimpc_flag ? "Yes" : "No")
                << ", chunk_data_size: " << chunk_data_size << " bytes)";

            /// 5. 테스트 Task 타입인지 확인
            bool is_test_task = (task.task_type == TaskType::TEST_ECHO ||
                task.task_type == TaskType::TEST_COMPUTE ||
                task.task_type == TaskType::TEST_DELAY ||
                task.task_type == TaskType::TEST_FAIL);

            if (is_test_task && chunk_data_size > 0) {
                /// Handle TestChunk
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
                /// Handle BimPcChunk
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
                /// Handle PointCloudChunk
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
                /// Handle case with no chunk data (빈 테스트 Task 포함)
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

            /// Get task from queqe
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

            /// 테스트 Task 처리
            if (item.is_test) {
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

                /// Send Test result back to Master
                sendTestResult(test_result);
                updateStats(test_result.success, processing_time / 1000.0);
            }
            /// Handle branch processing between BimPcChunk and PointCloudChunk
            else if (item.is_bimpc) {
                /// Process BimPcChunk
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

                /// Send BimPc result back to Master
                sendBimPcResult(result);
                updateStats(result.success, processing_time / 1000.0);
            }
            else {
                /// Process PointCloudChunk
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

                /// Send result back to Master
                sendTaskResult(result);
                updateStats(result.success, processing_time / 1000.0);
            }
        }

        ILOG << "Processing thread " << thread_id << " stopped";
    }

    void sendTaskResult(const ProcessingResult& result) {
        try {
            std::vector<uint8_t> result_data = NetworkUtils::serializeResult(result);

            /// Add is_bimpc flag (1 byte)
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

            /// Add is_bimpc flag (1 byte)
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
            // Check more frequently for quick shutdown support
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
    const size_t processing_threads_ = 1;

    // Task queue
    std::queue<TaskQueueItem> task_queue_;
    std::mutex task_queue_mutex_;
    std::condition_variable task_queue_cv_;

    // Processing threads
    std::vector<std::thread> processing_thread_pool_;
    std::thread status_thread_;

    // Input thread
    std::thread input_thread_;

    // Statistics
    std::mutex stats_mutex_;
    std::atomic<uint32_t> completed_tasks_{ 0 };
    std::atomic<uint32_t> failed_tasks_{ 0 };
    double total_processing_time_{ 0.0 };
    double avg_processing_time_{ 0.0 };
};

// Global variable (for signal handler)
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
    // Setup signal handler
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