#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <signal.h>
#include <queue>

#include "../include/PointCloudTypes.h"
#include "../include/NetworkManager.h"
#include "../include/TaskManager.h"

using namespace DPApp;

/// 즉시 종료(작업 중단) 신호로 쓸 전역 플래그
namespace DPApp {
    std::atomic<bool> g_cancel_requested{ false };
}

class SlaveApplication {
private:
    // 작업별 중단 플래그들을 관리
    std::map<uint32_t, std::shared_ptr<std::atomic<bool>>> active_task_cancellations_;
    std::mutex cancellation_mutex_;
public:
    SlaveApplication() : running_(false), server_port_(8080), processing_threads_(1) {}

    ~SlaveApplication() {
        stop();
    }

    bool initialize(int argc, char* argv[]) {
        // 명령행 인수 파싱
        parseCommandLine(argc, argv);

        // 네트워크 클라이언트 설정
        client_ = std::make_unique<NetworkClient>();
        client_->setMessageCallback([this](const NetworkMessage& msg, const std::string& sender_id) {
            handleMessage(msg, sender_id);
            });

        client_->setConnectionCallback([this](const std::string& client_id, bool connected) {
            handleConnection(client_id, connected);
            });

        // 작업 처리기 설정
        task_processor_ = std::make_unique<TaskProcessor>();

        // 커스텀 프로세서 등록 (필요한 경우)
        registerCustomProcessors();

        return true;
    }

    bool start() {
        if (running_) return false;

        DPApp::g_cancel_requested.store(false, std::memory_order_relaxed);

        std::cout << "Starting DPApp Slave Worker..." << std::endl;
        std::cout << "Server: " << server_address_ << ":" << server_port_ << std::endl;
        std::cout << "Processing threads: " << processing_threads_ << std::endl;
        std::cout << "Slave ID: " << client_->getSlaveId() << std::endl;

        // 서버에 연결
        if (!client_->connect(server_address_, server_port_)) {
            std::cerr << "Failed to connect to master server" << std::endl;
            return false;
        }

        running_ = true;

        // 작업 처리 스레드들 시작
        for (size_t i = 0; i < processing_threads_; ++i) {
            processing_thread_pool_.emplace_back(&SlaveApplication::processingLoop, this, i);
        }

        // 상태 보고 스레드 시작
        status_thread_ = std::thread(&SlaveApplication::statusLoop, this);

        std::cout << "Slave worker started successfully" << std::endl;

        return true;
    }

    void stop() {
        if (!running_) return;

        std::cout << "Stopping slave worker..." << std::endl;

        // 1. running 플래그를 먼저 false로 설정
        running_ = false;

        // 2. 클라이언트 연결을 먼저 해제 (네트워크 스레드 종료를 위해)
        if (client_) {
            client_->disconnect();
        }

        // 3. 작업 큐 조건 변수 통지
        task_queue_cv_.notify_all();

        // 4. 상태 스레드 종료 대기
        if (status_thread_.joinable()) {
            status_thread_.join();
        }

        // 5. 모든 처리 스레드 종료 대기
        for (auto& thread : processing_thread_pool_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
        processing_thread_pool_.clear();

        std::cout << "Slave worker stopped" << std::endl;
    }

    void run() {
        if (!start()) {
            return;
        }

        printHelp();

        std::string command;
        while (running_) {
            // 강제 종료 요청 확인
            if (shutdown_requested_) {
                std::cout << "Shutdown requested by master, stopping..." << std::endl;
                break;
            }

            // 사용자 명령 입력 처리
            if (std::getline(std::cin, command)) {
                if (!processCommand(command)) {
                    break; // quit 명령 처리됨
                }
            }
        }

        stop();
    }

private:
    std::atomic<bool> shutdown_requested_{ false };

    struct TaskQueueItem {
        ProcessingTask task;
        PointCloudChunk chunk;
        std::chrono::steady_clock::time_point received_time;

        TaskQueueItem(const ProcessingTask& t, const PointCloudChunk& c)
            : task(t), chunk(c), received_time(std::chrono::steady_clock::now()) {
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
                if (i + 1 < argc) {
                    processing_threads_ = std::stoul(argv[++i]);
                    if (processing_threads_ == 0) {
                        processing_threads_ = 1;
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
        std::cout << "Usage: slave [options]" << std::endl;
        std::cout << "Options:" << std::endl;
        std::cout << "  -s, --server <address>   Master server address (default: localhost)" << std::endl;
        std::cout << "  -p, --port <port>        Master server port (default: 8080)" << std::endl;
        std::cout << "  -t, --threads <count>    Processing thread count (default: 1)" << std::endl;
        std::cout << "  -h, --help               Show this help message" << std::endl;
    }

    void printHelp() {
        std::cout << "=== DPApp Slave Console Commands ===\n";
        std::cout << "status        - Show worker status\n";
        std::cout << "stats         - Show processing statistics\n";
        std::cout << "tasks         - Show supported task types\n";
        std::cout << "queue         - Show task queue status\n";
        std::cout << "quit          - Graceful stop (finish current task)\n";
        std::cout << "quit-now      - Immediate stop (cancel current task)\n";
        std::cout << "help          - Show this help\n";
        std::cout << "====================================\n";
    }

    // processCommand 함수를 bool 반환형으로 변경
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
        /// quit: 그레이스풀 종료 ---
        else if (cmd == "quit" || cmd == "exit") {
            std::cout << "[GRACEFUL] finish current task then stop." << std::endl;
            // 새 작업은 더 이상 처리하지 않도록
            running_ = false;
            // 대기 중 스레드 깨우기
            task_queue_cv_.notify_all();
            // run() 루프를 빠져나가 stop() 호출로 이동
            return false;
        }
        /// 즉시 종료: 진행 중 작업도 중단 시도 ---
        else if (cmd == "quit-now" || cmd == "exit-now" || cmd == "quit!") {
            std::cout << "[IMMEDIATE] cancel in-flight task and stop NOW." << std::endl;
            DPApp::g_cancel_requested.store(true, std::memory_order_relaxed);
            running_ = false;
            task_queue_cv_.notify_all();
            return false;
        }
        else {
            std::cout << "Unknown command: " << cmd << std::endl;
            std::cout << "Type 'help' for available commands" << std::endl;
        }
        return true;
    }

    void handleMessage(const NetworkMessage& message, const std::string& sender_id) {
        switch (message.header.type) {
        case MessageType::TASK_ASSIGNMENT:
            handleTaskAssignment(message);
            break;

        case MessageType::HEARTBEAT:
            // 마스터로부터의 하트비트에 대한 응답
            sendHeartbeatResponse();
            break;

        case MessageType::SHUTDOWN:
            std::cout << "Received shutdown command from master" << std::endl;
            shutdown_requested_ = true;
            break;

        default:
            std::cout << "Unknown message type from master" << std::endl;
            break;
        }
    }

    void handleConnection(const std::string& client_id, bool connected) {
        if (connected) {
            std::cout << "Connected to master server as: " << client_id << std::endl;
        }
        else {
            std::cout << "Disconnected from master server" << std::endl;
            if (running_) {
                std::cout << "Connection lost, attempting to reconnect..." << std::endl;
                // 재연결 로직은 NetworkClient에서 처리
            }
        }
    }

    void handleTaskAssignment(const NetworkMessage& message) {
        try {
            // 메시지 데이터에서 작업과 청크 정보 분리
            // 실제로는 더 정교한 직렬화/역직렬화가 필요

            // 임시로 간단한 분리 (실제 구현에서는 프로토콜 정의 필요)
            if (message.data.size() < sizeof(uint32_t)) {
                throw std::runtime_error("Invalid task assignment message");
            }

            // 여기서는 NetworkUtils의 함수들을 사용하여 분리
            // 실제 구현에서는 메시지 프로토콜을 더 정교하게 정의해야 함

            ProcessingTask task;
            PointCloudChunk chunk;

            // 임시 구현: 하드코딩된 작업 생성
            task.task_id = 1;  // 실제로는 메시지에서 파싱
            task.chunk_id = 1;
            task.task_type = "filter";  // 실제로는 메시지에서 파싱

            // 임시 청크 데이터 (실제로는 메시지에서 파싱)
            chunk.chunk_id = 1;
            chunk.points.resize(1000);
            for (size_t i = 0; i < chunk.points.size(); ++i) {
                chunk.points[i] = Point3D(i * 0.1, i * 0.1, i * 0.01);
            }

            // 작업을 큐에 추가
            {
                std::lock_guard<std::mutex> lock(task_queue_mutex_);
                task_queue_.emplace(task, chunk);
            }

            task_queue_cv_.notify_one();

            std::cout << "Received task " << task.task_id
                << " (type: " << task.task_type
                << ", chunk: " << chunk.chunk_id
                << ", points: " << chunk.points.size() << ")" << std::endl;

        }
        catch (const std::exception& e) {
            std::cerr << "Error processing task assignment: " << e.what() << std::endl;
        }
    }

    void sendHeartbeatResponse() {
        NetworkMessage response(MessageType::HEARTBEAT,
            std::vector<uint8_t>(client_->getSlaveId().begin(),
                client_->getSlaveId().end()));
        client_->sendMessage(response);
    }

    void processingLoop(size_t thread_id) {
        std::cout << "Processing thread " << thread_id << " started" << std::endl;

        while (running_) {
            TaskQueueItem item(ProcessingTask{}, PointCloudChunk{});

            // 작업 큐에서 작업 가져오기
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

            std::cout << "Thread " << thread_id << " processing task " << item.task.task_id << std::endl;

            // 작업 처리
            auto start_time = std::chrono::steady_clock::now();
            ProcessingResult result = task_processor_->processTask(item.task, item.chunk);
            auto end_time = std::chrono::steady_clock::now();

            auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                end_time - start_time).count();

            std::cout << "Thread " << thread_id << " completed task " << item.task.task_id
                << " in " << processing_time << "ms (success: "
                << (result.success ? "Yes" : "No") << ")" << std::endl;

            // 결과를 마스터에게 전송
            sendTaskResult(result);

            // 통계 업데이트
            updateStats(result.success, processing_time / 1000.0);
        }

        std::cout << "Processing thread " << thread_id << " stopped" << std::endl;
    }

    void sendTaskResult(const ProcessingResult& result) {
        try {
            std::vector<uint8_t> result_data = NetworkUtils::serializeResult(result);
            NetworkMessage message(MessageType::TASK_RESULT, result_data);

            if (client_->sendMessage(message)) {
                std::cout << "Task result " << result.task_id << " sent to master" << std::endl;
            }
            else {
                std::cerr << "Failed to send task result " << result.task_id << " to master" << std::endl;
            }

        }
        catch (const std::exception& e) {
            std::cerr << "Error sending task result: " << e.what() << std::endl;
        }
    }

    void statusLoop() {
        while (running_) {
            // 더 자주 확인하여 빠른 종료 지원
            for (int i = 0; i < 60 && running_; ++i) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            if (running_) {
                std::cout << "=== Worker Status Update ===" << std::endl;
                printProcessingStats();
                printQueueStatus();
                std::cout << "============================" << std::endl;
            }
        }
    }

    void registerCustomProcessors() {
        // 필요한 경우 커스텀 프로세서 등록
        // 예: 특별한 포인트클라우드 처리 알고리즘

        /*
        task_processor_->registerProcessor("custom_filter",
            [](const ProcessingTask& task, const PointCloudChunk& chunk) -> ProcessingResult {
                ProcessingResult result;
                result.task_id = task.task_id;
                result.chunk_id = task.chunk_id;
                result.success = true;

                // 커스텀 처리 로직
                result.processed_points = chunk.points; // 임시

                return result;
            });
        */
    }

    void updateStats(bool success, double processing_time) {
        std::lock_guard<std::mutex> lock(stats_mutex_);

        if (success) {
            completed_tasks_++;
        }
        else {
            failed_tasks_++;
        }

        total_processing_time_ += processing_time;

        if (completed_tasks_ + failed_tasks_ > 0) {
            avg_processing_time_ = total_processing_time_ / (completed_tasks_ + failed_tasks_);
        }
    }

    void printWorkerStatus() {
        std::cout << "=== Worker Status ===" << std::endl;
        std::cout << "Connected: " << (client_->isConnected() ? "Yes" : "No") << std::endl;
        std::cout << "Slave ID: " << client_->getSlaveId() << std::endl;
        std::cout << "Server: " << server_address_ << ":" << server_port_ << std::endl;
        std::cout << "Processing threads: " << processing_threads_ << std::endl;
        std::cout << "Running: " << (running_ ? "Yes" : "No") << std::endl;
        std::cout << "===================" << std::endl;
    }

    void printProcessingStats() {
        std::lock_guard<std::mutex> lock(stats_mutex_);

        std::cout << "=== Processing Statistics ===" << std::endl;
        std::cout << "Completed tasks: " << completed_tasks_ << std::endl;
        std::cout << "Failed tasks: " << failed_tasks_ << std::endl;

        uint32_t total_tasks = completed_tasks_ + failed_tasks_;
        if (total_tasks > 0) {
            double success_rate = static_cast<double>(completed_tasks_) / total_tasks * 100;
            std::cout << "Success rate: " << success_rate << "%" << std::endl;
        }

        std::cout << "Average processing time: " << avg_processing_time_ << "s" << std::endl;
        std::cout << "Total processing time: " << total_processing_time_ << "s" << std::endl;
        std::cout << "=============================" << std::endl;
    }

    void printSupportedTasks() {
        auto task_types = task_processor_->getSupportedTaskTypes();

        std::cout << "=== Supported Task Types ===" << std::endl;
        for (const auto& task_type : task_types) {
            std::cout << "- " << task_type << std::endl;
        }
        std::cout << "============================" << std::endl;
    }

    void printQueueStatus() {
        std::lock_guard<std::mutex> lock(task_queue_mutex_);

        std::cout << "=== Task Queue Status ===" << std::endl;
        std::cout << "Queue size: " << task_queue_.size() << std::endl;
        std::cout << "=========================" << std::endl;
    }

private:
    std::unique_ptr<NetworkClient> client_;
    std::unique_ptr<TaskProcessor> task_processor_;

    std::atomic<bool> running_;
    std::string server_address_ = "localhost";
    uint16_t server_port_;
    size_t processing_threads_;

    // 작업 큐
    std::queue<TaskQueueItem> task_queue_;
    std::mutex task_queue_mutex_;
    std::condition_variable task_queue_cv_;

    // 처리 스레드들
    std::vector<std::thread> processing_thread_pool_;
    std::thread status_thread_;

    // 통계
    std::mutex stats_mutex_;
    std::atomic<uint32_t> completed_tasks_{ 0 };
    std::atomic<uint32_t> failed_tasks_{ 0 };
    std::atomic<double> total_processing_time_{ 0.0 };
    std::atomic<double> avg_processing_time_{ 0.0 };
};

// 전역 변수 (시그널 핸들러용)
static SlaveApplication* g_app = nullptr;

void signalHandler(int signal) {
    std::cout << std::endl << "Received signal " << signal << ", shutting down..." << std::endl;
    if (g_app) {
        g_app->stop();
    }
    exit(0);
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

    app.run();

    return 0;
}