#ifdef _WIN32
#define NOMINMAX   // Windows 헤더 포함하기 전에 정의
#include <io.h>    // For _close, _fileno
#endif

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <signal.h>
#include <queue>
#include <set>
#include <tuple>
#include <iomanip>

#ifndef _WIN32
#include <unistd.h>  // For close, STDIN_FILENO
#endif

#include "../include/PointCloudTypes.h"
#include "../include/NetworkManager.h"
#include "../include/TaskManager.h"
#include "../include/Logger.h"
#include "../include/RuntimeConfig.h"

using namespace DPApp;

/// 즉시 종료(작업 중단) 신호로 쓸 전역 플래그
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
        /// Initialize logger
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

        ILOG << "Starting DPApp Slave Worker...";
        ILOG << "Server: " << server_address_ << ":" << server_port_;
        ILOG << "Processing threads: " << processing_threads_;
        ILOG << "Slave ID: " << client_->getSlaveId();

        // 서버에 연결
        if (!client_->connect(server_address_, server_port_)) {
            ELOG << "Failed to connect to master server";
            return false;
        }

        running_ = true;

        // 작업 처리 스레드들 시작
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

        /// 1. Set running flag to false first
        running_ = false;
        shutdown_requested_ = true;

        /// 2. Notify all waiting threads about shutdown
        task_queue_cv_.notify_all();

        /// 3. Processing threads 종료 (타임아웃 적용)
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

        /// 4. Status thread 종료
        if (status_thread_.joinable()) {
            try {
                status_thread_.join();
            }
            catch (const std::exception& e) {
                WLOG << "Error joining status thread: " << e.what();
                status_thread_.detach();
            }
        }

#ifdef _WIN32
        try {
            if (_fileno(stdin) != -1) {
                _close(_fileno(stdin));
            }
        }
        catch (...) {
            // 이미 닫혔거나 오류 발생 시 무시
        }
#else
        try {
            if (fcntl(STDIN_FILENO, F_GETFD) != -1) {
                close(STDIN_FILENO);
            }
        }
        catch (...) {
            // 이미 닫혔거나 오류 발생 시 무시
        }
#endif

        if (input_thread_.joinable()) {
            try {
                input_thread_.join();
            }
            catch (const std::exception& e) {
                WLOG << "Error joining input thread: " << e.what();
                input_thread_.detach();
            }
        }

        /// 5. 네트워크 클라이언트 해제
        if (client_) {
            try {
                client_->disconnect();
            }
            catch (const std::exception& e) {
                WLOG << "Error disconnecting client: " << e.what();
            }
        }

        ILOG << "Slave worker stopped completely";
    }

    /// Graceful shutdown from network disconnect :
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

        // 입력 처리를 별도 스레드로 분리
        input_thread_ = std::thread([this]() {
            std::string command;
            while (running_ && !shutdown_requested_ && !force_exit_) {
                if (std::getline(std::cin, command)) {
                    if (!processCommand(command)) {
                        running_ = false;  // quit 명령 처리
                        break;
                    }
                }

                // 입력이 없어도 주기적으로 종료 조건 확인
                if (shutdown_requested_ || force_exit_) {
                    break;
                }
            }
            });

        // 메인 루프는 주기적으로 상태를 체크
        while (running_ && !shutdown_requested_ && !force_exit_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // 연결 상태 체크
            if (!client_->isConnected()) {
                WLOG << "Lost connection to master server";
                break;
            }
        }

        /// 종료 이유 로깅
        if (shutdown_requested_) {
            ILOG << "Shutdown requested by master, stopping...";
        }
        else if (force_exit_) {
            ILOG << "Force exit requested, stopping immediately...";
        }
        else if (!running_) {
            ILOG << "Normal shutdown requested, stopping...";
        }

        if (shutdown_requested_) {
            ILOG << "Shutdown requested by master, stopping...";
        }

        stop();

        if (force_exit_) {
            std::exit(0);
        }
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
        ILOG << "Usage: slave [options]";
        ILOG << "Options:";
        ILOG << "  -s, --server <address>   Master server address (default: localhost)";
        ILOG << "  -p, --port <port>        Master server port (default: 8080)";
        ILOG << "  -t, --threads <count>    Processing thread count (default: 1)";
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
            ILOG << "[GRACEFUL] finish current task then stop.";
            /// 대기 중 스레드 깨우기
            task_queue_cv_.notify_all();
            /// run() 루프를 빠져나가서 stop() 호출로 이동
            return false;
        }
        /// 즉시 종료: 진행 중 작업도 중단 시도 ---
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

            /// 즉시 종료 플래그 설정
            shutdown_requested_ = true;
            force_exit_ = true;
            running_ = false;

            // 모든 대기 중인 스레드 깨우기
            task_queue_cv_.notify_all();
#ifdef _WIN32
            _close(_fileno(stdin));
#else
            close(STDIN_FILENO);
#endif
            /// 네트워크 연결 즉시 해제하여 블로킹 해제
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
            /// Only attempt reconnection if we're still supposed to be running
            /// and the disconnection wasn't initiated by us
            if (running_ && !shutdown_requested_) {
                WLOG << "Connection lost, attempting to reconnect...";
                /// Reconnection logic would go here
            }
            else {
                /// If we initiated the shutdown, don't try to reconnect
                ILOG << "Shutdown in progress, not attempting reconnection";
            }
        }
    }

    void handleTaskAssignment(const NetworkMessage& message) {
        try {
            const std::vector<uint8_t>& combined_data = message.data;

            /// Check the size of combined_data
            if (combined_data.size() < sizeof(uint32_t)) {
                throw std::runtime_error("Message too small to contain task data size");
            }

            /// Read the first 4 bytes
            uint32_t task_data_size = 0;
            std::memcpy(&task_data_size, combined_data.data(), sizeof(uint32_t));

            size_t offset = sizeof(uint32_t);
            if (combined_data.size() < offset + task_data_size) {
                throw std::runtime_error("Message is smaller than expected task data size");
            }

            /// Extract task data and deserialization
            std::vector<uint8_t> task_data(combined_data.begin() + offset, combined_data.begin() + offset + task_data_size);
            ProcessingTask task = NetworkUtils::deserializeTask(task_data);

            /// Update offset
            offset += task_data_size;
            if (combined_data.size() <= offset) {
                throw std::runtime_error("No chunk data found after task data");
            }

            /// Deserialize the rest data (chunk_data)
            std::vector<uint8_t> chunk_data(combined_data.begin() + offset, combined_data.end());
            PointCloudChunk chunk = NetworkUtils::deserializeChunk(chunk_data);

            /// 작업을 큐에 추가
            {
                std::lock_guard<std::mutex> lock(task_queue_mutex_);
                task_queue_.emplace(task, chunk);
            }

            task_queue_cv_.notify_one();

            ILOG << "Received task " << task.task_id
                << " (type: " << taskStr(task.task_type)
                << ", chunk: " << chunk.chunk_id
                << ", points: " << chunk.points.size() << ")";

        }
        catch (const std::exception& e) {
            ELOG << "Error processing task assignment: " << e.what();

            // 에러 응답 전송
            ProcessingResult error_result;
            error_result.task_id = 0; // 파싱에 실패했으므로 알 수 없음
            error_result.chunk_id = 0;
            error_result.success = false;
            error_result.error_message = "Task assignment parsing failed: " + std::string(e.what());
            sendTaskResult(error_result);
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
            TaskQueueItem item(ProcessingTask{}, PointCloudChunk{});

            /// 작업 큐에서 작업 가져오기
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

            ILOG << "Thread " << thread_id << " processing task " << item.task.task_id;

            /// 작업 처리
            auto start_time = std::chrono::steady_clock::now();

            ProcessingResult result;
            try {
                // 취소 가능한 작업 처리
                {
                    std::lock_guard<std::mutex> cancel_lock(cancellation_mutex_);
                    active_task_cancellations_[item.task.task_id] = std::make_shared<std::atomic<bool>>(false);
                }

                result = task_processor_->processTask(item.task, item.chunk);

                // 취소 플래그 제거
                {
                    std::lock_guard<std::mutex> cancel_lock(cancellation_mutex_);
                    active_task_cancellations_.erase(item.task.task_id);
                }

            }
            catch (const std::exception& e) {
                result.task_id = item.task.task_id;
                result.chunk_id = item.task.chunk_id;
                result.success = false;
                result.error_message = "Processing exception: " + std::string(e.what());

                // 취소 플래그 제거
                {
                    std::lock_guard<std::mutex> cancel_lock(cancellation_mutex_);
                    active_task_cancellations_.erase(item.task.task_id);
                }
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

            /// 결과를 마스터에게 전송
            sendTaskResult(result);

            /// 통계 업데이트
            updateStats(result.success, processing_time / 1000.0);
        }

        ILOG << "Processing thread " << thread_id << " stopped";
    }

    void sendTaskResult(const ProcessingResult& result) {
        try {
            std::vector<uint8_t> result_data = NetworkUtils::serializeResult(result);
            NetworkMessage message(MessageType::TASK_RESULT, result_data);

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

    void statusLoop() {
        while (running_) {
            /// 더 자주 확인하여 빠른 종료 지원
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

    void registerCustomProcessors() {
        /// registerCustomProcessors는 이런 경우에 :
        /// 
        /// 특정 프로젝트나 클라이언트만을 위한 특수 알고리즘
        ///     설정 파일이나 명령행 옵션에 따라 달라지는 기능
        ///     실험적이거나 임시적인 기능
        ///     외부 라이브러리나 하드웨어에 의존하는 기능
        /// Compare to PointCloudProcessors in TaskManager.h

        ILOG << "Registering custom point cloud processors...";

        /// 필요한 경우 커스텀 프로세서 등록
        /// 
        /* sample templet
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

        // 1. 노이즈 제거 필터 (Statistical Outlier Removal)
        task_processor_->registerProcessor(TaskType::OUTLIER_FILTER,
            [](const ProcessingTask& task, const PointCloudChunk& chunk) -> ProcessingResult {
                ProcessingResult result;
                result.task_id = task.task_id;
                result.chunk_id = task.chunk_id;
                result.success = true;

                try {
                    // 통계적 이상치 제거 알고리즘
                    std::vector<Point3D> filtered_points;
                    const double std_dev_threshold = 2.0; // 표준편차 임계값
                    const int k_neighbors = std::min(10, static_cast<int>(chunk.points.size())); // 이웃 점 개수

                    if (chunk.points.size() < 3) {
                        result.processed_points = chunk.points;
                        return result;
                    }

                    for (const auto& point : chunk.points) {
                        // 각 점에 대해 k개 이웃점과의 평균 거리 계산
                        std::vector<double> distances;
                        for (const auto& neighbor : chunk.points) {
                            if (&point != &neighbor) {
                                double dist = std::sqrt(
                                    std::pow(point.x - neighbor.x, 2) +
                                    std::pow(point.y - neighbor.y, 2) +
                                    std::pow(point.z - neighbor.z, 2)
                                );
                                distances.push_back(dist);
                            }
                        }

                        // 거리들을 정렬하고 k개만 선택
                        std::sort(distances.begin(), distances.end());
                        if (distances.size() > k_neighbors) {
                            distances.resize(k_neighbors);
                        }

                        if (distances.empty()) {
                            filtered_points.push_back(point);
                            continue;
                        }

                        // 평균과 표준편차 계산
                        double mean = 0.0;
                        for (double d : distances) mean += d;
                        mean /= distances.size();

                        double variance = 0.0;
                        for (double d : distances) {
                            variance += std::pow(d - mean, 2);
                        }
                        variance /= distances.size();
                        double std_dev = std::sqrt(variance);

                        // 임계값 내의 점만 유지
                        if (mean <= std_dev_threshold * std_dev) {
                            filtered_points.push_back(point);
                        }
                    }

                    result.processed_points = filtered_points;
                }
                catch (const std::exception& e) {
                    result.success = false;
                    result.error_message = "Outlier filter error: " + std::string(e.what());
                }

                return result;
            });

        // 2. 복셀 그리드 다운샘플링
        task_processor_->registerProcessor(TaskType::VOXEL_DOWNSAMPLE,
            [](const ProcessingTask& task, const PointCloudChunk& chunk) -> ProcessingResult {
                ProcessingResult result;
                result.task_id = task.task_id;
                result.chunk_id = task.chunk_id;
                result.success = true;

                try {
                    const double voxel_size = 0.1; // 복셀 크기
                    std::map<std::tuple<int, int, int>, std::vector<Point3D>> voxel_map;

                    // 각 점을 복셀에 할당
                    for (const auto& point : chunk.points) {
                        int vx = static_cast<int>(std::floor(point.x / voxel_size));
                        int vy = static_cast<int>(std::floor(point.y / voxel_size));
                        int vz = static_cast<int>(std::floor(point.z / voxel_size));

                        voxel_map[std::make_tuple(vx, vy, vz)].push_back(point);
                    }

                    // 각 복셀의 중심점으로 다운샘플링
                    std::vector<Point3D> downsampled_points;
                    for (const auto& voxel_pair : voxel_map) {
                        const auto& points_in_voxel = voxel_pair.second;

                        Point3D centroid(0, 0, 0);
                        for (const auto& p : points_in_voxel) {
                            centroid.x += p.x;
                            centroid.y += p.y;
                            centroid.z += p.z;
                        }
                        centroid.x /= points_in_voxel.size();
                        centroid.y /= points_in_voxel.size();
                        centroid.z /= points_in_voxel.size();

                        downsampled_points.push_back(centroid);
                    }

                    result.processed_points = downsampled_points;
                }
                catch (const std::exception& e) {
                    result.success = false;
                    result.error_message = "Voxel downsample error: " + std::string(e.what());
                }

                return result;
            });

        // 3. 평면 세그멘테이션 (RANSAC 기반)
        task_processor_->registerProcessor(TaskType::PLANE_SEGMENTATION,
            [](const ProcessingTask& task, const PointCloudChunk& chunk) -> ProcessingResult {
                ProcessingResult result;
                result.task_id = task.task_id;
                result.chunk_id = task.chunk_id;
                result.success = true;

                try {
                    if (chunk.points.size() < 3) {
                        result.processed_points = chunk.points;
                        return result;
                    }

                    const int max_iterations = 1000;
                    const double distance_threshold = 0.02;
                    const int min_inliers = static_cast<int>(chunk.points.size() * 0.1); // 최소 10%

                    std::vector<Point3D> best_inliers;
                    std::random_device rd;
                    std::mt19937 gen(rd());
                    std::uniform_int_distribution<> dis(0, static_cast<int>(chunk.points.size() - 1));

                    for (int iter = 0; iter < max_iterations; ++iter) {
                        // 랜덤하게 3개 점 선택하여 평면 방정식 구성
                        std::set<int> selected_indices;
                        while (selected_indices.size() < 3) {
                            selected_indices.insert(dis(gen));
                        }

                        std::vector<int> indices(selected_indices.begin(), selected_indices.end());

                        const Point3D& p1 = chunk.points[indices[0]];
                        const Point3D& p2 = chunk.points[indices[1]];
                        const Point3D& p3 = chunk.points[indices[2]];

                        // 평면 법선 벡터 계산 (외적)
                        double nx = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
                        double ny = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
                        double nz = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);

                        double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
                        if (norm < 1e-6) continue; // 퇴화된 경우 건너뛰기

                        nx /= norm; ny /= norm; nz /= norm;
                        double d = -(nx * p1.x + ny * p1.y + nz * p1.z);

                        // 인라이어 계산
                        std::vector<Point3D> inliers;
                        for (const auto& point : chunk.points) {
                            double distance = std::abs(nx * point.x + ny * point.y + nz * point.z + d);
                            if (distance < distance_threshold) {
                                inliers.push_back(point);
                            }
                        }

                        if (inliers.size() > best_inliers.size() && inliers.size() >= min_inliers) {
                            best_inliers = inliers;
                        }
                    }

                    result.processed_points = best_inliers;
                }
                catch (const std::exception& e) {
                    result.success = false;
                    result.error_message = "Plane segmentation error: " + std::string(e.what());
                }

                return result;
            });

        ILOG << "Custom processors registered successfully!";
        ILOG << "Available processors: outlier_filter, voxel_downsample, plane_segmentation";
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
        ILOG << "=== Worker Status ===";
        ILOG << "Connected: " << (client_->isConnected() ? "Yes" : "No");
        ILOG << "Slave ID: " << client_->getSlaveId();
        ILOG << "Server: " << server_address_ << ":" << server_port_;
        ILOG << "Processing threads: " << processing_threads_;
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
        std::atomic<bool> force_exit_{ false };  /// 강제 종료 플래그

        // 작업별 중단 플래그들을 관리
        std::map<uint32_t, std::shared_ptr<std::atomic<bool>>> active_task_cancellations_;
        std::mutex cancellation_mutex_;

        std::unique_ptr<NetworkClient> client_;
        std::unique_ptr<TaskProcessor> task_processor_;

        std::atomic<bool> running_;
        std::string server_address_ = "localhost";
        uint16_t server_port_;
        size_t processing_threads_;

        /// 작업 큐
        std::queue<TaskQueueItem> task_queue_;
        std::mutex task_queue_mutex_;
        std::condition_variable task_queue_cv_;

        /// 처리 스레드들
        std::vector<std::thread> processing_thread_pool_;
        std::thread status_thread_;

        /// Input thread
        std::thread input_thread_;

        /// 통계
        std::mutex stats_mutex_;
        std::atomic<uint32_t> completed_tasks_{ 0 };
        std::atomic<uint32_t> failed_tasks_{ 0 };
        std::atomic<double> total_processing_time_{ 0.0 };
        std::atomic<double> avg_processing_time_{ 0.0 };
};

/// 전역 변수 (시그널 핸들러용)
static SlaveApplication* g_app = nullptr;

void signalHandler(int signal) {
    ILOG << "";
    ILOG << "Received signal " << signal << ", shutting down...";
    if (g_app) {
        g_app->stop();
    }
    exit(0);
}

int main(int argc, char* argv[]) {
    /// 시그널 핸들러 설정
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