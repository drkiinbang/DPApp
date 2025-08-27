#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <signal.h>
#include <queue>
#include <set>
#include <tuple>

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

        // 1. Set running flag to false first
        running_ = false;

        // 2. Notify all waiting threads about shutdown
        task_queue_cv_.notify_all();

        // 3. Wait for processing threads to complete current tasks
        for (auto& thread : processing_thread_pool_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
        processing_thread_pool_.clear();

        // 4. Wait for status thread to finish
        if (status_thread_.joinable()) {
            status_thread_.join();
        }

        // 5. Disconnect client AFTER all threads are stopped
        // This prevents the client thread from being interrupted while other threads are still running
        if (client_) {
            client_->disconnect();
        }

        std::cout << "Slave worker stopped" << std::endl;
    }

    /// Graceful shutdown from network disconnect :
    void handleNetworkDisconnect() {
        if (running_) {
            std::cout << "Network disconnected, initiating graceful shutdown..." << std::endl;
            running_ = false;
            task_queue_cv_.notify_all();
        }
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
            /// 대기 중 스레드 깨우기
            task_queue_cv_.notify_all();
            /// run() 루프를 빠져나가 stop() 호출로 이동
            return false;
        }
        /// 즉시 종료: 진행 중 작업도 중단 시도 ---
        else if (cmd == "quit-now" || cmd == "exit-now" || cmd == "quit!") {
            std::cout << "[IMMEDIATE] cancel in-flight task and stop NOW." << std::endl;
            DPApp::g_cancel_requested.store(true, std::memory_order_relaxed);
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
            /// 마스터로부터의 하트비트에 대한 응답
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
            /// Only attempt reconnection if we're still supposed to be running
            /// and the disconnection wasn't initiated by us
            if (running_ && !shutdown_requested_) {
                std::cout << "Connection lost, attempting to reconnect..." << std::endl;
                /// Reconnection logic would go here
            }
            else {
                /// If we initiated the shutdown, don't try to reconnect
                std::cout << "Shutdown in progress, not attempting reconnection" << std::endl;
            }
        }
    }

    void handleTaskAssignment(const NetworkMessage& message) {
        try {
            /// 메시지 데이터에서 작업과 청크 정보 분리
            /// 실제로는 더 정교한 직렬화/역직렬화가 필요

            /// 임시로 간단한 분리 (실제 구현에서는 프로토콜 정의 필요)
            if (message.data.size() < sizeof(uint32_t)) {
                throw std::runtime_error("Invalid task assignment message");
            }

            /// 여기서는 NetworkUtils의 함수들을 사용하여 분리
            /// 실제 구현에서는 메시지 프로토콜을 더 정교하게 정의해야 함

            ProcessingTask task;
            PointCloudChunk chunk;

            /// 임시 구현: 하드코딩된 작업 생성
            task.task_id = 1;  /// 실제로는 메시지에서 파싱
            task.chunk_id = 1;
            task.task_type = "filter";  /// 실제로는 메시지에서 파싱

            /// 임시 청크 데이터 (실제로는 메시지에서 파싱)
            chunk.chunk_id = 1;
            chunk.points.resize(1000);
            for (size_t i = 0; i < chunk.points.size(); ++i) {
                chunk.points[i] = Point3D(i * 0.1, i * 0.1, i * 0.01);
            }

            /// 작업을 큐에 추가
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

            std::cout << "Thread " << thread_id << " processing task " << item.task.task_id << std::endl;

            /// 작업 처리
            auto start_time = std::chrono::steady_clock::now();
            ProcessingResult result = task_processor_->processTask(item.task, item.chunk);
            auto end_time = std::chrono::steady_clock::now();

            auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                end_time - start_time).count();

            std::cout << "Thread " << thread_id << " completed task " << item.task.task_id
                << " in " << processing_time << "ms (success: "
                << (result.success ? "Yes" : "No") << ")" << std::endl;

            /// 결과를 마스터에게 전송
            sendTaskResult(result);

            /// 통계 업데이트
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
            /// 더 자주 확인하여 빠른 종료 지원
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
        /// registerCustomProcessors는 이런 경우에 :
        /// 
        /// 특정 프로젝트나 클라이언트만을 위한 특수 알고리즘
        ///     설정 파일이나 명령행 옵션에 따라 달라지는 기능
        ///     실험적이거나 임시적인 기능
        ///     외부 라이브러리나 하드웨어에 의존하는 기능
        /// Compare to PointCloudProcessors in TaskManager.h
        
        std::cout << "Registering custom point cloud processors..." << std::endl;

        /// 필요한 경우 커스텀 프로세서 등록
        /// 예: 특별한 포인트클라우드 처리 알고리즘
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
        task_processor_->registerProcessor("outlier_filter",
            [](const ProcessingTask& task, const PointCloudChunk& chunk) -> ProcessingResult {
                ProcessingResult result;
                result.task_id = task.task_id;
                result.chunk_id = task.chunk_id;
                result.success = true;

                // 통계적 이상치 제거 알고리즘
                std::vector<Point3D> filtered_points;
                const double std_dev_threshold = 2.0; // 표준편차 임계값
                const int k_neighbors = 10; // 이웃 점 개수

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
                std::cout << "Outlier filter: " << chunk.points.size()
                    << " -> " << filtered_points.size() << " points" << std::endl;

                return result;
            });

        // 2. 복셀 그리드 다운샘플링
        task_processor_->registerProcessor("voxel_downsample",
            [](const ProcessingTask& task, const PointCloudChunk& chunk) -> ProcessingResult {
                ProcessingResult result;
                result.task_id = task.task_id;
                result.chunk_id = task.chunk_id;
                result.success = true;

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
                std::cout << "Voxel downsample: " << chunk.points.size()
                    << " -> " << downsampled_points.size() << " points" << std::endl;

                return result;
            });

        // 3. 평면 세그멘테이션 (RANSAC 기반)
        task_processor_->registerProcessor("plane_segmentation",
            [](const ProcessingTask& task, const PointCloudChunk& chunk) -> ProcessingResult {
                ProcessingResult result;
                result.task_id = task.task_id;
                result.chunk_id = task.chunk_id;
                result.success = true;

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
                    std::vector<int> indices(3);
                    for (int i = 0; i < 3; ++i) {
                        indices[i] = dis(gen);
                    }

                    // 3개 점이 모두 다른지 확인
                    if (indices[0] == indices[1] || indices[1] == indices[2] || indices[0] == indices[2]) {
                        continue;
                    }

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
                std::cout << "Plane segmentation: found " << best_inliers.size()
                    << " inliers out of " << chunk.points.size() << " points" << std::endl;

                return result;
            });

        // 4. 포인트클라우드 압축
        task_processor_->registerProcessor("compress",
            [](const ProcessingTask& task, const PointCloudChunk& chunk) -> ProcessingResult {
                ProcessingResult result;
                result.task_id = task.task_id;
                result.chunk_id = task.chunk_id;
                result.success = true;

                // 간단한 양자화 기반 압축
                const double quantization_step = 0.001; // 1mm 정밀도

                std::vector<Point3D> compressed_points;
                std::set<std::tuple<int, int, int>> quantized_points;

                for (const auto& point : chunk.points) {
                    int qx = static_cast<int>(std::round(point.x / quantization_step));
                    int qy = static_cast<int>(std::round(point.y / quantization_step));
                    int qz = static_cast<int>(std::round(point.z / quantization_step));

                    auto quantized = std::make_tuple(qx, qy, qz);
                    if (quantized_points.find(quantized) == quantized_points.end()) {
                        quantized_points.insert(quantized);

                        Point3D compressed_point;
                        compressed_point.x = qx * quantization_step;
                        compressed_point.y = qy * quantization_step;
                        compressed_point.z = qz * quantization_step;
                        compressed_points.push_back(compressed_point);
                    }
                }

                result.processed_points = compressed_points;
                std::cout << "Compression: " << chunk.points.size()
                    << " -> " << compressed_points.size() << " points" << std::endl;

                return result;
            });

        // 5. 균등 샘플링
        task_processor_->registerProcessor("uniform_sample",
            [](const ProcessingTask& task, const PointCloudChunk& chunk) -> ProcessingResult {
                ProcessingResult result;
                result.task_id = task.task_id;
                result.chunk_id = task.chunk_id;
                result.success = true;

                const int target_points = (std::min)(static_cast<int>(chunk.points.size() / 2), 1000);

                if (chunk.points.size() <= target_points) {
                    result.processed_points = chunk.points;
                    return result;
                }

                std::vector<Point3D> sampled_points;
                double step = static_cast<double>(chunk.points.size()) / target_points;

                for (int i = 0; i < target_points; ++i) {
                    int index = static_cast<int>(i * step);
                    if (index < chunk.points.size()) {
                        sampled_points.push_back(chunk.points[index]);
                    }
                }

                result.processed_points = sampled_points;
                std::cout << "Uniform sampling: " << chunk.points.size()
                    << " -> " << sampled_points.size() << " points" << std::endl;

                return result;
            });

        std::cout << "Custom processors registered successfully!" << std::endl;
        std::cout << "Available processors: outlier_filter, voxel_downsample, "
            << "plane_segmentation, compress, uniform_sample" << std::endl;
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

    /// 작업 큐
    std::queue<TaskQueueItem> task_queue_;
    std::mutex task_queue_mutex_;
    std::condition_variable task_queue_cv_;

    /// 처리 스레드들
    std::vector<std::thread> processing_thread_pool_;
    std::thread status_thread_;

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
    std::cout << std::endl << "Received signal " << signal << ", shutting down..." << std::endl;
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

    app.run();

    return 0;
}