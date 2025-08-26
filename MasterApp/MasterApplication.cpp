#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <signal.h>

#include "../include/PointCloudTypes.h"
#include "../include/NetworkManager.h"
#include "../include/TaskManager.h"

using namespace DPApp;

class MasterApplication {
public:
    MasterApplication() : running_(false), server_port_(8080) {}
    
    ~MasterApplication() {
        stop();
    }
    
    bool initialize(int argc, char* argv[]) {
        // 명령행 인수 파싱
        parseCommandLine(argc, argv);
        
        // 네트워크 서버 설정
        server_ = std::make_unique<NetworkServer>();
        server_->setMessageCallback([this](const NetworkMessage& msg, const std::string& client_id) {
            handleMessage(msg, client_id);
        });
        
        server_->setConnectionCallback([this](const std::string& client_id, bool connected) {
            handleConnection(client_id, connected);
        });
        
        // 작업 관리자 설정
        task_manager_ = std::make_unique<TaskManager>();
        task_manager_->setTaskCompletedCallback([this](const TaskInfo& task, const ProcessingResult& result) {
            handleTaskCompleted(task, result);
        });
        
        task_manager_->setTaskFailedCallback([this](const TaskInfo& task, const std::string& error) {
            handleTaskFailed(task, error);
        });
        
        return true;
    }
    
    bool start() {
        if (running_) return false;
        
        std::cout << "Starting DPApp Master Server..." << std::endl;
        std::cout << "Port: " << server_port_ << std::endl;
        std::cout << "Max points per chunk: " << max_points_per_chunk_ << std::endl;
        
        // 네트워크 서버 시작
        if (!server_->start(server_port_)) {
            std::cerr << "Failed to start network server" << std::endl;
            return false;
        }
        
        running_ = true;
        
        // 백그라운드 스레드 시작
        task_assignment_thread_ = std::thread(&MasterApplication::taskAssignmentLoop, this);
        status_thread_ = std::thread(&MasterApplication::statusLoop, this);
        timeout_thread_ = std::thread(&MasterApplication::timeoutLoop, this);
        
        std::cout << "Master server started successfully" << std::endl;
        
        return true;
    }
    
    void stop() {
        if (!running_) return;
        
        std::cout << "Stopping master server..." << std::endl;
        
        running_ = false;
        
        // 스레드 종료 대기
        if (task_assignment_thread_.joinable()) {
            task_assignment_thread_.join();
        }
        if (status_thread_.joinable()) {
            status_thread_.join();
        }
        if (timeout_thread_.joinable()) {
            timeout_thread_.join();
        }
        
        // 서버 종료
        if (server_) {
            server_->stop();
        }
        
        std::cout << "Master server stopped" << std::endl;
    }
    
    void run() {
        if (!start()) {
            return;
        }
        
        printHelp();
        
        std::string command;
        while (running_ && std::getline(std::cin, command)) {
            processCommand(command);
        }
    }
    
    // 포인트클라우드 파일 로드 및 처리 시작
    bool loadAndProcessPointCloud(const std::string& filename, const std::string& task_type) {
        auto point_cloud = std::make_shared<PointCloud>();
        
        std::cout << "Loading point cloud: " << filename << std::endl;
        
        if (!point_cloud->loadFromFile(filename)) {
            std::cerr << "Failed to load point cloud file: " << filename << std::endl;
            return false;
        }
        
        std::cout << "Loaded " << point_cloud->getPointCount() << " points" << std::endl;
        
        // 청크로 분할
        std::cout << "Splitting into chunks..." << std::endl;
        auto chunks = point_cloud->splitIntoChunks(max_points_per_chunk_);
        
        std::cout << "Created " << chunks.size() << " chunks" << std::endl;
        
        // 작업 생성
        createTasksFromChunks(chunks, task_type);
        
        return true;
    }

private:
    void parseCommandLine(int argc, char* argv[]) {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            
            if (arg == "-p" || arg == "--port") {
                if (i + 1 < argc) {
                    server_port_ = static_cast<uint16_t>(std::stoi(argv[++i]));
                }
            } else if (arg == "-c" || arg == "--chunk-size") {
                if (i + 1 < argc) {
                    max_points_per_chunk_ = std::stoul(argv[++i]);
                }
            } else if (arg == "-h" || arg == "--help") {
                printUsage();
                exit(0);
            }
        }
    }
    
    void printUsage() {
        std::cout << "Usage: master [options]" << std::endl;
        std::cout << "Options:" << std::endl;
        std::cout << "  -p, --port <port>        Server port (default: 8080)" << std::endl;
        std::cout << "  -c, --chunk-size <size>  Max points per chunk (default: 10000)" << std::endl;
        std::cout << "  -h, --help               Show this help message" << std::endl;
    }
    
    void printHelp() {
        std::cout << std::endl;
        std::cout << "=== DPApp Master Console Commands ===" << std::endl;
        std::cout << "load <file> <task_type>  - Load point cloud and create processing tasks" << std::endl;
        std::cout << "status                   - Show system status" << std::endl;
        std::cout << "slaves                   - List connected slaves" << std::endl;
        std::cout << "tasks                    - List all tasks" << std::endl;
        std::cout << "progress                 - Show overall progress" << std::endl;
        std::cout << "results                  - Show completed results count" << std::endl;
        std::cout << "save <file>              - Save merged results to file" << std::endl;
        std::cout << "clear                    - Clear all completed tasks" << std::endl;
        std::cout << "help                     - Show this help" << std::endl;
        std::cout << "quit                     - Stop and exit" << std::endl;
        std::cout << "=====================================" << std::endl;
        std::cout << std::endl;
    }
    
    void processCommand(const std::string& command) {
        std::istringstream iss(command);
        std::string cmd;
        iss >> cmd;
        
        if (cmd == "load") {
            std::string filename, task_type;
            iss >> filename >> task_type;
            
            if (filename.empty() || task_type.empty()) {
                std::cout << "Usage: load <filename> <task_type>" << std::endl;
                std::cout << "Task types: filter, classify, estimate_normals, remove_outliers, downsample" << std::endl;
            } else {
                loadAndProcessPointCloud(filename, task_type);
            }
        } else if (cmd == "status") {
            printSystemStatus();
        } else if (cmd == "slaves") {
            printSlaveStatus();
        } else if (cmd == "tasks") {
            printTaskStatus();
        } else if (cmd == "progress") {
            printProgress();
        } else if (cmd == "results") {
            printResults();
        } else if (cmd == "save") {
            std::string filename;
            iss >> filename;
            if (filename.empty()) {
                std::cout << "Usage: save <filename>" << std::endl;
            } else {
                saveResults(filename);
            }
        } else if (cmd == "clear") {
            clearCompletedTasks();
        } else if (cmd == "help") {
            printHelp();
        } else if (cmd == "quit" || cmd == "exit") {
            stop();
        } else if (!cmd.empty()) {
            std::cout << "Unknown command: " << cmd << std::endl;
            std::cout << "Type 'help' for available commands" << std::endl;
        }
    }
    
    void handleMessage(const NetworkMessage& message, const std::string& client_id) {
        switch (message.header.type) {
            case MessageType::SLAVE_REGISTER:
                std::cout << "Slave registration from: " << client_id << std::endl;
                task_manager_->registerSlave(client_id);
                break;
                
            case MessageType::SLAVE_UNREGISTER:
                std::cout << "Slave unregistration from: " << client_id << std::endl;
                task_manager_->unregisterSlave(client_id);
                break;
                
            case MessageType::HEARTBEAT:
                task_manager_->updateSlaveHeartbeat(client_id);
                break;
                
            case MessageType::TASK_RESULT:
                handleTaskResult(message, client_id);
                break;
                
            default:
                std::cout << "Unknown message type from " << client_id << std::endl;
                break;
        }
    }
    
    void handleConnection(const std::string& client_id, bool connected) {
        if (connected) {
            std::cout << "Client connected: " << client_id << std::endl;
        } else {
            std::cout << "Client disconnected: " << client_id << std::endl;
            task_manager_->unregisterSlave(client_id);
        }
    }
    
    void handleTaskResult(const NetworkMessage& message, const std::string& client_id) {
        try {
            ProcessingResult result = NetworkUtils::deserializeResult(message.data);
            task_manager_->completeTask(result.task_id, result);
        } catch (const std::exception& e) {
            std::cerr << "Error processing task result from " << client_id 
                      << ": " << e.what() << std::endl;
        }
    }
    
    void handleTaskCompleted(const TaskInfo& task, const ProcessingResult& result) {
        std::cout << "Task " << task.task_id << " completed by " 
                  << task.assigned_slave << std::endl;
    }
    
    void handleTaskFailed(const TaskInfo& task, const std::string& error) {
        std::cout << "Task " << task.task_id << " failed: " << error << std::endl;
    }
    
    void createTasksFromChunks(const std::vector<PointCloudChunk>& chunks, const std::string& task_type) {
        std::vector<uint8_t> default_params; // 기본 파라미터
        
        // 작업 타입에 따른 기본 파라미터 설정
        if (task_type == "filter") {
            double min_z = -100.0, max_z = 100.0;
            default_params.resize(sizeof(double) * 2);
            std::memcpy(default_params.data(), &min_z, sizeof(double));
            std::memcpy(default_params.data() + sizeof(double), &max_z, sizeof(double));
        } else if (task_type == "downsample") {
            double ratio = 0.1; // 10%
            default_params.resize(sizeof(double));
            std::memcpy(default_params.data(), &ratio, sizeof(double));
        }
        
        for (const auto& chunk : chunks) {
            auto chunk_ptr = std::make_shared<PointCloudChunk>(chunk);
            uint32_t task_id = task_manager_->addTask(task_type, chunk_ptr, default_params);
            std::cout << "Created task " << task_id << " for chunk " << chunk.chunk_id 
                      << " (" << chunk.points.size() << " points)" << std::endl;
        }
    }
    
    void taskAssignmentLoop() {
        while (running_) {
            // 작업 할당 시도
            task_manager_->assignTasksToSlaves();
            
            // 할당된 작업들을 슬레이브에게 전송
            sendAssignedTasks();
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    void sendAssignedTasks() {
        auto tasks = task_manager_->getTasksByStatus(TaskStatus::ASSIGNED);
        
        for (auto& task_info : tasks) {
            // 작업을 IN_PROGRESS로 변경
            task_manager_->updateTaskStatus(task_info.task_id, TaskStatus::IN_PROGRESS);
            
            // ProcessingTask 구조체 생성
            ProcessingTask task;
            task.task_id = task_info.task_id;
            task.chunk_id = task_info.chunk_id;
            task.task_type = task_info.task_type;
            task.parameters = task_info.parameters;
            
            // 메시지 생성 (작업 정보 + 청크 데이터)
            auto task_data = NetworkUtils::serializeTask(task);
            auto chunk_data = NetworkUtils::serializeChunk(*task_info.chunk_data);
            
            // 합친 데이터 생성
            std::vector<uint8_t> combined_data;
            combined_data.insert(combined_data.end(), task_data.begin(), task_data.end());
            combined_data.insert(combined_data.end(), chunk_data.begin(), chunk_data.end());
            
            NetworkMessage message(MessageType::TASK_ASSIGNMENT, combined_data);
            
            if (server_->sendMessage(task_info.assigned_slave, message)) {
                std::cout << "Task " << task_info.task_id 
                          << " sent to " << task_info.assigned_slave << std::endl;
            } else {
                std::cerr << "Failed to send task " << task_info.task_id 
                          << " to " << task_info.assigned_slave << std::endl;
                task_manager_->failTask(task_info.task_id, "Failed to send task");
            }
        }
    }
    
    void statusLoop() {
        while (running_) {
            std::this_thread::sleep_for(std::chrono::seconds(30));
            
            if (running_) {
                std::cout << "=== Status Update ===" << std::endl;
                printProgress();
                std::cout << "Connected slaves: " << server_->getClientCount() << std::endl;
                std::cout << "===================" << std::endl;
            }
        }
    }
    
    void timeoutLoop() {
        while (running_) {
            task_manager_->checkTimeouts();
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }
    }
    
    void printSystemStatus() {
        std::cout << "=== System Status ===" << std::endl;
        std::cout << "Server running: " << (server_->isRunning() ? "Yes" : "No") << std::endl;
        std::cout << "Server port: " << server_port_ << std::endl;
        std::cout << "Connected slaves: " << server_->getClientCount() << std::endl;
        printProgress();
        std::cout << "===================" << std::endl;
    }
    
    void printSlaveStatus() {
        auto slaves = task_manager_->getAllSlaves();
        
        std::cout << "=== Slave Status ===" << std::endl;
        std::cout << "Total slaves: " << slaves.size() << std::endl;
        
        for (const auto& slave : slaves) {
            std::cout << "Slave: " << slave.slave_id << std::endl;
            std::cout << "  Active: " << (slave.is_active ? "Yes" : "No") << std::endl;
            std::cout << "  Busy: " << (slave.is_busy ? "Yes" : "No") << std::endl;
            std::cout << "  Completed tasks: " << slave.completed_tasks << std::endl;
            std::cout << "  Failed tasks: " << slave.failed_tasks << std::endl;
            std::cout << "  Avg processing time: " << slave.avg_processing_time << "s" << std::endl;
        }
        std::cout << "==================" << std::endl;
    }
    
    void printTaskStatus() {
        std::cout << "=== Task Status ===" << std::endl;
        std::cout << "Pending: " << task_manager_->getPendingTaskCount() << std::endl;
        std::cout << "Active: " << task_manager_->getActiveTaskCount() << std::endl;
        std::cout << "Completed: " << task_manager_->getCompletedTaskCount() << std::endl;
        std::cout << "Failed: " << task_manager_->getFailedTaskCount() << std::endl;
        std::cout << "==================" << std::endl;
    }
    
    void printProgress() {
        double progress = task_manager_->getOverallProgress();
        std::cout << "Overall progress: " << (progress * 100) << "%" << std::endl;
    }
    
    void printResults() {
        auto results = task_manager_->getCompletedResults();
        std::cout << "Completed results: " << results.size() << std::endl;
        
        // 간단한 통계
        size_t total_processed_points = 0;
        for (const auto& result : results) {
            total_processed_points += result.processed_points.size();
        }
        
        std::cout << "Total processed points: " << total_processed_points << std::endl;
    }
    
    void saveResults(const std::string& filename) {
        auto results = task_manager_->getCompletedResults();
        
        if (results.empty()) {
            std::cout << "No results to save" << std::endl;
            return;
        }
        
        // 결과 병합
        PointCloud merged_cloud;
        for (const auto& result : results) {
            merged_cloud.addPoints(result.processed_points);
        }
        
        if (merged_cloud.saveToFile(filename)) {
            std::cout << "Results saved to: " << filename << std::endl;
            std::cout << "Total points: " << merged_cloud.getPointCount() << std::endl;
        } else {
            std::cout << "Failed to save results to: " << filename << std::endl;
        }
    }
    
    void clearCompletedTasks() {
        task_manager_->clearCompletedResults();
        std::cout << "Completed tasks cleared" << std::endl;
    }
    
private:
    std::unique_ptr<NetworkServer> server_;
    std::unique_ptr<TaskManager> task_manager_;
    
    std::atomic<bool> running_;
    uint16_t server_port_;
    size_t max_points_per_chunk_ = 10000;
    
    std::thread task_assignment_thread_;
    std::thread status_thread_;
    std::thread timeout_thread_;
};

// 전역 변수 (시그널 핸들러용)
static MasterApplication* g_app = nullptr;

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
    
    MasterApplication app;
    g_app = &app;
    
    if (!app.initialize(argc, argv)) {
        std::cerr << "Failed to initialize master application" << std::endl;
        return 1;
    }
    
    app.run();
    
    return 0;
}