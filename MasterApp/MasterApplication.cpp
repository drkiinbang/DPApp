#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <signal.h>
#include <sstream>
#include <iomanip>

#include "../include/PointCloudTypes.h"
#include "../include/NetworkManager.h"
#include "../include/TaskManager.h"
#include "RestApiServer.h"
#include "../include/RuntimeConfig.h"
#include "../include/Logger.h"

using namespace DPApp;

class MasterApplication {
private:
    DPApp::RuntimeConfig cfg_;

public:
    bool isDaemonMode() const { return daemon_mode_; }

    MasterApplication() : running_(false), server_port_(8080), api_port_(8081) {}

    ~MasterApplication() {
        stop();
    }

    bool initialize(int argc, char* argv[]) {
        /// Initialize logger
        std::time_t t = std::time(nullptr);
        std::tm tm{};
#ifdef _WIN32
        localtime_s(&tm, &t); // Windows
#else
        localtime_r(&t, &tm); // Linux/Unix
#endif
        char logFilename[512];
        std::strftime(logFilename, sizeof(logFilename), "MasterApp_%Y-%m-%d_%H-%M-%S.log", &tm);
        DPApp::Logger::initialize(logFilename);
        DPApp::Logger::setMinLevel(DPApp::LogLevel::INFO);

        parseCommandLine(argc, argv);

        /// Set network server
        server_ = std::make_unique<NetworkServer>();
        server_->setMessageCallback([this](const NetworkMessage& msg, const std::string& client_id) {
            handleMessage(msg, client_id);
            });

        server_->setConnectionCallback([this](const std::string& client_id, bool connected) {
            handleConnection(client_id, connected);
            });

        // 작업 관리자 설정
        task_manager_ = std::make_unique<TaskManager>(cfg_.task_timeout_seconds);
        task_manager_->setTaskCompletedCallback([this](const TaskInfo& task, const ProcessingResult& result) {
            handleTaskCompleted(task, result);
            });

        task_manager_->setTaskFailedCallback([this](const TaskInfo& task, const std::string& error) {
            handleTaskFailed(task, error);
            });

        // REST API 서버 설정
        setupRestApiRoutes();

        return true;
    }
    
    bool start() {
        if (running_) return false;

        cfg_ = DPApp::RuntimeConfig::loadFromEnv();

        ILOG << "Starting DPApp Master Server...";
        ILOG << "Main Port: " << server_port_;
        ILOG << "REST API Port: " << api_port_;
        ILOG << "Max points per chunk: " << max_points_per_chunk_;

        // 네트워크 서버 시작
        if (!server_->start(server_port_)) {
            ELOG << "Failed to start network server";
            return false;
        }

        // REST API 서버 시작
        if (!api_server_->start(api_port_)) {
            ELOG << "Failed to start REST API server";
            return false;
        }

        running_ = true;

        // 백그라운드 스레드 시작
        task_assignment_thread_ = std::thread(&MasterApplication::taskAssignmentLoop, this);
        status_thread_ = std::thread(&MasterApplication::statusLoop, this);
        timeout_thread_ = std::thread(&MasterApplication::timeoutLoop, this);

        ILOG << "Master server started successfully";
        ILOG << "REST API available at: http://localhost:" << api_port_ << "/api";

        return true;
    }
    
    void stop() {
        if (!running_) return;

        ILOG << "Stopping master server...";

        shutdownAllSlaves();
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 메시지 전송을 위한 짧은 대기

        running_ = false;

        /// 1. 새로운 작업 할당 중지
        task_manager_->requestStop();

        /// 2. 진행 중인 작업 완료 대기 (타임아웃 포함)
        auto start_time = std::chrono::steady_clock::now();
        while (task_manager_->getActiveTaskCount() > 0 &&
            std::chrono::steady_clock::now() - start_time < std::chrono::seconds(30)) {
            ILOG << "Waiting for " << task_manager_->getActiveTaskCount() << " active tasks to complete...";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        /// 3. 스레드 종료 대기
        if (task_assignment_thread_.joinable()) {
            task_assignment_thread_.join();
        }
        if (status_thread_.joinable()) {
            status_thread_.join();
        }
        if (timeout_thread_.joinable()) {
            timeout_thread_.join();
        }

        /// 4. 서버들 종료
        if (api_server_) {
            api_server_->stop();
        }
        if (server_) {
            server_->stop();
        }

        ILOG << "Master server stopped";
    }
    
    void run() {
        if (!start()) {
            return;
        }

        if (daemon_mode_) {
            runDaemon();
        }
        else {
            runInteractive();
        }
    }
    
    void runDaemon() {
        ILOG << "Master server running in daemon mode...";
        ILOG << "REST API: http://localhost:" << api_port_ << "/api";
        ILOG << "Use REST API to control the server.";

        // 메인 스레드는 단순히 대기
        while (running_) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    void runInteractive() {
        printHelp();

        std::string command;
        while (running_ && std::getline(std::cin, command)) {
            processCommand(command);
        }
    }

    // 포인트클라우드 파일 로드 및 처리 시작
    bool loadAndProcessPointCloud(const std::string& filename, const TaskType task_type) {
        auto point_cloud = std::make_shared<PointCloud>();
        
        ILOG << "Loading point cloud: " << filename;
        
        if (!point_cloud->loadFromFile(filename)) {
            ELOG << "Failed to load point cloud file: " << filename;
            return false;
        }
        
        ILOG << "Loaded " << point_cloud->getPointCount() << " points";
        
        /// 청크로 분할
        ILOG << "Splitting into chunks...";
        auto chunks = point_cloud->splitIntoChunks(max_points_per_chunk_);
        
        ILOG << "Created " << chunks.size() << " chunks";
        
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
            }
            else if (arg == "--api-port") {
                if (i + 1 < argc) {
                    api_port_ = static_cast<uint16_t>(std::stoi(argv[++i]));
                }
            }
            else if (arg == "-d" || arg == "--daemon") {
                daemon_mode_ = true;
            }
            else if (arg == "-c" || arg == "--chunk-size") {
                if (i + 1 < argc) {
                    max_points_per_chunk_ = std::stoul(argv[++i]);
                }
            }
            else if (arg == "-h" || arg == "--help") {
                printUsage();
                exit(0);
            }
        }
    }
    
    void shutdownSlave(const std::string& slave_id) {
        NetworkMessage shutdown_msg(MessageType::SHUTDOWN, std::vector<uint8_t>());

        if (server_->sendMessage(slave_id, shutdown_msg)) {
            ILOG << "Shutdown command sent to slave: " << slave_id;
        }
        else {
            WLOG << "Failed to send shutdown command to slave: " << slave_id;
        }
    }

    void shutdownAllSlaves() {
        auto connected_clients = server_->getConnectedClients();

        if (connected_clients.empty()) {
            ILOG << "No connected slaves to shutdown";
            return;
        }

        NetworkMessage shutdown_msg(MessageType::SHUTDOWN, std::vector<uint8_t>());

        for (const auto& client_id : connected_clients) {
            if (server_->sendMessage(client_id, shutdown_msg)) {
                ILOG << "Shutdown command sent to slave: " << client_id;
            }
            else {
                WLOG << "Failed to send shutdown command to slave: " << client_id;
            }
        }

        ILOG << "Shutdown commands sent to " << connected_clients.size() << " slaves";
    }

    void printUsage() {
        ILOG << "Usage: master [options]" ;
        ILOG << "Options:" ;
        ILOG << "  -p, --port <port>        Main server port (default: 8080)" ;
        ILOG << "  --api-port <port>        REST API port (default: 8081)" ;
        ILOG << "  -d, --daemon             Run in daemon mode" ;
        ILOG << "  -c, --chunk-size <size>  Max points per chunk (default: 10000)" ;
        ILOG << "  -h, --help               Show this help message" ;
    }
    
    void printHelp() {
        ILOG << "";
        ILOG << "=== DPApp Master Console Commands ===" ;
        ILOG << "load <file> <task_type>  - Load point cloud and create processing tasks" ;
        ILOG << "status                   - Show system status" ;
        ILOG << "slaves                   - List connected slaves" ;
        ILOG << "tasks                    - List all tasks" ;
        ILOG << "progress                 - Show overall progress" ;
        ILOG << "results                  - Show completed results count" ;
        ILOG << "save <file>              - Save merged results to file" ;
        ILOG << "clear                    - Clear all completed tasks" ;
        ILOG << "shutdown [slave_id]      - Shutdown specific slave or all slaves" ;  // 추가
        ILOG << "help                     - Show this help" ;
        ILOG << "quit                     - Stop and exit" ;
        ILOG << "=====================================" ;
        ILOG << "";
    }
    
    void processCommand(const std::string& command) {
        std::istringstream iss(command);
        std::string cmd;
        iss >> cmd;
        
        if (cmd == "load") {
            std::string filename, task_type_str;
            iss >> filename >> task_type_str;
            
            if (filename.empty() || task_type_str.empty()) {
                WLOG << "Usage: load <filename> <task_type>" ;
                WLOG << "Task types: filter, classify, estimate_normals, remove_outliers, downsample" ;
            } else {
                auto task_type = strTask(task_type_str);
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
                WLOG << "Usage: save <filename>";
            } else {
                saveResults(filename);
            }
        } else if (cmd == "clear") {
            clearCompletedTasks();
        }
        else if (cmd == "shutdown") {
            std::string slave_id;
            iss >> slave_id;

            if (slave_id.empty()) {
                /// 모든 슬레이브 종료
                shutdownAllSlaves();
            }
            else {
                /// 특정 슬레이브 종료
                shutdownSlave(slave_id);
            }
        } else if (cmd == "help") {
            printHelp();
        } else if (cmd == "quit" || cmd == "exit") {
            stop();
        } else if (!cmd.empty()) {
            WLOG << "Unknown command: " << cmd;
            WLOG << "Type 'help' for available commands";
        }
    }
    
    void handleMessage(const NetworkMessage& message, const std::string& client_id) {
        switch (message.header.type) {
            case MessageType::SLAVE_REGISTER:
                ILOG << "Slave registration from: " << client_id ;
                task_manager_->registerSlave(client_id);
                break;
                
            case MessageType::SLAVE_UNREGISTER:
                ILOG << "Slave unregistration from: " << client_id ;
                task_manager_->unregisterSlave(client_id);
                break;
                
            case MessageType::HEARTBEAT:
                task_manager_->updateSlaveHeartbeat(client_id);
                break;
                
            case MessageType::TASK_RESULT:
                handleTaskResult(message, client_id);
                break;
                
            default:
                WLOG << "Unknown message type from " << client_id ;
                break;
        }
    }
    
    void handleConnection(const std::string& client_id, bool connected) {
        if (connected) {
            ILOG << "Client connected: " << client_id ;
        } else {
            ILOG << "Client disconnected: " << client_id ;
            task_manager_->unregisterSlave(client_id);
        }
    }
    
    void handleTaskResult(const NetworkMessage& message, const std::string& client_id) {
        try {
            ProcessingResult result = NetworkUtils::deserializeResult(message.data);
            task_manager_->completeTask(result.task_id, result);
        } catch (const std::exception& e) {
            ELOG << "Error processing task result from " << client_id 
                      << ": " << e.what() ;
        }
    }
    
    void handleTaskCompleted(const TaskInfo& task, const ProcessingResult& result) {
        ILOG << "Task " << task.task_id << " completed by " 
                  << task.assigned_slave ;
    }
    
    void handleTaskFailed(const TaskInfo& task, const std::string& error) {
        WLOG << "Task " << task.task_id << " failed: " << error ;
    }
    
    void createTasksFromChunks(const std::vector<PointCloudChunk>& chunks, const TaskType task_type) {
        std::vector<uint8_t> default_params; // 기본 파라미터
        
        // 작업 타입에 따른 기본 파라미터 설정
        if (task_type == TaskType::FILTER) {
            double min_z = -100.0, max_z = 100.0;
            default_params.resize(sizeof(double) * 2);
            std::memcpy(default_params.data(), &min_z, sizeof(double));
            std::memcpy(default_params.data() + sizeof(double), &max_z, sizeof(double));
        } else if (task_type == TaskType::DOWNSAMPLE) {
            double ratio = 0.1; // 10%
            default_params.resize(sizeof(double));
            std::memcpy(default_params.data(), &ratio, sizeof(double));
        }
        
        for (const auto& chunk : chunks) {
            auto chunk_ptr = std::make_shared<PointCloudChunk>(chunk);
            uint32_t task_id = task_manager_->addTask(task_type, chunk_ptr, default_params);
            ILOG << "Created task " << task_id << " for chunk " << chunk.chunk_id 
                      << " (" << chunk.points.size() << " points)" ;
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
            
            /// 메시지 생성 (작업 정보 + 청크 데이터)
            auto task_data = NetworkUtils::serializeTask(task);
            auto chunk_data = NetworkUtils::serializeChunk(*task_info.chunk_data);
            
            /// 합친 데이터 생성
            std::vector<uint8_t> combined_data;
            uint32_t task_data_size = static_cast<uint32_t>(task_data.size());
            /// Insert 4byte data (task_data_size)
            combined_data.insert(combined_data.end(),
                reinterpret_cast<uint8_t*>(&task_data_size),
                reinterpret_cast<uint8_t*>(&task_data_size) + sizeof(uint32_t));

            /// Insert task_data
            combined_data.insert(combined_data.end(), task_data.begin(), task_data.end());
            /// Insert chunk_data
            combined_data.insert(combined_data.end(), chunk_data.begin(), chunk_data.end());
            
            NetworkMessage message(MessageType::TASK_ASSIGNMENT, combined_data);
            
            if (server_->sendMessage(task_info.assigned_slave, message)) {
                ILOG << "Task " << task_info.task_id 
                          << " sent to " << task_info.assigned_slave ;
            } else {
                ELOG << "Failed to send task " << task_info.task_id 
                          << " to " << task_info.assigned_slave ;
                task_manager_->failTask(task_info.task_id, "Failed to send task");
            }
        }
    }
    
    void statusLoop() {
        while (running_) {
            std::this_thread::sleep_for(std::chrono::seconds(30));
            
            if (running_) {
                ILOG << "=== Status Update ===" ;
                printProgress();
                ILOG << "Connected slaves: " << server_->getClientCount() ;
                ILOG << "===================" ;
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
        ILOG << "=== System Status ===" ;
        ILOG << "Server running: " << (server_->isRunning() ? "Yes" : "No") ;
        ILOG << "Server port: " << server_port_ ;
        ILOG << "Connected slaves: " << server_->getClientCount() ;
        printProgress();
        ILOG << "===================" ;
    }
    
    void printSlaveStatus() {
        auto slaves = task_manager_->getAllSlaves();
        
        ILOG << "=== Slave Status ===" ;
        ILOG << "Total slaves: " << slaves.size() ;
        
        for (const auto& slave : slaves) {
            ILOG << "Slave: " << slave.slave_id ;
            ILOG << "  Active: " << (slave.is_active ? "Yes" : "No") ;
            ILOG << "  Busy: " << (slave.is_busy ? "Yes" : "No") ;
            ILOG << "  Completed tasks: " << slave.completed_tasks ;
            ILOG << "  Failed tasks: " << slave.failed_tasks ;
            ILOG << "  Avg processing time: " << slave.avg_processing_time << "s" ;
        }
        ILOG << "==================" ;
    }
    
    void printTaskStatus() {
        ILOG << "=== Task Status ===" ;
        ILOG << "Pending: " << task_manager_->getPendingTaskCount() ;
        ILOG << "Active: " << task_manager_->getActiveTaskCount() ;
        ILOG << "Completed: " << task_manager_->getCompletedTaskCount() ;
        ILOG << "Failed: " << task_manager_->getFailedTaskCount() ;
        ILOG << "==================" ;
    }
    
    void printProgress() {
        double progress = task_manager_->getOverallProgress();
        ILOG << "Overall progress: " << (progress * 100) << "%" ;
    }
    
    void printResults() {
        auto results = task_manager_->getCompletedResults();
        ILOG << "Completed results: " << results.size() ;
        
        // 간단한 통계
        size_t total_processed_points = 0;
        for (const auto& result : results) {
            total_processed_points += result.processed_points.size();
        }
        
        ILOG << "Total processed points: " << total_processed_points ;
    }
    
    void saveResults(const std::string& filename) {
        auto results = task_manager_->getCompletedResults();
        
        if (results.empty()) {
            WLOG << "No results to save" ;
            return;
        }
        
        // 결과 병합
        PointCloud merged_cloud;
        for (const auto& result : results) {
            merged_cloud.addPoints(result.processed_points);
        }
        
        if (merged_cloud.saveToFile(filename)) {
            ILOG << "Results saved to: " << filename ;
            ILOG << "Total points: " << merged_cloud.getPointCount() ;
        } else {
            WLOG << "Failed to save results to: " << filename ;
        }
    }
    
    void clearCompletedTasks() {
        task_manager_->clearCompletedResults();
        ILOG << "Completed tasks cleared" ;
    }

    // Add these missing handler functions to MasterApplication.cpp
// Place these in the private REST API handlers section

    HttpResponse handleGetTasks(const HttpRequest& req) {
        auto pending_tasks = task_manager_->getTasksByStatus(TaskStatus::PENDING);
        auto active_tasks = task_manager_->getTasksByStatus(TaskStatus::IN_PROGRESS);
        auto completed_tasks = task_manager_->getTasksByStatus(TaskStatus::COMPLETED);
        auto failed_tasks = task_manager_->getTasksByStatus(TaskStatus::FAILED);

        std::ostringstream json;
        json << "{\n";
        json << "  \"success\": true,\n";
        json << "  \"data\": {\n";
        json << "    \"summary\": {\n";
        json << "      \"pending_count\": " << pending_tasks.size() << ",\n";
        json << "      \"active_count\": " << active_tasks.size() << ",\n";
        json << "      \"completed_count\": " << completed_tasks.size() << ",\n";
        json << "      \"failed_count\": " << failed_tasks.size() << ",\n";
        json << "      \"total_count\": " << (pending_tasks.size() + active_tasks.size() + completed_tasks.size() + failed_tasks.size()) << "\n";
        json << "    },\n";
        json << "    \"tasks\": {\n";

        // Pending tasks
        json << "      \"pending\": [\n";
        for (size_t i = 0; i < pending_tasks.size(); ++i) {
            const auto& task = pending_tasks[i];
            json << "        {\n";
            json << "          \"task_id\": " << task.task_id << ",\n";
            json << "          \"chunk_id\": " << task.chunk_id << ",\n";
            json << "          \"task_type\": \"" << std::string(taskStr(task.task_type)) << "\",\n";
            json << "          \"status\": \"pending\",\n";
            json << "          \"assigned_slave\": \"" << task.assigned_slave << "\"\n";
            json << "        }";
            if (i < pending_tasks.size() - 1) json << ",";
            json << "\n";
        }
        json << "      ],\n";

        // Active tasks
        json << "      \"active\": [\n";
        for (size_t i = 0; i < active_tasks.size(); ++i) {
            const auto& task = active_tasks[i];
            json << "        {\n";
            json << "          \"task_id\": " << task.task_id << ",\n";
            json << "          \"chunk_id\": " << task.chunk_id << ",\n";
            json << "          \"task_type\": \"" << taskStr(task.task_type) << "\",\n";
            json << "          \"status\": \"in_progress\",\n";
            json << "          \"assigned_slave\": \"" << task.assigned_slave << "\"\n";
            json << "        }";
            if (i < active_tasks.size() - 1) json << ",";
            json << "\n";
        }
        json << "      ],\n";

        // Completed tasks (show only recent ones to avoid huge response)
        json << "      \"completed\": [\n";
        size_t completed_limit = (std::min)(completed_tasks.size(), size_t(10)); // Show only last 10
        for (size_t i = 0; i < completed_limit; ++i) {
            const auto& task = completed_tasks[i];
            json << "        {\n";
            json << "          \"task_id\": " << task.task_id << ",\n";
            json << "          \"chunk_id\": " << task.chunk_id << ",\n";
            json << "          \"task_type\": \"" << taskStr(task.task_type) << "\",\n";
            json << "          \"status\": \"completed\",\n";
            json << "          \"assigned_slave\": \"" << task.assigned_slave << "\"\n";
            json << "        }";
            if (i < completed_limit - 1) json << ",";
            json << "\n";
        }
        json << "      ],\n";

        // Failed tasks
        json << "      \"failed\": [\n";
        for (size_t i = 0; i < failed_tasks.size(); ++i) {
            const auto& task = failed_tasks[i];
            json << "        {\n";
            json << "          \"task_id\": " << task.task_id << ",\n";
            json << "          \"chunk_id\": " << task.chunk_id << ",\n";
            json << "          \"task_type\": \"" << taskStr(task.task_type) << "\",\n";
            json << "          \"status\": \"failed\",\n";
            json << "          \"assigned_slave\": \"" << task.assigned_slave << "\"\n";
            json << "        }";
            if (i < failed_tasks.size() - 1) json << ",";
            json << "\n";
        }
        json << "      ]\n";
        json << "    }\n";
        json << "  }\n";
        json << "}";

        HttpResponse response;
        response.body = json.str();
        return response;
    }

    HttpResponse handleGetTask(const HttpRequest& req) {
        std::string task_id_str = req.path_params.at("id");
        uint32_t task_id;

        try {
            task_id = static_cast<uint32_t>(std::stoul(task_id_str));
        }
        catch (const std::exception&) {
            return createErrorResponse(400, "Invalid task ID format");
        }

        // Search through all task statuses to find the task
        std::vector<TaskStatus> all_statuses = {
            TaskStatus::PENDING,
            TaskStatus::IN_PROGRESS,
            TaskStatus::COMPLETED,
            TaskStatus::FAILED
        };

        for (const auto& status : all_statuses) {
            auto tasks = task_manager_->getTasksByStatus(status);
            for (const auto& task : tasks) {
                if (task.task_id == task_id) {
                    std::string status_str;
                    switch (status) {
                    case TaskStatus::PENDING: status_str = "pending"; break;
                    case TaskStatus::IN_PROGRESS: status_str = "in_progress"; break;
                    case TaskStatus::COMPLETED: status_str = "completed"; break;
                    case TaskStatus::FAILED: status_str = "failed"; break;
                    }

                    std::ostringstream json;
                    json << "{\n";
                    json << "  \"success\": true,\n";
                    json << "  \"data\": {\n";
                    json << "    \"task_id\": " << task.task_id << ",\n";
                    json << "    \"chunk_id\": " << task.chunk_id << ",\n";
                    json << "    \"task_type\": \"" << taskStr(task.task_type) << "\",\n";
                    json << "    \"status\": \"" << status_str << "\",\n";
                    json << "    \"assigned_slave\": \"" << task.assigned_slave << "\",\n";
                    json << "    \"parameters_size\": " << task.parameters.size();

                    if (task.chunk_data) {
                        json << ",\n    \"chunk_points\": " << task.chunk_data->points.size();
                    }

                    json << "\n  }\n";
                    json << "}";

                    HttpResponse response;
                    response.body = json.str();
                    return response;
                }
            }
        }

        return createErrorResponse(404, "Task not found");
    }

    HttpResponse handleClearTasks(const HttpRequest& req) {
        // Get completed tasks count before clearing
        auto completed_results = task_manager_->getCompletedResults();
        size_t completed_count = completed_results.size();

        // Clear completed tasks
        task_manager_->clearCompletedResults();

        std::ostringstream json;
        json << "{\n";
        json << "  \"success\": true,\n";
        json << "  \"message\": \"Completed tasks cleared successfully\",\n";
        json << "  \"data\": {\n";
        json << "    \"cleared_tasks_count\": " << completed_count << ",\n";
        json << "    \"timestamp\": \"" << getCurrentTimestamp() << "\"\n";
        json << "  }\n";
        json << "}";

        HttpResponse response;
        response.body = json.str();
        return response;
    }

    void setupRestApiRoutes() {
        api_server_ = std::make_unique<RestApiServer>();

        // 시스템 상태
        api_server_->GET("/api/status", [this](const HttpRequest& req) {
            return handleGetStatus(req);
            });

        // 슬레이브 관리
        api_server_->GET("/api/slaves", [this](const HttpRequest& req) {
            return handleGetSlaves(req);
            });

        api_server_->GET("/api/slaves/{id}", [this](const HttpRequest& req) {
            return handleGetSlave(req);
            });

        api_server_->POST("/api/slaves/{id}/shutdown", [this](const HttpRequest& req) {
            return handleShutdownSlave(req);
            });

        api_server_->POST("/api/slaves/shutdown", [this](const HttpRequest& req) {
            return handleShutdownAllSlaves(req);
            });

        // 작업 관리
        api_server_->GET("/api/tasks", [this](const HttpRequest& req) {
            return handleGetTasks(req);
            });

        api_server_->GET("/api/tasks/{id}", [this](const HttpRequest& req) {
            return handleGetTask(req);
            });

        api_server_->DEL("/api/tasks", [this](const HttpRequest& req) {
            return handleClearTasks(req);
            });

        // 파일 처리
        api_server_->POST("/api/files/load", [this](const HttpRequest& req) {
            return handleLoadFile(req);
            });

        // 진행상황 및 결과
        api_server_->GET("/api/progress", [this](const HttpRequest& req) {
            return handleGetProgress(req);
            });

        api_server_->GET("/api/results", [this](const HttpRequest& req) {
            return handleGetResults(req);
            });

        api_server_->POST("/api/results/save", [this](const HttpRequest& req) {
            return handleSaveResults(req);
            });

        // 서버 제어
        api_server_->POST("/api/shutdown", [this](const HttpRequest& req) {
            return handleShutdownServer(req);
            });

        // 헬스체크
        api_server_->GET("/api/health", [this](const HttpRequest& req) {
            HttpResponse response;
            response.status_code = 200;
            response.body = R"({"success": true, "status": "healthy", "timestamp": ")" + getCurrentTimestamp() + "\"}";
            return response;
            });
    }

private: /// REST API 핸들러들
	HttpResponse handleGetStatus(const HttpRequest& req) {
		std::ostringstream json;
		json << "{\n";
		json << "  \"success\": true,\n";
		json << "  \"data\": {\n";
		json << "    \"server_running\": " << (server_->isRunning() ? "true" : "false") << ",\n";
		json << "    \"server_port\": " << server_port_ << ",\n";
		json << "    \"api_port\": " << api_port_ << ",\n";
		json << "    \"connected_slaves\": " << server_->getClientCount() << ",\n";
		json << "    \"pending_tasks\": " << task_manager_->getPendingTaskCount() << ",\n";
		json << "    \"active_tasks\": " << task_manager_->getActiveTaskCount() << ",\n";
		json << "    \"completed_tasks\": " << task_manager_->getCompletedTaskCount() << ",\n";
		json << "    \"failed_tasks\": " << task_manager_->getFailedTaskCount() << ",\n";
		json << "    \"overall_progress\": " << std::fixed << std::setprecision(2) << (task_manager_->getOverallProgress() * 100) << ",\n";
		json << "    \"timestamp\": \"" << getCurrentTimestamp() << "\"\n";
		json << "  }\n";
		json << "}";

		HttpResponse response;
		response.body = json.str();
		return response;
	}

	HttpResponse handleGetSlaves(const HttpRequest& req) {
		auto slaves = task_manager_->getAllSlaves();

		std::ostringstream json;
		json << "{\n";
		json << "  \"success\": true,\n";
		json << "  \"data\": {\n";
		json << "    \"total_slaves\": " << slaves.size() << ",\n";
		json << "    \"slaves\": [\n";

		for (size_t i = 0; i < slaves.size(); ++i) {
			const auto& slave = slaves[i];
			json << "      {\n";
			json << "        \"id\": \"" << slave.slave_id << "\",\n";
			json << "        \"active\": " << (slave.is_active ? "true" : "false") << ",\n";
			json << "        \"busy\": " << (slave.is_busy ? "true" : "false") << ",\n";
			json << "        \"completed_tasks\": " << slave.completed_tasks << ",\n";
			json << "        \"failed_tasks\": " << slave.failed_tasks << ",\n";
			json << "        \"avg_processing_time\": " << std::fixed << std::setprecision(2) << slave.avg_processing_time << "\n";
			json << "      }";
			if (i < slaves.size() - 1) json << ",";
			json << "\n";
		}

		json << "    ]\n";
		json << "  }\n";
		json << "}";

		HttpResponse response;
		response.body = json.str();
		return response;
	}

	HttpResponse handleGetSlave(const HttpRequest& req) {
		std::string slave_id = req.path_params.at("id");
		auto slaves = task_manager_->getAllSlaves();

		for (const auto& slave : slaves) {
			if (slave.slave_id == slave_id) {
				std::ostringstream json;
				json << "{\n";
				json << "  \"success\": true,\n";
				json << "  \"data\": {\n";
				json << "    \"id\": \"" << slave.slave_id << "\",\n";
				json << "    \"active\": " << (slave.is_active ? "true" : "false") << ",\n";
				json << "    \"busy\": " << (slave.is_busy ? "true" : "false") << ",\n";
				json << "    \"completed_tasks\": " << slave.completed_tasks << ",\n";
				json << "    \"failed_tasks\": " << slave.failed_tasks << ",\n";
				json << "    \"avg_processing_time\": " << std::fixed << std::setprecision(2) << slave.avg_processing_time << "\n";
				json << "  }\n";
				json << "}";

				HttpResponse response;
				response.body = json.str();
				return response;
			}
		}

		return createErrorResponse(404, "Slave not found");
	}

    HttpResponse handleLoadFile(const HttpRequest& req) {
        // 1) Content-Type 확인
        auto it_ct = req.headers.find("Content-Type");
        std::string content_type = (it_ct != req.headers.end() ? it_ct->second : "");
        if (content_type.find("application/json") == std::string::npos) {
            return createErrorResponse(400, "Content-Type must be application/json");
        }

        // 2) JSON 파싱
        auto parsed = DPApp::MiniJson::parse_object(req.body);
        if (!parsed) {
            return createErrorResponse(400, "Invalid JSON body");
        }

        // 3) 필수/옵션 필드 추출
        auto filename = DPApp::MiniJson::get<std::string>(*parsed, "filename");
        std::optional<std::string> task_type_str = DPApp::MiniJson::get<std::string>(*parsed, "task_type");
        
        if (!filename || !task_type_str) {
            return createErrorResponse(400, "Missing 'filename' or 'task_type'");
        }

        TaskType task_type = strTask(task_type_str.value());

        if (loadAndProcessPointCloud(*filename, task_type)) {
            std::ostringstream json;
            json << "{\n";
            json << "  \"success\": true,\n";
            json << "  \"message\": \"Point cloud processing started\",\n";
            json << "  \"data\": {\n";
            json << "    \"filename\": \"" << *filename << "\",\n";
            json << "    \"task_type\": \"" << taskStr(task_type) << "\"\n";
            json << "  }\n";
            json << "}";
            HttpResponse response;
            response.body = json.str();
            return response;
        }
        else {
            return createErrorResponse(500, "Failed to load point cloud file");
        }
    }


	HttpResponse handleGetProgress(const HttpRequest& req) {
		double progress = task_manager_->getOverallProgress();

		std::ostringstream json;
		json << "{\n";
		json << "  \"success\": true,\n";
		json << "  \"data\": {\n";
		json << "    \"overall_progress\": " << std::fixed << std::setprecision(2) << (progress * 100) << ",\n";
		json << "    \"pending_tasks\": " << task_manager_->getPendingTaskCount() << ",\n";
		json << "    \"active_tasks\": " << task_manager_->getActiveTaskCount() << ",\n";
		json << "    \"completed_tasks\": " << task_manager_->getCompletedTaskCount() << ",\n";
		json << "    \"failed_tasks\": " << task_manager_->getFailedTaskCount() << "\n";
		json << "  }\n";
		json << "}";

		HttpResponse response;
		response.body = json.str();
		return response;
	}

	HttpResponse handleGetResults(const HttpRequest& req) {
		auto results = task_manager_->getCompletedResults();

		size_t total_processed_points = 0;
		for (const auto& result : results) {
			total_processed_points += result.processed_points.size();
		}

		std::ostringstream json;
		json << "{\n";
		json << "  \"success\": true,\n";
		json << "  \"data\": {\n";
		json << "    \"completed_results\": " << results.size() << ",\n";
		json << "    \"total_processed_points\": " << total_processed_points << "\n";
		json << "  }\n";
		json << "}";

		HttpResponse response;
		response.body = json.str();
		return response;
	}

	HttpResponse handleSaveResults(const HttpRequest& req) {
        // 1) Content-Type 확인
        auto it_ct = req.headers.find("Content-Type");
        std::string content_type = (it_ct != req.headers.end() ? it_ct->second : "");
        if (content_type.find("application/json") == std::string::npos) {
            return createErrorResponse(400, "Content-Type must be application/json");
        }

        // 2) JSON 파싱
        auto parsed = DPApp::MiniJson::parse_object(req.body);
        if (!parsed) {
            return createErrorResponse(400, "Invalid JSON body");
        }

        // 3) 필수 필드
        auto filename0 = DPApp::MiniJson::get<std::string>(*parsed, "filename");
        if (!filename0 || filename0->empty()) {
            return createErrorResponse(400, "Missing 'filename'");
        }

        std::string filename = filename0.value();        

		auto results = task_manager_->getCompletedResults();

		if (results.empty()) {
			return createErrorResponse(400, "No results to save");
		}

		// 결과 병합
		PointCloud merged_cloud;
		for (const auto& result : results) {
			merged_cloud.addPoints(result.processed_points);
		}

		if (merged_cloud.saveToFile(filename)) {
			std::ostringstream json;
			json << "{\n";
			json << "  \"success\": true,\n";
			json << "  \"message\": \"Results saved successfully\",\n";
			json << "  \"data\": {\n";
			json << "    \"filename\": \"" << filename << "\",\n";
			json << "    \"total_points\": " << merged_cloud.getPointCount() << "\n";
			json << "  }\n";
			json << "}";

			HttpResponse response;
			response.body = json.str();
			return response;
		}
		else {
			return createErrorResponse(500, "Failed to save results");
		}
	}

	HttpResponse handleShutdownSlave(const HttpRequest& req) {
		std::string slave_id = req.path_params.at("id");

		NetworkMessage shutdown_msg(MessageType::SHUTDOWN, std::vector<uint8_t>());

		if (server_->sendMessage(slave_id, shutdown_msg)) {
			std::ostringstream json;
			json << "{\n";
			json << "  \"success\": true,\n";
			json << "  \"message\": \"Shutdown command sent to slave\",\n";
			json << "  \"data\": {\n";
			json << "    \"slave_id\": \"" << slave_id << "\"\n";
			json << "  }\n";
			json << "}";

			HttpResponse response;
			response.body = json.str();
			return response;
		}
		else {
			return createErrorResponse(500, "Failed to send shutdown command");
		}
	}

	HttpResponse handleShutdownAllSlaves(const HttpRequest& req) {
		auto connected_clients = server_->getConnectedClients();

		if (connected_clients.empty()) {
			return createErrorResponse(400, "No connected slaves to shutdown");
		}

		NetworkMessage shutdown_msg(MessageType::SHUTDOWN, std::vector<uint8_t>());
		int success_count = 0;

		for (const auto& client_id : connected_clients) {
			if (server_->sendMessage(client_id, shutdown_msg)) {
				success_count++;
			}
		}

		std::ostringstream json;
		json << "{\n";
		json << "  \"success\": true,\n";
		json << "  \"message\": \"Shutdown commands sent\",\n";
		json << "  \"data\": {\n";
		json << "    \"total_slaves\": " << connected_clients.size() << ",\n";
		json << "    \"shutdown_sent\": " << success_count << "\n";
		json << "  }\n";
		json << "}";

		HttpResponse response;
		response.body = json.str();
		return response;
	}

	HttpResponse handleShutdownServer(const HttpRequest& req) {
		// 비동기적으로 서버 종료 (응답을 보낸 후)
		std::thread([this]() {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			stop();
			}).detach();

		std::ostringstream json;
		json << "{\n";
		json << "  \"success\": true,\n";
		json << "  \"message\": \"Server shutdown initiated\"\n";
		json << "}";

		HttpResponse response;
		response.body = json.str();
		return response;
	}

private: /// utils
	HttpResponse createErrorResponse(int status_code, const std::string& error) {
		HttpResponse response;
		response.status_code = status_code;

		switch (status_code) {
		case 400: response.status_text = "Bad Request"; break;
		case 404: response.status_text = "Not Found"; break;
		case 500: response.status_text = "Internal Server Error"; break;
		default: response.status_text = "Error"; break;
		}

		std::ostringstream json;
		json << "{\n";
		json << "  \"success\": false,\n";
		json << "  \"error\": \"" << error << "\"\n";
		json << "}";

		response.body = json.str();
		return response;
	}

	std::string extractJsonValue(const std::string& json, const std::string& key) {
		// 간단한 JSON 값 추출 (실제로는 JSON 라이브러리 사용 권장)
		std::string search = "\"" + key + "\"";
		size_t pos = json.find(search);
		if (pos == std::string::npos) return "";

		pos = json.find(":", pos);
		if (pos == std::string::npos) return "";

		pos = json.find("\"", pos);
		if (pos == std::string::npos) return "";

		size_t end_pos = json.find("\"", pos + 1);
		if (end_pos == std::string::npos) return "";

		return json.substr(pos + 1, end_pos - pos - 1);
	}

    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        std::time_t tt = std::chrono::system_clock::to_time_t(now);

        std::tm tm_utc{};
#ifdef _WIN32
        // gmtime_s( out_tm, in_time_t ) : 0 이면 성공
        if (gmtime_s(&tm_utc, &tt) != 0) {
            return ""; // 필요 시 로깅/예외 처리로 교체
        }
#else
        // gmtime_r( in_time_t, out_tm ) : nullptr 아니면 성공
        if (gmtime_r(&tt, &tm_utc) == nullptr) {
            return ""; // 필요 시 로깅/예외 처리로 교체
        }
#endif

        std::ostringstream oss;
        oss << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%SZ");
        return oss.str();
    }

private:
	std::unique_ptr<NetworkServer> server_;
	std::unique_ptr<TaskManager> task_manager_;
	std::unique_ptr<RestApiServer> api_server_;

	std::atomic<bool> running_;
	uint16_t server_port_;
	uint16_t api_port_;
	size_t max_points_per_chunk_ = 10000;
	bool daemon_mode_ = false;

	std::thread task_assignment_thread_;
	std::thread status_thread_;
	std::thread timeout_thread_;
};

// 전역 변수 (시그널 핸들러용)
static MasterApplication* g_app = nullptr;

void signalHandler(int signal) {
    ILOG << "";
    ILOG << "Received signal " << signal << ", shutting down...";
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
        ELOG << "Failed to initialize master application" ;
        return 1;
    }
    
    app.run();
    
    return 0;
}