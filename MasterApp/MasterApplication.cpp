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
    TaskType current_task_type_;
    std::vector<uint8_t> current_parameters_;

    std::unique_ptr<NetworkServer> server_;
    std::unique_ptr<TaskManager> task_manager_;
    std::unique_ptr<RestApiServer> api_server_;

    std::atomic<bool> running_;
    uint16_t server_port_;
    uint16_t api_port_;
    uint32_t max_points_per_chunk_ = 10000;
    bool daemon_mode_ = false;

    std::thread task_assignment_thread_;
    std::thread status_thread_;
    std::thread timeout_thread_;

public:
    bool isDaemonMode() const { return daemon_mode_; }

    MasterApplication() : running_(false), server_port_(8080), api_port_(8081) {}

    ~MasterApplication() {
        stop();
    }

    bool initialize(int argc, char* argv[]) {
        std::time_t t = std::time(nullptr);
        std::tm tm{};
#ifdef _WIN32
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        char logFilename[512];
        std::strftime(logFilename, sizeof(logFilename), "MasterApp_%Y-%m-%d_%H-%M-%S.log", &tm);
        DPApp::Logger::initialize(logFilename);
        DPApp::Logger::setMinLevel(DPApp::LogLevel::INFO);

        parseCommandLine(argc, argv);

        server_ = std::make_unique<NetworkServer>();
        server_->setMessageCallback([this](const NetworkMessage& msg, const std::string& client_id) {
            handleMessage(msg, client_id);
            });

        server_->setConnectionCallback([this](const std::string& client_id, bool connected) {
            handleConnection(client_id, connected);
            });

        setupRestApiRoutes();
        return true;
    }

    bool initializeTaskManager(TaskType task_type) {
        current_task_type_ = task_type;
        current_parameters_.clear();

        if (task_type == TaskType::CONVERT_PTS) {
            double offset[3] = { 0.0, 0.0, 0.0 };
            current_parameters_.resize(sizeof(double) * 3);
            std::memcpy(current_parameters_.data(), offset, sizeof(double) * 3);
        }

        task_manager_ = std::make_unique<TaskManager>(
            cfg_.task_timeout_seconds,
            current_task_type_,
            current_parameters_
        );

        task_manager_->setTaskCompletedCallback([this](const TaskInfo& task, const ProcessingResult& result) {
            handleTaskCompleted(task, result);
            });

        task_manager_->setTaskFailedCallback([this](const TaskInfo& task, const std::string& error) {
            handleTaskFailed(task, error);
            });

        ILOG << "TaskManager initialized for task type: " << taskStr(task_type);
        return true;
    }

    bool start() {
        if (running_) return false;

        cfg_ = DPApp::RuntimeConfig::loadFromEnv();

        ILOG << "Starting DPApp Master Server...";
        ILOG << "Main Port: " << server_port_;
        ILOG << "REST API Port: " << api_port_;
        ILOG << "Max points per chunk: " << max_points_per_chunk_;

        if (!server_->start(server_port_)) {
            ELOG << "Failed to start network server";
            return false;
        }

        if (!api_server_->start(api_port_)) {
            ELOG << "Failed to start REST API server";
            return false;
        }

        running_ = true;

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
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        running_ = false;

        if (task_manager_) {
            task_manager_->requestStop();
        }

        auto start_time = std::chrono::steady_clock::now();
        while (task_manager_ && task_manager_->getActiveTaskCount() > 0 &&
            std::chrono::steady_clock::now() - start_time < std::chrono::seconds(30)) {
            ILOG << "Waiting for " << task_manager_->getActiveTaskCount() << " active tasks to complete...";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        if (task_assignment_thread_.joinable()) {
            task_assignment_thread_.join();
        }
        if (status_thread_.joinable()) {
            status_thread_.join();
        }
        if (timeout_thread_.joinable()) {
            timeout_thread_.join();
        }

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

    bool loadAndProcessPointCloud(const std::string& filename, const TaskType task_type) {
        if (!task_manager_ || task_manager_->getCurrentTaskType() != task_type) {
            if (!initializeTaskManager(task_type)) {
                ELOG << "Failed to initialize TaskManager for task type: " << taskStr(task_type);
                return false;
            }
        }

        ILOG << "Loading point cloud: " << filename;
        ILOG << "Task type: " << taskStr(task_type);

        auto chunks = PointCloudLoader::loadPointCloudFileInChunks(filename, max_points_per_chunk_);

        if (chunks.empty()) {
            ELOG << "Failed to load point cloud file: " << filename;
            return false;
        }

        ILOG << "Loaded " << chunks.size() << " chunks";

        createTasksFromChunks(chunks);

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
        ILOG << "Usage: master [options]";
        ILOG << "Options:";
        ILOG << "  -p, --port <port>        Main server port (default: 8080)";
        ILOG << "  --api-port <port>        REST API port (default: 8081)";
        ILOG << "  -d, --daemon             Run in daemon mode";
        ILOG << "  -c, --chunk-size <size>  Max points per chunk (default: 10000)";
        ILOG << "  -h, --help               Show this help message";
    }

    void printHelp() {
        ILOG << "";
        ILOG << "=== DPApp Master Console Commands ===";
        ILOG << "load <file> <task_type>  - Load point cloud and create processing tasks.";
        //ILOG << "pts2bim_dist <filename> <task_type>  - Load bim/point cloud and create processing tasks for computing distances.";
        ILOG << "status                   - Show system status";
        ILOG << "slaves                   - List connected slaves";
        ILOG << "tasks                    - List all tasks";
        ILOG << "progress                 - Show overall progress";
        ILOG << "results                  - Show completed results count";
        ILOG << "save <file>              - Save merged results to file";
        ILOG << "clear                    - Clear all completed tasks";
        ILOG << "shutdown [slave_id]      - Shutdown specific slave or all slaves";
        ILOG << "help                     - Show this help";
        ILOG << "quit                     - Stop and exit";
        ILOG << "=====================================";
        ILOG << "";
        ILOG << "--- Available Task Types (one per session) ---";
        ILOG << "  Usage for 'load': load C:/data/scan.xyz <task_type_name>";
        ILOG << "";
        ILOG << "  [Name]                [Description]";
        ILOG << "  --------------------  ----------------------------------------------------";
        ILOG << "  convert_pts           Convert points with coordinate adjustments";
        ILOG << "  bim                   Calculate distances from origin";
        ILOG << "  ";
        ILOG << "  Note: Only one task type can be processed per session.";
        ILOG << "        Restart the application to change task type.";
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
                WLOG << "Usage: load <filename> <task_type>";
                WLOG << "Task types: convert_pts, bim";
                WLOG << "Note: Only one task type per session";
            }
            else {
                auto task_type = strTask(task_type_str);

                if (task_manager_ && task_manager_->getCurrentTaskType() != task_type) {
                    WLOG << "Cannot change task type during session.";
                    WLOG << "Current task type: " << taskStr(task_manager_->getCurrentTaskType());
                    WLOG << "Requested task type: " << taskStr(task_type);
                    WLOG << "Please restart the application to change task type.";
                    return;
                }

                loadAndProcessPointCloud(filename, task_type);
            }
        }
        else if (cmd == "pts2bim_dist") {
            std::string gltfPath, pc2Path;
            iss >> gltfPath >> pc2Path;
            if (gltfPath.empty() || pc2Path.empty()) {
                WLOG << "Usage: pts2bim_dist <glt/glbf_Path> <pointclluds2_path>";
            }
            else {
                TaskType task_type = TaskType::BIM_PC2_DIST;

                if (task_manager_ && task_manager_->getCurrentTaskType() != task_type) {
                    WLOG << "Cannot change task type during session.";
                    WLOG << "Current task type: " << taskStr(task_manager_->getCurrentTaskType());
                    WLOG << "Requested task type: " << taskStr(task_type);
                    WLOG << "Please restart the application to change task type.";
                    return;
                }

                loadAndProcessPointCloud(pc2Path, task_type);
            }
        }
        else if (cmd == "status") {
            printSystemStatus();
        }
        else if (cmd == "slaves") {
            printSlaveStatus();
        }
        else if (cmd == "tasks") {
            printTaskStatus();
        }
        else if (cmd == "progress") {
            printProgress();
        }
        else if (cmd == "results") {
            printResults();
        }
        else if (cmd == "save") {
            std::string filename;
            iss >> filename;
            if (filename.empty()) {
                WLOG << "Usage: save <filename>";
            }
            else {
                saveResults(filename);
            }
        }
        else if (cmd == "clear") {
            clearCompletedTasks();
        }
        else if (cmd == "shutdown") {
            std::string slave_id;
            iss >> slave_id;

            if (slave_id.empty()) {
                shutdownAllSlaves();
            }
            else {
                shutdownSlave(slave_id);
            }
        }
        else if (cmd == "help") {
            printHelp();
        }
        else if (cmd == "quit" || cmd == "exit") {
            stop();
        }
        else if (!cmd.empty()) {
            WLOG << "Unknown command: " << cmd;
            WLOG << "Type 'help' for available commands";
        }
    }

    void handleMessage(const NetworkMessage& message, const std::string& client_id) {
        switch (message.header.type) {
        case MessageType::SLAVE_REGISTER:
            ILOG << "Slave registration from: " << client_id;
            if (task_manager_) {
                task_manager_->registerSlave(client_id);
            }
            break;

        case MessageType::SLAVE_UNREGISTER:
            ILOG << "Slave unregistration from: " << client_id;
            if (task_manager_) {
                task_manager_->unregisterSlave(client_id);
            }
            break;

        case MessageType::HEARTBEAT:
            if (task_manager_) {
                task_manager_->updateSlaveHeartbeat(client_id);
            }
            break;

        case MessageType::TASK_RESULT:
            handleTaskResult(message, client_id);
            break;

        default:
            WLOG << "Unknown message type from " << client_id;
            break;
        }
    }

    void handleConnection(const std::string& client_id, bool connected) {
        if (connected) {
            ILOG << "Client connected: " << client_id;
        }
        else {
            ILOG << "Client disconnected: " << client_id;
            if (task_manager_) {
                task_manager_->unregisterSlave(client_id);
            }
        }
    }

    void handleTaskResult(const NetworkMessage& message, const std::string& client_id) {
        try {
            ProcessingResult result = NetworkUtils::deserializeResult(message.data);
            if (task_manager_) {
                task_manager_->completeTask(result.task_id, result);
            }
        }
        catch (const std::exception& e) {
            ELOG << "Error processing task result from " << client_id
                << ": " << e.what();
        }
    }

    void handleTaskCompleted(const TaskInfo& task, const ProcessingResult& result) {
        ILOG << "Task " << task.task_id << " completed by "
            << task.assigned_slave;
    }

    void handleTaskFailed(const TaskInfo& task, const std::string& error) {
        WLOG << "Task " << task.task_id << " failed: " << error;
    }

    void createTasksFromChunks(const std::vector<std::shared_ptr<PointCloudChunk>>& chunks) {
        ILOG << "Creating " << chunks.size() << " tasks for " << taskStr(current_task_type_);

        for (const auto& chunk_ptr : chunks) {
            uint32_t task_id = task_manager_->addTask(chunk_ptr);
            ILOG << "Created task " << task_id << " for chunk " << chunk_ptr->chunk_id
                << " (" << chunk_ptr->points.size() << " points)";
        }
    }

    void taskAssignmentLoop() {
        while (running_) {
            if (task_manager_) {
                task_manager_->assignTasksToSlaves();
                sendAssignedTasks();
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    void sendAssignedTasks() {
        if (!task_manager_) return;

        auto tasks = task_manager_->getTasksByStatus(TaskStatus::ASSIGNED);

        for (auto& task_info : tasks) {
            task_manager_->updateTaskStatus(task_info.task_id, TaskStatus::IN_PROGRESS);

            ProcessingTask task;
            task.task_id = task_info.task_id;
            task.chunk_id = task_info.chunk_id;
            task.task_type = task_info.task_type;
            task.parameters = task_info.parameters;

            auto task_data = NetworkUtils::serializeTask(task);
            auto chunk_data = NetworkUtils::serializeChunk(*task_info.chunk_data);

            std::vector<uint8_t> combined_data;
            uint32_t task_data_size = static_cast<uint32_t>(task_data.size());
            combined_data.insert(combined_data.end(),
                reinterpret_cast<uint8_t*>(&task_data_size),
                reinterpret_cast<uint8_t*>(&task_data_size) + sizeof(uint32_t));

            combined_data.insert(combined_data.end(), task_data.begin(), task_data.end());
            combined_data.insert(combined_data.end(), chunk_data.begin(), chunk_data.end());

            NetworkMessage message(MessageType::TASK_ASSIGNMENT, combined_data);

            if (server_->sendMessage(task_info.assigned_slave, message)) {
                ILOG << "Task " << task_info.task_id
                    << " sent to " << task_info.assigned_slave;
            }
            else {
                ELOG << "Failed to send task " << task_info.task_id
                    << " to " << task_info.assigned_slave;
                task_manager_->failTask(task_info.task_id, "Failed to send task");
            }
        }
    }

    void statusLoop() {
        while (running_) {
            std::this_thread::sleep_for(std::chrono::seconds(30));

            if (running_) {
                ILOG << "=== Status Update ===";
                printProgress();
                ILOG << "Connected slaves: " << server_->getClientCount();
                ILOG << "===================";
            }
        }
    }

    void timeoutLoop() {
        while (running_) {
            if (task_manager_) {
                task_manager_->checkTimeouts();
            }
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }
    }

    void printSystemStatus() {
        ILOG << "=== System Status ===";
        ILOG << "Server running: " << (server_->isRunning() ? "Yes" : "No");
        ILOG << "Server port: " << server_port_;
        ILOG << "Connected slaves: " << server_->getClientCount();
        if (task_manager_) {
            ILOG << "Current task type: " << taskStr(task_manager_->getCurrentTaskType());
        }
        else {
            ILOG << "No task session active";
        }
        printProgress();
        ILOG << "===================";
    }

    void printSlaveStatus() {
        if (!task_manager_) {
            ILOG << "No task session active - no slaves registered";
            return;
        }

        auto slaves = task_manager_->getAllSlaves();

        ILOG << "=== Slave Status ===";
        ILOG << "Total slaves: " << slaves.size();

        for (const auto& slave : slaves) {
            ILOG << "Slave: " << slave.slave_id;
            ILOG << "  Active: " << (slave.is_active ? "Yes" : "No");
            ILOG << "  Busy: " << (slave.is_busy ? "Yes" : "No");
            ILOG << "  Completed tasks: " << slave.completed_tasks;
            ILOG << "  Failed tasks: " << slave.failed_tasks;
            ILOG << "  Avg processing time: " << slave.avg_processing_time << "s";
        }
        ILOG << "==================";
    }

    void printTaskStatus() {
        if (!task_manager_) {
            ILOG << "No task session active";
            return;
        }

        ILOG << "=== Task Status ===";
        ILOG << "Task type: " << taskStr(task_manager_->getCurrentTaskType());
        ILOG << "Pending: " << task_manager_->getPendingTaskCount();
        ILOG << "Active: " << task_manager_->getActiveTaskCount();
        ILOG << "Completed: " << task_manager_->getCompletedTaskCount();
        ILOG << "Failed: " << task_manager_->getFailedTaskCount();
        ILOG << "==================";
    }

    void printProgress() {
        if (!task_manager_) {
            ILOG << "Overall progress: No active session";
            return;
        }

        double progress = task_manager_->getOverallProgress();
        ILOG << "Overall progress: " << (progress * 100) << "%";
    }

    void printResults() {
        if (!task_manager_) {
            ILOG << "No task session active - no results";
            return;
        }

        auto results = task_manager_->getCompletedResults();
        ILOG << "Completed results: " << results.size();

        size_t total_processed_points = 0;
        for (const auto& result : results) {
            total_processed_points += result.processed_points.size();
        }

        ILOG << "Total processed points: " << total_processed_points;
    }

    void saveResults(const std::string& filename) {
        if (!task_manager_) {
            WLOG << "No task session active - no results to save";
            return;
        }

        auto results = task_manager_->getCompletedResults();

        if (results.empty()) {
            WLOG << "No results to save";
            return;
        }

        std::ofstream outfile(filename);
        if (!outfile.is_open()) {
            WLOG << "Failed to open output file: " << filename;
            return;
        }

        size_t total_points = 0;

        for (const auto& result : results) {
            for (const auto& point : result.processed_points) {
                outfile << point.x << " " << point.y << " " << point.z;

                outfile << "\n";
                total_points++;
            }
        }

        outfile.close();

        ILOG << "Results saved to: " << filename;
        ILOG << "Total points: " << total_points;
    }

    void clearCompletedTasks() {
        if (!task_manager_) {
            WLOG << "No task session active - no tasks to clear";
            return;
        }

        task_manager_->clearCompletedResults();
        ILOG << "Completed tasks cleared";
    }

    void setupRestApiRoutes() {
        api_server_ = std::make_unique<RestApiServer>();

        api_server_->GET("/api/status", [this](const HttpRequest& req) {
            return handleGetStatus(req);
            });

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

        api_server_->GET("/api/tasks", [this](const HttpRequest& req) {
            return handleGetTasks(req);
            });

        api_server_->GET("/api/tasks/{id}", [this](const HttpRequest& req) {
            return handleGetTask(req);
            });

        api_server_->DEL("/api/tasks", [this](const HttpRequest& req) {
            return handleClearTasks(req);
            });

        api_server_->POST("/api/files/load", [this](const HttpRequest& req) {
            return handleLoadFile(req);
            });

        api_server_->GET("/api/progress", [this](const HttpRequest& req) {
            return handleGetProgress(req);
            });

        api_server_->GET("/api/results", [this](const HttpRequest& req) {
            return handleGetResults(req);
            });

        api_server_->POST("/api/results/save", [this](const HttpRequest& req) {
            return handleSaveResults(req);
            });

        api_server_->POST("/api/shutdown", [this](const HttpRequest& req) {
            return handleShutdownServer(req);
            });

        api_server_->GET("/api/health", [this](const HttpRequest& req) {
            HttpResponse response;
            response.status_code = 200;
            response.body = R"({"success": true, "status": "healthy", "timestamp": ")" + getCurrentTimestamp() + "\"}";
            return response;
            });
    }

    HttpResponse handleGetStatus(const HttpRequest& req) {
        std::ostringstream json;
        json << "{\n";
        json << "  \"success\": true,\n";
        json << "  \"data\": {\n";
        json << "    \"server_running\": " << (server_->isRunning() ? "true" : "false") << ",\n";
        json << "    \"server_port\": " << server_port_ << ",\n";
        json << "    \"api_port\": " << api_port_ << ",\n";
        json << "    \"connected_slaves\": " << server_->getClientCount() << ",\n";

        if (task_manager_) {
            json << "    \"current_task_type\": \"" << taskStr(task_manager_->getCurrentTaskType()) << "\",\n";
            json << "    \"pending_tasks\": " << task_manager_->getPendingTaskCount() << ",\n";
            json << "    \"active_tasks\": " << task_manager_->getActiveTaskCount() << ",\n";
            json << "    \"completed_tasks\": " << task_manager_->getCompletedTaskCount() << ",\n";
            json << "    \"failed_tasks\": " << task_manager_->getFailedTaskCount() << ",\n";
            json << "    \"overall_progress\": " << std::fixed << std::setprecision(2) << (task_manager_->getOverallProgress() * 100) << ",\n";
        }
        else {
            json << "    \"current_task_type\": null,\n";
            json << "    \"pending_tasks\": 0,\n";
            json << "    \"active_tasks\": 0,\n";
            json << "    \"completed_tasks\": 0,\n";
            json << "    \"failed_tasks\": 0,\n";
            json << "    \"overall_progress\": 0,\n";
        }

        json << "    \"timestamp\": \"" << getCurrentTimestamp() << "\"\n";
        json << "  }\n";
        json << "}";

        HttpResponse response;
        response.body = json.str();
        return response;
    }

    HttpResponse handleGetSlaves(const HttpRequest& req) {
        if (!task_manager_) {
            HttpResponse response;
            response.body = R"({"success": false, "error": "No task session active"})";
            return response;
        }

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
        if (!task_manager_) {
            return createErrorResponse(400, "No task session active");
        }

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

    HttpResponse handleGetTasks(const HttpRequest& req) {
        if (!task_manager_) {
            return createErrorResponse(400, "No task session active");
        }

        auto pending_tasks = task_manager_->getTasksByStatus(TaskStatus::PENDING);
        auto active_tasks = task_manager_->getTasksByStatus(TaskStatus::IN_PROGRESS);
        auto completed_tasks = task_manager_->getTasksByStatus(TaskStatus::COMPLETED);
        auto failed_tasks = task_manager_->getTasksByStatus(TaskStatus::FAILED);

        std::ostringstream json;
        json << "{\n";
        json << "  \"success\": true,\n";
        json << "  \"data\": {\n";
        json << "    \"task_type\": \"" << taskStr(task_manager_->getCurrentTaskType()) << "\",\n";
        json << "    \"summary\": {\n";
        json << "      \"pending_count\": " << pending_tasks.size() << ",\n";
        json << "      \"active_count\": " << active_tasks.size() << ",\n";
        json << "      \"completed_count\": " << completed_tasks.size() << ",\n";
        json << "      \"failed_count\": " << failed_tasks.size() << ",\n";
        json << "      \"total_count\": " << (pending_tasks.size() + active_tasks.size() + completed_tasks.size() + failed_tasks.size()) << "\n";
        json << "    }\n";
        json << "  }\n";
        json << "}";

        HttpResponse response;
        response.body = json.str();
        return response;
    }

    HttpResponse handleGetTask(const HttpRequest& req) {
        if (!task_manager_) {
            return createErrorResponse(400, "No task session active");
        }

        std::string task_id_str = req.path_params.at("id");
        uint32_t task_id;

        try {
            task_id = static_cast<uint32_t>(std::stoul(task_id_str));
        }
        catch (const std::exception&) {
            return createErrorResponse(400, "Invalid task ID format");
        }

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
                    json << "    \"assigned_slave\": \"" << task.assigned_slave << "\"\n";
                    json << "  }\n";
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
        if (!task_manager_) {
            return createErrorResponse(400, "No task session active");
        }

        auto completed_results = task_manager_->getCompletedResults();
        size_t completed_count = completed_results.size();

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

    HttpResponse handleLoadFile(const HttpRequest& req) {
        auto it_ct = req.headers.find("Content-Type");
        std::string content_type = (it_ct != req.headers.end() ? it_ct->second : "");
        if (content_type.find("application/json") == std::string::npos) {
            return createErrorResponse(400, "Content-Type must be application/json");
        }

        auto parsed = DPApp::MiniJson::parse_object(req.body);
        if (!parsed) {
            return createErrorResponse(400, "Invalid JSON body");
        }

        auto filename = DPApp::MiniJson::get<std::string>(*parsed, "filename");
        std::optional<std::string> task_type_str = DPApp::MiniJson::get<std::string>(*parsed, "task_type");

        if (!filename || !task_type_str) {
            return createErrorResponse(400, "Missing 'filename' or 'task_type'");
        }

        TaskType task_type = strTask(task_type_str.value());

        if (task_manager_ && task_manager_->getCurrentTaskType() != task_type) {
            std::ostringstream error_msg;
            error_msg << "Cannot change task type during session. Current: "
                << taskStr(task_manager_->getCurrentTaskType())
                << ", Requested: " << taskStr(task_type)
                << ". Please restart the application.";
            return createErrorResponse(400, error_msg.str());
        }

        if (loadAndProcessPointCloud(*filename, task_type)) {
            std::ostringstream json;
            json << "{\n";
            json << "  \"success\": true,\n";
            json << "  \"message\": \"Point cloud processing started\",\n";
            json << "  \"data\": {\n";
            json << "    \"filename\": \"" << *filename << "\",\n";
            json << "    \"task_type\": \"" << taskStr(task_type) << "\",\n";
            json << "    \"session_mode\": \"single_task_type\"\n";
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
        if (!task_manager_) {
            HttpResponse response;
            response.body = R"({"success": true, "data": {"overall_progress": 0, "pending_tasks": 0, "active_tasks": 0, "completed_tasks": 0, "failed_tasks": 0, "message": "No task session active"}})";
            return response;
        }

        double progress = task_manager_->getOverallProgress();

        std::ostringstream json;
        json << "{\n";
        json << "  \"success\": true,\n";
        json << "  \"data\": {\n";
        json << "    \"task_type\": \"" << taskStr(task_manager_->getCurrentTaskType()) << "\",\n";
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
        if (!task_manager_) {
            HttpResponse response;
            response.body = R"({"success": true, "data": {"completed_results": 0, "total_processed_points": 0, "message": "No task session active"}})";
            return response;
        }

        auto results = task_manager_->getCompletedResults();

        size_t total_processed_points = 0;
        for (const auto& result : results) {
            total_processed_points += result.processed_points.size();
        }

        std::ostringstream json;
        json << "{\n";
        json << "  \"success\": true,\n";
        json << "  \"data\": {\n";
        json << "    \"task_type\": \"" << taskStr(task_manager_->getCurrentTaskType()) << "\",\n";
        json << "    \"completed_results\": " << results.size() << ",\n";
        json << "    \"total_processed_points\": " << total_processed_points << "\n";
        json << "  }\n";
        json << "}";

        HttpResponse response;
        response.body = json.str();
        return response;
    }

    HttpResponse handleSaveResults(const HttpRequest& req) {
        if (!task_manager_) {
            return createErrorResponse(400, "No task session active");
        }

        auto it_ct = req.headers.find("Content-Type");
        std::string content_type = (it_ct != req.headers.end() ? it_ct->second : "");
        if (content_type.find("application/json") == std::string::npos) {
            return createErrorResponse(400, "Content-Type must be application/json");
        }

        auto parsed = DPApp::MiniJson::parse_object(req.body);
        if (!parsed) {
            return createErrorResponse(400, "Invalid JSON body");
        }

        auto filename0 = DPApp::MiniJson::get<std::string>(*parsed, "filename");
        if (!filename0 || filename0->empty()) {
            return createErrorResponse(400, "Missing 'filename'");
        }

        std::string filename = filename0.value();
        auto results = task_manager_->getCompletedResults();

        if (results.empty()) {
            return createErrorResponse(400, "No results to save");
        }

        std::ofstream outfile(filename);
        if (!outfile.is_open()) {
            return createErrorResponse(500, "Failed to open output file");
        }

        size_t total_points = 0;

        for (const auto& result : results) {
            for (const auto& point : result.processed_points) {
                outfile << point.x << " " << point.y << " " << point.z;

                outfile << "\n";
                total_points++;
            }
        }

        outfile.close();

        std::ostringstream json;
        json << "{\n";
        json << "  \"success\": true,\n";
        json << "  \"message\": \"Results saved successfully\",\n";
        json << "  \"data\": {\n";
        json << "    \"filename\": \"" << filename << "\",\n";
        json << "    \"total_points\": " << total_points << ",\n";
        json << "    \"task_type\": \"" << taskStr(task_manager_->getCurrentTaskType()) << "\"\n";
        json << "  }\n";
        json << "}";

        HttpResponse response;
        response.body = json.str();
        return response;
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

    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        std::time_t tt = std::chrono::system_clock::to_time_t(now);

        std::tm tm_utc{};
#ifdef _WIN32
        if (gmtime_s(&tm_utc, &tt) != 0) {
            return "";
        }
#else
        if (gmtime_r(&tt, &tm_utc) == nullptr) {
            return "";
        }
#endif

        std::ostringstream oss;
        oss << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%SZ");
        return oss.str();
    }
};

// Global variable (for signal handler)
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
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    MasterApplication app;
    g_app = &app;

    if (!app.initialize(argc, argv)) {
        ELOG << "Failed to initialize master application";
        return 1;
    }

    app.run();

    return 0;
}