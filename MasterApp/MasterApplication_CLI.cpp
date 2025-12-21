/**
 * @file MasterApplication_CLI.cpp
 * @brief Command Line Interface implementation
 * 
 * This file contains:
 * - Command line parsing
 * - Console command processing
 * - Status printing functions
 * - Results display and saving
 */

#include "MasterApplication.h"

/// =========================================
/// Command Line Parsing
/// =========================================

void MasterApplication::parseCommandLine(int argc, char* argv[]) {
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
        else if (arg == "--external-ip") {
            if (i + 1 < argc) {
                master_external_ip_ = argv[++i];
            }
        }
        else if (arg == "-h" || arg == "--help") {
            printUsage();
            exit(0);
        }
    }
}

void MasterApplication::printUsage() {
    ILOG << "Usage: master [options]";
    ILOG << "Options:";
    ILOG << "  -p, --port <port>        Main server port (default: 8080)";
    ILOG << "  --api-port <port>        REST API port (default: 8081)";
    ILOG << "  -d, --daemon             Run in daemon mode";
    ILOG << "  -c, --chunk-size <size>  Max points per chunk (default: 10000)";
    ILOG << "  --external-ip <ip>       External IP for slaves to connect";
    ILOG << "  -h, --help               Show this help message";
}

void MasterApplication::printHelp() {
    ILOG << "";
    ILOG << "=== DPApp Master Console Commands ===";
    ILOG << "bimpc <mesh_folder> <pc_file>  - Process BIM mesh + PointCloud";
    ILOG << "load <file> <task_type>        - Load file and create tasks";
    ILOG << "status                         - Show system status";
    ILOG << "slaves                         - List connected slaves";
    ILOG << "agents                         - List registered agents";
    ILOG << "tasks                          - List all tasks";
    ILOG << "progress                       - Show overall progress";
    ILOG << "results                        - Show completed results count";
    ILOG << "save <file>                    - Save merged results to file";
    ILOG << "clear                          - Clear all completed tasks";
    ILOG << "shutdown [slave_id]            - Shutdown specific slave or all";
    ILOG << "start-slaves [threads]         - Start slaves on all agents";
    ILOG << "stop-slaves                    - Stop slaves on all agents";
    ILOG << "help                           - Show this help";
    ILOG << "quit                           - Stop and exit";
    ILOG << "=====================================";
    ILOG << "";
    ILOG << "--- BIM-PointCloud Processing ---";
    ILOG << "  bimpc C:/data/meshes C:/data/scan.las";
    ILOG << "  - mesh_folder: Folder containing .gltf/.glb files";
    ILOG << "  - pc_file: PointCloud file (.las, .xyz, .pts)";
    ILOG << "";
}

/// =========================================
/// Command Processing
/// =========================================

void MasterApplication::processCommand(const std::string& command) {
    std::istringstream iss(command);
    std::string cmd;
    iss >> cmd;

    if (cmd == "bimpc") {
        std::string mesh_folder, pc_file;
        iss >> mesh_folder >> pc_file;

        if (mesh_folder.empty() || pc_file.empty()) {
            WLOG << "Usage: bimpc <mesh_folder> <pointcloud_file>";
            WLOG << "Example: bimpc C:/data/meshes C:/data/scan.las";
        }
        else {
            runBimPcProcess(mesh_folder, pc_file);
        }
        return;
    }
    else if (cmd == "load") {
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
    else if (cmd == "agents") {
        printAgentStatus();
    }
    else if (cmd == "start-slaves") {
        uint32_t threads = 4;
        std::string threads_str;
        if (iss >> threads_str) {
            try {
                threads = static_cast<uint32_t>(std::stoul(threads_str));
            }
            catch (...) {}
        }
        int started = startSlavesOnAllAgents(threads);
        ILOG << "Started slaves on " << started << " agents";
    }
    else if (cmd == "stop-slaves") {
        int stopped = stopSlavesOnAllAgents(false);
        ILOG << "Stopped slaves on " << stopped << " agents";
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

/// =========================================
/// Slave Control
/// =========================================

void MasterApplication::shutdownSlave(const std::string& slave_id) {
    NetworkMessage shutdown_msg(MessageType::SHUTDOWN, std::vector<uint8_t>());

    if (server_->sendMessage(slave_id, shutdown_msg)) {
        ILOG << "Shutdown command sent to slave: " << slave_id;
    }
    else {
        WLOG << "Failed to send shutdown command to slave: " << slave_id;
    }
}

void MasterApplication::shutdownAllSlaves() {
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

/// =========================================
/// Status Printing
/// =========================================

void MasterApplication::printSystemStatus() {
    ILOG << "=== System Status ===";
    ILOG << "Server running: " << (server_->isRunning() ? "Yes" : "No");
    ILOG << "Server port: " << server_port_;
    ILOG << "Connected slaves: " << server_->getClientCount();
    ILOG << "Registered agents: " << agents_.size();
    if (task_manager_) {
        ILOG << "Current task type: " << taskStr(task_manager_->getCurrentTaskType());
    }
    else {
        ILOG << "No task session active";
    }
    printProgress();
    ILOG << "===================";
}

void MasterApplication::printSlaveStatus() {
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

void MasterApplication::printAgentStatus() {
    std::lock_guard<std::mutex> lock(agents_mutex_);

    ILOG << "=== Agent Status ===";
    ILOG << "Total agents: " << agents_.size();

    for (const auto& agent : agents_) {
        ILOG << "Agent: " << agent.agent_id;
        ILOG << "  Host: " << agent.host << ":" << agent.port;
        ILOG << "  Online: " << (agent.online ? "Yes" : "No");
    }
    ILOG << "==================";
}

void MasterApplication::printTaskStatus() {
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

void MasterApplication::printProgress() {
    if (!task_manager_) {
        ILOG << "Overall progress: No active session";
        return;
    }

    double progress = task_manager_->getOverallProgress();
    ILOG << "Overall progress: " << (progress * 100) << "%";
}

void MasterApplication::printResults() {
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

/// =========================================
/// Results Management
/// =========================================

void MasterApplication::saveResults(const std::string& filename) {
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
            outfile << point[0] << " " << point[1] << " " << point[2];
            outfile << "\n";
            total_points++;
        }
    }

    outfile.close();

    ILOG << "Results saved to: " << filename;
    ILOG << "Total points: " << total_points;
}

void MasterApplication::clearCompletedTasks() {
    if (!task_manager_) {
        WLOG << "No task session active - no tasks to clear";
        return;
    }

    task_manager_->clearCompletedResults();
    ILOG << "Completed tasks cleared";
}
