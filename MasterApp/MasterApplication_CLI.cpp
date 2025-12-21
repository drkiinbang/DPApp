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
    ILOG << "";
    ILOG << "=== Test Commands ===";
    ILOG << "test echo <count> [time_ms]    - Run TEST_ECHO tasks";
    ILOG << "test compute <count> [time_ms] - Run TEST_COMPUTE tasks";
    ILOG << "test delay <count> <extra_ms> [time_ms] - Run TEST_DELAY tasks";
    ILOG << "test fail <count> <fail_n> [time_ms]    - Run TEST_FAIL tasks";
    ILOG << "test status                    - Show test results summary";
    ILOG << "test help                      - Show test command help";
    ILOG << "";
    ILOG << "help                           - Show this help";
    ILOG << "quit                           - Shutdown server";
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
    else if (cmd == "test") {
        std::string rest_of_line;
        std::getline(iss, rest_of_line);
        runTestCommand(rest_of_line);
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

/// =========================================
/// Test Commands Implementation
/// =========================================

void MasterApplication::runTestCommand(const std::string& args) {
    std::istringstream iss(args);
    std::string sub_cmd;
    iss >> sub_cmd;

    if (sub_cmd.empty() || sub_cmd == "help") {
        printTestHelp();
    }
    else if (sub_cmd == "status") {
        printTestStatus();
    }
    else if (sub_cmd == "echo") {
        uint32_t count = 1;
        uint32_t time_ms = 10000;
        iss >> count;
        if (iss >> time_ms) {}  // optional
        runTestEcho(count, time_ms);
    }
    else if (sub_cmd == "compute") {
        uint32_t count = 1;
        uint32_t time_ms = 10000;
        iss >> count;
        if (iss >> time_ms) {}  // optional
        runTestCompute(count, time_ms);
    }
    else if (sub_cmd == "delay") {
        uint32_t count = 1;
        uint32_t extra_ms = 5000;
        uint32_t time_ms = 10000;
        iss >> count >> extra_ms;
        if (iss >> time_ms) {}  // optional
        runTestDelay(count, extra_ms, time_ms);
    }
    else if (sub_cmd == "fail") {
        uint32_t count = 1;
        uint32_t fail_n = 2;
        uint32_t time_ms = 10000;
        iss >> count >> fail_n;
        if (iss >> time_ms) {}  // optional
        runTestFail(count, fail_n, time_ms);
    }
    else {
        WLOG << "Unknown test command: " << sub_cmd;
        printTestHelp();
    }
}

void MasterApplication::printTestHelp() {
    ILOG << "";
    ILOG << "=== Test Commands ===";
    ILOG << "test echo <count> [time_ms]";
    ILOG << "    - Run TEST_ECHO tasks (data round-trip)";
    ILOG << "    - count: number of tasks (default: 1)";
    ILOG << "    - time_ms: processing time per task (default: 10000ms)";
    ILOG << "";
    ILOG << "test compute <count> [time_ms]";
    ILOG << "    - Run TEST_COMPUTE tasks (square calculation)";
    ILOG << "    - Results are verified for correctness";
    ILOG << "";
    ILOG << "test delay <count> <extra_ms> [time_ms]";
    ILOG << "    - Run TEST_DELAY tasks (timeout testing)";
    ILOG << "    - extra_ms: additional delay on top of base time";
    ILOG << "";
    ILOG << "test fail <count> <fail_n> [time_ms]";
    ILOG << "    - Run TEST_FAIL tasks (retry testing)";
    ILOG << "    - fail_n: number of times to fail before success";
    ILOG << "";
    ILOG << "test status";
    ILOG << "    - Show test results summary";
    ILOG << "";
    ILOG << "Examples:";
    ILOG << "  test echo 5           - 5 echo tasks, 10s each";
    ILOG << "  test compute 10 15000 - 10 compute tasks, 15s each";
    ILOG << "  test delay 3 20000    - 3 delay tasks with 20s extra delay";
    ILOG << "  test fail 2 3         - 2 tasks that fail 3 times then succeed";
    ILOG << "";
}

void MasterApplication::runTestEcho(uint32_t count, uint32_t base_time_ms) {
    ILOG << "Starting TEST_ECHO: " << count << " tasks, " << base_time_ms << "ms each";

    // TaskManager 초기화 (아직 없으면)
    if (!task_manager_) {
        if (!initializeTaskManager(TaskType::TEST_ECHO)) {
            ELOG << "Failed to initialize TaskManager for TEST_ECHO";
            return;
        }
    }

    // 테스트 청크 생성
    std::vector<TestChunk> chunks;
    for (uint32_t i = 0; i < count; ++i) {
        TestChunk chunk = TestChunk::generate(i, 100, base_time_ms);
        // TEST_ECHO: expected_output은 input과 동일해야 함
        chunk.expected_output = chunk.input_numbers;
        chunks.push_back(chunk);
    }

    createTestTasks(TaskType::TEST_ECHO, chunks);

    ILOG << "Created " << count << " TEST_ECHO tasks";
}

void MasterApplication::runTestCompute(uint32_t count, uint32_t base_time_ms) {
    ILOG << "Starting TEST_COMPUTE: " << count << " tasks, " << base_time_ms << "ms each";

    if (!task_manager_) {
        if (!initializeTaskManager(TaskType::TEST_COMPUTE)) {
            ELOG << "Failed to initialize TaskManager for TEST_COMPUTE";
            return;
        }
    }

    std::vector<TestChunk> chunks;
    for (uint32_t i = 0; i < count; ++i) {
        TestChunk chunk = TestChunk::generate(i, 100, base_time_ms);
        chunks.push_back(chunk);
    }

    createTestTasks(TaskType::TEST_COMPUTE, chunks);

    ILOG << "Created " << count << " TEST_COMPUTE tasks";
}

void MasterApplication::runTestDelay(uint32_t count, uint32_t extra_delay_ms, uint32_t base_time_ms) {
    ILOG << "Starting TEST_DELAY: " << count << " tasks, base=" << base_time_ms
        << "ms, extra=" << extra_delay_ms << "ms";

    if (!task_manager_) {
        if (!initializeTaskManager(TaskType::TEST_DELAY)) {
            ELOG << "Failed to initialize TaskManager for TEST_DELAY";
            return;
        }
    }

    std::vector<TestChunk> chunks;
    for (uint32_t i = 0; i < count; ++i) {
        TestChunk chunk = TestChunk::generate(i, 100, base_time_ms);
        chunk.extra_delay_ms = extra_delay_ms;
        chunks.push_back(chunk);
    }

    createTestTasks(TaskType::TEST_DELAY, chunks);

    ILOG << "Created " << count << " TEST_DELAY tasks (total time: "
        << (base_time_ms + extra_delay_ms) << "ms each)";
}

void MasterApplication::runTestFail(uint32_t count, uint32_t fail_count, uint32_t base_time_ms) {
    ILOG << "Starting TEST_FAIL: " << count << " tasks, fail " << fail_count
        << " times before success, " << base_time_ms << "ms each";

    if (!task_manager_) {
        if (!initializeTaskManager(TaskType::TEST_FAIL)) {
            ELOG << "Failed to initialize TaskManager for TEST_FAIL";
            return;
        }
    }

    std::vector<TestChunk> chunks;
    for (uint32_t i = 0; i < count; ++i) {
        TestChunk chunk = TestChunk::generate(i, 100, base_time_ms);
        chunk.fail_count = fail_count;
        chunks.push_back(chunk);
    }

    createTestTasks(TaskType::TEST_FAIL, chunks);

    ILOG << "Created " << count << " TEST_FAIL tasks";
}

void MasterApplication::createTestTasks(TaskType test_type, const std::vector<TestChunk>& chunks) {
    if (!task_manager_) {
        ELOG << "TaskManager not initialized";
        return;
    }

    // TaskManager에 Task 추가
    for (auto chunk : chunks) {  // 복사본 사용 (chunk_id 수정 위해)
        // 전역 고유 chunk_id 할당
        {
            std::lock_guard<std::mutex> lock(test_mutex_);
            chunk.chunk_id = next_test_chunk_id_++;
            test_chunks_.push_back(chunk);  // 수정된 chunk_id로 저장
        }

        auto pc_chunk = std::make_shared<PointCloudChunk>();
        pc_chunk->chunk_id = chunk.chunk_id;
        // points는 비워둠 - 실제 데이터는 TestChunk에서 전송

        uint32_t task_id = task_manager_->addTask(pc_chunk);

        // Task 타입을 테스트 타입으로 변경
        TaskInfo* task_info = task_manager_->getTaskById(task_id);
        if (task_info) {
            task_info->task_type = test_type;
        }

        ILOG << "Created test task " << task_id << " (type: " << taskStr(test_type)
            << ", chunk: " << chunk.chunk_id << ")";
    }
}

void MasterApplication::printTestStatus() {
    std::lock_guard<std::mutex> lock(test_mutex_);

    ILOG << "";
    ILOG << "=== Test Status ===";
    ILOG << "Test chunks created: " << test_chunks_.size();
    ILOG << "Test results received: " << test_results_.size();

    if (test_results_.empty()) {
        ILOG << "No test results yet";
        ILOG << "===================";
        return;
    }

    uint32_t success_count = 0;
    uint32_t fail_count = 0;
    uint32_t verified_count = 0;
    double total_time = 0;

    for (const auto& result : test_results_) {
        if (result.success) {
            success_count++;
            total_time += result.processing_time_ms;

            // 검증
            for (const auto& chunk : test_chunks_) {
                if (chunk.chunk_id == result.chunk_id) {
                    TestResult mutable_result = result;
                    if (TestProcessors::verifyResult(chunk, mutable_result)) {
                        verified_count++;
                    }
                    break;
                }
            }
        }
        else {
            fail_count++;
        }
    }

    ILOG << "";
    ILOG << "Results:";
    ILOG << "  Success: " << success_count << "/" << test_results_.size();
    ILOG << "  Failed:  " << fail_count;
    ILOG << "  Verified: " << verified_count << "/" << success_count;

    if (success_count > 0) {
        ILOG << "  Avg time: " << (total_time / success_count) << "ms";
    }

    ILOG << "===================";
    ILOG << "";
}