/**
 * @file MasterApplication.cpp
 * @brief Core implementation of MasterApplication
 *
 * This file contains:
 * - Constructor/Destructor
 * - Initialization and lifecycle management
 * - Network message handlers
 * - Task assignment and processing loops
 * - PointCloud loading and BimPc processing
 */

#include "MasterApplication.h"
#include "../include/TaskManager.h"

 /// =========================================
 /// Global variable (for signal handler)
 /// =========================================
MasterApplication* g_app = nullptr;

/// =========================================
/// PointCloudLoader namespace implementation
/// =========================================

namespace DPApp {
    namespace PointCloudLoader {
        std::vector<std::shared_ptr<BimPcChunk>> loadBimPcChunks(
            const std::string& bim_folder,
            const std::string& pointcloud_file)
        {
            std::vector<std::shared_ptr<BimPcChunk>> chunks;

            try {
                std::cout << "Loading BIM mesh folder: " << bim_folder << std::endl;
                std::cout << "Loading PointCloud file: " << pointcloud_file << std::endl;

                /// 1. Load mesh data
                std::vector<chunkbim::MeshChunk> bimData;
                auto retval = loadGltf(bim_folder, bimData, 0);
                std::cout << "Loaded " << bimData.size() << " bim chunks" << std::endl;

                if (bimData.empty()) {
                    std::cerr << "No BIM data loaded" << std::endl;
                    return chunks;
                }

                /// 2. Load point cloud data
                std::vector<pctree::XYZPoint> pc;
                if (!loadLasFile(pointcloud_file, pc)) {
                    return chunks;
                }

                size_t numChunks = bimData.size();
                size_t numChunkPts = (pc.size() + numChunks - 1) / numChunks;

                for (size_t i = 0; i < numChunks; ++i) {
                    chunks.emplace_back(std::make_shared<BimPcChunk>());
                    auto& bimpc_chunk = chunks.back();

                    bimpc_chunk->chunk_id = static_cast<uint32_t>(i);

                    size_t start_idx = i * numChunkPts;
                    size_t end_idx = (std::min)(start_idx + numChunkPts, pc.size());

                    if (start_idx < pc.size()) {
                        bimpc_chunk->points.assign(
                            pc.begin() + start_idx,
                            pc.begin() + end_idx
                        );
                    }

                    bimpc_chunk->bim = std::move(bimData[i]);
                    bimpc_chunk->calculateBounds();

                    std::cout << "Created BimPcChunk " << bimpc_chunk->chunk_id
                        << " - points: " << bimpc_chunk->points.size()
                        << ", vertices: " << bimpc_chunk->bim.vertices.size()
                        << ", faces: " << bimpc_chunk->bim.faces.size()
                        << std::endl;
                }

                std::cout << "Created " << chunks.size() << " BimPcChunks" << std::endl;
            }
            catch (const std::exception& e) {
                std::cerr << "Error loading BimPcChunks: " << e.what() << std::endl;
            }

            return chunks;
        }
    }
}

/// =========================================
/// Constructor / Destructor
/// =========================================

MasterApplication::MasterApplication()
    : running_(false), server_port_(8080), api_port_(8081) {
}

MasterApplication::~MasterApplication() {
    stop();
}

/// =========================================
/// Initialization
/// =========================================

bool MasterApplication::initialize(int argc, char* argv[]) {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &t);
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

    /// Load agents configuration
    loadAgentsFromConfig("agents_config.json");

    return true;
}

bool MasterApplication::initializeTaskManager(TaskType task_type) {
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

    if (server_) {
        auto connected_clients = server_->getConnectedClients();
        for (const auto& client_id : connected_clients) {
            task_manager_->registerSlave(client_id);
            ILOG << "Auto-registered existing slave: " << client_id;
        }
        ILOG << "Registered " << connected_clients.size() << " existing slaves to TaskManager";
    }

    ILOG << "TaskManager initialized for task type: " << taskStr(task_type);
    return true;
}

/// =========================================
/// Lifecycle Management
/// =========================================

bool MasterApplication::start() {
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

void MasterApplication::stop() {
    if (!running_) return;

    ILOG << "Stopping master server...";

    shutdownAllSlaves();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    running_ = false;

    if (task_manager_) {
        task_manager_->requestStop();
    }

    if (server_) {
        server_->stop();
    }
    if (api_server_) {
        api_server_->stop();
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

    ILOG << "Master server stopped";
}

void MasterApplication::run() {
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

void MasterApplication::runDaemon() {
    ILOG << "Master server running in daemon mode...";
    ILOG << "REST API: http://localhost:" << api_port_ << "/api";
    ILOG << "Use REST API to control the server.";

    while (running_) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void MasterApplication::runInteractive() {
    printHelp();

    std::string command;
    while (running_ && std::getline(std::cin, command)) {
        processCommand(command);
    }
}

/// =========================================
/// BIM-PointCloud Processing
/// =========================================

void MasterApplication::runBimPcProcess(const std::string& mesh_folder, const std::string& pointcloud_file) {
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "BIM-PointCloud Distributed Processing" << std::endl;
    std::cout << std::string(70, '=') << std::endl;
    std::cout << "Mesh folder: " << mesh_folder << std::endl;
    std::cout << "PointCloud file: " << pointcloud_file << std::endl;

    if (!initializeTaskManager(TaskType::BIM_PC2_DIST)) {
        std::cerr << "Failed to initialize TaskManager" << std::endl;
        return;
    }

    auto slaves = task_manager_->getAllSlaves();
    if (slaves.empty()) {
        std::cerr << "WARNING: No slaves connected! Tasks will remain pending." << std::endl;
        std::cerr << "Please connect at least one slave before running." << std::endl;
    }
    else {
        std::cout << "Connected slaves: " << slaves.size() << std::endl;
        for (const auto& slave : slaves) {
            std::cout << "  - " << slave.slave_id << std::endl;
        }
    }

    std::cout << "\nLoading BIM and PointCloud data..." << std::endl;
    auto chunks = PointCloudLoader::loadBimPcChunks(mesh_folder, pointcloud_file);

    if (chunks.empty()) {
        std::cerr << "Failed to load BimPcChunks" << std::endl;
        return;
    }

    std::cout << "Created " << chunks.size() << " BimPcChunks" << std::endl;

    std::cout << "\nCreating " << chunks.size() << " tasks..." << std::endl;
    for (const auto& chunk : chunks) {
        uint32_t task_id = task_manager_->addTask(chunk);
        ILOG << "Created task " << task_id << " for BimPcChunk " << chunk->chunk_id;
    }

    std::cout << "\nWaiting for tasks to complete..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(300);

    while (task_manager_->getPendingTaskCount() > 0 || task_manager_->getActiveTaskCount() > 0) {
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            std::cerr << "Timeout waiting for tasks to complete" << std::endl;
            break;
        }

        auto pending = task_manager_->getPendingTaskCount();
        auto active = task_manager_->getActiveTaskCount();
        auto completed = task_manager_->getCompletedTaskCount();

        std::cout << "  Pending: " << pending
            << ", Active: " << active
            << ", Completed: " << completed << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cout << "\n" << std::string(70, '-') << std::endl;
    std::cout << "PROCESSING RESULTS" << std::endl;
    std::cout << std::string(70, '-') << std::endl;

    auto results = task_manager_->getBimPcResults();

    int success_count = 0;
    int fail_count = 0;
    double total_processing_time = 0;
    uint32_t total_points = 0;
    uint32_t total_faces = 0;

    for (const auto& result : results) {
        if (result.success) {
            success_count++;
            total_processing_time += result.processing_time_ms;
            total_points += result.total_points_processed;
            total_faces += result.total_faces_processed;

            std::cout << "+ Task " << result.task_id
                << " (chunk " << result.chunk_id << "): "
                << result.total_points_processed << " points, "
                << result.total_faces_processed << " faces, "
                << result.processing_time_ms << "ms" << std::endl;

            std::cout << "    Distance: min=" << result.min_distance
                << ", max=" << result.max_distance
                << ", avg=" << result.avg_distance << std::endl;

            std::cout << "    Classification: within=" << result.points_within_threshold
                << ", outside=" << result.points_outside_threshold << std::endl;
        }
        else {
            fail_count++;
            std::cout << "x Task " << result.task_id
                << " (chunk " << result.chunk_id << "): FAILED - "
                << result.error_message << std::endl;
        }
    }

    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "SUMMARY" << std::endl;
    std::cout << std::string(70, '=') << std::endl;
    std::cout << "Total chunks: " << chunks.size() << std::endl;
    std::cout << "Successful: " << success_count << std::endl;
    std::cout << "Failed: " << fail_count << std::endl;
    std::cout << "Total points processed: " << total_points << std::endl;
    std::cout << "Total faces processed: " << total_faces << std::endl;
    std::cout << "Total processing time: " << total_processing_time << " ms" << std::endl;

    if (success_count == static_cast<int>(chunks.size())) {
        std::cout << "\n+++ ALL TASKS COMPLETED SUCCESSFULLY +++" << std::endl;
    }
    else if (success_count > 0) {
        std::cout << "\n*** PARTIAL SUCCESS ***" << std::endl;
    }
    else {
        std::cout << "\nxxx ALL TASKS FAILED xxx" << std::endl;
    }
    std::cout << std::string(70, '=') << "\n" << std::endl;
}

bool MasterApplication::loadAndProcessPointCloud(const std::string& filename, const TaskType task_type) {
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

/// =========================================
/// Network Message Handlers
/// =========================================

void MasterApplication::handleMessage(const NetworkMessage& message, const std::string& client_id) {
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

void MasterApplication::handleConnection(const std::string& client_id, bool connected) {
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

void MasterApplication::handleTaskResult(const NetworkMessage& message, const std::string& client_id) {
    try {
        if (message.data.size() < 1) {
            ELOG << "Invalid task result message from " << client_id;
            return;
        }

        uint8_t result_type = message.data[0];
        std::vector<uint8_t> result_data(message.data.begin() + 1, message.data.end());

        // result_type: 0 = ProcessingResult, 1 = BimPcResult, 2 = TestResult
        if (result_type == 2) {
            // 테스트 결과 처리
            TestResult result = TestResult::deserialize(result_data);
            handleTestResult(result);

            // TaskManager 상태 업데이트
            if (task_manager_) {
                ProcessingResult simple_result;
                simple_result.task_id = result.task_id;
                simple_result.chunk_id = result.chunk_id;
                simple_result.success = result.success;
                simple_result.error_message = result.error_message;
                task_manager_->completeTask(result.task_id, simple_result);
            }

            ILOG << "TestResult received for task " << result.task_id
                << " from " << client_id
                << " (success: " << (result.success ? "Yes" : "No")
                << ", time: " << result.processing_time_ms << "ms)";
        }
        else if (result_type == 1) {
            // BimPc 결과 처리
            BimPcResult result = NetworkUtils::deserializeBimPcResult(result_data);
            if (task_manager_) {
                task_manager_->addBimPcResult(result);

                ProcessingResult simple_result;
                simple_result.task_id = result.task_id;
                simple_result.chunk_id = result.chunk_id;
                simple_result.success = result.success;
                simple_result.error_message = result.error_message;
                task_manager_->completeTask(result.task_id, simple_result);
            }
            ILOG << "BimPcResult received for task " << result.task_id
                << " from " << client_id;
        }
        else {
            // 일반 결과 처리
            ProcessingResult result = NetworkUtils::deserializeResult(result_data);
            if (task_manager_) {
                task_manager_->completeTask(result.task_id, result);
            }
        }
    }
    catch (const std::exception& e) {
        ELOG << "Error processing task result from " << client_id
            << ": " << e.what();
    }
}

/// =========================================
/// Test Result Handling
/// =========================================

void MasterApplication::handleTestResult(const TestResult& result) {
    std::lock_guard<std::mutex> lock(test_mutex_);

    // 결과 저장
    test_results_.push_back(result);

    // 해당 청크 찾아서 검증
    for (auto& chunk : test_chunks_) {
        if (chunk.chunk_id == result.chunk_id) {
            TestResult mutable_result = result;
            bool verified = TestProcessors::verifyResult(chunk, mutable_result);

            if (verified) {
                ILOG << "[TEST] Task " << result.task_id << " VERIFIED - all "
                    << result.output_numbers.size() << " results correct";
            }
            else if (result.success) {
                WLOG << "[TEST] Task " << result.task_id << " MISMATCH - "
                    << mutable_result.mismatch_count << " errors found";
            }
            break;
        }
    }
}

void MasterApplication::handleTaskCompleted(const TaskInfo& task, const ProcessingResult& result) {
    ILOG << "Task " << task.task_id << " completed by " << task.assigned_slave;
}

void MasterApplication::handleTaskFailed(const TaskInfo& task, const std::string& error) {
    WLOG << "Task " << task.task_id << " failed: " << error;
}

/// =========================================
/// Task Creation
/// =========================================

void MasterApplication::createTasksFromChunks(const std::vector<std::shared_ptr<PointCloudChunk>>& chunks) {
    ILOG << "Creating " << chunks.size() << " tasks for " << taskStr(current_task_type_);

    for (const auto& chunk_ptr : chunks) {
        uint32_t task_id = task_manager_->addTask(chunk_ptr);
        ILOG << "Created task " << task_id << " for chunk " << chunk_ptr->chunk_id
            << " (" << chunk_ptr->points.size() << " points)";
    }
}

void MasterApplication::createTasksFromChunks(const std::vector<std::shared_ptr<BimPcChunk>>& chunks) {
    ILOG << "Creating " << chunks.size() << " tasks for " << taskStr(current_task_type_);

    for (const auto& chunk_ptr : chunks) {
        uint32_t task_id = task_manager_->addTask(chunk_ptr);
        ILOG << "Created task " << task_id << " for chunk " << chunk_ptr->chunk_id
            << " (" << chunk_ptr->points.size() << " points)";
    }
}

/// =========================================
/// Background Loops
/// =========================================

void MasterApplication::taskAssignmentLoop() {
    while (running_) {
        if (task_manager_) {
            task_manager_->assignTasksToSlaves();
            sendAssignedTasks();
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void MasterApplication::sendAssignedTasks() {
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
        std::vector<uint8_t> chunk_data;

        // 테스트 Task 타입인지 확인
        bool is_test_task = (task_info.task_type == TaskType::TEST_ECHO ||
            task_info.task_type == TaskType::TEST_COMPUTE ||
            task_info.task_type == TaskType::TEST_DELAY ||
            task_info.task_type == TaskType::TEST_FAIL);

        bool is_bimpc_task = (task_info.task_type == TaskType::BIM_PC2_DIST &&
            task_info.bimpc_chunk_data != nullptr);

        if (is_test_task) {
            // 테스트 청크 데이터 직렬화
            std::lock_guard<std::mutex> lock(test_mutex_);
            for (const auto& test_chunk : test_chunks_) {
                if (test_chunk.chunk_id == task_info.chunk_id) {
                    chunk_data = test_chunk.serialize();
                    ILOG << "Task " << task_info.task_id << " has TestChunk data ("
                        << test_chunk.input_numbers.size() << " numbers, "
                        << "base_time: " << test_chunk.base_processing_time_ms << "ms)";
                    break;
                }
            }
        }
        else if (is_bimpc_task) {
            chunk_data = NetworkUtils::serializeBimPcChunk(*task_info.bimpc_chunk_data);
            ILOG << "Task " << task_info.task_id << " has BimPcChunk data ("
                << task_info.bimpc_chunk_data->points.size() << " points, "
                << task_info.bimpc_chunk_data->bim.faces.size() << " faces)";
        }
        else if (task_info.chunk_data) {
            chunk_data = NetworkUtils::serializeChunk(*task_info.chunk_data);
            ILOG << "Task " << task_info.task_id << " has PointCloudChunk data ("
                << task_info.chunk_data->points.size() << " points)";
        }
        else {
            ILOG << "Task " << task_info.task_id << " has no chunk data";
        }

        std::vector<uint8_t> combined_data;

        uint32_t task_data_size = static_cast<uint32_t>(task_data.size());
        combined_data.insert(combined_data.end(),
            reinterpret_cast<uint8_t*>(&task_data_size),
            reinterpret_cast<uint8_t*>(&task_data_size) + sizeof(uint32_t));
        combined_data.insert(combined_data.end(), task_data.begin(), task_data.end());

        uint8_t is_bimpc_flag = is_bimpc_task ? 1 : 0;
        combined_data.push_back(is_bimpc_flag);

        uint32_t chunk_data_size = static_cast<uint32_t>(chunk_data.size());
        combined_data.insert(combined_data.end(),
            reinterpret_cast<uint8_t*>(&chunk_data_size),
            reinterpret_cast<uint8_t*>(&chunk_data_size) + sizeof(uint32_t));

        if (!chunk_data.empty()) {
            combined_data.insert(combined_data.end(), chunk_data.begin(), chunk_data.end());
        }

        NetworkMessage message(MessageType::TASK_ASSIGNMENT, combined_data);

        if (server_->sendMessage(task_info.assigned_slave, message)) {
            ILOG << "Task " << task_info.task_id << " sent to " << task_info.assigned_slave;
        }
        else {
            ELOG << "Failed to send task " << task_info.task_id
                << " to " << task_info.assigned_slave;
            task_manager_->failTask(task_info.task_id, "Failed to send task");
        }
    }
}

void MasterApplication::statusLoop() {
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

void MasterApplication::timeoutLoop() {
    while (running_) {
        if (task_manager_) {
            task_manager_->checkTimeouts();
        }
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }
}

/// =========================================
/// Main Entry Point
/// =========================================

static BOOL WINAPI ConsoleHandler(DWORD signal) {
    if (signal == CTRL_C_EVENT || signal == CTRL_BREAK_EVENT) {
        ILOG << "";
        ILOG << "Ctrl+C received, shutting down...";
        if (g_app) {
            g_app->stop();
        }
        return TRUE;
    }
    return FALSE;
}

int main(int argc, char* argv[]) {
    SetConsoleCtrlHandler(ConsoleHandler, TRUE);

    MasterApplication app;
    g_app = &app;

    if (!app.initialize(argc, argv)) {
        ELOG << "Failed to initialize master application";
        return 1;
    }

    app.run();

    return 0;
}