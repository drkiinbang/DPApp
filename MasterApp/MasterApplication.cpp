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
#include "../include/IcpTypes.h"
#include "../include/bimtree/IcpCore.hpp"
#include "../include/bimtree/IcpSerialization.hpp"

#include <iostream>
#include <atomic>

/// [ToDo] Delete the follwing lines for clean-up
 /// Aftet completing compile test, comment out the follwing lines
#ifdef _DEBUG

/*
#ifdef _DEBUG
void testIcpTypesCompile() {
    // Transform4x4 테스트
    icp::Transform4x4 t1 = icp::Transform4x4::identity();
    icp::Transform4x4 t2 = icp::Transform4x4::translation(1.0f, 2.0f, 3.0f);
    icp::Transform4x4 t3 = t1 * t2;

    float x = 0, y = 0, z = 0;
    t3.transformPoint(1.0f, 1.0f, 1.0f, x, y, z);

    float tx, ty, tz;
    t3.getTranslation(tx, ty, tz);

    // IcpConfig 테스트
    icp::IcpConfig config;
    config.maxIterations = 100;
    config.convergenceThreshold = 1e-6f;
    bool valid = config.isValid();

    // IcpChunk 테스트
    icp::IcpChunk chunk;
    chunk.chunk_id = 1;
    chunk.sourcePoints.push_back({ 1.0f, 2.0f, 3.0f });
    chunk.targetPoints.push_back({ 4.0f, 5.0f, 6.0f });
    chunk.calculateSourceBounds();
    chunk.calculateTargetBounds();
    size_t size = chunk.getApproximateSize();
    bool chunkValid = chunk.isValid();

    // IcpResult 테스트
    icp::IcpResult result;
    result.chunk_id = 1;
    result.success = true;
    result.converged = true;
    result.finalRMSE = 0.01f;
    result.initialRMSE = 0.1f;
    float improvement = result.getImprovementRatio();
    bool good = result.isGood(0.05f);

    // IcpJobStatus 테스트
    icp::IcpJobStatus status = icp::IcpJobStatus::PENDING;
    const char* statusStr = icp::statusToString(status);

    // IcpJob 테스트
    icp::IcpJob job;
    job.jobId = "test_job_001";
    job.lasFilePath = "C:/data/test.las";
    job.bimFolderPath = "C:/data/bim";
    job.status = icp::IcpJobStatus::LOADING_DATA;
    float progress = job.getProgress();
    bool finished = job.isFinished();
    bool running = job.isRunning();

    // IcpStatistics 테스트
    icp::IcpStatistics stats = icp::IcpStatistics::fromJob(job);

    // TaskType 테스트 (PointCloudTypes.h 수정 확인)
    DPApp::TaskType icpTask = DPApp::TaskType::ICP_FINE_ALIGNMENT;
    const char* taskName = DPApp::taskStr(icpTask);
    DPApp::TaskType parsed = DPApp::strTask("icp_fine");

    // 사용하지 않는 변수 경고 방지
    (void)x; (void)y; (void)z;
    (void)tx; (void)ty; (void)tz;
    (void)valid; (void)size; (void)chunkValid;
    (void)improvement; (void)good;
    (void)statusStr; (void)progress;
    (void)finished; (void)running;
    (void)taskName; (void)parsed;
    (void)stats;
}
#endif
*/

/*
void testIcpCoreCompile() {
    std::cout << "=== Phase 2 IcpCore Test ===" << std::endl;

    /// 1. 테스트 데이터 생성
    std::vector<std::array<float, 3>> sourcePoints = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        {1.0f, 1.0f, 0.0f},
        {1.0f, 0.0f, 1.0f},
        {0.0f, 1.0f, 1.0f},
        {1.0f, 1.0f, 1.0f}
    };

    /// Target = Source + (0.1, 0.2, 0.3) translation
    std::vector<std::array<float, 3>> targetPoints;
    for (const auto& p : sourcePoints) {
        targetPoints.push_back({ p[0] + 0.1f, p[1] + 0.2f, p[2] + 0.3f });
    }

    /// 2. KD-Tree 테스트
    icp::PointCloudAdaptor adaptor(targetPoints);
    icp::KdTree3D tree(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();
    std::cout << "KD-Tree built successfully" << std::endl;

    /// 3. Correspondence 찾기 테스트
    std::vector<icp::Correspondence> correspondences;
    icp::findCorrespondences(sourcePoints, tree, 1.0f, correspondences);
    std::cout << "Found " << correspondences.size() << " correspondences" << std::endl;

    /// 4. SVD 변환 추정 테스트
    icp::Transform4x4 transform = icp::estimateRigidTransformSVD(
        sourcePoints, targetPoints, correspondences);

    float tx, ty, tz;
    transform.getTranslation(tx, ty, tz);
    std::cout << "Estimated translation: (" << tx << ", " << ty << ", " << tz << ")" << std::endl;
    std::cout << "Expected translation: (0.1, 0.2, 0.3)" << std::endl;

    /// 5. RMSE 계산 테스트
    float rmse = icp::computeRMSE(correspondences);
    std::cout << "Initial RMSE: " << rmse << std::endl;

    /// 6. 다운샘플링 테스트
    auto downsampled = icp::downsampleUniform(sourcePoints, 2);
    std::cout << "Downsampled: " << sourcePoints.size() << " -> " << downsampled.size() << std::endl;

    /// 7. 전체 ICP 테스트
    icp::IcpChunk chunk;
    chunk.chunk_id = 1;
    chunk.sourcePoints = sourcePoints;
    chunk.targetPoints = targetPoints;
    chunk.initialTransform = icp::Transform4x4::identity();
    chunk.config.maxIterations = 50;
    chunk.config.convergenceThreshold = 1e-6f;
    chunk.config.maxCorrespondenceDistance = 1.0f;
    chunk.config.minCorrespondences = 3;
    chunk.config.useNormalFiltering = false;

    std::atomic<bool> cancelRequested{ false };
    icp::IcpResult result = icp::runIcp(chunk, cancelRequested);

    std::cout << "\n=== ICP Result ===" << std::endl;
    std::cout << "Success: " << (result.success ? "Yes" : "No") << std::endl;
    std::cout << "Converged: " << (result.converged ? "Yes" : "No") << std::endl;
    std::cout << "Iterations: " << result.actualIterations << std::endl;
    std::cout << "Initial RMSE: " << result.initialRMSE << std::endl;
    std::cout << "Final RMSE: " << result.finalRMSE << std::endl;
    std::cout << "Processing time: " << result.processingTimeMs << " ms" << std::endl;

    result.transform.getTranslation(tx, ty, tz);
    std::cout << "Final translation: (" << tx << ", " << ty << ", " << tz << ")" << std::endl;

    /// 8. 결과 집계 테스트
    std::vector<icp::IcpResult> results = { result };
    icp::Transform4x4 aggregated = icp::aggregateResultsWeighted(results);
    const icp::IcpResult* best = icp::selectBestResult(results);

    std::cout << "\nAggregation test: " << (best != nullptr ? "OK" : "Failed") << std::endl;
    std::cout << "=== Phase 2 Test Complete ===" << std::endl;

    (void)aggregated;  // 경고 방지
}
*/

/// 부동소수점 비교 헬퍼
inline bool floatEquals(float a, float b, float epsilon = 1e-6f) {
    return std::abs(a - b) < epsilon;
}

/// [ToDo] 직렬화 테스트 더 이상 필요없으면 삭제
/// Phase 3 직렬화 테스트
void testIcpSerializationCompile() {
    std::cout << "=== Phase 3 Serialization Test ===" << std::endl;

    bool allPassed = true;

    /// 1. Transform4x4 직렬화 테스트
    {
        icp::Transform4x4 original = icp::Transform4x4::translation(1.5f, 2.5f, 3.5f);

        std::vector<uint8_t> buffer;
        icp::serializeTransform(original, buffer);

        size_t offset = 0;
        icp::Transform4x4 restored = icp::deserializeTransform(buffer.data(), offset);

        float tx1, ty1, tz1, tx2, ty2, tz2;
        original.getTranslation(tx1, ty1, tz1);
        restored.getTranslation(tx2, ty2, tz2);

        bool match = floatEquals(tx1, tx2) && floatEquals(ty1, ty2) && floatEquals(tz1, tz2);
        std::cout << "Transform4x4 serialization: " << (match ? "PASS" : "FAIL") << std::endl;
        allPassed &= match;
    }

    /// 2. IcpConfig 직렬화 테스트
    {
        icp::IcpConfig original;
        original.maxIterations = 100;
        original.convergenceThreshold = 1e-6f;
        original.maxCorrespondenceDistance = 2.0f;
        original.downsampleRatio = 5;
        original.useNormalFiltering = true;
        original.verbose = true;

        std::vector<uint8_t> buffer;
        icp::serializeConfig(original, buffer);

        size_t offset = 0;
        icp::IcpConfig restored = icp::deserializeConfig(buffer.data(), offset);

        bool match = (original.maxIterations == restored.maxIterations) &&
            (original.downsampleRatio == restored.downsampleRatio) &&
            floatEquals(original.convergenceThreshold, restored.convergenceThreshold) &&
            floatEquals(original.maxCorrespondenceDistance, restored.maxCorrespondenceDistance) &&
            (original.useNormalFiltering == restored.useNormalFiltering) &&
            (original.verbose == restored.verbose);

        std::cout << "IcpConfig serialization: " << (match ? "PASS" : "FAIL") << std::endl;
        allPassed &= match;
    }

    /// 3. IcpChunk 직렬화 테스트
    {
        icp::IcpChunk original;
        original.chunk_id = 42;
        original.sourcePoints = { {1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f} };
        original.targetPoints = { {7.0f, 8.0f, 9.0f}, {10.0f, 11.0f, 12.0f}, {13.0f, 14.0f, 15.0f} };
        original.faceIndices = { 0, 1, 2 };
        original.faceNormals = { {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f} };
        original.initialTransform = icp::Transform4x4::translation(0.1f, 0.2f, 0.3f);
        original.config.maxIterations = 75;
        original.source_min_x = -1.0f; original.source_max_x = 10.0f;
        original.source_min_y = -2.0f; original.source_max_y = 20.0f;
        original.source_min_z = -3.0f; original.source_max_z = 30.0f;

        std::vector<uint8_t> buffer = icp::serializeIcpChunk(original);
        icp::IcpChunk restored = icp::deserializeIcpChunk(buffer);

        bool match = (original.chunk_id == restored.chunk_id) &&
            (original.sourcePoints.size() == restored.sourcePoints.size()) &&
            (original.targetPoints.size() == restored.targetPoints.size()) &&
            (original.faceIndices.size() == restored.faceIndices.size()) &&
            (original.faceNormals.size() == restored.faceNormals.size()) &&
            (original.config.maxIterations == restored.config.maxIterations) &&
            floatEquals(original.source_min_x, restored.source_min_x) &&
            floatEquals(original.source_max_x, restored.source_max_x);

        /// 포인트 데이터 확인
        if (match && original.sourcePoints.size() > 0) {
            match = floatEquals(original.sourcePoints[0][0], restored.sourcePoints[0][0]) &&
                floatEquals(original.sourcePoints[0][1], restored.sourcePoints[0][1]) &&
                floatEquals(original.sourcePoints[0][2], restored.sourcePoints[0][2]);
        }

        std::cout << "IcpChunk serialization: " << (match ? "PASS" : "FAIL")
            << " (buffer size: " << buffer.size() << " bytes)" << std::endl;
        allPassed &= match;
    }

    /// 4. IcpResult 직렬화 테스트
    {
        icp::IcpResult original;
        original.chunk_id = 123;
        original.transform = icp::Transform4x4::translation(5.0f, 6.0f, 7.0f);
        original.localTransform = icp::Transform4x4::translation(0.5f, 0.6f, 0.7f);
        original.finalRMSE = 0.0123f;
        original.initialRMSE = 0.5678f;
        original.actualIterations = 25;
        original.correspondenceCount = 1000;
        original.sourcePointCount = 5000;
        original.targetPointCount = 3000;
        original.converged = true;
        original.success = true;
        original.processingTimeMs = 123.456;
        original.errorMessage = "";  // 빈 메시지

        std::vector<uint8_t> buffer = icp::serializeIcpResult(original);
        icp::IcpResult restored = icp::deserializeIcpResult(buffer);

        bool match = (original.chunk_id == restored.chunk_id) &&
            floatEquals(original.finalRMSE, restored.finalRMSE) &&
            floatEquals(original.initialRMSE, restored.initialRMSE) &&
            (original.actualIterations == restored.actualIterations) &&
            (original.correspondenceCount == restored.correspondenceCount) &&
            (original.sourcePointCount == restored.sourcePointCount) &&
            (original.targetPointCount == restored.targetPointCount) &&
            (original.converged == restored.converged) &&
            (original.success == restored.success) &&
            (std::abs(original.processingTimeMs - restored.processingTimeMs) < 0.001);

        std::cout << "IcpResult serialization (success): " << (match ? "PASS" : "FAIL")
            << " (buffer size: " << buffer.size() << " bytes)" << std::endl;
        allPassed &= match;
    }

    /// 5. IcpResult with error message 직렬화 테스트
    {
        icp::IcpResult original;
        original.chunk_id = 999;
        original.success = false;
        original.converged = false;
        original.errorMessage = "Test error message: something went wrong!";

        std::vector<uint8_t> buffer = icp::serializeIcpResult(original);
        icp::IcpResult restored = icp::deserializeIcpResult(buffer);

        bool match = (original.chunk_id == restored.chunk_id) &&
            (original.success == restored.success) &&
            (original.converged == restored.converged) &&
            (original.errorMessage == restored.errorMessage);

        std::cout << "IcpResult serialization (error): " << (match ? "PASS" : "FAIL")
            << " (message: \"" << restored.errorMessage << "\")" << std::endl;
        allPassed &= match;
    }

    /// 6. NetworkUtils wrapper 테스트
    {
        icp::IcpChunk chunk;
        chunk.chunk_id = 777;
        chunk.sourcePoints = { {1.0f, 2.0f, 3.0f} };
        chunk.targetPoints = { {4.0f, 5.0f, 6.0f} };

        auto buffer = DPApp::NetworkUtils::serializeIcpChunk(chunk);
        auto restored = DPApp::NetworkUtils::deserializeIcpChunk(buffer);

        bool match = (chunk.chunk_id == restored.chunk_id);
        std::cout << "NetworkUtils wrapper: " << (match ? "PASS" : "FAIL") << std::endl;
        allPassed &= match;
    }

    std::cout << "\n=== Phase 3 Test Result: " << (allPassed ? "ALL PASSED" : "SOME FAILED")
        << " ===" << std::endl;
}

#endif

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
                auto retval = loadGltf(bim_folder, bimData);
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
    loadAgentsFromConfig("master_config.json");

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

/// [ToDo] Delete the follwing lines for clean-up
/// Aftet completing compile test, comment out the follwing lines
#ifdef _DEBUG
    /// Phase 1 test
    //testIcpTypesCompile();
    
    /// Phase 2 test
    //testIcpCoreCompile();
    //return 0;  // 테스트만 실행하고 종료

    /// Phase 3 test
    //testIcpSerializationCompile();
    //return 0;
 
#endif

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