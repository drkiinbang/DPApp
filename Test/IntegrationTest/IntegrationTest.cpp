// IntegrationTest.cpp - Updated for latest codebase
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <memory>
#include <fstream>
#include <cassert>

#include "../../include/PointCloudTypes.h"
#include "../../include/NetworkManager.h"
#include "../../include/TaskManager.h"
#include "../../include/Logger.h"

using namespace DPApp;
using namespace std::chrono_literals;

int main_simpleTest() {
    std::cout << "=== Simple Connection Test ===" << std::endl;

    // Logger 초기화
    DPApp::Logger::initialize("integration_test.log");
    DPApp::Logger::setMinLevel(DPApp::LogLevel::INFO);

    auto server = std::make_unique<NetworkServer>();

    bool client_connected = false;
    server->setConnectionCallback([&](const std::string& client_id, bool connected) {
        std::cout << "Server callback: " << client_id << " " << (connected ? "connected" : "disconnected") << std::endl;
        if (connected) client_connected = true;
        });

    server->setMessageCallback([](const NetworkMessage& msg, const std::string& client_id) {
        std::cout << "Server received message type " << static_cast<int>(msg.header.type)
            << " from " << client_id << std::endl;
        });

    if (!server->start(9999)) {
        std::cerr << "Failed to start server" << std::endl;
        return 1;
    }

    std::cout << "Server started on port 9999" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    auto client = std::make_unique<NetworkClient>();

    client->setConnectionCallback([](const std::string& client_id, bool connected) {
        std::cout << "Client callback: " << client_id << " " << (connected ? "connected" : "disconnected") << std::endl;
        });

    std::cout << "Attempting client connection..." << std::endl;
    if (!client->connect("127.0.0.1", 9999)) {
        std::cerr << "Client connection failed" << std::endl;
        server->stop();
        return 1;
    }

    std::cout << "Connection established, maintaining for 10 seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout << "Disconnecting client..." << std::endl;
    client->disconnect();

    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "Stopping server..." << std::endl;
    server->stop();

    std::cout << "Test completed" << std::endl;
    return 0;
}

class IntegrationTestFramework {
public:
    struct TestResult {
        std::string test_name;
        bool passed;
        std::string error_message;
        double duration_ms;
    };

    static std::vector<TestResult> runAllTests();

private:
    static TestResult testBasicNetworkCommunication();
    static TestResult testMasterSlaveRegistration();
    static TestResult testTaskAssignmentAndExecution();
    static TestResult testMultipleWorkersScenario();
    static TestResult testWorkerDisconnectionRecovery();
    static TestResult testDataIntegrity();
    static TestResult testHeartbeatSystem();
    static TestResult testBIMComparisonTasks();

    static bool waitForCondition(std::function<bool()> condition, int timeout_seconds = 10);
    static void createTestPointCloudData(const std::string& filename);
    static void cleanupTestFiles();
};

std::vector<IntegrationTestFramework::TestResult> IntegrationTestFramework::runAllTests() {
    std::vector<TestResult> results;

    std::cout << "=== Starting Integration Tests ===" << std::endl;

    // Logger 초기화
    DPApp::Logger::initialize("integration_test.log");
    DPApp::Logger::setMinLevel(DPApp::LogLevel::INFO);

    createTestPointCloudData("test_data.xyz");

    std::vector<std::function<TestResult()>> tests = {
        testBasicNetworkCommunication,
        testMasterSlaveRegistration,
        testTaskAssignmentAndExecution,
        testMultipleWorkersScenario,
        testWorkerDisconnectionRecovery,
        testDataIntegrity,
        testHeartbeatSystem,
        testBIMComparisonTasks
    };

    for (auto& test : tests) {
        auto start = std::chrono::high_resolution_clock::now();
        TestResult result = test();
        auto end = std::chrono::high_resolution_clock::now();

        result.duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
        results.push_back(result);

        std::cout << "[" << (result.passed ? "PASS" : "FAIL") << "] "
            << result.test_name << " (" << result.duration_ms << "ms)";
        if (!result.passed) {
            std::cout << " - " << result.error_message;
        }
        std::cout << std::endl;

        std::this_thread::sleep_for(1s);
    }

    cleanupTestFiles();

    int passed = 0, failed = 0;
    for (const auto& result : results) {
        if (result.passed) passed++; else failed++;
    }

    std::cout << "\n=== Test Summary ===" << std::endl;
    std::cout << "Passed: " << passed << ", Failed: " << failed << std::endl;

    return results;
}

IntegrationTestFramework::TestResult IntegrationTestFramework::testBasicNetworkCommunication() {
    TestResult result{ "Basic Network Communication", false, "", 0 };

    try {
        auto server = std::make_unique<NetworkServer>();

        std::atomic<bool> server_received_connection{ false };
        std::atomic<bool> client_callback_called{ false };

        server->setConnectionCallback([&](const std::string& client_id, bool connected) {
            if (connected) {
                std::cout << "Server callback: client connected " << client_id << std::endl;
                server_received_connection = true;
            }
            });

        if (!server->start(8081)) {
            result.error_message = "Failed to start server";
            return result;
        }

        std::cout << "Server started, waiting..." << std::endl;
        std::this_thread::sleep_for(2s);

        auto client = std::make_unique<NetworkClient>();

        client->setConnectionCallback([&](const std::string& client_id, bool connected) {
            std::cout << "Client callback: " << client_id << " connected=" << connected << std::endl;
            if (connected) {
                client_callback_called = true;
            }
            });

        std::cout << "Connecting client..." << std::endl;
        if (!client->connect("127.0.0.1", 8081)) {
            result.error_message = "Failed to connect client";
            server->stop();
            return result;
        }

        std::cout << "Waiting for callbacks..." << std::endl;
        bool success = waitForCondition([&]() {
            return server_received_connection.load() && client_callback_called.load();
            }, 15);

        if (!success) {
            result.error_message = "Connection callbacks timeout";
            client->disconnect();
            server->stop();
            return result;
        }

        std::cout << "Connection established, maintaining..." << std::endl;
        std::this_thread::sleep_for(3s);

        std::cout << "Disconnecting..." << std::endl;
        client->disconnect();
        std::this_thread::sleep_for(1s);
        server->stop();

        result.passed = true;
        std::cout << "Basic network test completed successfully" << std::endl;
    }
    catch (const std::exception& e) {
        result.error_message = std::string("Exception: ") + e.what();
    }

    return result;
}

IntegrationTestFramework::TestResult IntegrationTestFramework::testMasterSlaveRegistration() {
    TestResult result{ "Master-Slave Registration", false, "", 0 };

    try {
        auto server = std::make_unique<NetworkServer>();
        auto task_manager = std::make_unique<TaskManager>(300); // timeout_seconds 매개변수 추가

        std::atomic<int> registered_slaves{ 0 };
        std::atomic<int> connected_clients{ 0 };

        server->setConnectionCallback([&](const std::string& client_id, bool connected) {
            if (connected) {
                connected_clients++;
                std::cout << "Server connection callback: " << client_id << " connected" << std::endl;
            }
            else {
                connected_clients--;
                std::cout << "Server connection callback: " << client_id << " disconnected" << std::endl;
            }
            });

        server->setMessageCallback([&](const NetworkMessage& msg, const std::string& client_id) {
            std::cout << "Received message type " << static_cast<int>(msg.header.type)
                << " from " << client_id << std::endl;
            if (msg.header.type == MessageType::SLAVE_REGISTER) {
                task_manager->registerSlave(client_id);
                registered_slaves++;
                std::cout << "Registered slave: " << client_id
                    << " (total: " << registered_slaves.load() << ")" << std::endl;
            }
            });

        if (!server->start(8082)) {
            result.error_message = "Failed to start server";
            return result;
        }

        std::cout << "Server started on 8082, waiting for stabilization..." << std::endl;
        std::this_thread::sleep_for(2s);

        std::vector<std::unique_ptr<NetworkClient>> clients;
        for (int i = 0; i < 3; ++i) {
            std::cout << "Creating client " << i << "..." << std::endl;
            auto client = std::make_unique<NetworkClient>();

            if (client->connect("127.0.0.1", 8082)) {
                clients.push_back(std::move(client));
                std::cout << "Client " << i << " connected" << std::endl;
                std::this_thread::sleep_for(1s);
            }
            else {
                std::cout << "Failed to connect client " << i << std::endl;
            }
        }

        std::cout << "All clients connected, waiting for registration..." << std::endl;

        bool registration_success = waitForCondition([&]() {
            std::cout << "Checking registration: registered=" << registered_slaves.load()
                << ", connected=" << connected_clients.load() << std::endl;
            return registered_slaves.load() >= 3;
            }, 30);

        if (!registration_success) {
            result.error_message = "Slave registration timeout - registered: " +
                std::to_string(registered_slaves.load()) +
                ", connected: " + std::to_string(connected_clients.load());
            server->stop();
            return result;
        }

        auto slaves = task_manager->getAllSlaves();
        std::cout << "TaskManager reports " << slaves.size() << " slaves" << std::endl;

        if (slaves.size() < 3) {
            result.error_message = "Expected at least 3 slaves, got " + std::to_string(slaves.size());
            server->stop();
            return result;
        }

        for (auto& client : clients) {
            client->disconnect();
        }

        std::this_thread::sleep_for(2s);
        server->stop();

        result.passed = true;
    }
    catch (const std::exception& e) {
        result.error_message = std::string("Exception: ") + e.what();
    }

    return result;
}

IntegrationTestFramework::TestResult IntegrationTestFramework::testTaskAssignmentAndExecution() {
    TestResult result{ "Task Assignment and Execution", false, "", 0 };

    try {
        auto server = std::make_unique<NetworkServer>();
        auto task_manager = std::make_unique<TaskManager>(300);

        // 테스트용 청크 데이터 생성
        auto chunk = std::make_shared<PointCloudChunk>();
        chunk->chunk_id = 1;
        for (int i = 0; i < 100; ++i) {
            chunk->points.emplace_back(i * 0.1, i * 0.1, i * 0.1);
        }

        // 올바른 TaskType enum 사용
        std::vector<uint8_t> params;
        uint32_t task_id = task_manager->addTask(TaskType::FILTER, chunk, params);

        task_manager->registerSlave("test_worker");

        bool assigned = task_manager->assignTasksToSlaves();
        if (!assigned) {
            result.error_message = "Failed to assign tasks";
            return result;
        }

        auto assigned_tasks = task_manager->getTasksByStatus(TaskStatus::ASSIGNED);
        if (assigned_tasks.empty()) {
            result.error_message = "No tasks were assigned";
            return result;
        }

        // 작업 완료 시뮬레이션
        ProcessingResult proc_result;
        proc_result.task_id = task_id;
        proc_result.chunk_id = 1;
        proc_result.success = true;
        proc_result.processed_points = chunk->points;

        bool completed = task_manager->completeTask(task_id, proc_result);
        if (!completed) {
            result.error_message = "Failed to complete task";
            return result;
        }

        auto completed_tasks = task_manager->getTasksByStatus(TaskStatus::COMPLETED);
        if (completed_tasks.empty()) {
            result.error_message = "Task was not marked as completed";
            return result;
        }

        result.passed = true;
    }
    catch (const std::exception& e) {
        result.error_message = std::string("Exception: ") + e.what();
    }

    return result;
}

IntegrationTestFramework::TestResult IntegrationTestFramework::testMultipleWorkersScenario() {
    TestResult result{ "Multiple Workers Scenario", false, "", 0 };

    try {
        auto server = std::make_unique<NetworkServer>();
        auto task_manager = std::make_unique<TaskManager>(300);

        if (!server->start(8083)) {
            result.error_message = "Failed to start server";
            return result;
        }

        // 여러 작업 생성 - 다양한 TaskType 사용
        std::vector<uint32_t> task_ids;
        std::vector<TaskType> task_types = {
            TaskType::FILTER
        };

        for (int i = 0; i < 5; ++i) {
            auto chunk = std::make_shared<PointCloudChunk>();
            chunk->chunk_id = i;
            for (int j = 0; j < 50; ++j) {
                chunk->points.emplace_back(j * 0.1, j * 0.1, j * 0.1);
            }

            std::vector<uint8_t> params;
            TaskType task_type = task_types[i % task_types.size()];
            uint32_t task_id = task_manager->addTask(task_type, chunk, params);
            task_ids.push_back(task_id);
        }

        // 여러 워커 등록
        for (int i = 0; i < 3; ++i) {
            task_manager->registerSlave("worker_" + std::to_string(i));
        }

        bool assigned = task_manager->assignTasksToSlaves();
        if (!assigned) {
            result.error_message = "Failed to assign tasks to multiple workers";
            server->stop();
            return result;
        }

        auto assigned_tasks = task_manager->getTasksByStatus(TaskStatus::ASSIGNED);
        if (assigned_tasks.size() < 3) {
            result.error_message = "Insufficient tasks assigned to workers";
            server->stop();
            return result;
        }

        server->stop();
        result.passed = true;
    }
    catch (const std::exception& e) {
        result.error_message = std::string("Exception: ") + e.what();
    }

    return result;
}

IntegrationTestFramework::TestResult IntegrationTestFramework::testWorkerDisconnectionRecovery() {
    TestResult result{ "Worker Disconnection Recovery", false, "", 0 };

    try {
        auto task_manager = std::make_unique<TaskManager>(300);

        task_manager->registerSlave("worker_1");

        auto chunk = std::make_shared<PointCloudChunk>();
        chunk->chunk_id = 1;
        std::vector<uint8_t> params;
        uint32_t task_id = task_manager->addTask(TaskType::FILTER, chunk, params);

        task_manager->assignTasksToSlaves();

        // 워커 연결 해제 시뮬레이션
        task_manager->unregisterSlave("worker_1");

        // 작업이 다시 대기 상태로 돌아가는지 확인
        auto pending_tasks = task_manager->getTasksByStatus(TaskStatus::PENDING);
        if (pending_tasks.empty()) {
            result.error_message = "Task was not returned to pending after worker disconnection";
            return result;
        }

        // 새 워커 등록 및 재할당
        task_manager->registerSlave("worker_2");
        bool reassigned = task_manager->assignTasksToSlaves();

        if (!reassigned) {
            result.error_message = "Failed to reassign task to new worker";
            return result;
        }

        result.passed = true;
    }
    catch (const std::exception& e) {
        result.error_message = std::string("Exception: ") + e.what();
    }

    return result;
}

IntegrationTestFramework::TestResult IntegrationTestFramework::testDataIntegrity() {
    TestResult result{ "Data Integrity", false, "", 0 };

    try {
        // 포인트클라우드 직렬화/역직렬화 테스트
        PointCloudChunk original_chunk;
        original_chunk.chunk_id = 42;
        original_chunk.min_x = -10.0; original_chunk.max_x = 10.0;
        original_chunk.min_y = -5.0;  original_chunk.max_y = 5.0;
        original_chunk.min_z = 0.0;   original_chunk.max_z = 15.0;

        for (int i = 0; i < 10; ++i) {
            Point3D point;
            point.x = i * 1.0;
            point.y = i * 0.5;
            point.z = i * 1.5;
            point.intensity = i * 10.0f;
            point.r = 255;
            point.g = 128;
            point.b = 64;
            point.classification = i % 10;
            original_chunk.points.push_back(point);
        }

        // 직렬화
        auto serialized = NetworkUtils::serializeChunk(original_chunk);

        // 역직렬화
        auto deserialized_chunk = NetworkUtils::deserializeChunk(serialized);

        // 데이터 비교
        if (deserialized_chunk.chunk_id != original_chunk.chunk_id) {
            result.error_message = "Chunk ID mismatch after serialization";
            return result;
        }

        if (deserialized_chunk.points.size() != original_chunk.points.size()) {
            result.error_message = "Point count mismatch after serialization";
            return result;
        }

        for (size_t i = 0; i < original_chunk.points.size(); ++i) {
            const auto& orig = original_chunk.points[i];
            const auto& deser = deserialized_chunk.points[i];

            if (std::abs(orig.x - deser.x) > 1e-6 ||
                std::abs(orig.y - deser.y) > 1e-6 ||
                std::abs(orig.z - deser.z) > 1e-6) {
                result.error_message = "Point coordinate mismatch at index " + std::to_string(i);
                return result;
            }

            if (orig.r != deser.r || orig.g != deser.g || orig.b != deser.b) {
                result.error_message = "Point color mismatch at index " + std::to_string(i);
                return result;
            }
        }

        // Task 직렬화/역직렬화 테스트
        ProcessingTask original_task;
        original_task.task_id = 123;
        original_task.chunk_id = 456;
        original_task.task_type = TaskType::BIM_DISTANCE_CALCULATION;
        original_task.parameters = { 1, 2, 3, 4, 5 };

        auto task_serialized = NetworkUtils::serializeTask(original_task);
        auto task_deserialized = NetworkUtils::deserializeTask(task_serialized);

        if (task_deserialized.task_id != original_task.task_id ||
            task_deserialized.chunk_id != original_task.chunk_id ||
            task_deserialized.task_type != original_task.task_type ||
            task_deserialized.parameters != original_task.parameters) {
            result.error_message = "Task serialization/deserialization failed";
            return result;
        }

        result.passed = true;
    }
    catch (const std::exception& e) {
        result.error_message = std::string("Exception: ") + e.what();
    }

    return result;
}

IntegrationTestFramework::TestResult IntegrationTestFramework::testHeartbeatSystem() {
    TestResult result{ "Heartbeat System", false, "", 0 };

    try {
        auto task_manager = std::make_unique<TaskManager>(300);

        task_manager->registerSlave("heartbeat_worker");
        task_manager->updateSlaveHeartbeat("heartbeat_worker");

        auto slaves = task_manager->getAllSlaves();
        if (slaves.empty()) {
            result.error_message = "No slaves found";
            return result;
        }

        bool found_worker = false;
        for (const auto& slave : slaves) {
            if (slave.slave_id == "heartbeat_worker" && slave.is_active) {
                found_worker = true;
                break;
            }
        }

        if (!found_worker) {
            result.error_message = "Worker not found or not active after heartbeat";
            return result;
        }

        // 타임아웃 테스트 (짧은 타임아웃으로 설정)
        auto short_timeout_manager = std::make_unique<TaskManager>(1); // 1초 타임아웃
        short_timeout_manager->registerSlave("timeout_test_worker");

        // 하트비트 없이 2초 대기
        std::this_thread::sleep_for(2s);

        // 타임아웃 체크 수행
        short_timeout_manager->checkTimeouts();

        auto slaves_after_timeout = short_timeout_manager->getAllSlaves();
        bool worker_still_active = false;
        for (const auto& slave : slaves_after_timeout) {
            if (slave.slave_id == "timeout_test_worker" && slave.is_active) {
                worker_still_active = true;
                break;
            }
        }

        // 타임아웃된 워커는 비활성화되어야 함
        if (worker_still_active) {
            result.error_message = "Worker should be inactive after timeout";
            return result;
        }

        result.passed = true;
    }
    catch (const std::exception& e) {
        result.error_message = std::string("Exception: ") + e.what();
    }

    return result;
}

IntegrationTestFramework::TestResult IntegrationTestFramework::testBIMComparisonTasks() {
    TestResult result{ "BIM Comparison Tasks", false, "", 0 };

    try {
        auto task_manager = std::make_unique<TaskManager>(300);

        // BIM 비교 작업을 위한 테스트 데이터 생성
        auto chunk = std::make_shared<PointCloudChunk>();
        chunk->chunk_id = 1;
        for (int i = 0; i < 50; ++i) {
            Point3D point;
            point.x = i * 0.1;
            point.y = i * 0.1;
            point.z = i * 0.1;
            chunk->points.push_back(point);
        }

        // BIM 비교 파라미터 생성
        std::string bim_folder = "./test_bim";
        std::string bim_pattern = "*.gltf";
        std::string pc_file = "./test_pc.las";
        float distance_threshold = 10.0f;
        bool enable_color_coding = true;

        std::vector<uint8_t> parameters;

        // BIM 폴더 경로
        uint32_t bim_path_len = static_cast<uint32_t>(bim_folder.length());
        parameters.insert(parameters.end(),
            reinterpret_cast<const uint8_t*>(&bim_path_len),
            reinterpret_cast<const uint8_t*>(&bim_path_len) + sizeof(uint32_t));
        parameters.insert(parameters.end(), bim_folder.begin(), bim_folder.end());

        // BIM 파일 패턴
        uint32_t bim_pattern_len = static_cast<uint32_t>(bim_pattern.length());
        parameters.insert(parameters.end(),
            reinterpret_cast<const uint8_t*>(&bim_pattern_len),
            reinterpret_cast<const uint8_t*>(&bim_pattern_len) + sizeof(uint32_t));
        parameters.insert(parameters.end(), bim_pattern.begin(), bim_pattern.end());

        // 원본 Point Cloud 파일 경로
        uint32_t pc_path_len = static_cast<uint32_t>(pc_file.length());
        parameters.insert(parameters.end(),
            reinterpret_cast<const uint8_t*>(&pc_path_len),
            reinterpret_cast<const uint8_t*>(&pc_path_len) + sizeof(uint32_t));
        parameters.insert(parameters.end(), pc_file.begin(), pc_file.end());

        // 거리 임계값과 색상 코딩 플래그
        parameters.insert(parameters.end(),
            reinterpret_cast<const uint8_t*>(&distance_threshold),
            reinterpret_cast<const uint8_t*>(&distance_threshold) + sizeof(float));

        uint8_t color_flag = enable_color_coding ? 1 : 0;
        parameters.push_back(color_flag);

        // BIM 비교 작업 생성
        uint32_t task_id = task_manager->addTask(TaskType::BIM_DISTANCE_CALCULATION, chunk, parameters);

        if (task_id == 0) {
            result.error_message = "Failed to create BIM comparison task";
            return result;
        }

        // 워커 등록 및 작업 할당
        task_manager->registerSlave("bim_worker");
        bool assigned = task_manager->assignTasksToSlaves();

        if (!assigned) {
            result.error_message = "Failed to assign BIM comparison task";
            return result;
        }

        auto assigned_tasks = task_manager->getTasksByStatus(TaskStatus::ASSIGNED);
        bool bim_task_found = false;
        for (const auto& task : assigned_tasks) {
            if (task.task_type == TaskType::BIM_DISTANCE_CALCULATION) {
                bim_task_found = true;
                break;
            }
        }

        if (!bim_task_found) {
            result.error_message = "BIM comparison task was not assigned";
            return result;
        }

        result.passed = true;
    }
    catch (const std::exception& e) {
        result.error_message = std::string("Exception: ") + e.what();
    }

    return result;
}

bool IntegrationTestFramework::waitForCondition(std::function<bool()> condition, int timeout_seconds) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(timeout_seconds)) {
        if (condition()) {
            return true;
        }
        std::this_thread::sleep_for(100ms);
    }
    return false;
}

void IntegrationTestFramework::createTestPointCloudData(const std::string& filename) {
    std::ofstream file(filename);
    file << "# Test Point Cloud Data\n";
    for (int i = 0; i < 1000; ++i) {
        file << (i * 0.1) << " " << (i * 0.05) << " " << (i * 0.02) << " "
            << (i % 256) << " " << ((i * 2) % 256) << " " << ((i * 3) % 256) << " "
            << ((i * 4) % 256) << "\n";
    }
    file.close();
}

void IntegrationTestFramework::cleanupTestFiles() {
    std::remove("test_data.xyz");
    DPApp::Logger::shutdown();
}

int main() {
    bool simpleTest = false;

    if (simpleTest) {
        return main_simpleTest();
    }

    std::cout << "DPApp Integration Test Suite" << std::endl;
    std::cout << "=============================" << std::endl;

    auto results = IntegrationTestFramework::runAllTests();

    for (const auto& result : results) {
        if (!result.passed) {
            return 1;
        }
    }

    std::cout << "\nAll tests passed!" << std::endl;
    return 0;
}