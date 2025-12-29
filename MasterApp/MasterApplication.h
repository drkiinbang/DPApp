#pragma once

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <cstdint>
#include <atomic>
#include <mutex>
#include <vector>
#include <memory>
#include <map>

#include "../include/PointCloudTypes.h"
#include "../include/NetworkManager.h"
#include "../include/TaskManagerTypes.h"  // TaskManager class declaration
#include "../include/TestTask.h"          // Test Task processing
#include "RestApiServer.h"
#include "../include/RuntimeConfig.h"
#include "../include/Logger.h"
#include "../include/IcpTypes.h"

using namespace DPApp;

/// =========================================
/// Forward Declarations
/// =========================================

namespace DPApp {
    namespace PointCloudLoader {
        std::vector<std::shared_ptr<BimPcChunk>> loadBimPcChunks(
            const std::string& bim_folder,
            const std::string& pointcloud_file);
    }
}

/// =========================================
/// HTTP Client Response (for Agent communication)
/// =========================================

struct HttpClientResponse {
    bool success = false;
    int status_code = 0;
    std::string body;
};

/// =========================================
/// MasterApplication Class Declaration
/// =========================================

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

    /// =========================================
    /// Agent Management
    /// =========================================

    struct AgentInfo {
        std::string agent_id;
        std::string host;
        uint16_t port = 8092;
        bool online = false;
    };

    std::vector<AgentInfo> agents_;
    std::mutex agents_mutex_;
    std::string master_external_ip_ = "127.0.0.1";

    std::thread task_assignment_thread_;
    std::thread status_thread_;
    std::thread timeout_thread_;

    /// =========================================
    /// Test Task Management
    /// =========================================

    std::vector<TestChunk> test_chunks_;           // 테스트 청크 저장
    std::vector<TestResult> test_results_;         // 테스트 결과 저장
    std::mutex test_mutex_;                        // 테스트 데이터 동기화
    uint32_t next_test_chunk_id_ = 0;              // 전역 고유 청크 ID 카운터
    /// icp member variables
    std::map<std::string, std::shared_ptr<icp::IcpJob>> icp_jobs_;
    std::mutex icp_jobs_mutex_;
    uint32_t next_icp_task_id_ = 10000;

public:
    MasterApplication();
    ~MasterApplication();

    /// =========================================
    /// Core Functions (MasterApplication.cpp)
    /// =========================================

    bool isDaemonMode() const { return daemon_mode_; }
    bool initialize(int argc, char* argv[]);
    bool initializeTaskManager(TaskType task_type);
    bool start();
    void stop();
    void run();

    void runBimPcProcess(const std::string& mesh_folder, const std::string& pointcloud_file);
    void runDaemon();
    void runInteractive();
    bool loadAndProcessPointCloud(const std::string& filename, const TaskType task_type);

    /// Network callbacks
    void handleMessage(const NetworkMessage& message, const std::string& client_id);
    void handleConnection(const std::string& client_id, bool connected);
    void handleTaskResult(const NetworkMessage& message, const std::string& client_id);
    void handleTaskCompleted(const TaskInfo& task, const ProcessingResult& result);
    void handleTaskFailed(const TaskInfo& task, const std::string& error);

    /// Task management
    void createTasksFromChunks(const std::vector<std::shared_ptr<PointCloudChunk>>& chunks);
    void createTasksFromChunks(const std::vector<std::shared_ptr<BimPcChunk>>& chunks);
    void taskAssignmentLoop();
    void sendAssignedTasks();
    void statusLoop();
    void timeoutLoop();

    /// =========================================
    /// CLI Functions (MasterApplication_CLI.cpp)
    /// =========================================

    void parseCommandLine(int argc, char* argv[]);
    void printUsage();
    void printHelp();
    void processCommand(const std::string& command);

    void shutdownSlave(const std::string& slave_id);
    void shutdownAllSlaves();

    void printSystemStatus();
    void printSlaveStatus();
    void printAgentStatus();
    void printTaskStatus();
    void printProgress();
    void printResults();
    void saveResults(const std::string& filename);
    void clearCompletedTasks();

    /// =========================================
    /// Test Commands (MasterApplication_CLI.cpp)
    /// =========================================

    void runTestCommand(const std::string& args);
    void runTestEcho(uint32_t count, uint32_t base_time_ms = 10000);
    void runTestCompute(uint32_t count, uint32_t base_time_ms = 10000);
    void runTestDelay(uint32_t count, uint32_t extra_delay_ms, uint32_t base_time_ms = 10000);
    void runTestFail(uint32_t count, uint32_t fail_count, uint32_t base_time_ms = 10000);
    void printTestStatus();
    void printTestHelp();
    void handleTestResult(const TestResult& result);
    void createTestTasks(TaskType test_type, const std::vector<TestChunk>& chunks);

    /// =========================================
    /// REST API Functions (MasterApplication_REST.cpp)
    /// =========================================

    void setupRestApiRoutes();
        
    HttpResponse handleGetStatus(const HttpRequest& req);
    HttpResponse handleGetSlaves(const HttpRequest& req);
    HttpResponse handleGetSlave(const HttpRequest& req);
    HttpResponse handleGetTasks(const HttpRequest& req);
    HttpResponse handleGetTask(const HttpRequest& req);
    HttpResponse handleClearTasks(const HttpRequest& req);
    HttpResponse handleLoadFile(const HttpRequest& req);
    HttpResponse handleGetProgress(const HttpRequest& req);
    HttpResponse handleGetResults(const HttpRequest& req);
    HttpResponse handleSaveResults(const HttpRequest& req);
    HttpResponse handleShutdownSlave(const HttpRequest& req);
    HttpResponse handleShutdownAllSlaves(const HttpRequest& req);
    HttpResponse handleShutdownServer(const HttpRequest& req);
    HttpResponse createErrorResponse(int status_code, const std::string& error);
    std::string getCurrentTimestamp();

    /// icp
    void setupIcpApiRoutes();
    HttpResponse handleIcpStart(const HttpRequest& req);
    HttpResponse handleIcpJobs(const HttpRequest& req);
    HttpResponse handleIcpJobStatus(const HttpRequest& req);
    HttpResponse handleIcpJobResult(const HttpRequest& req);
    HttpResponse handleIcpJobCancel(const HttpRequest& req);
    HttpResponse handleIcpJobStats(const HttpRequest& req);
    void processIcpJob(std::shared_ptr<icp::IcpJob> job);
    std::string generateIcpJobId();
    void handleIcpResult(const icp::IcpResult& result, uint32_t task_id);

    /// =========================================
    /// Agent Functions (MasterApplication_Agent.cpp)
    /// =========================================

    /// HTTP Client
    HttpClientResponse httpPost(const std::string& host, uint16_t port,
        const std::string& path, const std::string& body_content);
    HttpClientResponse httpGet(const std::string& host, uint16_t port, const std::string& path);

    /// Agent Management
    bool loadAgentsFromConfig(const std::string& config_path);
    bool isAgentOnline(const std::string& host, uint16_t port);
    bool startSlaveOnAgent(const std::string& host, uint16_t port, uint32_t threads = 4);
    bool stopSlavesOnAgent(const std::string& host, uint16_t port, bool force = false);
    int startSlavesOnAllAgents(uint32_t threads = 4);
    int stopSlavesOnAllAgents(bool force = false);

    /// Agent REST Handlers
    HttpResponse handleGetAgents(const HttpRequest& req);
    HttpResponse handleAddAgent(const HttpRequest& req);
    HttpResponse handleRemoveAgent(const HttpRequest& req);
    HttpResponse handleTestAgent(const HttpRequest& req);
    HttpResponse handleStartSlavesOnAgents(const HttpRequest& req);
    HttpResponse handleStopSlavesOnAgents(const HttpRequest& req);

    /// Pause/Resume REST Handlers
    HttpResponse handlePauseSlave(const HttpRequest& req);
    HttpResponse handleResumeSlave(const HttpRequest& req);
    HttpResponse handlePauseAllSlaves(const HttpRequest& req);
    HttpResponse handleResumeAllSlaves(const HttpRequest& req);
};

/// =========================================
/// Global variable (for signal handler)
/// =========================================

extern MasterApplication* g_app;