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
#include <unordered_map>

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
/// 전방 선언 (Forward Declarations)
/// =========================================

/// 주어진 IcpChunk::chunk_id가 어느 BIM 부재(element)에 해당하는지 식별한다.
/// 이를 통해 청크별 ICP 결과를 부재 단위로 이름 붙여 저장할 수 있다
/// (IcpTypes.h의 IcpJob::elementResults 참고).
struct IcpElementInfo {
    std::string name;
    int revitId = 0;
};

namespace DPApp {
    namespace PointCloudLoader {
        std::vector<std::shared_ptr<BimPcChunk>> loadBimPcChunks(
            const std::string& bim_folder,
            const std::string& pointcloud_file);

        /// coarseAlignedPoints는 Master가 들고 있는 대량 포인트 배열로, BIM의 절대좌표계
        /// 대비 (offsetX, offsetY, offsetZ)만큼 이동된 float로 저장되어 있다 (IcpTypes.h의
        /// icp::computeBimRebaseOffset / IcpJob::rebaseOffsetX 참고). 각 부재별로 만들어지는
        /// IcpChunk::sourcePoints도 동일한 float+이동 표현을 그대로 유지한다
        /// (chunk.offsetX/Y/Z에 그 offset이 기록되며, 사용하는 쪽에서 runIcp() 맨 앞에서
        /// 한 번만 승격+offset 복원을 수행한다). outElementInfo가 null이 아니면, 실제로
        /// 생성된 청크 하나당 항목 하나씩 채워진다(반환되는 벡터와 순서/인덱스가 동일하므로
        /// outElementInfo[i]가 chunks[i]를 설명함) -- 점이 너무 적거나 메시가 퇴화되어
        /// 건너뛴 부재는 청크도 만들어지지 않으므로 여기서도 함께 생략된다.
        std::vector<std::shared_ptr<icp::IcpChunk>> loadIcpElementChunks(
            const std::vector<chunkbim::MeshChunk>& elements,
            const std::vector<std::array<float, 3>>& coarseAlignedPoints,
            double offsetX, double offsetY, double offsetZ,
            const icp::IcpConfig& config,
            std::vector<IcpElementInfo>* outElementInfo = nullptr);
    }
}

/// =========================================
/// HTTP 클라이언트 응답 (Agent 통신용)
/// =========================================

struct HttpClientResponse {
    bool success = false;
    int status_code = 0;
    std::string body;
};

/// =========================================
/// MasterApplication 클래스 선언
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
    /// Agent 관리
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
    /// 테스트 태스크 관리
    /// =========================================

    std::vector<TestChunk> test_chunks_;           // 테스트 청크 목록
    std::vector<TestResult> test_results_;         // 테스트 결과 목록
    std::mutex test_mutex_;                        // 테스트 데이터 동기화
    uint32_t next_test_chunk_id_ = 0;              // 다음 테스트 청크 ID 카운터
    /// icp 관련 멤버 변수
    std::map<std::string, std::shared_ptr<icp::IcpJob>> icp_jobs_;
    std::mutex icp_jobs_mutex_;
    uint32_t next_icp_task_id_ = 10000;
    /// 네트워크로 분배된 ICP task_id를 그 task가 속한 job으로 역매핑한다.
    /// 이를 통해 handleIcpResult()가 올바른 IcpJob을 찾아 갱신할 수 있다.
    /// icp_jobs_mutex_로 보호된다. processIcpJob()이 fine alignment를 여러
    /// Slave에 분산시킬 때(BIM 부재당 청크 1개) 채워진다; CHANGELOG_2026-07-11.md 참고.
    std::unordered_map<uint32_t, std::string> icp_task_to_job_;
    /// task_manager_는 initializeTaskManager()가 현재 세션(bimpc / test / 분산 ICP)에
    /// 맞춰 매번 재생성하는 단일 공유 자원이다. 이 뮤텍스는 분산 ICP 작업의
    /// 분배+대기+통합 단계 전체 동안 계속 잡혀 있어서, 두 번째 ICP 작업이 동시에
    /// initializeTaskManager()를 호출해 첫 번째 작업이 쓰고 있는 TaskManager를
    /// 가로채가는 것을 막는다. 이 락을 즉시 얻지 못한 작업은 이 락을 기다리며
    /// 블로킹되는 대신 Master 단독 fine alignment로 대체(fallback) 처리한다.
    std::mutex icp_dispatch_mutex_;

public:
    MasterApplication();
    ~MasterApplication();

    /// =========================================
    /// 핵심 함수들 (MasterApplication.cpp)
    /// =========================================

    bool isDaemonMode() const { return daemon_mode_; }
    bool initialize(int argc, char* argv[]);
    bool initializeTaskManager(TaskType task_type);
    bool start();
    void stop();
    void run();

    void runBimPcProcess(const std::string& mesh_folder, const std::string& pointcloud_file);
    void runGenerateSyntheticPointCloud(
        const std::string& bim_folder, const std::string& output_las,
        int num_elements, double grid_size, double noise_sigma,
        double shift_x, double shift_y, double shift_z, double rot_deg);
    void runDaemon();
    void runInteractive();
    bool loadAndProcessPointCloud(const std::string& filename, const TaskType task_type);

    /// 네트워크 콜백
    void handleMessage(const NetworkMessage& message, const std::string& client_id);
    void handleConnection(const std::string& client_id, bool connected);
    void handleTaskResult(const NetworkMessage& message, const std::string& client_id);
    void handleTaskCompleted(const TaskInfo& task, const ProcessingResult& result);
    void handleTaskFailed(const TaskInfo& task, const std::string& error);

    /// 태스크 관리
    void createTasksFromChunks(const std::vector<std::shared_ptr<PointCloudChunk>>& chunks);
    void createTasksFromChunks(const std::vector<std::shared_ptr<BimPcChunk>>& chunks);
    void taskAssignmentLoop();
    void sendAssignedTasks();
    void statusLoop();
    void timeoutLoop();

    /// =========================================
    /// CLI 함수들 (MasterApplication_CLI.cpp)
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
    /// 테스트 명령어 (MasterApplication_CLI.cpp)
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
    /// REST API 함수들 (MasterApplication_REST.cpp)
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

    /// icp 관련
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
    /// Agent 함수들 (MasterApplication_Agent.cpp)
    /// =========================================

    /// HTTP 클라이언트
    HttpClientResponse httpPost(const std::string& host, uint16_t port,
        const std::string& path, const std::string& body_content);
    HttpClientResponse httpGet(const std::string& host, uint16_t port, const std::string& path);

    /// Agent 관리
    bool loadAgentsFromConfig(const std::string& config_path);
    bool isAgentOnline(const std::string& host, uint16_t port);
    bool startSlaveOnAgent(const std::string& host, uint16_t port, uint32_t threads = 4);
    bool stopSlavesOnAgent(const std::string& host, uint16_t port, bool force = false);
    int startSlavesOnAllAgents(uint32_t threads = 4);
    int stopSlavesOnAllAgents(bool force = false);

    /// Agent REST 핸들러
    HttpResponse handleGetAgents(const HttpRequest& req);
    HttpResponse handleAddAgent(const HttpRequest& req);
    HttpResponse handleRemoveAgent(const HttpRequest& req);
    HttpResponse handleTestAgent(const HttpRequest& req);
    HttpResponse handleStartSlavesOnAgents(const HttpRequest& req);
    HttpResponse handleStopSlavesOnAgents(const HttpRequest& req);

    /// 일시정지/재개 REST 핸들러
    HttpResponse handlePauseSlave(const HttpRequest& req);
    HttpResponse handleResumeSlave(const HttpRequest& req);
    HttpResponse handlePauseAllSlaves(const HttpRequest& req);
    HttpResponse handleResumeAllSlaves(const HttpRequest& req);
};

/// =========================================
/// 전역 변수 (시그널 핸들러용)
/// =========================================

extern MasterApplication* g_app;