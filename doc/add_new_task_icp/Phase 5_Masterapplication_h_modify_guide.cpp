/// ============================================================================
/// MasterApplication.h 수정 가이드 - ICP Task 지원 추가
/// ============================================================================
/// 
/// 파일 위치: F:\repository\DPApp\MasterApp\MasterApplication.h
/// 
/// 총 4곳을 수정합니다.
/// ============================================================================


/// ============================================================================
/// 수정 1: Include 추가 (Line 21 근처, <memory> 아래)
/// ============================================================================

#include <map>


/// ============================================================================
/// 수정 2: Include 추가 (Line 29 근처, Logger.h 아래)
/// ============================================================================

#include "../include/IcpTypes.h"


/// ============================================================================
/// 수정 3: ICP 멤버 변수 추가 (Line 101 근처, Test Task 멤버 아래)
/// ============================================================================
/// 
/// [위치] "uint32_t next_test_chunk_id_ = 0;" 아래에 추가:

    /// =========================================
    /// ICP Task Management
    /// =========================================

    std::map<std::string, std::shared_ptr<icp::IcpJob>> icp_jobs_;
    std::mutex icp_jobs_mutex_;
    uint32_t next_icp_task_id_ = 10000;  /// ICP 태스크 ID 시작값


/// ============================================================================
/// 수정 4: ICP REST API 함수 선언 추가 (Line 191 근처, REST API 섹션에)
/// ============================================================================
/// 
/// [위치] "HttpResponse handleShutdownServer(const HttpRequest& req);" 아래에 추가:

    /// =========================================
    /// ICP REST API Functions (MasterApplication_ICP.cpp)
    /// =========================================

    void setupIcpApiRoutes();

    /// ICP REST Handlers
    HttpResponse handleIcpStart(const HttpRequest& req);
    HttpResponse handleIcpJobs(const HttpRequest& req);
    HttpResponse handleIcpJobStatus(const HttpRequest& req);
    HttpResponse handleIcpJobResult(const HttpRequest& req);
    HttpResponse handleIcpJobCancel(const HttpRequest& req);
    HttpResponse handleIcpJobStats(const HttpRequest& req);

    /// ICP Job Processing (internal)
    void processIcpJob(std::shared_ptr<icp::IcpJob> job);
    std::string generateIcpJobId();

    /// ICP Result Handling
    void handleIcpResult(const icp::IcpResult& result, uint32_t task_id);


/// ============================================================================
/// 수정 완료 후 전체 구조 (참고용)
/// ============================================================================
/// 
/// MasterApplication.h 수정 후 구조:
/// 
/// [Private Members]
///   - cfg_, current_task_type_, etc.
///   - server_, task_manager_, api_server_
///   - AgentInfo, agents_, etc.
///   - test_chunks_, test_results_, test_mutex_
///   - icp_jobs_, icp_jobs_mutex_          <-- 추가
/// 
/// [Public Functions]
///   - Core Functions
///   - CLI Functions
///   - Test Commands
///   - REST API Functions
///     - handleGetStatus, handleGetSlaves, etc.
///     - handleShutdownServer
///     - setupIcpApiRoutes                 <-- 추가
///     - handleIcpStart, handleIcpJobs     <-- 추가
///     - handleIcpJobStatus, etc.          <-- 추가
///   - Agent Functions
/// 
/// ============================================================================
/// 빌드 확인
/// ============================================================================
/// 
/// 이 수정만으로는 빌드가 되지 않습니다.
/// Phase 5-2 (MasterApplication_ICP.cpp) 생성 후 빌드하세요.
/// ============================================================================
