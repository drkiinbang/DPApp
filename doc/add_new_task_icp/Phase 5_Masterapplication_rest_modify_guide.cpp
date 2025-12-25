/// ============================================================================
/// MasterApplication_REST.cpp 수정 가이드 - ICP API 라우트 연결
/// ============================================================================
/// 
/// 파일 위치: F:\repository\DPApp\MasterApp\MasterApplication_REST.cpp
/// 
/// 1곳만 수정합니다.
/// ============================================================================


/// ============================================================================
/// 수정: setupRestApiRoutes() 함수 끝에 ICP 라우트 설정 추가
/// ============================================================================
/// 
/// [위치] setupRestApiRoutes() 함수의 마지막 부분 (Line 125 근처)
/// 
/// [기존 코드]
void MasterApplication::setupRestApiRoutes() {
    api_server_ = std::make_unique<RestApiServer>();

    api_server_->GET("/api/status", [this](const HttpRequest& req) {
        return handleGetStatus(req);
    });

    // ... 다른 라우트들 ...

    api_server_->POST("/api/slaves/{id}/resume", [this](const HttpRequest& req) {
        return handleResumeSlave(req);
    });

    api_server_->POST("/api/slaves/resume", [this](const HttpRequest& req) {
        return handleResumeAllSlaves(req);
    });

    // <-- 여기에 추가!
}

/// [수정 후]
void MasterApplication::setupRestApiRoutes() {
    api_server_ = std::make_unique<RestApiServer>();

    // ... 기존 라우트들 ...

    api_server_->POST("/api/slaves/{id}/resume", [this](const HttpRequest& req) {
        return handleResumeSlave(req);
    });

    api_server_->POST("/api/slaves/resume", [this](const HttpRequest& req) {
        return handleResumeAllSlaves(req);
    });

    /// =========================================
    /// ICP API Routes
    /// =========================================
    setupIcpApiRoutes();
}


/// ============================================================================
/// 빌드 확인
/// ============================================================================
/// 
/// 1. MasterApplication.h 수정 완료
/// 2. MasterApplication_ICP.cpp 추가 완료
/// 3. MasterApplication_REST.cpp 수정 완료
/// 4. MasterApp.vcxproj에 MasterApplication_ICP.cpp 추가
/// 
/// 5. MasterApp 프로젝트 빌드
/// ============================================================================


/// ============================================================================
/// Visual Studio 프로젝트에 파일 추가 방법
/// ============================================================================
/// 
/// 1. 솔루션 탐색기에서 MasterApp 프로젝트 우클릭
/// 2. 추가 → 기존 항목
/// 3. MasterApplication_ICP.cpp 선택
/// 
/// 또는 .vcxproj 파일 직접 수정:
/// <ItemGroup> 섹션에 추가:
///   <ClCompile Include="MasterApplication_ICP.cpp" />
/// ============================================================================
