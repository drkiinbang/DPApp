/// ============================================
/// LauncherService.cpp
/// Master 컴퓨터에서 실행되는 Agent 서비스
/// REST API를 통해 Master 프로세스의 생명주기를 관리
/// Windows 버전
/// ============================================

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <tlhelp32.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cstdint>
#include <csignal>

#include "Logger.h"
#include "MiniJson.h"
#include "RESTApiServer.h"

using namespace DPApp;

/// ============================================
/// 전역 종료 플래그 (사용 전에 반드시 선언되어야 함)
/// ============================================
static std::atomic<bool> g_shutdown_requested{ false };
static std::atomic<bool> g_shutdown_complete{ false };

/// ============================================
/// Master 프로세스 상태
/// ============================================

enum class MasterStatus : uint8_t {
    STOPPED = 0,
    STARTING = 1,
    RUNNING = 2,
    STOPPING = 3,
    FAILED = 4
};

/// MasterStatus를 문자열로 변환
inline const char* masterStatusStr(MasterStatus status) {
    switch (status) {
    case MasterStatus::STOPPED: return "stopped";
    case MasterStatus::STARTING: return "starting";
    case MasterStatus::RUNNING: return "running";
    case MasterStatus::STOPPING: return "stopping";
    case MasterStatus::FAILED: return "failed";
    default: return "unknown";
    }
}

/// ============================================
/// 블로킹된 accept()를 깨우기 위한 헬퍼 함수
/// ============================================
static void wakeUpServer(uint16_t port) {
    /// accept()를 깨우기 위해 더미 연결을 보냄
    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) return;

    /// 짧은 타임아웃 설정
    DWORD timeout = 100;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    /// 그냥 연결하고 닫기 - 이것으로 accept()가 깨어남
    connect(sock, (sockaddr*)&addr, sizeof(addr));
    closesocket(sock);
}

/// ============================================
/// LauncherService 클래스
/// ============================================

class LauncherService {
public:
    LauncherService() = default;

    ~LauncherService() {
        /// 소멸자에서는 이미 멈춰있다면 stop()을 다시 호출하지 않아야 함
        /// stop()은 main()에서 명시적으로 호출됨
    }

    /// 명령줄 인자로 런처 초기화
    bool initialize(int argc, char* argv[]) {
        /// 기본 설정
        launcher_port_ = 8090;
        ///[ToDo] exe 파일 이름을 설정 파일에 넣을 것
        std::string master_executable_ = "MasterApp.exe";
        master_server_port_ = 8080;
        master_api_port_ = 8081;
        config_file_ = "launcher_config.json";

        /// 명령줄 인자 파싱
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];

            if ((arg == "-p" || arg == "--port") && i + 1 < argc) {
                launcher_port_ = static_cast<uint16_t>(std::stoi(argv[++i]));
            }
            else if ((arg == "-m" || arg == "--master") && i + 1 < argc) {
                master_executable_ = argv[++i];
            }
            else if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
                config_file_ = argv[++i];
            }
            else if (arg == "-h" || arg == "--help") {
                printUsage(argv[0]);
                return false;
            }
        }

#ifdef _DEBUG
        master_executable_ = std::string("MasterAppd.exe");
#endif

        /// 설정 파일이 있으면 로딩
        loadConfig();

        /// 로거 초기화
        Logger::initialize("launcher.log");

        ILOG << "===========================================";
        ILOG << "DPApp Launcher Service Starting (Windows)";
        ILOG << "===========================================";
        ILOG << "Launcher port: " << launcher_port_;
        ILOG << "Master executable: " << master_executable_;

        return true;
    }

    /// 런처 서비스 시작
    bool start() {
        running_ = true;

        /// REST API 라우트 설정
        setupApiRoutes();

        /// API 서버 시작
        if (!api_server_->start(launcher_port_)) {
            ELOG << "Failed to start REST API server on port " << launcher_port_;
            return false;
        }

        ILOG << "Launcher REST API started on port " << launcher_port_;

        /// 모니터 스레드 시작
        monitor_thread_ = std::thread(&LauncherService::monitorLoop, this);

        return true;
    }

    /// 런처 서비스 중지
    void stop() {
        /// 재진입 방지
        bool expected = true;
        if (!running_.compare_exchange_strong(expected, false)) {
            return;
        }

        ILOG << "Stopping launcher service...";

        /// 실행 중이면 master 중지
        stopMaster(false);

        /// 중지하기 전에 서버의 accept()를 깨움
        wakeUpServer(launcher_port_);

        /// API 서버 중지
        if (api_server_) {
            api_server_->stop();
        }

        /// 타임아웃을 두고 모니터 스레드를 기다림
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }

        ILOG << "Launcher service stopped";
    }

    /// 메인 루프 실행 (블로킹)
    void run() {
        ILOG << "Launcher service running. Press Ctrl+C to exit.";

        while (running_ && !g_shutdown_requested.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        /// 루프 종료
        running_ = false;
    }

    /// 런처 포트 조회
    uint16_t getPort() const { return launcher_port_; }

private:
    /// 사용법 출력
    void printUsage(const char* program) {
        std::cout << "Usage: " << program << " [options]\n"
            << "Options:\n"
            << "  -p, --port <port>      Launcher API port (default: 8090)\n"
            << "  -m, --master <path>    Master executable path (default: MasterApp.exe)\n"
            << "  -c, --config <file>    Configuration file (default: launcher_config.json)\n"
            << "  -h, --help             Show this help message\n";
    }

    /// JSON 파일로부터 설정 로딩
    void loadConfig() {
        std::ifstream file(config_file_);
        if (!file.is_open()) {
            ILOG << "Config file not found, using defaults: " << config_file_;
            return;
        }

        std::stringstream buffer;
        buffer << file.rdbuf();

        auto parsed = MiniJson::parse_object(buffer.str());
        if (!parsed) {
            WLOG << "Invalid JSON in config file";
            return;
        }

        if (auto v = MiniJson::get<double>(*parsed, "launcher_port"))
            launcher_port_ = static_cast<uint16_t>(*v);
        if (auto v = MiniJson::get<std::string>(*parsed, "master_executable"))
            master_executable_ = *v;
        if (auto v = MiniJson::get<double>(*parsed, "master_server_port"))
            master_server_port_ = static_cast<uint16_t>(*v);
        if (auto v = MiniJson::get<double>(*parsed, "master_api_port"))
            master_api_port_ = static_cast<uint16_t>(*v);
        if (auto v = MiniJson::get<std::string>(*parsed, "master_config_file"))
            master_config_file_ = *v;

        ILOG << "Loaded configuration from " << config_file_;
    }

    /// REST API 라우트 설정
    void setupApiRoutes() {
        api_server_ = std::make_unique<RestApiServer>();

        /// 헬스 체크 엔드포인트
        api_server_->GET("/api/health", [this](const HttpRequest& req) {
            return handleHealth(req);
            });

        /// 런처와 master 상태 조회
        api_server_->GET("/api/status", [this](const HttpRequest& req) {
            return handleGetStatus(req);
            });

        /// Master 프로세스 시작
        api_server_->POST("/api/master/start", [this](const HttpRequest& req) {
            return handleStartMaster(req);
            });

        /// Master 프로세스 중지
        api_server_->POST("/api/master/stop", [this](const HttpRequest& req) {
            return handleStopMaster(req);
            });

        /// Master 프로세스 재시작
        api_server_->POST("/api/master/restart", [this](const HttpRequest& req) {
            return handleRestartMaster(req);
            });

        /// 런처 서비스 종료
        api_server_->POST("/api/shutdown", [this](const HttpRequest& req) {
            return handleShutdown(req);
            });
    }

    /// =========================================
    /// Master 프로세스 관리
    /// =========================================

    /// Master 프로세스 시작
    bool startMaster() {
        std::lock_guard<std::mutex> lock(master_mutex_);

        if (master_status_ == MasterStatus::RUNNING && isMasterRunning()) {
            WLOG << "Master is already running with PID: " << master_pid_;
            return true;
        }

        master_status_ = MasterStatus::STARTING;

        /// 명령줄 구성
        std::ostringstream cmd;
        cmd << master_executable_;
        cmd << " --daemon";
        cmd << " --port " << master_server_port_;
        cmd << " --api-port " << master_api_port_;

        if (!master_config_file_.empty()) {
            cmd << " --config " << master_config_file_;
        }

        std::string cmdLine = cmd.str();
        ILOG << "Starting master: " << cmdLine;

        /// Windows API를 이용해 프로세스 생성
        STARTUPINFOA si = { sizeof(si) };
        PROCESS_INFORMATION pi = {};

        /// CreateProcessA를 위해 수정 가능한 char 배열로 변환
        std::vector<char> cmdBuffer(cmdLine.begin(), cmdLine.end());
        cmdBuffer.push_back('\0');

        BOOL success = CreateProcessA(
            NULL,
            cmdBuffer.data(),
            NULL,
            NULL,
            FALSE,
            CREATE_NEW_CONSOLE,
            NULL,
            NULL,
            &si,
            &pi
        );

        if (!success) {
            DWORD error = GetLastError();
            ELOG << "Failed to create master process. Error code: " << error;
            master_status_ = MasterStatus::FAILED;
            return false;
        }

        master_pid_ = pi.dwProcessId;
        master_handle_ = pi.hProcess;
        CloseHandle(pi.hThread);

        master_start_time_ = std::chrono::steady_clock::now();

        ILOG << "Master process started with PID: " << master_pid_;

        /// 잠시 대기한 뒤 프로세스가 실행 중인지 확인
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (isMasterRunning()) {
            master_status_ = MasterStatus::RUNNING;
            ILOG << "Master process verified running";
            return true;
        }
        else {
            ELOG << "Master process terminated immediately after start";
            master_status_ = MasterStatus::FAILED;
            master_pid_ = 0;
            if (master_handle_ != NULL) {
                CloseHandle(master_handle_);
                master_handle_ = NULL;
            }
            return false;
        }
    }

    /// Master 프로세스 중지
    bool stopMaster(bool force = false) {
        std::lock_guard<std::mutex> lock(master_mutex_);

        if (master_pid_ == 0) {
            ILOG << "No master process to stop";
            master_status_ = MasterStatus::STOPPED;
            return true;
        }

        master_status_ = MasterStatus::STOPPING;
        ILOG << "Stopping master process (PID: " << master_pid_ << ", force: " << force << ")";

        bool stopped = false;

        if (!force) {
            /// REST API를 통한 정상 종료를 먼저 시도
            stopped = sendShutdownToMaster();

            if (stopped) {
                /// 프로세스가 종료될 때까지 대기
                if (master_handle_ != NULL) {
                    DWORD waitResult = WaitForSingleObject(master_handle_, 5000);
                    if (waitResult == WAIT_OBJECT_0) {
                        ILOG << "Master process exited gracefully";
                        stopped = true;
                    }
                    else {
                        WLOG << "Master did not exit gracefully within timeout";
                        stopped = false;
                    }
                }
            }
        }

        /// 정상 종료가 실패했거나 강제 종료가 요청되면 강제 종료
        if (!stopped || force) {
            if (master_handle_ != NULL) {
                ILOG << "Force terminating master process";
                TerminateProcess(master_handle_, 1);
                WaitForSingleObject(master_handle_, 3000);
            }
        }

        /// 정리
        if (master_handle_ != NULL) {
            CloseHandle(master_handle_);
            master_handle_ = NULL;
        }
        master_pid_ = 0;
        master_status_ = MasterStatus::STOPPED;

        ILOG << "Master process stopped";
        return true;
    }

    /// Master 프로세스가 아직 실행 중인지 확인
    bool isMasterRunning() {
        if (master_handle_ == NULL) return false;

        DWORD exitCode;
        if (GetExitCodeProcess(master_handle_, &exitCode)) {
            return (exitCode == STILL_ACTIVE);
        }
        return false;
    }

    /// REST API를 통해 master에 종료 명령 전송
    bool sendShutdownToMaster() {
        SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == INVALID_SOCKET) {
            WLOG << "Failed to create socket for master shutdown";
            return false;
        }

        /// 타임아웃 설정
        DWORD timeout = 3000;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));

        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(master_api_port_);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

        if (connect(sock, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
            WLOG << "Failed to connect to master API for shutdown";
            closesocket(sock);
            return false;
        }

        /// 종료 요청 전송
        std::string request =
            "POST /api/shutdown HTTP/1.1\r\n"
            "Host: 127.0.0.1\r\n"
            "Content-Length: 0\r\n"
            "Connection: close\r\n"
            "\r\n";

        send(sock, request.c_str(), static_cast<int>(request.length()), 0);

        /// 응답 수신 (간단히)
        char buffer[512];
        recv(sock, buffer, sizeof(buffer) - 1, 0);

        closesocket(sock);

        ILOG << "Shutdown command sent to master";
        return true;
    }

    /// master 상태를 확인하는 모니터 루프
    void monitorLoop() {
        ILOG << "Monitor thread started";

        while (running_ && !g_shutdown_requested.load()) {
            {
                std::lock_guard<std::mutex> lock(master_mutex_);

                if (master_status_ == MasterStatus::RUNNING) {
                    if (!isMasterRunning()) {
                        WLOG << "Master process terminated unexpectedly";
                        master_status_ = MasterStatus::STOPPED;
                        master_pid_ = 0;
                        if (master_handle_ != NULL) {
                            CloseHandle(master_handle_);
                            master_handle_ = NULL;
                        }
                    }
                }
            }

            /// 5초마다 확인 (단, 종료 플래그는 더 자주 확인)
            for (int i = 0; i < 50 && running_ && !g_shutdown_requested.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        ILOG << "Monitor thread exiting";
    }

    /// =========================================
    /// REST API 핸들러
    /// =========================================

    HttpResponse handleHealth(const HttpRequest& req) {
        HttpResponse response;
        response.status_code = 200;
        response.body = R"({"success": true, "status": "healthy", "service": "launcher"})";
        return response;
    }

    HttpResponse handleGetStatus(const HttpRequest& req) {
        std::lock_guard<std::mutex> lock(master_mutex_);

        bool master_running = (master_pid_ != 0) && isMasterRunning();

        std::ostringstream json;
        json << "{\n";
        json << "  \"success\": true,\n";
        json << "  \"data\": {\n";
        json << "    \"launcher\": {\n";
        json << "      \"port\": " << launcher_port_ << ",\n";
        json << "      \"status\": \"running\"\n";
        json << "    },\n";
        json << "    \"master\": {\n";
        json << "      \"pid\": " << master_pid_ << ",\n";
        json << "      \"status\": \"" << masterStatusStr(master_status_) << "\",\n";
        json << "      \"running\": " << (master_running ? "true" : "false") << ",\n";
        json << "      \"server_port\": " << master_server_port_ << ",\n";
        json << "      \"api_port\": " << master_api_port_ << "\n";
        json << "    }\n";
        json << "  }\n";
        json << "}";

        HttpResponse response;
        response.body = json.str();
        return response;
    }

    HttpResponse handleStartMaster(const HttpRequest& req) {
        auto parsed = MiniJson::parse_object(req.body);
        if (parsed) {
            if (auto v = MiniJson::get<double>(*parsed, "server_port"))
                master_server_port_ = static_cast<uint16_t>(*v);
            if (auto v = MiniJson::get<double>(*parsed, "api_port"))
                master_api_port_ = static_cast<uint16_t>(*v);
        }

        if (startMaster()) {
            std::ostringstream json;
            json << "{\n";
            json << "  \"success\": true,\n";
            json << "  \"message\": \"Master started\",\n";
            json << "  \"data\": {\n";
            json << "    \"pid\": " << master_pid_ << ",\n";
            json << "    \"server_port\": " << master_server_port_ << ",\n";
            json << "    \"api_port\": " << master_api_port_ << "\n";
            json << "  }\n";
            json << "}";

            HttpResponse response;
            response.body = json.str();
            return response;
        }

        return createErrorResponse(500, "Failed to start master");
    }

    HttpResponse handleStopMaster(const HttpRequest& req) {
        bool force = false;

        auto parsed = MiniJson::parse_object(req.body);
        if (parsed) {
            if (auto v = MiniJson::get<bool>(*parsed, "force"))
                force = *v;
        }

        if (stopMaster(force)) {
            HttpResponse response;
            response.body = R"({"success": true, "message": "Master stopped"})";
            return response;
        }

        return createErrorResponse(500, "Failed to stop master");
    }

    HttpResponse handleRestartMaster(const HttpRequest& req) {
        bool force = false;

        auto parsed = MiniJson::parse_object(req.body);
        if (parsed) {
            if (auto v = MiniJson::get<bool>(*parsed, "force"))
                force = *v;
        }

        stopMaster(force);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (startMaster()) {
            std::ostringstream json;
            json << "{\n";
            json << "  \"success\": true,\n";
            json << "  \"message\": \"Master restarted\",\n";
            json << "  \"data\": { \"pid\": " << master_pid_ << " }\n";
            json << "}";

            HttpResponse response;
            response.body = json.str();
            return response;
        }

        return createErrorResponse(500, "Failed to restart master");
    }

    HttpResponse handleShutdown(const HttpRequest& req) {
        ILOG << "Shutdown requested via API";
        g_shutdown_requested = true;

        HttpResponse response;
        response.body = R"({"success": true, "message": "Launcher shutdown initiated"})";
        return response;
    }

    HttpResponse createErrorResponse(int code, const std::string& message) {
        HttpResponse response;
        response.status_code = code;

        switch (code) {
        case 400: response.status_text = "Bad Request"; break;
        case 404: response.status_text = "Not Found"; break;
        case 500: response.status_text = "Internal Server Error"; break;
        default: response.status_text = "Error"; break;
        }

        std::ostringstream json;
        json << "{\n";
        json << "  \"success\": false,\n";
        json << "  \"error\": \"" << message << "\"\n";
        json << "}";

        response.body = json.str();
        return response;
    }

private:
    /// 설정
    uint16_t launcher_port_ = 8090;
    std::string master_executable_ = "MasterApp.exe";
    uint16_t master_server_port_ = 8080;
    uint16_t master_api_port_ = 8081;
    std::string master_config_file_;
    std::string config_file_ = "launcher_config.json";

    /// 런타임 상태
    std::atomic<bool> running_{ false };
    std::unique_ptr<RestApiServer> api_server_;
    std::thread monitor_thread_;

    /// Master 프로세스 상태
    std::mutex master_mutex_;
    DWORD master_pid_ = 0;
    HANDLE master_handle_ = NULL;
    MasterStatus master_status_ = MasterStatus::STOPPED;
    std::chrono::steady_clock::time_point master_start_time_;
};

/// ============================================
/// 콘솔 제어 처리를 위한 전역 인스턴스
/// ============================================

static LauncherService* g_launcher = nullptr;

/// Ctrl+C를 위한 Windows 콘솔 제어 핸들러
static BOOL WINAPI ConsoleHandler(DWORD signal) {
    if (signal == CTRL_C_EVENT || signal == CTRL_BREAK_EVENT) {
        /// 종료 플래그만 설정 - 여기서 복잡한 작업을 수행하지 말 것
        g_shutdown_requested = true;

        /// accept()를 풀어주기 위해 서버를 깨움
        if (g_launcher) {
            wakeUpServer(g_launcher->getPort());
        }

        /// 정상 종료가 완료될 때까지 대기 (최대 5초)
        for (int i = 0; i < 50 && !g_shutdown_complete.load(); ++i) {
            Sleep(100);
        }

        return TRUE;
    }
    return FALSE;
}

/// ============================================
/// 메인 진입점
/// ============================================

int main(int argc, char* argv[]) {
    /// 디버그 모드에서 abort 메시지 박스 비활성화
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);

    LauncherService launcher;
    g_launcher = &launcher;

    /// Windows 콘솔 제어 핸들러 설정
    SetConsoleCtrlHandler(ConsoleHandler, TRUE);

    /// 런처 초기화
    if (!launcher.initialize(argc, argv)) {
        return 1;
    }

    /// 런처 서비스 시작
    if (!launcher.start()) {
        return 1;
    }

    /// 메인 루프 실행 (Ctrl+C 또는 종료 요청까지 블로킹)
    launcher.run();

    /// 정상 종료
    ILOG << "Initiating graceful shutdown...";
    launcher.stop();

    ILOG << "Launcher terminated successfully.";
    Logger::shutdown();

    /// 종료가 완료되었음을 알림
    g_shutdown_complete = true;

    return 0;
}
