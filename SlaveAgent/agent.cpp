/// ============================================
/// SlaveAgent.cpp
/// Worker 컴퓨터에서 실행되는 Agent 서비스
/// REST API를 통해 Slave 프로세스의 생명주기를 관리
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
/// Slave 프로세스 정보
/// ============================================

struct SlaveProcessInfo {
    DWORD pid = 0;
    HANDLE handle = NULL;
    std::string master_address;
    uint16_t master_port = 0;
    std::chrono::steady_clock::time_point start_time;
    bool running = false;

    SlaveProcessInfo() = default;

    SlaveProcessInfo(DWORD p, HANDLE h, const std::string& addr, uint16_t port)
        : pid(p), handle(h), master_address(addr), master_port(port),
        start_time(std::chrono::steady_clock::now()), running(true) {
    }
};

/// ============================================
/// SlaveAgent 클래스
/// ============================================

class SlaveAgent {
public:
    SlaveAgent() = default;

    ~SlaveAgent() {
        /// 소멸자에서는 이미 멈춰있다면 stop()을 다시 호출하지 않아야 함
        /// stop()은 main()에서 명시적으로 호출됨
    }

    /// 명령줄 인자로 agent 초기화
    bool initialize(int argc, char* argv[]) {
        /// 기본 설정
        agent_port_ = 8092;
        slave_executable_ = "SlaveApp.exe";
        max_slaves_ = 4;
        config_file_ = "agent_config.json";

        /// 명령줄 인자 파싱
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];

            if ((arg == "-p" || arg == "--port") && i + 1 < argc) {
                agent_port_ = static_cast<uint16_t>(std::stoi(argv[++i]));
            }
            else if ((arg == "-s" || arg == "--slave") && i + 1 < argc) {
                slave_executable_ = argv[++i];
            }
            else if ((arg == "-m" || arg == "--max-slaves") && i + 1 < argc) {
                max_slaves_ = static_cast<uint32_t>(std::stoul(argv[++i]));
            }
            else if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
                config_file_ = argv[++i];
            }
            else if (arg == "-h" || arg == "--help") {
                printUsage(argv[0]);
                return false;
            }
        }

        /// 설정 파일이 있으면 로딩
        loadConfig();

        /// 로거 초기화
        Logger::initialize("slave_agent.log");

        ILOG << "===========================================";
        ILOG << "DPApp Slave Agent Starting (Windows)";
        ILOG << "===========================================";
        ILOG << "Agent port: " << agent_port_;
        ILOG << "Slave executable: " << slave_executable_;
        ILOG << "Max slaves: " << max_slaves_;

        return true;
    }

    /// agent 서비스 시작
    bool start() {
        running_ = true;

        /// REST API 라우트 설정
        setupApiRoutes();

        /// API 서버 시작
        if (!api_server_->start(agent_port_)) {
            ELOG << "Failed to start REST API server on port " << agent_port_;
            return false;
        }

        ILOG << "SlaveAgent REST API started on port " << agent_port_;

        /// 모니터 스레드 시작
        monitor_thread_ = std::thread(&SlaveAgent::monitorLoop, this);

        return true;
    }

    /// agent 서비스 중지
    void stop() {
        /// 재진입 방지
        bool expected = true;
        if (!running_.compare_exchange_strong(expected, false)) {
            return;
        }

        ILOG << "Stopping SlaveAgent...";

        /// 모든 slave 중지
        stopAllSlaves(true);

        /// 중지하기 전에 서버의 accept()를 깨움
        wakeUpServer(agent_port_);

        /// API 서버 중지
        if (api_server_) {
            api_server_->stop();
        }

        /// 모니터 스레드 대기
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }

        ILOG << "SlaveAgent stopped";
    }

    /// 메인 루프 실행 (블로킹)
    void run() {
        ILOG << "SlaveAgent running. Press Ctrl+C to exit.";

        while (running_ && !g_shutdown_requested.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        /// 루프 종료
        running_ = false;
    }

    /// agent 포트 조회
    uint16_t getPort() const { return agent_port_; }

private:
    /// 사용법 출력
    void printUsage(const char* program) {
        std::cout << "Usage: " << program << " [options]\n"
            << "Options:\n"
            << "  -p, --port <port>        Agent API port (default: 8092)\n"
            << "  -s, --slave <path>       Slave executable path (default: SlaveApp.exe)\n"
            << "  -m, --max-slaves <num>   Maximum concurrent slaves (default: 8)\n"
            << "  -c, --config <file>      Configuration file (default: agent_config.json)\n"
            << "  -h, --help               Show this help message\n";
    }

    /// 파일로부터 설정 로딩
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

        if (auto v = MiniJson::get<double>(*parsed, "agent_port"))
            agent_port_ = static_cast<uint16_t>(*v);
        if (auto v = MiniJson::get<std::string>(*parsed, "slave_executable"))
            slave_executable_ = *v;
        if (auto v = MiniJson::get<double>(*parsed, "max_slaves"))
            max_slaves_ = static_cast<uint32_t>(*v);
        if (auto v = MiniJson::get<std::string>(*parsed, "working_directory"))
            working_directory_ = *v;

        ILOG << "Loaded configuration from " << config_file_;
    }

    /// REST API 라우트 설정
    void setupApiRoutes() {
        api_server_ = std::make_unique<RestApiServer>();

        /// 헬스 체크
        api_server_->GET("/api/health", [this](const HttpRequest& req) {
            return handleHealth(req);
            });

        /// agent와 slave 상태 조회
        api_server_->GET("/api/status", [this](const HttpRequest& req) {
            return handleGetStatus(req);
            });

        /// slave 목록 조회
        api_server_->GET("/api/slaves", [this](const HttpRequest& req) {
            return handleGetSlaves(req);
            });

        /// 새 slave 시작
        api_server_->POST("/api/slaves/start", [this](const HttpRequest& req) {
            return handleStartSlave(req);
            });

        /// PID로 특정 slave 중지
        api_server_->POST("/api/slaves/stop", [this](const HttpRequest& req) {
            return handleStopSlave(req);
            });

        /// 모든 slave 중지
        api_server_->POST("/api/slaves/stop-all", [this](const HttpRequest& req) {
            return handleStopAllSlaves(req);
            });

        /// agent 종료
        api_server_->POST("/api/shutdown", [this](const HttpRequest& req) {
            return handleShutdown(req);
            });
    }

    /// =========================================
    /// Slave 프로세스 관리
    /// =========================================

    /// 새 slave 프로세스 시작
    bool startSlave(const std::string& master_address,
        uint16_t master_port,
        SlaveProcessInfo& out_info,
        uint32_t threads = 1) {
        std::lock_guard<std::mutex> lock(slaves_mutex_);

        /// 최대 slave 개수 제한 확인
        size_t running_count = countRunningSlaves();
        if (running_count >= max_slaves_) {
            WLOG << "Max slaves limit reached (" << max_slaves_ << ")";
            return false;
        }

        /// 명령줄 구성
        /// [수정 이력] 예전에는 `threads`가 /api/slaves/start에서 받아져 여기까지
        /// 전달되기는 했지만, 그 뒤 조용히 버려졌다 -- SlaveApplication.cpp에 -t 플래그가
        /// 아예 없어서 항상 하드코딩된 스레드 1개로 실행되었기 때문에 SlaveApp.exe는
        /// 스레드 개수를 전혀 받지 못했다.
        std::ostringstream cmd;
        cmd << slave_executable_;
        cmd << " -s " << master_address;
        cmd << " -p " << master_port;
        if (threads > 1) {
            cmd << " -t " << threads;
        }

        std::string command = cmd.str();
        ILOG << "Starting slave: " << command;

        /// 시작 정보 설정
        STARTUPINFOA si;
        ZeroMemory(&si, sizeof(si));
        si.cb = sizeof(si);
        si.dwFlags = STARTF_USESHOWWINDOW;
#ifdef _DEBUG
        si.wShowWindow = SW_SHOW;
#else
        //si.wShowWindow = SW_HIDE;
        si.wShowWindow = SW_SHOW;
#endif

        PROCESS_INFORMATION pi;
        ZeroMemory(&pi, sizeof(pi));

        /// 지정되어 있으면 작업 디렉터리 설정
        const char* work_dir = working_directory_.empty() ? NULL : working_directory_.c_str();

        /// 프로세스 생성
        if (!CreateProcessA(
            NULL,
            const_cast<char*>(command.c_str()),
            NULL,
            NULL,
            FALSE,
            CREATE_NEW_CONSOLE,
            NULL,
            work_dir,
            &si,
            &pi)) {
            DWORD error = GetLastError();
            ELOG << "CreateProcess failed: " << error;
            return false;
        }

        /// 프로세스 정보 저장
        SlaveProcessInfo info(pi.dwProcessId, pi.hProcess, master_address, master_port);
        slaves_.push_back(info);

        CloseHandle(pi.hThread);

        out_info = info;
        ILOG << "Slave started (PID: " << pi.dwProcessId << ") -> "
            << master_address << ":" << master_port;

        return true;
    }

    /// PID로 slave 중지
    bool stopSlave(DWORD pid, bool force) {
        std::lock_guard<std::mutex> lock(slaves_mutex_);

        auto it = std::find_if(slaves_.begin(), slaves_.end(),
            [pid](const SlaveProcessInfo& s) { return s.pid == pid; });

        if (it == slaves_.end()) {
            WLOG << "Slave not found: PID " << pid;
            return false;
        }

        ILOG << "Stopping slave (PID: " << pid << ", force: " << force << ")";

        bool result = false;
        if (it->handle != NULL) {
            if (force) {
                result = TerminateProcess(it->handle, 1) != 0;
            }
            else {
                /// 정상 종료를 먼저 시도
                /// Ctrl+C를 보내거나 다른 방법을 사용
                /// 단순화를 위해 그냥 종료
                result = TerminateProcess(it->handle, 0) != 0;
            }

            /// 프로세스가 종료될 때까지 잠시 대기
            WaitForSingleObject(it->handle, 3000);
            CloseHandle(it->handle);
        }

        slaves_.erase(it);
        ILOG << "Slave stopped (PID: " << pid << ")";

        return true;
    }

    /// 모든 slave 중지
    bool stopAllSlaves(bool force) {
        std::vector<DWORD> pids;

        {
            std::lock_guard<std::mutex> lock(slaves_mutex_);
            for (const auto& slave : slaves_) {
                pids.push_back(slave.pid);
            }
        }

        bool all_ok = true;
        for (DWORD pid : pids) {
            if (!stopSlave(pid, force)) {
                all_ok = false;
            }
        }

        return all_ok;
    }

    /// 실행 중인 slave 개수 계산
    size_t countRunningSlaves() {
        /// 뮤텍스가 이미 잡혀 있다고 가정함
        size_t count = 0;
        for (const auto& slave : slaves_) {
            if (slave.running) {
                count++;
            }
        }
        return count;
    }

    /// 프로세스가 아직 실행 중인지 확인
    bool isProcessRunning(HANDLE handle) {
        if (handle == NULL) return false;

        DWORD exitCode;
        if (GetExitCodeProcess(handle, &exitCode)) {
            return exitCode == STILL_ACTIVE;
        }
        return false;
    }

    /// =========================================
    /// 모니터 루프
    /// =========================================

    void monitorLoop() {
        ILOG << "Monitor thread started";

        while (running_ && !g_shutdown_requested.load()) {
            /// 주기적으로 slave 상태 확인
            {
                std::lock_guard<std::mutex> lock(slaves_mutex_);

                for (auto it = slaves_.begin(); it != slaves_.end(); ) {
                    bool running = isProcessRunning(it->handle);

                    if (!running && it->running) {
                        WLOG << "Slave terminated (PID: " << it->pid << ")";

                        if (it->handle != NULL) {
                            CloseHandle(it->handle);
                        }
                        it = slaves_.erase(it);
                    }
                    else {
                        it->running = running;
                        ++it;
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
        response.body = R"({"success": true, "status": "healthy", "service": "slave_agent"})";
        return response;
    }

    HttpResponse handleGetStatus(const HttpRequest& req) {
        std::lock_guard<std::mutex> lock(slaves_mutex_);

        size_t running_count = 0;
        for (const auto& slave : slaves_) {
            if (slave.running) running_count++;
        }

        std::ostringstream json;
        json << "{\n";
        json << "  \"success\": true,\n";
        json << "  \"data\": {\n";
        json << "    \"agent\": {\n";
        json << "      \"port\": " << agent_port_ << ",\n";
        json << "      \"status\": \"running\",\n";
        json << "      \"max_slaves\": " << max_slaves_ << "\n";
        json << "    },\n";
        json << "    \"slaves\": {\n";
        json << "      \"total\": " << slaves_.size() << ",\n";
        json << "      \"running\": " << running_count << ",\n";
        json << "      \"available\": " << (max_slaves_ - running_count) << "\n";
        json << "    }\n";
        json << "  }\n";
        json << "}";

        HttpResponse response;
        response.body = json.str();
        return response;
    }

    HttpResponse handleGetSlaves(const HttpRequest& req) {
        std::lock_guard<std::mutex> lock(slaves_mutex_);

        std::ostringstream json;
        json << "{\n";
        json << "  \"success\": true,\n";
        json << "  \"data\": {\n";
        json << "    \"count\": " << slaves_.size() << ",\n";
        json << "    \"slaves\": [\n";

        for (size_t i = 0; i < slaves_.size(); ++i) {
            const auto& s = slaves_[i];

            auto uptime = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - s.start_time).count();

            json << "      {\n";
            json << "        \"pid\": " << s.pid << ",\n";
            json << "        \"master_address\": \"" << s.master_address << "\",\n";
            json << "        \"master_port\": " << s.master_port << ",\n";
            json << "        \"running\": " << (s.running ? "true" : "false") << ",\n";
            json << "        \"uptime_seconds\": " << uptime << "\n";
            json << "      }" << (i + 1 < slaves_.size() ? "," : "") << "\n";
        }

        json << "    ]\n";
        json << "  }\n";
        json << "}";

        HttpResponse response;
        response.body = json.str();
        return response;
    }

    HttpResponse handleStartSlave(const HttpRequest& req) {
        /// 요청 본문 파싱
        auto parsed = MiniJson::parse_object(req.body);
        if (!parsed) {
            return createErrorResponse(400, "Invalid JSON body");
        }

        /// 필수 매개변수 조회
        auto master_address = MiniJson::get<std::string>(*parsed, "master_address");
        if (!master_address || master_address->empty()) {
            return createErrorResponse(400, "Missing 'master_address'");
        }

        /// 선택적 매개변수와 기본값
        uint16_t master_port = 8080;

        if (auto v = MiniJson::get<double>(*parsed, "master_port")) {
            master_port = static_cast<uint16_t>(*v);
        }

        uint32_t threads = 1;
        if (auto v = MiniJson::get<double>(*parsed, "threads")) {
            if (*v > 0) {
                threads = static_cast<uint32_t>(*v);
            }
        }

        /// slave 시작
        SlaveProcessInfo info;
        if (startSlave(*master_address, master_port, info, threads)) {
            std::ostringstream json;
            json << "{\n";
            json << "  \"success\": true,\n";
            json << "  \"message\": \"Slave started\",\n";
            json << "  \"data\": {\n";
            json << "    \"pid\": " << info.pid << ",\n";
            json << "    \"master_address\": \"" << info.master_address << "\",\n";
            json << "    \"master_port\": " << info.master_port << ",\n";
            json << "  }\n";
            json << "}";

            HttpResponse response;
            response.body = json.str();
            return response;
        }

        return createErrorResponse(500, "Failed to start slave (max limit reached or process error)");
    }

    HttpResponse handleStopSlave(const HttpRequest& req) {
        /// 요청 본문 파싱
        auto parsed = MiniJson::parse_object(req.body);
        if (!parsed) {
            return createErrorResponse(400, "Invalid JSON body");
        }

        /// PID 조회
        auto pid_opt = MiniJson::get<double>(*parsed, "pid");
        if (!pid_opt) {
            return createErrorResponse(400, "Missing 'pid'");
        }

        DWORD pid = static_cast<DWORD>(*pid_opt);

        /// force 플래그 조회
        bool force = false;
        if (auto v = MiniJson::get<bool>(*parsed, "force")) {
            force = *v;
        }

        if (stopSlave(pid, force)) {
            HttpResponse response;
            response.body = R"({"success": true, "message": "Slave stopped"})";
            return response;
        }

        return createErrorResponse(404, "Slave not found");
    }

    HttpResponse handleStopAllSlaves(const HttpRequest& req) {
        /// force 플래그 조회
        bool force = false;
        auto parsed = MiniJson::parse_object(req.body);
        if (parsed) {
            if (auto v = MiniJson::get<bool>(*parsed, "force")) {
                force = *v;
            }
        }

        size_t count = 0;
        {
            std::lock_guard<std::mutex> lock(slaves_mutex_);
            count = slaves_.size();
        }

        stopAllSlaves(force);

        std::ostringstream json;
        json << "{\n";
        json << "  \"success\": true,\n";
        json << "  \"message\": \"All slaves stopped\",\n";
        json << "  \"data\": { \"stopped_count\": " << count << " }\n";
        json << "}";

        HttpResponse response;
        response.body = json.str();
        return response;
    }

    HttpResponse handleShutdown(const HttpRequest& req) {
        ILOG << "Shutdown requested via API";

        /// detach된 스레드를 쓰는 대신 종료 플래그를 설정
        g_shutdown_requested = true;

        HttpResponse response;
        response.body = R"({"success": true, "message": "Agent shutdown initiated"})";
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
    uint16_t agent_port_ = 8092;
    ///[ToDo] exe 파일 이름을 설정 파일에 넣을 것
#ifdef _DEBUG
    std::string slave_executable_ = "SlaveAppd.exe";
#else
    std::string slave_executable_ = "SlaveApp.exe";
#endif
    uint32_t max_slaves_ = 4;
    std::string working_directory_;
    std::string config_file_ = "agent_config.json";

    /// 상태
    std::atomic<bool> running_{ false };
    std::unique_ptr<RestApiServer> api_server_;
    std::thread monitor_thread_;

    /// Slave 프로세스들
    std::mutex slaves_mutex_;
    std::vector<SlaveProcessInfo> slaves_;
};

/// ============================================
/// 시그널 처리를 위한 전역 인스턴스
/// ============================================

static SlaveAgent* g_agent = nullptr;

/// Ctrl+C를 위한 Windows 콘솔 제어 핸들러
static BOOL WINAPI ConsoleHandler(DWORD signal) {
    if (signal == CTRL_C_EVENT || signal == CTRL_BREAK_EVENT) {
        /// 종료 플래그만 설정 - 여기서 복잡한 작업을 수행하지 말 것
        g_shutdown_requested = true;

        /// accept()를 풀어주기 위해 서버를 깨움
        if (g_agent) {
            wakeUpServer(g_agent->getPort());
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

    SlaveAgent agent;
    g_agent = &agent;

    /// 콘솔 핸들러 설정
    SetConsoleCtrlHandler(ConsoleHandler, TRUE);

    if (!agent.initialize(argc, argv)) {
        return 1;
    }

    if (!agent.start()) {
        return 1;
    }

    /// 메인 루프 실행 (Ctrl+C 또는 종료 요청까지 블로킹)
    agent.run();

    /// 정상 종료
    ILOG << "Initiating graceful shutdown...";
    agent.stop();

    ILOG << "SlaveAgent terminated successfully.";
    Logger::shutdown();

    /// 종료가 완료되었음을 알림
    g_shutdown_complete = true;

    return 0;
}
