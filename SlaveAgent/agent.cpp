/// ============================================
/// SlaveAgent.cpp
/// Agent service running on Worker machines
/// Manages Slave process lifecycle via REST API
/// Windows version
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
/// Global shutdown flags (must be declared before use)
/// ============================================
static std::atomic<bool> g_shutdown_requested{ false };
static std::atomic<bool> g_shutdown_complete{ false };

/// ============================================
/// Helper function to wake up blocked accept()
/// ============================================
static void wakeUpServer(uint16_t port) {
    /// Send dummy connection to wake up accept()
    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) return;

    /// Set short timeout
    DWORD timeout = 100;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    /// Just connect and close - this wakes up accept()
    connect(sock, (sockaddr*)&addr, sizeof(addr));
    closesocket(sock);
}

/// ============================================
/// Slave Process Information
/// ============================================

struct SlaveProcessInfo {
    DWORD pid = 0;
    HANDLE handle = NULL;
    std::string master_address;
    uint16_t master_port = 0;
    uint32_t threads = 1;
    std::chrono::steady_clock::time_point start_time;
    bool running = false;

    SlaveProcessInfo() = default;

    SlaveProcessInfo(DWORD p, HANDLE h, const std::string& addr, uint16_t port, uint32_t t)
        : pid(p), handle(h), master_address(addr), master_port(port), threads(t),
        start_time(std::chrono::steady_clock::now()), running(true) {
    }
};

/// ============================================
/// SlaveAgent Class
/// ============================================

class SlaveAgent {
public:
    SlaveAgent() = default;

    ~SlaveAgent() {
        /// Destructor should not call stop() again if already stopped
        /// The stop() is called explicitly from main()
    }

    /// Initialize agent with command line arguments
    bool initialize(int argc, char* argv[]) {
        /// Default configuration
        agent_port_ = 8092;
        slave_executable_ = "SlaveApp.exe";
        max_slaves_ = 8;
        config_file_ = "agent_config.json";

        /// Parse command line arguments
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

        /// Load configuration file if exists
        loadConfig();

        /// Initialize logger
        Logger::initialize("slave_agent.log");

        ILOG << "===========================================";
        ILOG << "DPApp Slave Agent Starting (Windows)";
        ILOG << "===========================================";
        ILOG << "Agent port: " << agent_port_;
        ILOG << "Slave executable: " << slave_executable_;
        ILOG << "Max slaves: " << max_slaves_;

        return true;
    }

    /// Start the agent service
    bool start() {
        running_ = true;

        /// Setup REST API routes
        setupApiRoutes();

        /// Start API server
        if (!api_server_->start(agent_port_)) {
            ELOG << "Failed to start REST API server on port " << agent_port_;
            return false;
        }

        ILOG << "SlaveAgent REST API started on port " << agent_port_;

        /// Start monitor thread
        monitor_thread_ = std::thread(&SlaveAgent::monitorLoop, this);

        return true;
    }

    /// Stop the agent service
    void stop() {
        /// Prevent re-entry
        bool expected = true;
        if (!running_.compare_exchange_strong(expected, false)) {
            return;
        }

        ILOG << "Stopping SlaveAgent...";

        /// Stop all slaves
        stopAllSlaves(true);

        /// Wake up the server accept() before stopping
        wakeUpServer(agent_port_);

        /// Stop API server
        if (api_server_) {
            api_server_->stop();
        }

        /// Wait for monitor thread
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }

        ILOG << "SlaveAgent stopped";
    }

    /// Run main loop (blocking)
    void run() {
        ILOG << "SlaveAgent running. Press Ctrl+C to exit.";

        while (running_ && !g_shutdown_requested.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        /// Exit loop
        running_ = false;
    }

    /// Get agent port
    uint16_t getPort() const { return agent_port_; }

private:
    /// Print usage information
    void printUsage(const char* program) {
        std::cout << "Usage: " << program << " [options]\n"
            << "Options:\n"
            << "  -p, --port <port>        Agent API port (default: 8092)\n"
            << "  -s, --slave <path>       Slave executable path (default: SlaveApp.exe)\n"
            << "  -m, --max-slaves <num>   Maximum concurrent slaves (default: 8)\n"
            << "  -c, --config <file>      Configuration file (default: agent_config.json)\n"
            << "  -h, --help               Show this help message\n";
    }

    /// Load configuration from file
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

    /// Setup REST API routes
    void setupApiRoutes() {
        api_server_ = std::make_unique<RestApiServer>();

        /// Health check
        api_server_->GET("/api/health", [this](const HttpRequest& req) {
            return handleHealth(req);
            });

        /// Get agent and slaves status
        api_server_->GET("/api/status", [this](const HttpRequest& req) {
            return handleGetStatus(req);
            });

        /// Get slaves list
        api_server_->GET("/api/slaves", [this](const HttpRequest& req) {
            return handleGetSlaves(req);
            });

        /// Start a new slave
        api_server_->POST("/api/slaves/start", [this](const HttpRequest& req) {
            return handleStartSlave(req);
            });

        /// Stop a specific slave by PID
        api_server_->POST("/api/slaves/stop", [this](const HttpRequest& req) {
            return handleStopSlave(req);
            });

        /// Stop all slaves
        api_server_->POST("/api/slaves/stop-all", [this](const HttpRequest& req) {
            return handleStopAllSlaves(req);
            });

        /// Shutdown agent
        api_server_->POST("/api/shutdown", [this](const HttpRequest& req) {
            return handleShutdown(req);
            });
    }

    /// =========================================
    /// Slave Process Management
    /// =========================================

    /// Start a new slave process
    bool startSlave(const std::string& master_address,
        uint16_t master_port,
        uint32_t threads,
        SlaveProcessInfo& out_info) {
        std::lock_guard<std::mutex> lock(slaves_mutex_);

        /// Check max slaves limit
        size_t running_count = countRunningSlaves();
        if (running_count >= max_slaves_) {
            WLOG << "Max slaves limit reached (" << max_slaves_ << ")";
            return false;
        }

        /// Build command line
        std::ostringstream cmd;
        cmd << slave_executable_;
        cmd << " -s " << master_address;
        cmd << " -p " << master_port;
        cmd << " -t " << threads;

        std::string command = cmd.str();
        ILOG << "Starting slave: " << command;

        /// Setup startup info
        STARTUPINFOA si;
        ZeroMemory(&si, sizeof(si));
        si.cb = sizeof(si);
        si.dwFlags = STARTF_USESHOWWINDOW;
        si.wShowWindow = SW_HIDE;

        PROCESS_INFORMATION pi;
        ZeroMemory(&pi, sizeof(pi));

        /// Set working directory if specified
        const char* work_dir = working_directory_.empty() ? NULL : working_directory_.c_str();

        /// Create process
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

        /// Store process info
        SlaveProcessInfo info(pi.dwProcessId, pi.hProcess, master_address, master_port, threads);
        slaves_.push_back(info);

        CloseHandle(pi.hThread);

        out_info = info;
        ILOG << "Slave started (PID: " << pi.dwProcessId << ") -> "
            << master_address << ":" << master_port;

        return true;
    }

    /// Stop a slave by PID
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
                /// Try graceful shutdown first
                /// Send Ctrl+C or use other method
                /// For simplicity, just terminate
                result = TerminateProcess(it->handle, 0) != 0;
            }

            /// Wait briefly for process to exit
            WaitForSingleObject(it->handle, 3000);
            CloseHandle(it->handle);
        }

        slaves_.erase(it);
        ILOG << "Slave stopped (PID: " << pid << ")";

        return true;
    }

    /// Stop all slaves
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

    /// Count running slaves
    size_t countRunningSlaves() {
        /// Assumes mutex is already held
        size_t count = 0;
        for (const auto& slave : slaves_) {
            if (slave.running) {
                count++;
            }
        }
        return count;
    }

    /// Check if a process is still running
    bool isProcessRunning(HANDLE handle) {
        if (handle == NULL) return false;

        DWORD exitCode;
        if (GetExitCodeProcess(handle, &exitCode)) {
            return exitCode == STILL_ACTIVE;
        }
        return false;
    }

    /// =========================================
    /// Monitor Loop
    /// =========================================

    void monitorLoop() {
        ILOG << "Monitor thread started";

        while (running_ && !g_shutdown_requested.load()) {
            /// Check slave status periodically
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

            /// Check every 5 seconds (but check shutdown flag more frequently)
            for (int i = 0; i < 50 && running_ && !g_shutdown_requested.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        ILOG << "Monitor thread exiting";
    }

    /// =========================================
    /// REST API Handlers
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
            json << "        \"threads\": " << s.threads << ",\n";
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
        /// Parse request body
        auto parsed = MiniJson::parse_object(req.body);
        if (!parsed) {
            return createErrorResponse(400, "Invalid JSON body");
        }

        /// Get required parameters
        auto master_address = MiniJson::get<std::string>(*parsed, "master_address");
        if (!master_address || master_address->empty()) {
            return createErrorResponse(400, "Missing 'master_address'");
        }

        /// Get optional parameters with defaults
        uint16_t master_port = 8080;
        uint32_t threads = 4;

        if (auto v = MiniJson::get<double>(*parsed, "master_port")) {
            master_port = static_cast<uint16_t>(*v);
        }
        if (auto v = MiniJson::get<double>(*parsed, "threads")) {
            threads = static_cast<uint32_t>(*v);
        }

        /// Start slave
        SlaveProcessInfo info;
        if (startSlave(*master_address, master_port, threads, info)) {
            std::ostringstream json;
            json << "{\n";
            json << "  \"success\": true,\n";
            json << "  \"message\": \"Slave started\",\n";
            json << "  \"data\": {\n";
            json << "    \"pid\": " << info.pid << ",\n";
            json << "    \"master_address\": \"" << info.master_address << "\",\n";
            json << "    \"master_port\": " << info.master_port << ",\n";
            json << "    \"threads\": " << info.threads << "\n";
            json << "  }\n";
            json << "}";

            HttpResponse response;
            response.body = json.str();
            return response;
        }

        return createErrorResponse(500, "Failed to start slave (max limit reached or process error)");
    }

    HttpResponse handleStopSlave(const HttpRequest& req) {
        /// Parse request body
        auto parsed = MiniJson::parse_object(req.body);
        if (!parsed) {
            return createErrorResponse(400, "Invalid JSON body");
        }

        /// Get PID
        auto pid_opt = MiniJson::get<double>(*parsed, "pid");
        if (!pid_opt) {
            return createErrorResponse(400, "Missing 'pid'");
        }

        DWORD pid = static_cast<DWORD>(*pid_opt);

        /// Get force flag
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
        /// Get force flag
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

        /// Set shutdown flag instead of using detached thread
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
    /// Configuration
    uint16_t agent_port_ = 8092;
    ///[ToDo] set exe file name in config
#ifdef _DEBUG
    std::string slave_executable_ = "SlaveAppd.exe";
#else
    std::string slave_executable_ = "SlaveApp.exe";
#endif
    uint32_t max_slaves_ = 8;
    std::string working_directory_;
    std::string config_file_ = "agent_config.json";

    /// State
    std::atomic<bool> running_{ false };
    std::unique_ptr<RestApiServer> api_server_;
    std::thread monitor_thread_;

    /// Slave processes
    std::mutex slaves_mutex_;
    std::vector<SlaveProcessInfo> slaves_;
};

/// ============================================
/// Global instance for signal handling
/// ============================================

static SlaveAgent* g_agent = nullptr;

/// Windows console control handler for Ctrl+C
static BOOL WINAPI ConsoleHandler(DWORD signal) {
    if (signal == CTRL_C_EVENT || signal == CTRL_BREAK_EVENT) {
        /// Set shutdown flag only - do NOT perform complex operations here
        g_shutdown_requested = true;

        /// Wake up the server to unblock accept()
        if (g_agent) {
            wakeUpServer(g_agent->getPort());
        }

        /// Wait for graceful shutdown to complete (max 5 seconds)
        for (int i = 0; i < 50 && !g_shutdown_complete.load(); ++i) {
            Sleep(100);
        }

        return TRUE;
    }
    return FALSE;
}

/// ============================================
/// Main Entry Point
/// ============================================

int main(int argc, char* argv[]) {
    /// Disable abort message box in debug mode
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);

    SlaveAgent agent;
    g_agent = &agent;

    /// Setup console handler
    SetConsoleCtrlHandler(ConsoleHandler, TRUE);

    if (!agent.initialize(argc, argv)) {
        return 1;
    }

    if (!agent.start()) {
        return 1;
    }

    /// Run main loop (blocking until Ctrl+C or shutdown request)
    agent.run();

    /// Graceful shutdown
    ILOG << "Initiating graceful shutdown...";
    agent.stop();

    ILOG << "SlaveAgent terminated successfully.";
    Logger::shutdown();

    /// Signal that shutdown is complete
    g_shutdown_complete = true;

    return 0;
}