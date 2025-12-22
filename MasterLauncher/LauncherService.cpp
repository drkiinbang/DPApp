/// ============================================
/// LauncherService.cpp
/// Agent service running on Master machine
/// Manages Master process lifecycle via REST API
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
#include <cstdint>
#include <csignal>

#include "Logger.h"
#include "MiniJson.h"
#include "RESTApiServer.h"

using namespace DPApp;

/// ============================================
/// Global shutdown flag (must be declared before use)
/// ============================================
static std::atomic<bool> g_shutdown_requested{ false };
static std::atomic<bool> g_shutdown_complete{ false };

/// ============================================
/// Master Process Status
/// ============================================

enum class MasterStatus : uint8_t {
    STOPPED = 0,
    STARTING = 1,
    RUNNING = 2,
    STOPPING = 3,
    FAILED = 4
};

/// Convert MasterStatus to string representation
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
/// LauncherService Class
/// ============================================

class LauncherService {
public:
    LauncherService() = default;

    ~LauncherService() {
        /// Destructor should not call stop() again if already stopped
        /// The stop() is called explicitly from main()
    }

    /// Initialize launcher with command line arguments
    bool initialize(int argc, char* argv[]) {
        /// Default configuration
        launcher_port_ = 8090;
        master_executable_ = "master.exe";
        master_server_port_ = 8080;
        master_api_port_ = 8081;
        config_file_ = "launcher_config.json";

        /// Parse command line arguments
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

        /// Load configuration file if exists
        loadConfig();

        /// Initialize logger
        Logger::initialize("launcher.log");

        ILOG << "===========================================";
        ILOG << "DPApp Launcher Service Starting (Windows)";
        ILOG << "===========================================";
        ILOG << "Launcher port: " << launcher_port_;
        ILOG << "Master executable: " << master_executable_;

        return true;
    }

    /// Start the launcher service
    bool start() {
        running_ = true;

        /// Setup REST API routes
        setupApiRoutes();

        /// Start API server
        if (!api_server_->start(launcher_port_)) {
            ELOG << "Failed to start REST API server on port " << launcher_port_;
            return false;
        }

        ILOG << "Launcher REST API started on port " << launcher_port_;

        /// Start monitor thread
        monitor_thread_ = std::thread(&LauncherService::monitorLoop, this);

        return true;
    }

    /// Stop the launcher service
    void stop() {
        /// Prevent re-entry
        bool expected = true;
        if (!running_.compare_exchange_strong(expected, false)) {
            return;
        }

        ILOG << "Stopping launcher service...";

        /// Stop master if running
        stopMaster(false);

        /// Wake up the server accept() before stopping
        wakeUpServer(launcher_port_);

        /// Stop API server
        if (api_server_) {
            api_server_->stop();
        }

        /// Wait for monitor thread with timeout
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }

        ILOG << "Launcher service stopped";
    }

    /// Run main loop (blocking)
    void run() {
        ILOG << "Launcher service running. Press Ctrl+C to exit.";

        while (running_ && !g_shutdown_requested.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        /// Exit loop
        running_ = false;
    }

    /// Get launcher port
    uint16_t getPort() const { return launcher_port_; }

private:
    /// Print usage information
    void printUsage(const char* program) {
        std::cout << "Usage: " << program << " [options]\n"
            << "Options:\n"
            << "  -p, --port <port>      Launcher API port (default: 8090)\n"
            << "  -m, --master <path>    Master executable path (default: master.exe)\n"
            << "  -c, --config <file>    Configuration file (default: launcher_config.json)\n"
            << "  -h, --help             Show this help message\n";
    }

    /// Load configuration from JSON file
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
        if (auto v = MiniJson::get<std::string>(*parsed, "master_external_ip"))
            master_external_ip_ = *v;
        if (auto v = MiniJson::get<std::string>(*parsed, "master_config_file"))
            master_config_file_ = *v;

        ILOG << "Loaded configuration from " << config_file_;
    }

    /// Setup REST API routes
    void setupApiRoutes() {
        api_server_ = std::make_unique<RestApiServer>();

        /// Health check endpoint
        api_server_->GET("/api/health", [this](const HttpRequest& req) {
            return handleHealth(req);
            });

        /// Get launcher and master status
        api_server_->GET("/api/status", [this](const HttpRequest& req) {
            return handleGetStatus(req);
            });

        /// Start master process
        api_server_->POST("/api/master/start", [this](const HttpRequest& req) {
            return handleStartMaster(req);
            });

        /// Stop master process
        api_server_->POST("/api/master/stop", [this](const HttpRequest& req) {
            return handleStopMaster(req);
            });

        /// Restart master process
        api_server_->POST("/api/master/restart", [this](const HttpRequest& req) {
            return handleRestartMaster(req);
            });

        /// Shutdown launcher service
        api_server_->POST("/api/shutdown", [this](const HttpRequest& req) {
            return handleShutdown(req);
            });
    }

    /// =========================================
    /// Master Process Management
    /// =========================================

    /// Start master process
    bool startMaster() {
        std::lock_guard<std::mutex> lock(master_mutex_);

        if (master_status_ == MasterStatus::RUNNING && isMasterRunning()) {
            WLOG << "Master is already running with PID: " << master_pid_;
            return true;
        }

        master_status_ = MasterStatus::STARTING;

        /// Build command line
        std::ostringstream cmd;
        cmd << master_executable_;
        cmd << " --daemon";
        cmd << " --port " << master_server_port_;
        cmd << " --api-port " << master_api_port_;

        if (!master_external_ip_.empty()) {
            cmd << " --external-ip " << master_external_ip_;
        }
        if (!master_config_file_.empty()) {
            cmd << " --config " << master_config_file_;
        }

        std::string cmdLine = cmd.str();
        ILOG << "Starting master: " << cmdLine;

        /// Create process using Windows API
        STARTUPINFOA si = { sizeof(si) };
        PROCESS_INFORMATION pi = {};

        /// Convert to mutable char array for CreateProcessA
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

        /// Wait briefly and verify process is running
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

    /// Stop master process
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
            /// Try graceful shutdown first via REST API
            stopped = sendShutdownToMaster();

            if (stopped) {
                /// Wait for process to exit
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

        /// Force terminate if graceful shutdown failed or force requested
        if (!stopped || force) {
            if (master_handle_ != NULL) {
                ILOG << "Force terminating master process";
                TerminateProcess(master_handle_, 1);
                WaitForSingleObject(master_handle_, 3000);
            }
        }

        /// Cleanup
        if (master_handle_ != NULL) {
            CloseHandle(master_handle_);
            master_handle_ = NULL;
        }
        master_pid_ = 0;
        master_status_ = MasterStatus::STOPPED;

        ILOG << "Master process stopped";
        return true;
    }

    /// Check if master process is still running
    bool isMasterRunning() {
        if (master_handle_ == NULL) return false;

        DWORD exitCode;
        if (GetExitCodeProcess(master_handle_, &exitCode)) {
            return (exitCode == STILL_ACTIVE);
        }
        return false;
    }

    /// Send shutdown command to master via REST API
    bool sendShutdownToMaster() {
        SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == INVALID_SOCKET) {
            WLOG << "Failed to create socket for master shutdown";
            return false;
        }

        /// Set timeout
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

        /// Send shutdown request
        std::string request =
            "POST /api/shutdown HTTP/1.1\r\n"
            "Host: 127.0.0.1\r\n"
            "Content-Length: 0\r\n"
            "Connection: close\r\n"
            "\r\n";

        send(sock, request.c_str(), static_cast<int>(request.length()), 0);

        /// Receive response (brief)
        char buffer[512];
        recv(sock, buffer, sizeof(buffer) - 1, 0);

        closesocket(sock);

        ILOG << "Shutdown command sent to master";
        return true;
    }

    /// Monitor loop for checking master status
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
            if (auto v = MiniJson::get<std::string>(*parsed, "external_ip"))
                master_external_ip_ = *v;
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
    /// Configuration
    uint16_t launcher_port_ = 8090;
    std::string master_executable_ = "master.exe";
    uint16_t master_server_port_ = 8080;
    uint16_t master_api_port_ = 8081;
    std::string master_external_ip_;
    std::string master_config_file_;
    std::string config_file_ = "launcher_config.json";

    /// Runtime state
    std::atomic<bool> running_{ false };
    std::unique_ptr<RestApiServer> api_server_;
    std::thread monitor_thread_;

    /// Master process state
    std::mutex master_mutex_;
    DWORD master_pid_ = 0;
    HANDLE master_handle_ = NULL;
    MasterStatus master_status_ = MasterStatus::STOPPED;
    std::chrono::steady_clock::time_point master_start_time_;
};

/// ============================================
/// Global instance for console control handling
/// ============================================

static LauncherService* g_launcher = nullptr;

/// Windows console control handler for Ctrl+C
static BOOL WINAPI ConsoleHandler(DWORD signal) {
    if (signal == CTRL_C_EVENT || signal == CTRL_BREAK_EVENT) {
        /// Set shutdown flag only - do NOT perform complex operations here
        g_shutdown_requested = true;

        /// Wake up the server to unblock accept()
        if (g_launcher) {
            wakeUpServer(g_launcher->getPort());
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

    LauncherService launcher;
    g_launcher = &launcher;

    /// Setup Windows console control handler
    SetConsoleCtrlHandler(ConsoleHandler, TRUE);

    /// Initialize launcher
    if (!launcher.initialize(argc, argv)) {
        return 1;
    }

    /// Start launcher service
    if (!launcher.start()) {
        return 1;
    }

    /// Run main loop (blocking until Ctrl+C or shutdown request)
    launcher.run();

    /// Graceful shutdown
    ILOG << "Initiating graceful shutdown...";
    launcher.stop();

    ILOG << "Launcher terminated successfully.";
    Logger::shutdown();

    /// Signal that shutdown is complete
    g_shutdown_complete = true;

    return 0;
}