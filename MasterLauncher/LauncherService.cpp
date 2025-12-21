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

#include "Logger.h"
#include "MiniJson.h"
#include "RESTApiServer.h"

using namespace DPApp;

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
/// LauncherService Class
/// ============================================

class LauncherService {
public:
    LauncherService() = default;

    ~LauncherService() {
        stop();
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
        if (!running_) return;

        running_ = false;

        /// Stop master if running
        stopMaster(false);

        /// Stop API server
        if (api_server_) {
            api_server_->stop();
        }

        /// Wait for monitor thread
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }

        ILOG << "Launcher service stopped";
    }

    /// Run main loop (blocking)
    void run() {
        ILOG << "Launcher service running. Press Ctrl+C to exit.";

        while (running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

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

    /// Start master process using CreateProcess
    bool startMaster() {
        std::lock_guard<std::mutex> lock(master_mutex_);

        if (master_pid_ != 0 && isMasterRunning()) {
            WLOG << "Master is already running (PID: " << master_pid_ << ")";
            return false;
        }

        /// Build command line
        std::ostringstream cmd;
        cmd << master_executable_;
        cmd << " -p " << master_server_port_;
        cmd << " -a " << master_api_port_;
        cmd << " -d";  /// daemon mode

        if (!master_external_ip_.empty()) {
            cmd << " --external-ip " << master_external_ip_;
        }

        if (!master_config_file_.empty()) {
            cmd << " -c " << master_config_file_;
        }

        std::string command = cmd.str();
        ILOG << "Starting master: " << command;

        /// Setup startup info for CreateProcess
        STARTUPINFOA si;
        ZeroMemory(&si, sizeof(si));
        si.cb = sizeof(si);
        si.dwFlags = STARTF_USESHOWWINDOW;
        si.wShowWindow = SW_HIDE;

        PROCESS_INFORMATION pi;
        ZeroMemory(&pi, sizeof(pi));

        /// Create the master process
        if (!CreateProcessA(
            NULL,
            const_cast<char*>(command.c_str()),
            NULL,
            NULL,
            FALSE,
            CREATE_NEW_CONSOLE | DETACHED_PROCESS,
            NULL,
            NULL,
            &si,
            &pi)) {
            DWORD error = GetLastError();
            ELOG << "CreateProcess failed with error: " << error;
            master_status_ = MasterStatus::FAILED;
            return false;
        }

        /// Store process information
        master_pid_ = pi.dwProcessId;
        master_handle_ = pi.hProcess;
        CloseHandle(pi.hThread);

        master_status_ = MasterStatus::STARTING;
        master_start_time_ = std::chrono::steady_clock::now();

        ILOG << "Master process started (PID: " << master_pid_ << ")";
        return true;
    }

    /// Stop master process
    bool stopMaster(bool force) {
        std::lock_guard<std::mutex> lock(master_mutex_);

        if (master_pid_ == 0) {
            WLOG << "Master is not running";
            return true;
        }

        ILOG << "Stopping master (PID: " << master_pid_ << ", force: " << force << ")";
        master_status_ = MasterStatus::STOPPING;

        bool result = false;

        if (master_handle_ != NULL) {
            if (!force) {
                /// Try graceful shutdown via REST API first
                result = sendShutdownToMaster();

                if (result) {
                    /// Wait for process to exit gracefully
                    DWORD waitResult = WaitForSingleObject(master_handle_, 5000);
                    if (waitResult != WAIT_OBJECT_0) {
                        /// Process didn't exit, force terminate
                        WLOG << "Graceful shutdown timeout, forcing termination";
                        TerminateProcess(master_handle_, 1);
                    }
                }
                else {
                    /// REST API failed, force terminate
                    TerminateProcess(master_handle_, 1);
                }
            }
            else {
                /// Force terminate immediately
                result = TerminateProcess(master_handle_, 1) != 0;
            }

            /// Wait for process to fully terminate
            WaitForSingleObject(master_handle_, 3000);
            CloseHandle(master_handle_);
            master_handle_ = NULL;
        }

        master_pid_ = 0;
        master_status_ = MasterStatus::STOPPED;
        ILOG << "Master stopped";

        return true;
    }

    /// Send shutdown command to master via REST API
    bool sendShutdownToMaster() {
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            ELOG << "WSAStartup failed";
            return false;
        }

        SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == INVALID_SOCKET) {
            ELOG << "Failed to create socket";
            WSACleanup();
            return false;
        }

        /// Set socket timeout
        DWORD timeout = 5000;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));

        /// Connect to master REST API
        sockaddr_in server;
        ZeroMemory(&server, sizeof(server));
        server.sin_family = AF_INET;
        server.sin_port = htons(master_api_port_);
        inet_pton(AF_INET, "127.0.0.1", &server.sin_addr);  /// <-- ¼öÁ¤µÊ

        if (connect(sock, (sockaddr*)&server, sizeof(server)) == SOCKET_ERROR) {
            ELOG << "Failed to connect to master REST API";
            closesocket(sock);
            WSACleanup();
            return false;
        }

        /// Send HTTP POST request
        std::string request =
            "POST /api/shutdown HTTP/1.1\r\n"
            "Host: localhost\r\n"
            "Content-Length: 0\r\n"
            "Connection: close\r\n"
            "\r\n";

        if (send(sock, request.c_str(), static_cast<int>(request.length()), 0) == SOCKET_ERROR) {
            ELOG << "Failed to send shutdown request";
            closesocket(sock);
            WSACleanup();
            return false;
        }

        /// Wait for response
        char buffer[256];
        recv(sock, buffer, sizeof(buffer), 0);

        closesocket(sock);
        WSACleanup();

        ILOG << "Shutdown request sent to master";
        return true;
    }

    /// Check if master process is still running
    bool isMasterRunning() {
        if (master_pid_ == 0) return false;

        if (master_handle_ != NULL) {
            DWORD exitCode;
            if (GetExitCodeProcess(master_handle_, &exitCode)) {
                return exitCode == STILL_ACTIVE;
            }
        }

        /// Fallback: check process by PID using OpenProcess
        HANDLE hProcess = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, master_pid_);
        if (hProcess == NULL) {
            return false;
        }

        DWORD exitCode;
        bool running = GetExitCodeProcess(hProcess, &exitCode) && exitCode == STILL_ACTIVE;
        CloseHandle(hProcess);
        return running;
    }

    /// =========================================
    /// Monitor Loop
    /// =========================================

    /// Background thread to monitor master process status
    void monitorLoop() {
        while (running_) {
            {
                std::lock_guard<std::mutex> lock(master_mutex_);

                if (master_pid_ != 0) {
                    bool running = isMasterRunning();

                    if (running && master_status_ == MasterStatus::STARTING) {
                        /// Master has successfully started
                        master_status_ = MasterStatus::RUNNING;
                        ILOG << "Master is now running";
                    }
                    else if (!running && master_status_ == MasterStatus::RUNNING) {
                        /// Master terminated unexpectedly
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

            /// Check every 5 seconds
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }

    /// =========================================
    /// REST API Handlers
    /// =========================================

    /// Handle health check request
    HttpResponse handleHealth(const HttpRequest& req) {
        HttpResponse response;
        response.status_code = 200;
        response.body = R"({"success": true, "status": "healthy", "service": "launcher"})";
        return response;
    }

    /// Handle status request
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

    /// Handle master start request
    HttpResponse handleStartMaster(const HttpRequest& req) {
        /// Parse optional parameters from request body
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

    /// Handle master stop request
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

    /// Handle master restart request
    HttpResponse handleRestartMaster(const HttpRequest& req) {
        bool force = false;

        auto parsed = MiniJson::parse_object(req.body);
        if (parsed) {
            if (auto v = MiniJson::get<bool>(*parsed, "force"))
                force = *v;
        }

        /// Stop master first
        stopMaster(force);

        /// Wait a moment before restarting
        std::this_thread::sleep_for(std::chrono::seconds(1));

        /// Start master again
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

    /// Handle launcher shutdown request
    HttpResponse handleShutdown(const HttpRequest& req) {
        ILOG << "Shutdown requested via API";

        /// Schedule shutdown in separate thread to allow response to be sent
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            running_ = false;
            }).detach();

        HttpResponse response;
        response.body = R"({"success": true, "message": "Launcher shutdown initiated"})";
        return response;
    }

    /// Create error response with given status code and message
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
        ILOG << "Ctrl+C received, shutting down...";
        if (g_launcher) {
            g_launcher->stop();
        }
        return TRUE;
    }
    return FALSE;
}

/// ============================================
/// Main Entry Point
/// ============================================

int main(int argc, char* argv[]) {
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

    /// Run main loop (blocking)
    launcher.run();

    return 0;
}