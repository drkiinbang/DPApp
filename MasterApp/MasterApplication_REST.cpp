/**
 * @file MasterApplication_REST.cpp
 * @brief REST API implementation
 *
 * This file contains:
 * - REST API route setup
 * - All REST API handlers for status, slaves, tasks, results
 * - Error response helper
 * - Timestamp utility
 */

#include "MasterApplication.h"
#include "../include/MiniJson.h"
#include "../include/TaskManagerTypes.h"  // TaskManager.h 대신 경량 헤더 사용

 /// =========================================
 /// REST API Route Setup
 /// =========================================

void MasterApplication::setupRestApiRoutes() {
    api_server_ = std::make_unique<RestApiServer>();

    api_server_->GET("/api/status", [this](const HttpRequest& req) {
        return handleGetStatus(req);
        });

    api_server_->GET("/api/slaves", [this](const HttpRequest& req) {
        return handleGetSlaves(req);
        });

    api_server_->GET("/api/slaves/{id}", [this](const HttpRequest& req) {
        return handleGetSlave(req);
        });

    api_server_->POST("/api/slaves/{id}/shutdown", [this](const HttpRequest& req) {
        return handleShutdownSlave(req);
        });

    api_server_->POST("/api/slaves/shutdown", [this](const HttpRequest& req) {
        return handleShutdownAllSlaves(req);
        });

    api_server_->GET("/api/tasks", [this](const HttpRequest& req) {
        return handleGetTasks(req);
        });

    api_server_->GET("/api/tasks/{id}", [this](const HttpRequest& req) {
        return handleGetTask(req);
        });

    api_server_->DEL("/api/tasks", [this](const HttpRequest& req) {
        return handleClearTasks(req);
        });

    api_server_->POST("/api/files/load", [this](const HttpRequest& req) {
        return handleLoadFile(req);
        });

    api_server_->GET("/api/progress", [this](const HttpRequest& req) {
        return handleGetProgress(req);
        });

    api_server_->GET("/api/results", [this](const HttpRequest& req) {
        return handleGetResults(req);
        });

    api_server_->POST("/api/results/save", [this](const HttpRequest& req) {
        return handleSaveResults(req);
        });

    api_server_->POST("/api/shutdown", [this](const HttpRequest& req) {
        return handleShutdownServer(req);
        });

    api_server_->GET("/api/health", [this](const HttpRequest& req) {
        HttpResponse response;
        response.status_code = 200;
        response.body = R"({"success": true, "status": "healthy", "timestamp": ")" + getCurrentTimestamp() + "\"}";
        return response;
        });

    /// =========================================
    /// Agent Management Endpoints
    /// =========================================

    api_server_->GET("/api/agents", [this](const HttpRequest& req) {
        return handleGetAgents(req);
        });

    api_server_->POST("/api/agents", [this](const HttpRequest& req) {
        return handleAddAgent(req);
        });

    api_server_->DEL("/api/agents/{id}", [this](const HttpRequest& req) {
        return handleRemoveAgent(req);
        });

    api_server_->POST("/api/agents/{id}/test", [this](const HttpRequest& req) {
        return handleTestAgent(req);
        });

    api_server_->POST("/api/agents/start-slaves", [this](const HttpRequest& req) {
        return handleStartSlavesOnAgents(req);
        });

    api_server_->POST("/api/agents/stop-slaves", [this](const HttpRequest& req) {
        return handleStopSlavesOnAgents(req);
        });

    /// =========================================
    /// Slave Pause/Resume Endpoints
    /// =========================================

    api_server_->POST("/api/slaves/{id}/pause", [this](const HttpRequest& req) {
        return handlePauseSlave(req);
        });

    api_server_->POST("/api/slaves/{id}/resume", [this](const HttpRequest& req) {
        return handleResumeSlave(req);
        });

    api_server_->POST("/api/slaves/pause", [this](const HttpRequest& req) {
        return handlePauseAllSlaves(req);
        });

    api_server_->POST("/api/slaves/resume", [this](const HttpRequest& req) {
        return handleResumeAllSlaves(req);
        });

    /// ICP API Routes
    //setupIcpApiRoutes();
}

/// =========================================
/// Status Handlers
/// =========================================

HttpResponse MasterApplication::handleGetStatus(const HttpRequest& req) {
    std::ostringstream json;
    json << "{\n";
    json << "  \"success\": true,\n";
    json << "  \"data\": {\n";
    json << "    \"server_running\": " << (server_->isRunning() ? "true" : "false") << ",\n";
    json << "    \"server_port\": " << server_port_ << ",\n";
    json << "    \"api_port\": " << api_port_ << ",\n";
    json << "    \"connected_slaves\": " << server_->getClientCount() << ",\n";
    json << "    \"registered_agents\": " << agents_.size() << ",\n";

    if (task_manager_) {
        json << "    \"current_task_type\": \"" << taskStr(task_manager_->getCurrentTaskType()) << "\",\n";
        json << "    \"pending_tasks\": " << task_manager_->getPendingTaskCount() << ",\n";
        json << "    \"active_tasks\": " << task_manager_->getActiveTaskCount() << ",\n";
        json << "    \"completed_tasks\": " << task_manager_->getCompletedTaskCount() << ",\n";
        json << "    \"failed_tasks\": " << task_manager_->getFailedTaskCount() << ",\n";
        json << "    \"overall_progress\": " << std::fixed << std::setprecision(2) << (task_manager_->getOverallProgress() * 100) << ",\n";
    }
    else {
        json << "    \"current_task_type\": null,\n";
        json << "    \"pending_tasks\": 0,\n";
        json << "    \"active_tasks\": 0,\n";
        json << "    \"completed_tasks\": 0,\n";
        json << "    \"failed_tasks\": 0,\n";
        json << "    \"overall_progress\": 0,\n";
    }

    json << "    \"timestamp\": \"" << getCurrentTimestamp() << "\"\n";
    json << "  }\n";
    json << "}";

    HttpResponse response;
    response.body = json.str();
    return response;
}

/// =========================================
/// Slave Handlers
/// =========================================

HttpResponse MasterApplication::handleGetSlaves(const HttpRequest& req) {
    if (!task_manager_) {
        HttpResponse response;
        response.body = R"({"success": false, "error": "No task session active"})";
        return response;
    }

    auto slaves = task_manager_->getAllSlaves();

    std::ostringstream json;
    json << "{\n";
    json << "  \"success\": true,\n";
    json << "  \"data\": {\n";
    json << "    \"total_slaves\": " << slaves.size() << ",\n";
    json << "    \"slaves\": [\n";

    for (size_t i = 0; i < slaves.size(); ++i) {
        const auto& slave = slaves[i];
        json << "      {\n";
        json << "        \"id\": \"" << slave.slave_id << "\",\n";
        json << "        \"active\": " << (slave.is_active ? "true" : "false") << ",\n";
        json << "        \"busy\": " << (slave.is_busy ? "true" : "false") << ",\n";
        json << "        \"completed_tasks\": " << slave.completed_tasks << ",\n";
        json << "        \"failed_tasks\": " << slave.failed_tasks << ",\n";
        json << "        \"avg_processing_time\": " << std::fixed << std::setprecision(2) << slave.avg_processing_time << "\n";
        json << "      }";
        if (i < slaves.size() - 1) json << ",";
        json << "\n";
    }

    json << "    ]\n";
    json << "  }\n";
    json << "}";

    HttpResponse response;
    response.body = json.str();
    return response;
}

HttpResponse MasterApplication::handleGetSlave(const HttpRequest& req) {
    if (!task_manager_) {
        return createErrorResponse(400, "No task session active");
    }

    std::string slave_id = req.path_params.at("id");
    auto slaves = task_manager_->getAllSlaves();

    for (const auto& slave : slaves) {
        if (slave.slave_id == slave_id) {
            std::ostringstream json;
            json << "{\n";
            json << "  \"success\": true,\n";
            json << "  \"data\": {\n";
            json << "    \"id\": \"" << slave.slave_id << "\",\n";
            json << "    \"active\": " << (slave.is_active ? "true" : "false") << ",\n";
            json << "    \"busy\": " << (slave.is_busy ? "true" : "false") << ",\n";
            json << "    \"completed_tasks\": " << slave.completed_tasks << ",\n";
            json << "    \"failed_tasks\": " << slave.failed_tasks << ",\n";
            json << "    \"avg_processing_time\": " << std::fixed << std::setprecision(2) << slave.avg_processing_time << "\n";
            json << "  }\n";
            json << "}";

            HttpResponse response;
            response.body = json.str();
            return response;
        }
    }

    return createErrorResponse(404, "Slave not found");
}

HttpResponse MasterApplication::handleShutdownSlave(const HttpRequest& req) {
    std::string slave_id = req.path_params.at("id");

    NetworkMessage shutdown_msg(MessageType::SHUTDOWN, std::vector<uint8_t>());

    if (server_->sendMessage(slave_id, shutdown_msg)) {
        std::ostringstream json;
        json << "{\n";
        json << "  \"success\": true,\n";
        json << "  \"message\": \"Shutdown command sent to slave\",\n";
        json << "  \"data\": {\n";
        json << "    \"slave_id\": \"" << slave_id << "\"\n";
        json << "  }\n";
        json << "}";

        HttpResponse response;
        response.body = json.str();
        return response;
    }
    else {
        return createErrorResponse(500, "Failed to send shutdown command");
    }
}

HttpResponse MasterApplication::handleShutdownAllSlaves(const HttpRequest& req) {
    auto connected_clients = server_->getConnectedClients();

    if (connected_clients.empty()) {
        return createErrorResponse(400, "No connected slaves to shutdown");
    }

    NetworkMessage shutdown_msg(MessageType::SHUTDOWN, std::vector<uint8_t>());
    int success_count = 0;

    for (const auto& client_id : connected_clients) {
        if (server_->sendMessage(client_id, shutdown_msg)) {
            success_count++;
        }
    }

    std::ostringstream json;
    json << "{\n";
    json << "  \"success\": true,\n";
    json << "  \"message\": \"Shutdown commands sent\",\n";
    json << "  \"data\": {\n";
    json << "    \"total_slaves\": " << connected_clients.size() << ",\n";
    json << "    \"shutdown_sent\": " << success_count << "\n";
    json << "  }\n";
    json << "}";

    HttpResponse response;
    response.body = json.str();
    return response;
}

/// =========================================
/// Task Handlers
/// =========================================

HttpResponse MasterApplication::handleGetTasks(const HttpRequest& req) {
    if (!task_manager_) {
        return createErrorResponse(400, "No task session active");
    }

    auto pending_tasks = task_manager_->getTasksByStatus(TaskStatus::PENDING);
    auto active_tasks = task_manager_->getTasksByStatus(TaskStatus::IN_PROGRESS);
    auto completed_tasks = task_manager_->getTasksByStatus(TaskStatus::COMPLETED);
    auto failed_tasks = task_manager_->getTasksByStatus(TaskStatus::FAILED);

    std::ostringstream json;
    json << "{\n";
    json << "  \"success\": true,\n";
    json << "  \"data\": {\n";
    json << "    \"task_type\": \"" << taskStr(task_manager_->getCurrentTaskType()) << "\",\n";
    json << "    \"summary\": {\n";
    json << "      \"pending_count\": " << pending_tasks.size() << ",\n";
    json << "      \"active_count\": " << active_tasks.size() << ",\n";
    json << "      \"completed_count\": " << completed_tasks.size() << ",\n";
    json << "      \"failed_count\": " << failed_tasks.size() << ",\n";
    json << "      \"total_count\": " << (pending_tasks.size() + active_tasks.size() + completed_tasks.size() + failed_tasks.size()) << "\n";
    json << "    }\n";
    json << "  }\n";
    json << "}";

    HttpResponse response;
    response.body = json.str();
    return response;
}

HttpResponse MasterApplication::handleGetTask(const HttpRequest& req) {
    if (!task_manager_) {
        return createErrorResponse(400, "No task session active");
    }

    std::string task_id_str = req.path_params.at("id");
    uint32_t task_id;

    try {
        task_id = static_cast<uint32_t>(std::stoul(task_id_str));
    }
    catch (const std::exception&) {
        return createErrorResponse(400, "Invalid task ID format");
    }

    std::vector<TaskStatus> all_statuses = {
        TaskStatus::PENDING,
        TaskStatus::IN_PROGRESS,
        TaskStatus::COMPLETED,
        TaskStatus::FAILED
    };

    for (const auto& status : all_statuses) {
        auto tasks = task_manager_->getTasksByStatus(status);
        for (const auto& task : tasks) {
            if (task.task_id == task_id) {
                std::string status_str;
                switch (status) {
                case TaskStatus::PENDING: status_str = "pending"; break;
                case TaskStatus::IN_PROGRESS: status_str = "in_progress"; break;
                case TaskStatus::COMPLETED: status_str = "completed"; break;
                case TaskStatus::FAILED: status_str = "failed"; break;
                }

                std::ostringstream json;
                json << "{\n";
                json << "  \"success\": true,\n";
                json << "  \"data\": {\n";
                json << "    \"task_id\": " << task.task_id << ",\n";
                json << "    \"chunk_id\": " << task.chunk_id << ",\n";
                json << "    \"task_type\": \"" << taskStr(task.task_type) << "\",\n";
                json << "    \"status\": \"" << status_str << "\",\n";
                json << "    \"assigned_slave\": \"" << task.assigned_slave << "\"\n";
                json << "  }\n";
                json << "}";

                HttpResponse response;
                response.body = json.str();
                return response;
            }
        }
    }

    return createErrorResponse(404, "Task not found");
}

HttpResponse MasterApplication::handleClearTasks(const HttpRequest& req) {
    if (!task_manager_) {
        return createErrorResponse(400, "No task session active");
    }

    auto completed_results = task_manager_->getCompletedResults();
    size_t completed_count = completed_results.size();

    task_manager_->clearCompletedResults();

    std::ostringstream json;
    json << "{\n";
    json << "  \"success\": true,\n";
    json << "  \"message\": \"Completed tasks cleared successfully\",\n";
    json << "  \"data\": {\n";
    json << "    \"cleared_tasks_count\": " << completed_count << ",\n";
    json << "    \"timestamp\": \"" << getCurrentTimestamp() << "\"\n";
    json << "  }\n";
    json << "}";

    HttpResponse response;
    response.body = json.str();
    return response;
}

/// =========================================
/// File & Progress Handlers
/// =========================================

HttpResponse MasterApplication::handleLoadFile(const HttpRequest& req) {
    auto it_ct = req.headers.find("Content-Type");
    std::string content_type = (it_ct != req.headers.end() ? it_ct->second : "");
    if (content_type.find("application/json") == std::string::npos) {
        return createErrorResponse(400, "Content-Type must be application/json");
    }

    auto parsed = DPApp::MiniJson::parse_object(req.body);
    if (!parsed) {
        return createErrorResponse(400, "Invalid JSON body");
    }

    auto filename = DPApp::MiniJson::get<std::string>(*parsed, "filename");
    std::optional<std::string> task_type_str = DPApp::MiniJson::get<std::string>(*parsed, "task_type");

    if (!filename || !task_type_str) {
        return createErrorResponse(400, "Missing 'filename' or 'task_type'");
    }

    TaskType task_type = strTask(task_type_str.value());

    if (task_manager_ && task_manager_->getCurrentTaskType() != task_type) {
        std::ostringstream error_msg;
        error_msg << "Cannot change task type during session. Current: "
            << taskStr(task_manager_->getCurrentTaskType())
            << ", Requested: " << taskStr(task_type)
            << ". Please restart the application.";
        return createErrorResponse(400, error_msg.str());
    }

    if (loadAndProcessPointCloud(*filename, task_type)) {
        std::ostringstream json;
        json << "{\n";
        json << "  \"success\": true,\n";
        json << "  \"message\": \"Point cloud processing started\",\n";
        json << "  \"data\": {\n";
        json << "    \"filename\": \"" << *filename << "\",\n";
        json << "    \"task_type\": \"" << taskStr(task_type) << "\",\n";
        json << "    \"session_mode\": \"single_task_type\"\n";
        json << "  }\n";
        json << "}";
        HttpResponse response;
        response.body = json.str();
        return response;
    }
    else {
        return createErrorResponse(500, "Failed to load point cloud file");
    }
}

HttpResponse MasterApplication::handleGetProgress(const HttpRequest& req) {
    if (!task_manager_) {
        HttpResponse response;
        response.body = R"({"success": true, "data": {"overall_progress": 0, "pending_tasks": 0, "active_tasks": 0, "completed_tasks": 0, "failed_tasks": 0, "message": "No task session active"}})";
        return response;
    }

    double progress = task_manager_->getOverallProgress();

    std::ostringstream json;
    json << "{\n";
    json << "  \"success\": true,\n";
    json << "  \"data\": {\n";
    json << "    \"task_type\": \"" << taskStr(task_manager_->getCurrentTaskType()) << "\",\n";
    json << "    \"overall_progress\": " << std::fixed << std::setprecision(2) << (progress * 100) << ",\n";
    json << "    \"pending_tasks\": " << task_manager_->getPendingTaskCount() << ",\n";
    json << "    \"active_tasks\": " << task_manager_->getActiveTaskCount() << ",\n";
    json << "    \"completed_tasks\": " << task_manager_->getCompletedTaskCount() << ",\n";
    json << "    \"failed_tasks\": " << task_manager_->getFailedTaskCount() << "\n";
    json << "  }\n";
    json << "}";

    HttpResponse response;
    response.body = json.str();
    return response;
}

HttpResponse MasterApplication::handleGetResults(const HttpRequest& req) {
    if (!task_manager_) {
        HttpResponse response;
        response.body = R"({"success": true, "data": {"completed_results": 0, "total_processed_points": 0, "message": "No task session active"}})";
        return response;
    }

    auto results = task_manager_->getCompletedResults();

    size_t total_processed_points = 0;
    for (const auto& result : results) {
        total_processed_points += result.processed_points.size();
    }

    std::ostringstream json;
    json << "{\n";
    json << "  \"success\": true,\n";
    json << "  \"data\": {\n";
    json << "    \"task_type\": \"" << taskStr(task_manager_->getCurrentTaskType()) << "\",\n";
    json << "    \"completed_results\": " << results.size() << ",\n";
    json << "    \"total_processed_points\": " << total_processed_points << "\n";
    json << "  }\n";
    json << "}";

    HttpResponse response;
    response.body = json.str();
    return response;
}

HttpResponse MasterApplication::handleSaveResults(const HttpRequest& req) {
    if (!task_manager_) {
        return createErrorResponse(400, "No task session active");
    }

    auto it_ct = req.headers.find("Content-Type");
    std::string content_type = (it_ct != req.headers.end() ? it_ct->second : "");
    if (content_type.find("application/json") == std::string::npos) {
        return createErrorResponse(400, "Content-Type must be application/json");
    }

    auto parsed = DPApp::MiniJson::parse_object(req.body);
    if (!parsed) {
        return createErrorResponse(400, "Invalid JSON body");
    }

    auto filename0 = DPApp::MiniJson::get<std::string>(*parsed, "filename");
    if (!filename0 || filename0->empty()) {
        return createErrorResponse(400, "Missing 'filename'");
    }

    std::string filename = filename0.value();
    auto results = task_manager_->getCompletedResults();

    if (results.empty()) {
        return createErrorResponse(400, "No results to save");
    }

    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        return createErrorResponse(500, "Failed to open output file");
    }

    size_t total_points = 0;

    for (const auto& result : results) {
        for (const auto& point : result.processed_points) {
            outfile << point[0] << " " << point[1] << " " << point[2];
            outfile << "\n";
            total_points++;
        }
    }

    outfile.close();

    std::ostringstream json;
    json << "{\n";
    json << "  \"success\": true,\n";
    json << "  \"message\": \"Results saved successfully\",\n";
    json << "  \"data\": {\n";
    json << "    \"filename\": \"" << filename << "\",\n";
    json << "    \"total_points\": " << total_points << ",\n";
    json << "    \"task_type\": \"" << taskStr(task_manager_->getCurrentTaskType()) << "\"\n";
    json << "  }\n";
    json << "}";

    HttpResponse response;
    response.body = json.str();
    return response;
}

HttpResponse MasterApplication::handleShutdownServer(const HttpRequest& req) {
    std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        stop();
        }).detach();

    std::ostringstream json;
    json << "{\n";
    json << "  \"success\": true,\n";
    json << "  \"message\": \"Server shutdown initiated\"\n";
    json << "}";

    HttpResponse response;
    response.body = json.str();
    return response;
}

/// =========================================
/// Utility Functions
/// =========================================

HttpResponse MasterApplication::createErrorResponse(int status_code, const std::string& error) {
    HttpResponse response;
    response.status_code = status_code;

    switch (status_code) {
    case 400: response.status_text = "Bad Request"; break;
    case 404: response.status_text = "Not Found"; break;
    case 500: response.status_text = "Internal Server Error"; break;
    default: response.status_text = "Error"; break;
    }

    std::ostringstream json;
    json << "{\n";
    json << "  \"success\": false,\n";
    json << "  \"error\": \"" << error << "\"\n";
    json << "}";

    response.body = json.str();
    return response;
}

std::string MasterApplication::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);

    std::tm tm_utc{};
    if (gmtime_s(&tm_utc, &tt) != 0) {
        return "";
    }

    std::ostringstream oss;
    oss << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}