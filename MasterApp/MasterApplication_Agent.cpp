/**
 * @file MasterApplication_Agent.cpp
 * @brief Agent management and Slave control implementation
 * 
 * This file contains:
 * - HTTP client for Agent communication
 * - Agent configuration loading
 * - Agent management (start/stop slaves)
 * - Agent REST API handlers
 * - Slave Pause/Resume handlers
 */

#include "MasterApplication.h"
#include "../include/MiniJson.h"

/// =========================================
/// HTTP Client for Agent Communication
/// =========================================

HttpClientResponse MasterApplication::httpPost(const std::string& host, uint16_t port,
    const std::string& path, const std::string& body_content) {
    HttpClientResponse result;

    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        ELOG << "WSAStartup failed";
        return result;
    }

    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) {
        ELOG << "Failed to create socket";
        WSACleanup();
        return result;
    }

    DWORD timeout = 10000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));

    sockaddr_in server;
    ZeroMemory(&server, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port = htons(port);
    inet_pton(AF_INET, host.c_str(), &server.sin_addr);

    if (connect(sock, (sockaddr*)&server, sizeof(server)) == SOCKET_ERROR) {
        ELOG << "Failed to connect to " << host << ":" << port;
        closesocket(sock);
        WSACleanup();
        return result;
    }

    std::ostringstream request;
    request << "POST " << path << " HTTP/1.1\r\n";
    request << "Host: " << host << ":" << port << "\r\n";
    request << "Content-Type: application/json\r\n";
    request << "Content-Length: " << body_content.length() << "\r\n";
    request << "Connection: close\r\n";
    request << "\r\n";
    request << body_content;

    std::string req_str = request.str();
    if (send(sock, req_str.c_str(), static_cast<int>(req_str.length()), 0) == SOCKET_ERROR) {
        closesocket(sock);
        WSACleanup();
        return result;
    }

    std::string response_data;
    char buffer[4096];
    int bytes_received;

    while ((bytes_received = recv(sock, buffer, sizeof(buffer) - 1, 0)) > 0) {
        buffer[bytes_received] = '\0';
        response_data += buffer;
    }

    closesocket(sock);
    WSACleanup();

    if (!response_data.empty()) {
        size_t status_pos = response_data.find(" ");
        if (status_pos != std::string::npos) {
            try {
                result.status_code = std::stoi(response_data.substr(status_pos + 1, 3));
            }
            catch (...) {}
        }

        size_t body_pos = response_data.find("\r\n\r\n");
        if (body_pos != std::string::npos) {
            result.body = response_data.substr(body_pos + 4);
        }

        result.success = (result.status_code >= 200 && result.status_code < 300);
    }

    return result;
}

HttpClientResponse MasterApplication::httpGet(const std::string& host, uint16_t port, const std::string& path) {
    HttpClientResponse result;

    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        return result;
    }

    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) {
        WSACleanup();
        return result;
    }

    DWORD timeout = 10000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));

    sockaddr_in server;
    ZeroMemory(&server, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port = htons(port);
    inet_pton(AF_INET, host.c_str(), &server.sin_addr);

    if (connect(sock, (sockaddr*)&server, sizeof(server)) == SOCKET_ERROR) {
        closesocket(sock);
        WSACleanup();
        return result;
    }

    std::ostringstream request;
    request << "GET " << path << " HTTP/1.1\r\n";
    request << "Host: " << host << ":" << port << "\r\n";
    request << "Connection: close\r\n";
    request << "\r\n";

    std::string req_str = request.str();
    send(sock, req_str.c_str(), static_cast<int>(req_str.length()), 0);

    std::string response_data;
    char buffer[4096];
    int bytes_received;

    while ((bytes_received = recv(sock, buffer, sizeof(buffer) - 1, 0)) > 0) {
        buffer[bytes_received] = '\0';
        response_data += buffer;
    }

    closesocket(sock);
    WSACleanup();

    if (!response_data.empty()) {
        size_t status_pos = response_data.find(" ");
        if (status_pos != std::string::npos) {
            try {
                result.status_code = std::stoi(response_data.substr(status_pos + 1, 3));
            }
            catch (...) {}
        }

        size_t body_pos = response_data.find("\r\n\r\n");
        if (body_pos != std::string::npos) {
            result.body = response_data.substr(body_pos + 4);
        }

        result.success = (result.status_code >= 200 && result.status_code < 300);
    }

    return result;
}

/// =========================================
/// Agent Configuration
/// =========================================

bool MasterApplication::loadAgentsFromConfig(const std::string& config_path) {
    std::ifstream file(config_path);
    if (!file.is_open()) {
        WLOG << "Agents config file not found: " << config_path;
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();

    /// Use extended parser (supports arrays and nested objects)
    auto parsed = MiniJson::parse_object_ex(buffer.str());
    if (!parsed) {
        ELOG << "Invalid JSON in agents config";
        return false;
    }

    /// Get master_external_ip
    if (auto v = MiniJson::get_ex<std::string>(*parsed, "master_external_ip")) {
        master_external_ip_ = *v;
    }

    /// Get agents array
    auto agents_arr = MiniJson::get_array(*parsed, "agents");
    if (!agents_arr) {
        WLOG << "Missing 'agents' array in config";
        return false;
    }

    std::lock_guard<std::mutex> lock(agents_mutex_);
    agents_.clear();

    for (const auto& agent_val : *agents_arr) {
        if (!agent_val.is_object()) continue;

        const JsonObjectEx& obj = agent_val.as_object();

        AgentInfo agent;
        if (auto v = MiniJson::get_ex<std::string>(obj, "agent_id")) agent.agent_id = *v;
        if (auto v = MiniJson::get_ex<std::string>(obj, "host")) agent.host = *v;
        if (auto v = MiniJson::get_ex<double>(obj, "port")) agent.port = static_cast<uint16_t>(*v);

        if (!agent.host.empty()) {
            if (agent.agent_id.empty()) {
                agent.agent_id = agent.host + ":" + std::to_string(agent.port);
            }
            agent.online = false;
            agents_.push_back(agent);
            ILOG << "Loaded agent: " << agent.agent_id
                << " (" << agent.host << ":" << agent.port << ")";
        }
    }

    ILOG << "Loaded " << agents_.size() << " agents from config";
    return true;
}

/// =========================================
/// Agent Management Functions
/// =========================================

bool MasterApplication::isAgentOnline(const std::string& host, uint16_t port) {
    auto response = httpGet(host, port, "/api/health");
    return response.success;
}

bool MasterApplication::startSlaveOnAgent(const std::string& host, uint16_t port, uint32_t threads) {
    std::ostringstream body;
    body << "{\n";
    body << "  \"master_address\": \"" << master_external_ip_ << "\",\n";
    body << "  \"master_port\": " << server_port_ << ",\n";
    body << "  \"threads\": " << threads << "\n";
    body << "}";

    auto response = httpPost(host, port, "/api/slaves/start", body.str());

    if (response.success) {
        ILOG << "Started slave on agent " << host << ":" << port;
        return true;
    }
    else {
        ELOG << "Failed to start slave on agent " << host << ":" << port;
        return false;
    }
}

bool MasterApplication::stopSlavesOnAgent(const std::string& host, uint16_t port, bool force) {
    std::string body = force ? R"({"force": true})" : R"({})";
    auto response = httpPost(host, port, "/api/slaves/stop-all", body);

    if (response.success) {
        ILOG << "Stopped slaves on agent " << host << ":" << port;
        return true;
    }
    else {
        ELOG << "Failed to stop slaves on agent " << host << ":" << port;
        return false;
    }
}

int MasterApplication::startSlavesOnAllAgents(uint32_t threads) {
    std::lock_guard<std::mutex> lock(agents_mutex_);

    int started = 0;
    for (auto& agent : agents_) {
        if (startSlaveOnAgent(agent.host, agent.port, threads)) {
            agent.online = true;
            started++;
        }
    }
    return started;
}

int MasterApplication::stopSlavesOnAllAgents(bool force) {
    std::lock_guard<std::mutex> lock(agents_mutex_);

    int stopped = 0;
    for (auto& agent : agents_) {
        if (stopSlavesOnAgent(agent.host, agent.port, force)) {
            stopped++;
        }
    }
    return stopped;
}

/// =========================================
/// Agent REST API Handlers
/// =========================================

HttpResponse MasterApplication::handleGetAgents(const HttpRequest& req) {
    std::lock_guard<std::mutex> lock(agents_mutex_);

    std::ostringstream json;
    json << "{\n  \"success\": true,\n  \"data\": {\n";
    json << "    \"count\": " << agents_.size() << ",\n";
    json << "    \"agents\": [\n";

    for (size_t i = 0; i < agents_.size(); ++i) {
        const auto& a = agents_[i];
        json << "      {";
        json << "\"agent_id\": \"" << a.agent_id << "\", ";
        json << "\"host\": \"" << a.host << "\", ";
        json << "\"port\": " << a.port << ", ";
        json << "\"online\": " << (a.online ? "true" : "false");
        json << "}" << (i + 1 < agents_.size() ? "," : "") << "\n";
    }

    json << "    ]\n  }\n}";

    HttpResponse response;
    response.body = json.str();
    return response;
}

HttpResponse MasterApplication::handleAddAgent(const HttpRequest& req) {
    auto parsed = MiniJson::parse_object(req.body);
    if (!parsed) {
        return createErrorResponse(400, "Invalid JSON");
    }

    AgentInfo agent;
    if (auto v = MiniJson::get<std::string>(*parsed, "agent_id")) agent.agent_id = *v;
    if (auto v = MiniJson::get<std::string>(*parsed, "host")) agent.host = *v;
    if (auto v = MiniJson::get<double>(*parsed, "port")) agent.port = static_cast<uint16_t>(*v);

    if (agent.host.empty()) {
        return createErrorResponse(400, "Missing 'host'");
    }

    if (agent.agent_id.empty()) {
        agent.agent_id = agent.host + ":" + std::to_string(agent.port);
    }

    agent.online = isAgentOnline(agent.host, agent.port);

    {
        std::lock_guard<std::mutex> lock(agents_mutex_);
        agents_.push_back(agent);
    }

    std::ostringstream json;
    json << "{\n  \"success\": true,\n";
    json << "  \"message\": \"Agent added\",\n";
    json << "  \"data\": {\n";
    json << "    \"agent_id\": \"" << agent.agent_id << "\",\n";
    json << "    \"online\": " << (agent.online ? "true" : "false") << "\n";
    json << "  }\n}";

    HttpResponse response;
    response.body = json.str();
    return response;
}

HttpResponse MasterApplication::handleRemoveAgent(const HttpRequest& req) {
    std::string agent_id = req.path_params.at("id");

    {
        std::lock_guard<std::mutex> lock(agents_mutex_);
        auto it = std::find_if(agents_.begin(), agents_.end(),
            [&agent_id](const AgentInfo& a) { return a.agent_id == agent_id; });

        if (it == agents_.end()) {
            return createErrorResponse(404, "Agent not found");
        }

        agents_.erase(it);
    }

    HttpResponse response;
    response.body = R"({"success": true, "message": "Agent removed"})";
    return response;
}

HttpResponse MasterApplication::handleTestAgent(const HttpRequest& req) {
    std::string agent_id = req.path_params.at("id");

    std::lock_guard<std::mutex> lock(agents_mutex_);
    auto it = std::find_if(agents_.begin(), agents_.end(),
        [&agent_id](const AgentInfo& a) { return a.agent_id == agent_id; });

    if (it == agents_.end()) {
        return createErrorResponse(404, "Agent not found");
    }

    bool online = isAgentOnline(it->host, it->port);
    it->online = online;

    std::ostringstream json;
    json << "{\n  \"success\": true,\n";
    json << "  \"data\": {\n";
    json << "    \"agent_id\": \"" << it->agent_id << "\",\n";
    json << "    \"online\": " << (online ? "true" : "false") << "\n";
    json << "  }\n}";

    HttpResponse response;
    response.body = json.str();
    return response;
}

HttpResponse MasterApplication::handleStartSlavesOnAgents(const HttpRequest& req) {
    uint32_t threads = 4;

    auto parsed = MiniJson::parse_object(req.body);
    if (parsed) {
        if (auto v = MiniJson::get<double>(*parsed, "threads")) {
            threads = static_cast<uint32_t>(*v);
        }
    }

    int started = startSlavesOnAllAgents(threads);

    std::ostringstream json;
    json << "{\n  \"success\": true,\n";
    json << "  \"message\": \"Start command sent to agents\",\n";
    json << "  \"data\": { \"started_count\": " << started << " }\n";
    json << "}";

    HttpResponse response;
    response.body = json.str();
    return response;
}

HttpResponse MasterApplication::handleStopSlavesOnAgents(const HttpRequest& req) {
    bool force = false;

    auto parsed = MiniJson::parse_object(req.body);
    if (parsed) {
        if (auto v = MiniJson::get<bool>(*parsed, "force")) {
            force = *v;
        }
    }

    int stopped = stopSlavesOnAllAgents(force);

    std::ostringstream json;
    json << "{\n  \"success\": true,\n";
    json << "  \"message\": \"Stop command sent to agents\",\n";
    json << "  \"data\": { \"stopped_count\": " << stopped << " }\n";
    json << "}";

    HttpResponse response;
    response.body = json.str();
    return response;
}

/// =========================================
/// Slave Pause/Resume REST Handlers
/// =========================================

HttpResponse MasterApplication::handlePauseSlave(const HttpRequest& req) {
    std::string slave_id = req.path_params.at("id");

    NetworkMessage msg(MessageType::PAUSE, {});

    if (server_->sendMessage(slave_id, msg)) {
        HttpResponse response;
        response.body = R"({"success": true, "message": "Pause command sent"})";
        return response;
    }

    return createErrorResponse(500, "Failed to send pause command");
}

HttpResponse MasterApplication::handleResumeSlave(const HttpRequest& req) {
    std::string slave_id = req.path_params.at("id");

    NetworkMessage msg(MessageType::RESUME, {});

    if (server_->sendMessage(slave_id, msg)) {
        HttpResponse response;
        response.body = R"({"success": true, "message": "Resume command sent"})";
        return response;
    }

    return createErrorResponse(500, "Failed to send resume command");
}

HttpResponse MasterApplication::handlePauseAllSlaves(const HttpRequest& req) {
    auto clients = server_->getConnectedClients();
    NetworkMessage msg(MessageType::PAUSE, {});

    int count = 0;
    for (const auto& client_id : clients) {
        if (server_->sendMessage(client_id, msg)) count++;
    }

    std::ostringstream json;
    json << "{\n  \"success\": true,\n";
    json << "  \"data\": { \"paused_count\": " << count << " }\n";
    json << "}";

    HttpResponse response;
    response.body = json.str();
    return response;
}

HttpResponse MasterApplication::handleResumeAllSlaves(const HttpRequest& req) {
    auto clients = server_->getConnectedClients();
    NetworkMessage msg(MessageType::RESUME, {});

    int count = 0;
    for (const auto& client_id : clients) {
        if (server_->sendMessage(client_id, msg)) count++;
    }

    std::ostringstream json;
    json << "{\n  \"success\": true,\n";
    json << "  \"data\": { \"resumed_count\": " << count << " }\n";
    json << "}";

    HttpResponse response;
    response.body = json.str();
    return response;
}
