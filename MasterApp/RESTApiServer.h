// RestApiServer.h - Header-Only REST API Server
#pragma once

#include "RuntimeConfig.h"
#include "ThreadPool.h"
#include "MiniJson.h"

#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <map>
#include <vector>
#include <mutex>
#include <memory>
#include <regex>
#include <iostream>
#include <sstream>
#include <cstring>

// Windows-specific includes
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#endif

struct SecurityConfig {
	bool enable_auth = true;
	std::string api_key;
	std::vector<std::string> allowed_origins;
	size_t max_request_size = 64 * 1024;  /// 64KB
	size_t max_json_depth = 10;
	size_t max_filename_length = 1024;    /// File path length, 1MB
};

struct HttpRequest {
	std::string method;
	std::string path;
	std::string version;
	std::map<std::string, std::string> headers;
	std::string body;
	std::map<std::string, std::string> query_params;
	std::map<std::string, std::string> path_params;
	bool is_valid = true;
	std::string parse_error;
};

struct HttpResponse {
	int status_code = 200;
	std::string status_text = "OK";
	std::map<std::string, std::string> headers;
	std::string body;

	HttpResponse() {
		headers["Content-Type"] = "application/json";
		headers["Access-Control-Allow-Origin"] = "*";
		headers["Access-Control-Allow-Methods"] = "GET, POST, PUT, DELETE, OPTIONS";
		headers["Access-Control-Allow-Headers"] = "Content-Type, Authorization";
	}
};

class RestApiServer {
public:
	using RouteHandler = std::function<HttpResponse(const HttpRequest&)>;

private:
	DPApp::RuntimeConfig cfg_;
	std::unique_ptr<DPApp::ThreadPool> pool_;
	SecurityConfig security_config_;

public:
	RestApiServer() : running_(false), port_(0), winsock_initialized_(false) {
		cfg_ = DPApp::RuntimeConfig::loadFromEnv();
		pool_ = std::make_unique<DPApp::ThreadPool>(static_cast<size_t>(cfg_.rest_threadpool_size));

		/// Security configuration
		if (cfg_.api_key == "") {
			security_config_.enable_auth = false;
			security_config_.api_key = "";
		}
		else {
			security_config_.enable_auth = true;
			security_config_.api_key = cfg_.api_key;
		}

#ifdef _WIN32
		server_socket_ = INVALID_SOCKET;
#else
		server_socket_ = -1;
#endif
	}

	~RestApiServer() {
		stop();
		cleanupWinsock();
	}

	bool start(uint16_t port) {
		if (running_) return false;

		initializeWinsock();

		port_ = port;
		server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
#ifdef _WIN32
		if (server_socket_ == INVALID_SOCKET) {
#else
		if (server_socket_ < 0) {
#endif
			std::cerr << "Failed to create REST API server socket" << std::endl;
			return false;
		}

#ifdef _WIN32
		char opt = 1;
		setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#else
		int opt = 1;
		setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#endif

		sockaddr_in server_addr{};
		server_addr.sin_family = AF_INET;
		server_addr.sin_addr.s_addr = INADDR_ANY;
		server_addr.sin_port = htons(port_);

		if (bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
			std::cerr << "Failed to bind REST API server socket" << std::endl;
			closeSocket(server_socket_);
			return false;
		}

		if (listen(server_socket_, cfg_.server_backlog) < 0) {
			perror("Listen failed");
			return false;
		}

		running_ = true;
		server_thread_ = std::thread(&RestApiServer::serverLoop, this);

		std::cout << "REST API Server started on port " << port_ << std::endl;
		return true;
	}

	void stop() {
		if (!running_) return;

		running_ = false;

#ifdef _WIN32
		if (server_socket_ != INVALID_SOCKET) {
#else
		if (server_socket_ >= 0) {
#endif
			closeSocket(server_socket_);
			if (pool_) pool_->shutdown();
#ifdef _WIN32
			server_socket_ = INVALID_SOCKET;
#else
			server_socket_ = -1;
#endif
		}

		if (server_thread_.joinable()) {
			server_thread_.join();
		}
	}

	void GET(const std::string & pattern, RouteHandler handler) {
		addRoute("GET", pattern, handler);
	}

	void POST(const std::string & pattern, RouteHandler handler) {
		addRoute("POST", pattern, handler);
	}

	void PUT(const std::string & pattern, RouteHandler handler) {
		addRoute("PUT", pattern, handler);
	}

	void DEL(const std::string & pattern, RouteHandler handler) {
		addRoute("DELETE", pattern, handler);
	}

	inline HttpResponse createErrorResponse(int status_code, const std::string & error) {
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

private:
	struct Route {
		std::string method;
		std::regex pattern;
		std::vector<std::string> param_names;
		RouteHandler handler;
	};

	std::atomic<bool> running_;
	uint16_t port_;
#ifdef _WIN32
	SOCKET server_socket_;
#else
	int server_socket_;
#endif
	std::thread server_thread_;
	std::vector<Route> routes_;
	std::mutex routes_mutex_;
	bool winsock_initialized_;

private:
	inline void sendErrorAndClose(int client_socket, int status_code, const std::string & error) {
		HttpResponse error_response = createErrorResponse(status_code, error);
		std::string response_str = buildResponse(error_response);
		send(client_socket, response_str.c_str(), static_cast<int>(response_str.length()), 0);
		closeSocket(client_socket);
	}

	inline bool authenticateRequest(const HttpRequest & req) {
		auto auth_header = req.headers.find("Authorization");

		if (auth_header == req.headers.end()) {
			return false;
		}

		/// Validate API key
		std::string expected = "Bearer " + security_config_.api_key;
		return auth_header->second == expected;
	}

	inline void initializeWinsock() {
#ifdef _WIN32
		if (!winsock_initialized_) {
			WSADATA wsaData;
			int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
			if (result != 0) {
				std::cerr << "WSAStartup failed: " << result << std::endl;
			}
			else {
				winsock_initialized_ = true;
			}
		}
#endif
	}

	inline void cleanupWinsock() {
#ifdef _WIN32
		if (winsock_initialized_) {
			WSACleanup();
			winsock_initialized_ = false;
		}
#endif
	}

#ifdef _WIN32
	inline void closeSocket(SOCKET sock) {
		closesocket(sock);
	}
#else
	inline void closeSocket(int sock) {
		close(sock);
	}
#endif

	inline void addRoute(const std::string & method, const std::string & pattern, RouteHandler handler) {
		std::lock_guard<std::mutex> lock(routes_mutex_);

		Route route;
		route.method = method;
		route.handler = handler;

		// Extract parameters from pattern (e.g., /api/slaves/{id})
		std::string regex_pattern = pattern;
		std::regex param_regex(R"(\{([^}]+)\})");
		std::sregex_iterator iter(pattern.begin(), pattern.end(), param_regex);
		std::sregex_iterator end;

		for (; iter != end; ++iter) {
			route.param_names.push_back(iter->str(1));
			regex_pattern = std::regex_replace(regex_pattern, param_regex, "([^/]+)", std::regex_constants::format_first_only);
		}

		route.pattern = std::regex("^" + regex_pattern + "$");
		routes_.push_back(route);
	}

	inline void serverLoop() {
		while (running_) {
			sockaddr_in client_addr{};
#ifdef _WIN32
			int client_len = sizeof(client_addr);
			SOCKET client_socket = accept(server_socket_, (struct sockaddr*)&client_addr, &client_len);
			if (client_socket == INVALID_SOCKET) {
				if (!running_) break;
				continue;
			}
#else
			socklen_t client_len = sizeof(client_addr);
			int client_socket = accept(server_socket_, (struct sockaddr*)&client_addr, &client_len);
			if (client_socket < 0) {
				if (!running_) break;
				continue;
			}
#endif
			pool_->enqueue([this, client_socket]() {
				this->handleClient(client_socket);
				});
		}
	}

#ifdef _WIN32
	inline void handleClient(SOCKET client_socket) {
#else
	inline void handleClient(int client_socket) {
#endif
		// recv timeout (slow client 방어)
		if (cfg_.rest_socket_recv_timeout_ms > 0) {
#ifdef _WIN32
			DWORD tv = static_cast<DWORD>(cfg_.rest_socket_recv_timeout_ms);
			::setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
#else
			struct timeval tv;
			tv.tv_sec = cfg_.rest_socket_recv_timeout_ms / 1000;
			tv.tv_usec = (cfg_.rest_socket_recv_timeout_ms % 1000) * 1000;
			::setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif
		}

		char buffer[8192];
		int bytes_received = recv(client_socket, buffer, sizeof(buffer) - 1, 0);

		/// Check the length of bytes_received
		if (bytes_received > static_cast<int>(security_config_.max_request_size)) {
			sendErrorAndClose(static_cast<int>(client_socket), 413, "Request too large");
			return;
		}

		if (bytes_received > 0) {
			buffer[bytes_received] = '\0';
			std::string request_str(buffer);

			HttpRequest request = parseRequest(request_str);
			HttpResponse response = routeRequest(request);

			std::string response_str = buildResponse(response);
			send(client_socket, response_str.c_str(), static_cast<int>(response_str.length()), 0);
		}

		closeSocket(client_socket);
	}

	inline HttpRequest parseRequest(const std::string & request_str) {
		HttpRequest request;
		request.is_valid = true;

		/// 1. Check empty request
		if (request_str.empty()) {
			request.is_valid = false;
			request.parse_error = "Empty request";
			return request;
		}

		std::istringstream iss(request_str);
		std::string line;

		/// 2. Failure in reading the first line
		if (!std::getline(iss, line)) {
			request.is_valid = false;
			request.parse_error = "Cannot read request line";
			return request;
		}

		/// 2. Parse the first line
		std::istringstream first_line(line);
		first_line >> request.method >> request.path >> request.version;
		/// Check empty method
		if (request.method.empty()) {
			request.is_valid = false;
			request.parse_error = "Missing HTTP method";
			return request;
		}
		/// Check empty path
		if (request.path.empty()) {
			request.is_valid = false;
			request.parse_error = "Missing request path";
			return request;
		}
		/// Check the availability of method
		if (request.method != "GET" && request.method != "POST" &&
			request.method != "PUT" && request.method != "DELETE" &&
			request.method != "OPTIONS") {
			request.is_valid = false;
			request.parse_error = "Unsupported HTTP method: " + request.method;
			return request;
		}

		// Parse query parameters
		size_t query_pos = request.path.find('?');
		if (query_pos != std::string::npos) {
			std::string query = request.path.substr(query_pos + 1);
			request.path = request.path.substr(0, query_pos);
			request.query_params = parseQueryString(query);
		}

		/// 5. Parse headers
		while (std::getline(iss, line) && line != "\r" && !line.empty()) {
			size_t colon_pos = line.find(':');

			if (colon_pos == std::string::npos) {
				request.is_valid = false;
				request.parse_error = "Invalid header format: " + line;
				return request;
			}

			std::string key = line.substr(0, colon_pos);
			std::string value = line.substr(colon_pos + 1);

			/// key 공백 제거
			key.erase(0, key.find_first_not_of(' '));
			key.erase(key.find_last_not_of(' ') + 1);

			if (key.empty()) {
				request.is_valid = false;
				request.parse_error = "Empty header key";
				return request;
			}

			/// value 공백 제거
			value.erase(0, value.find_first_not_of(' '));
			value.erase(value.find_last_not_of("\r ") + 1);

			request.headers[key] = value;
		}

		/// Parse body
		std::string body_line;
		while (std::getline(iss, body_line)) {
			request.body += body_line + "\n";
		}

		if (!request.body.empty()) {
			request.body.pop_back(); // Remove last newline
		}

		/// 6. Validate content-Length (not essential)
		auto content_length_it = request.headers.find("Content-Length");
		if (content_length_it != request.headers.end()) {
			try {
				size_t expected_length = std::stoull(content_length_it->second);
				if (request.body.length() != expected_length) {
					request.is_valid = false;
					request.parse_error = "Content-Length mismatch";
					return request;
				}
			}
			catch (const std::exception&) {
				request.is_valid = false;
				request.parse_error = "Invalid Content-Length header";
				return request;
			}
		}

		return request;
	}

	inline std::string buildResponse(const HttpResponse & response) {
		std::ostringstream oss;

		oss << "HTTP/1.1 " << response.status_code << " " << response.status_text << "\r\n";

		for (const auto& header : response.headers) {
			oss << header.first << ": " << header.second << "\r\n";
		}

		oss << "Content-Length: " << response.body.length() << "\r\n";
		oss << "\r\n";
		oss << response.body;

		return oss.str();
	}

	inline HttpResponse routeRequest(const HttpRequest & request) {
		if (!request.is_valid) {
			return createErrorResponse(400, request.parse_error);
		}

		if (request.method.empty()) {
			return createErrorResponse(400, "Invalid request format");
		}

		std::lock_guard<std::mutex> lock(routes_mutex_);

		// Handle OPTIONS requests (CORS preflight)
		if (request.method == "OPTIONS") {
			HttpResponse response;
			response.status_code = 200;
			response.body = "";
			return response;
		}

		if (security_config_.enable_auth && !authenticateRequest(request)) {
			return createErrorResponse(401, "Unauthorized");
		}

		for (const auto& route : routes_) {
			if (route.method == request.method) {
				HttpRequest matched_request = request;
				if (matchRoute(route, request, matched_request)) {
					return route.handler(matched_request);
				}
			}
		}

		return createErrorResponse(404, "Endpoint not found");
	}

	inline bool matchRoute(const Route & route, const HttpRequest & request, HttpRequest & matched_request) {
		std::smatch matches;
		if (std::regex_match(request.path, matches, route.pattern)) {
			matched_request = request;

			// Extract path parameters
			for (size_t i = 0; i < route.param_names.size() && i + 1 < matches.size(); ++i) {
				matched_request.path_params[route.param_names[i]] = matches[i + 1].str();
			}

			return true;
		}
		return false;
	}

	inline std::map<std::string, std::string> parseQueryString(const std::string & query) {
		std::map<std::string, std::string> params;
		std::istringstream iss(query);
		std::string pair;

		while (std::getline(iss, pair, '&')) {
			size_t eq_pos = pair.find('=');
			if (eq_pos != std::string::npos) {
				std::string key = pair.substr(0, eq_pos);
				std::string value = pair.substr(eq_pos + 1);
				params[key] = value;
			}
		}

		return params;
	}
};