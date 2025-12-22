#pragma once
#include <string>
#include <cstdint>
#include <cstdlib>

namespace DPApp {
	/// Helper to read integers from environment variables
	inline int getenv_int(const char* name, int default_value) {
		int value = default_value;

#ifdef _WIN32
		char* buffer = nullptr;
		size_t len = 0;

		if (_dupenv_s(&buffer, &len, name) == 0 && buffer != nullptr) {
			try {
				value = std::stoi(buffer);
				free(buffer);
			}
			catch (...) {
				value = default_value;
				free(buffer);
			}
		}
#else
		const char* env_value = std::getenv(name);
		if (env_value != nullptr) {
			try {
				value = std::stoi(env_value);
			}
			catch (...) {
				value = default_value;
			}
		}
#endif

		return value;
	};

	/// Read strings from environment variables
	inline std::string getenv_string(const char* name, const std::string& default_value) {
		std::string value = default_value;
#ifdef _WIN32
		char* buffer = nullptr;
		size_t len = 0;
		if (_dupenv_s(&buffer, &len, name) == 0 && buffer != nullptr) {
			value = std::string(buffer);
			free(buffer);
		}
#else
		const char* env_value = std::getenv(name);
		if (env_value != nullptr) {
			value = std::string(env_value);
		}
#endif
		return value;
	}

	struct RuntimeConfig {
		/// Network/Server
		int server_backlog = 16; /// socket listen backlog
		int server_io_threads = 0; /// 0 means automatically determined based on hardware thread count

		/// Heartbeat & Task
		int heartbeat_timeout_seconds = 30; /// client heartbeat timeout
		int task_timeout_seconds = 3000; /// task processing timeout

		/// REST API
		int rest_threadpool_size = 8; /// REST request processing pool size
		int rest_socket_recv_timeout_ms = 10000; /// request reception timeout (milliseconds)

		std::string api_key = "";

		/// Logging level, etc. can be extended
		static RuntimeConfig loadFromEnv() {
			RuntimeConfig cfg;
			cfg.server_backlog = getenv_int("DPAPP_SERVER_BACKLOG", cfg.server_backlog);
			cfg.server_io_threads = getenv_int("DPAPP_SERVER_IO_THREADS", cfg.server_io_threads);
			cfg.heartbeat_timeout_seconds = getenv_int("DPAPP_HEARTBEAT_TIMEOUT", cfg.heartbeat_timeout_seconds);
			cfg.task_timeout_seconds = getenv_int("DPAPP_TASK_TIMEOUT", cfg.task_timeout_seconds);
			cfg.rest_threadpool_size = getenv_int("DPAPP_REST_THREADS", cfg.rest_threadpool_size);
			cfg.rest_socket_recv_timeout_ms = getenv_int("DPAPP_REST_RECV_TIMEOUT_MS", cfg.rest_socket_recv_timeout_ms);
			cfg.api_key = getenv_string("DPAPP_API_KEY", "");

			if (cfg.server_io_threads <= 0) {
				cfg.server_io_threads = static_cast<int>((std::max)(2u, std::thread::hardware_concurrency()));
			}

			if (cfg.rest_threadpool_size <= 0) {
				cfg.rest_threadpool_size = static_cast<int>((std::max)(4u, std::thread::hardware_concurrency()));
			}

			return cfg;
		}
	};
} /// namespace DPApp