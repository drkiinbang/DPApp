#pragma once
#include <string>
#include <cstdint>
#include <cstdlib>

namespace DPApp {
	/// 환경변수에서 정수 읽기 헬퍼
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

	/// 환경변수에서 문자열 읽기
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
		int server_backlog = 16; // 소켓 listen backlog
		int server_io_threads = 0; // 0이면 하드웨어 스레드 수 기반 자동 결정

		/// Heartbeat & Task
		int heartbeat_timeout_seconds = 30; // 클라이언트 heartbeat 타임아웃
		int task_timeout_seconds = 3000; // 작업 처리 타임아웃

		/// REST API
		int rest_threadpool_size = 8; // REST 요청 처리 풀 크기
		int rest_socket_recv_timeout_ms = 10000; // 요청 수신 타임아웃 (밀리초)

		std::string api_key = "";

		/// 로깅 레벨 등 확장 가능
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