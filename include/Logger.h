#pragma once

#include <string>
#include <fstream>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace DPApp {

	enum class LogLevel {
		DEBUG = 0,
		INFO = 1,
		WARN = 2,
		ERROR_ = 3,
		CRITICAL = 4
	};

	/// Helper class for stream-style logging
	class LogStream {
	public:
		LogStream(LogLevel level) : level_(level) {}

		/// Perform actual logging in the destructor
		~LogStream();

		/// << operator overloading
		template<typename T>
		LogStream& operator<<(const T& value) {
			stream_ << value;
			return *this;
		}

	private:
		LogLevel level_;
		std::ostringstream stream_;
	};

	class Logger {
	public:
		static void initialize(const std::string& log_file_path = "dpapp.log") {
			std::lock_guard<std::mutex> lock(log_mutex_);
			if (!log_file_.is_open()) {
				log_file_.open(log_file_path, std::ios::app);
			}
		}

		static void log(LogLevel level, const std::string& message) {
			if (static_cast<int>(level) < static_cast<int>(min_level_)) {
				return;
			}

			auto now = std::chrono::system_clock::now();
			std::time_t t = std::chrono::system_clock::to_time_t(now);

			std::tm tm_buf{}; /// Buffer for storing results
#if defined(_WIN32)
			/// Recommended for MSVC: thread-safe
			localtime_s(&tm_buf, &t);
#else
			/// POSIX: thread-safe
			localtime_r(&t, &tm_buf);
#endif

			std::lock_guard<std::mutex> lock(log_mutex_);

			/// Console output
			std::cout << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S")
				<< " [" << levelToString(level) << "] "
				<< message << std::endl;

			/// File output
			if (log_file_.is_open()) {
				log_file_ << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S")
					<< " [" << levelToString(level) << "] "
					<< message << std::endl;
				log_file_.flush();
			}
		}

		/// Functions for stream-style logging
		static LogStream debug() { return LogStream(LogLevel::DEBUG); }
		static LogStream info() { return LogStream(LogLevel::INFO); }
		static LogStream warn() { return LogStream(LogLevel::WARN); }
		static LogStream error() { return LogStream(LogLevel::ERROR_); }
		static LogStream critical() { return LogStream(LogLevel::CRITICAL); }

		static void setMinLevel(LogLevel level) { min_level_ = level; }

		static void shutdown() {
			std::lock_guard<std::mutex> lock(log_mutex_);
			if (log_file_.is_open()) {
				log_file_.close();
			}
		}

	private:
		static std::string levelToString(LogLevel level) {
			switch (level) {
			case LogLevel::DEBUG:    return "DEBUG";
			case LogLevel::INFO:     return "INFO";
			case LogLevel::WARN:     return "WARN";
			case LogLevel::ERROR_:    return "ERROR";
			case LogLevel::CRITICAL: return "CRITICAL";
			default:                 return "UNKNOWN";
			}
		}

		inline static std::mutex log_mutex_;
		inline static std::ofstream log_file_;
		inline static LogLevel min_level_ = LogLevel::INFO;
	};

	/// LogStream destructor implementation (after Logger is fully defined)
	inline LogStream::~LogStream() {
		Logger::log(level_, stream_.str());
	}

	/// Existing macros + Stream-style macros
#define LOG_DEBUG(msg) DPApp::Logger::log(DPApp::LogLevel::DEBUG, msg)
#define LOG_INFO(msg) DPApp::Logger::log(DPApp::LogLevel::INFO, msg)
#define LOG_WARN(msg) DPApp::Logger::log(DPApp::LogLevel::WARN, msg)
#define LOG_ERROR(msg) DPApp::Logger::log(DPApp::LogLevel::ERROR_, msg)
#define LOG_CRITICAL(msg) DPApp::Logger::log(DPApp::LogLevel::CRITICAL, msg)

/// Stream-style macros
#define DLOG DPApp::Logger::debug()
#define ILOG DPApp::Logger::info()
#define WLOG DPApp::Logger::warn()
#define ELOG DPApp::Logger::error()
#define CLOG DPApp::Logger::critical()

} /// namespace DPApp