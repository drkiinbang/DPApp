#pragma once
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>

namespace DPApp {
	class ThreadPool {
	public:
		explicit ThreadPool(size_t thread_count) {
			if (thread_count == 0) thread_count = 1;
			workers_.reserve(thread_count);
			for (size_t i = 0; i < thread_count; ++i) {
				workers_.emplace_back([this]() {
					for (;;) {
						std::function<void()> job;
						{
							std::unique_lock<std::mutex> lock(mtx_);
							cv_.wait(lock, [this] { return stopping_.load() || !tasks_.empty(); });
							if (stopping_.load() && tasks_.empty()) return;
							job = std::move(tasks_.front());
							tasks_.pop();
						}
						job();
					}
					});
			}
		}

		~ThreadPool() {
			shutdown();
		}

		ThreadPool(const ThreadPool&) = delete;
		ThreadPool& operator=(const ThreadPool&) = delete;

		void enqueue(std::function<void()> fn) {
			{
				std::lock_guard<std::mutex> lock(mtx_);
				if (stopping_.load()) return;
				tasks_.push(std::move(fn));
			}
			cv_.notify_one();
		}

		void shutdown() {
			bool expected = false;
			if (stopping_.compare_exchange_strong(expected, true)) {
				cv_.notify_all();
				for (auto& t : workers_) {
					if (t.joinable()) t.join();
				}
			}
		}

	private:
		std::vector<std::thread> workers_;
		std::queue<std::function<void()>> tasks_;

		std::mutex mtx_;
		std::condition_variable cv_;
		std::atomic<bool> stopping_{ false };
	};
} /// namespace DPApp