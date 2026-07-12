#pragma once
/**
 * @file TaskManagerTypes.h
 * @brief TaskManager 클래스 선언 및 인라인 구현
 *
 * 이 헤더는 header-only 라이브러리(tinygltf, stb_image)를 포함하지 않습니다.
 * TaskManager의 메서드를 호출해야 하는 파일에서 이 헤더를 include하세요.
 */

#ifdef _WIN32
#define NOMINMAX
#endif

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <atomic>
#include <functional>
#include <iostream>
#include <algorithm>
#include <optional>

#include "PointCloudTypes.h"
#include "IcpTypes.h"

namespace DPApp {

    /**
     * @brief 태스크 실행 상태 열거형
     */
    enum class TaskStatus {
        PENDING,
        ASSIGNED,
        IN_PROGRESS,
        COMPLETED,
        FAILED,
        TIMEOUT
    };

    /**
     * @brief 태스크 우선순위 레벨
     */
    enum class TaskPriority {
        LOW = 0,
        NORMAL = 1,
        HIGH = 2
    };

    /**
     * @brief 범용 태스크 정보 구조체
     */
    struct TaskInfo {
        uint32_t task_id;
        uint32_t chunk_id;
        TaskType task_type;
        TaskPriority priority;
        TaskStatus status;
        std::string assigned_slave;
        std::chrono::steady_clock::time_point created_time;
        std::chrono::steady_clock::time_point assigned_time;
        std::chrono::steady_clock::time_point completed_time;
        uint32_t retry_count;
        uint32_t max_retries;
        std::vector<uint8_t> parameters;
        std::shared_ptr<PointCloudChunk> chunk_data;
        std::shared_ptr<BimPcChunk> bimpc_chunk_data;
        std::shared_ptr<icp::IcpChunk> icp_chunk_data;

        TaskInfo() : task_id(0), chunk_id(0), task_type(TaskType::CONVERT_PTS),
            priority(TaskPriority::NORMAL), status(TaskStatus::PENDING),
            retry_count(0), max_retries(3), created_time(std::chrono::steady_clock::now()) {
        }

        double getExecutionTime() const {
            if (status != TaskStatus::COMPLETED) return 0.0;
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                completed_time - assigned_time);
            return duration.count() / 1000.0;
        }
    };

    /**
     * @brief Slave 워커 정보 및 통계
     */
    struct SlaveWorkerInfo {
        std::string slave_id;
        bool is_active;
        bool is_busy;
        std::string current_task_id;
        std::chrono::steady_clock::time_point last_heartbeat;
        uint32_t completed_tasks;
        uint32_t failed_tasks;
        double avg_processing_time;

        SlaveWorkerInfo() : is_active(false), is_busy(false), completed_tasks(0),
            failed_tasks(0), avg_processing_time(0.0),
            last_heartbeat(std::chrono::steady_clock::now()) {
        }

        double getSuccessRate() const {
            uint32_t total = completed_tasks + failed_tasks;
            return total > 0 ? static_cast<double>(completed_tasks) / total : 1.0;
        }
    };

    /**
     * @brief Task Manager class - 완전한 인라인 구현
     */
    class TaskManager {
    private:
        std::atomic<bool> stopping_{ false };
        TaskType current_task_type_;
        std::vector<uint8_t> task_parameters_;

        struct PendingFailCallback {
            bool should_invoke = false;
            TaskInfo task_info;
            std::string error_message;
        };

        // 멤버 변수
        std::map<uint32_t, TaskInfo> tasks_;
        std::map<std::string, SlaveWorkerInfo> slaves_;
        std::vector<ProcessingResult> completed_results_;
        std::vector<BimPcResult> bimpc_results_;

        mutable std::mutex tasks_mutex_;
        mutable std::mutex slaves_mutex_;
        mutable std::mutex results_mutex_;

        std::atomic<uint32_t> next_task_id_;
        uint32_t task_timeout_seconds_;
        size_t current_slave_index_;

    public:
        using TaskCompletedCallback = std::function<void(const TaskInfo&, const ProcessingResult&)>;
        using TaskFailedCallback = std::function<void(const TaskInfo&, const std::string&)>;

    private:
        TaskCompletedCallback task_completed_callback_;
        TaskFailedCallback task_failed_callback_;

    public:
        /// [수정] starting_task_id 매개변수 추가 -- MasterApplication::initializeTaskManager()가
        /// 매 작업(job)마다 새 TaskManager를 만들 때, task_id가 항상 1부터 다시 시작하지
        /// 않고 이전 TaskManager(이전 세대)가 마지막으로 쓴 값 다음부터 이어지도록 한다.
        /// task_id가 세대마다 재사용되면, 타임아웃되어 방치된 이전 세대의 task에 대한
        /// 늦은 Slave 응답이 새 세대의 진짜 task_id와 우연히 같아져서 completeTask()/
        /// icp_task_to_job_ 매핑을 조용히 오염시킬 수 있다(예전에 실제로 있었던 버그).
        explicit TaskManager(const uint32_t timeout_seconds, TaskType task_type, const std::vector<uint8_t>& parameters,
            uint32_t starting_task_id = 1)
            : next_task_id_(starting_task_id), task_timeout_seconds_(timeout_seconds), current_slave_index_(0),
            current_task_type_(task_type), task_parameters_(parameters) {
            std::cout << "TaskManager initialized for task type: " << taskStr(task_type)
                << " with " << timeout_seconds << "s timeout (starting task_id: " << starting_task_id << ")" << std::endl;
        }

        /// 다음에 발급될 task_id를 미리 확인한다(발급하지 않음). MasterApplication이 이
        /// TaskManager를 새 것으로 교체하기 직전에 호출해서, 다음 세대의 시작 task_id로
        /// 이어받는 데 쓴다.
        uint32_t peekNextTaskId() const {
            return next_task_id_.load();
        }

        ~TaskManager() {
            clearAllTasks();
            std::cout << "TaskManager destroyed" << std::endl;
        }

        void requestStop() {
            stopping_ = true;
            std::cout << "TaskManager stop requested" << std::endl;
        }

        bool isStopping() const {
            return stopping_;
        }

        TaskType getCurrentTaskType() const {
            return current_task_type_;
        }

        uint32_t addTask(std::shared_ptr<BimPcChunk> bimpc_chunk) {
            std::lock_guard<std::mutex> lock(tasks_mutex_);

            uint32_t task_id = next_task_id_++;

            TaskInfo task_info;
            task_info.task_id = task_id;
            task_info.chunk_id = bimpc_chunk->chunk_id;
            task_info.task_type = current_task_type_;
            task_info.priority = TaskPriority::NORMAL;
            task_info.parameters = task_parameters_;
            task_info.chunk_data = nullptr;
            task_info.bimpc_chunk_data = bimpc_chunk;

            tasks_[task_id] = std::move(task_info);

            std::cout << "Task added (BimPc): " << task_id
                << " (" << taskStr(current_task_type_) << ")"
                << " - points: " << bimpc_chunk->points.size()
                << ", indices: " << bimpc_chunk->bim.faces.size() << " faces"
                << std::endl;

            return task_id;
        }

        uint32_t addTask(std::shared_ptr<PointCloudChunk> pc_chunk) {
            std::lock_guard<std::mutex> lock(tasks_mutex_);

            uint32_t task_id = next_task_id_++;

            TaskInfo task_info;
            task_info.task_id = task_id;
            task_info.chunk_id = pc_chunk->chunk_id;
            task_info.task_type = current_task_type_;
            task_info.priority = TaskPriority::NORMAL;
            task_info.parameters = task_parameters_;
            task_info.chunk_data = pc_chunk;
            task_info.bimpc_chunk_data = nullptr;

            tasks_[task_id] = std::move(task_info);

            std::cout << "Task added (PointCloud chunk): " << task_id
                << " (" << taskStr(current_task_type_) << ")"
                << " - points: " << pc_chunk->points.size()
                << std::endl;

            return task_id;
        }

        uint32_t addTask(std::shared_ptr<icp::IcpChunk> icp_chunk) {
            std::lock_guard<std::mutex> lock(tasks_mutex_);

            uint32_t task_id = next_task_id_++;

            TaskInfo task_info;
            task_info.task_id = task_id;
            task_info.chunk_id = icp_chunk->chunk_id;
            task_info.task_type = current_task_type_;
            task_info.priority = TaskPriority::NORMAL;
            task_info.parameters = task_parameters_;
            task_info.chunk_data = nullptr;
            task_info.bimpc_chunk_data = nullptr;
            task_info.icp_chunk_data = icp_chunk;

            tasks_[task_id] = std::move(task_info);

            std::cout << "Task added (Icp): " << task_id
                << " (" << taskStr(current_task_type_) << ")"
                << " - source points: " << icp_chunk->sourcePoints.size()
                << ", target points: " << icp_chunk->targetPoints.size()
                << std::endl;

            return task_id;
        }

        void clearAllTasks() {
            std::lock_guard<std::mutex> lock(tasks_mutex_);
            size_t count = tasks_.size();
            tasks_.clear();
            std::cout << "Cleared " << count << " tasks" << std::endl;
        }

        bool assignTasksToSlaves() {
            if (stopping_) { return false; }

            std::scoped_lock lock(tasks_mutex_, slaves_mutex_);

            std::vector<uint32_t> pending_tasks;
            for (auto& [task_id, task_info] : tasks_) {
                if (task_info.status == TaskStatus::PENDING) {
                    pending_tasks.push_back(task_id);
                }
            }

            if (pending_tasks.empty()) {
                return false;
            }

            std::sort(pending_tasks.begin(), pending_tasks.end(),
                [this](uint32_t a, uint32_t b) {
                    return tasks_[a].priority > tasks_[b].priority;
                });

            int assigned_count = 0;

            for (uint32_t task_id : pending_tasks) {
                TaskInfo& task_info = tasks_[task_id];

                std::string selected_slave = selectSlaveForTask();
                if (!selected_slave.empty()) {
                    task_info.status = TaskStatus::ASSIGNED;
                    task_info.assigned_slave = selected_slave;
                    task_info.assigned_time = std::chrono::steady_clock::now();

                    slaves_[selected_slave].is_busy = true;
                    slaves_[selected_slave].current_task_id = std::to_string(task_id);

                    assigned_count++;
                    std::cout << "Task " << task_id << " assigned to " << selected_slave << std::endl;
                }
            }

            if (assigned_count > 0) {
                std::cout << "Assigned " << assigned_count << " tasks to slaves" << std::endl;
            }

            return assigned_count > 0;
        }

        bool updateTaskStatus(uint32_t task_id, TaskStatus status) {
            std::lock_guard<std::mutex> lock(tasks_mutex_);

            auto it = tasks_.find(task_id);
            if (it == tasks_.end()) {
                return false;
            }

            it->second.status = status;

            if (status == TaskStatus::IN_PROGRESS) {
                it->second.assigned_time = std::chrono::steady_clock::now();
            }
            else if (status == TaskStatus::COMPLETED) {
                it->second.completed_time = std::chrono::steady_clock::now();
            }

            return true;
        }

        bool completeTask(uint32_t task_id, const ProcessingResult& result) {
            TaskInfo task_copy;
            bool should_invoke_callback = false;

            {
                std::scoped_lock lock(tasks_mutex_, slaves_mutex_, results_mutex_);

                auto task_it = tasks_.find(task_id);
                if (task_it == tasks_.end()) {
                    return false;
                }

                TaskInfo& task_info = task_it->second;
                task_info.status = TaskStatus::COMPLETED;
                task_info.completed_time = std::chrono::steady_clock::now();

                if (!task_info.assigned_slave.empty()) {
                    auto slave_it = slaves_.find(task_info.assigned_slave);
                    if (slave_it != slaves_.end()) {
                        slave_it->second.is_busy = false;
                        slave_it->second.current_task_id.clear();
                        slave_it->second.completed_tasks++;

                        auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                            task_info.completed_time - task_info.assigned_time).count() / 1000.0;

                        updateSlaveStatistics(task_info.assigned_slave, true, processing_time);
                    }
                }

                completed_results_.push_back(result);

                if (task_completed_callback_) {
                    task_copy = task_info;
                    should_invoke_callback = true;
                }
            }

            if (should_invoke_callback) {
                task_completed_callback_(task_copy, result);
            }

            return true;
        }

        bool failTask(uint32_t task_id, const std::string& error_message) {
            PendingFailCallback pending;
            {
                std::scoped_lock lock(tasks_mutex_, slaves_mutex_);
                pending = failTaskInternalNoCallback(task_id, error_message);
            }

            if (pending.should_invoke && task_failed_callback_) {
                task_failed_callback_(pending.task_info, pending.error_message);
            }

            return true;
        }

        std::vector<TaskInfo> getTasksByStatus(TaskStatus status) {
            std::lock_guard<std::mutex> lock(tasks_mutex_);

            std::vector<TaskInfo> result;
            for (const auto& pair : tasks_) {
                if (pair.second.status == status) {
                    result.push_back(pair.second);
                }
            }

            return result;
        }

        std::vector<TaskInfo> getAllTasks() {
            std::lock_guard<std::mutex> lock(tasks_mutex_);

            std::vector<TaskInfo> result;
            for (const auto& pair : tasks_) {
                result.push_back(pair.second);
            }

            return result;
        }

        std::vector<TaskInfo> getAssignedTasks() {
            return getTasksByStatus(TaskStatus::ASSIGNED);
        }

        std::optional<TaskInfo> getTaskById(uint32_t task_id) {
            std::lock_guard<std::mutex> lock(tasks_mutex_);
            auto it = tasks_.find(task_id);
            if (it != tasks_.end()) {
                return it->second;
            }
            return std::nullopt;
        }

        bool setTaskType(uint32_t task_id, TaskType type) {
            std::lock_guard<std::mutex> lock(tasks_mutex_);
            auto it = tasks_.find(task_id);
            if (it == tasks_.end()) return false;
            it->second.task_type = type;
            return true;
        }

        void registerSlave(const std::string& slave_id) {
            std::lock_guard<std::mutex> lock(slaves_mutex_);

            if (slaves_.find(slave_id) == slaves_.end()) {
                SlaveWorkerInfo slave_info;
                slave_info.slave_id = slave_id;
                slave_info.is_active = true;
                slaves_[slave_id] = slave_info;
                std::cout << "Slave registered: " << slave_id << std::endl;
            }
            else {
                slaves_[slave_id].is_active = true;
                slaves_[slave_id].last_heartbeat = std::chrono::steady_clock::now();
                std::cout << "Slave re-registered: " << slave_id << std::endl;
            }
        }

        void unregisterSlave(const std::string& slave_id) {
            std::lock_guard<std::mutex> lock(slaves_mutex_);

            auto it = slaves_.find(slave_id);
            if (it != slaves_.end()) {
                it->second.is_active = false;
                std::cout << "Slave unregistered: " << slave_id << std::endl;
            }
        }

        void updateSlaveHeartbeat(const std::string& slave_id) {
            std::lock_guard<std::mutex> lock(slaves_mutex_);

            auto it = slaves_.find(slave_id);
            if (it != slaves_.end()) {
                it->second.last_heartbeat = std::chrono::steady_clock::now();
            }
        }

        std::vector<SlaveWorkerInfo> getAllSlaves() {
            std::lock_guard<std::mutex> lock(slaves_mutex_);

            std::vector<SlaveWorkerInfo> result;
            for (const auto& pair : slaves_) {
                result.push_back(pair.second);
            }

            return result;
        }

        size_t getPendingTaskCount() {
            std::lock_guard<std::mutex> lock(tasks_mutex_);

            size_t count = 0;
            for (const auto& pair : tasks_) {
                if (pair.second.status == TaskStatus::PENDING) {
                    count++;
                }
            }

            return count;
        }

        size_t getActiveTaskCount() {
            std::lock_guard<std::mutex> lock(tasks_mutex_);

            size_t count = 0;
            for (const auto& pair : tasks_) {
                if (pair.second.status == TaskStatus::ASSIGNED ||
                    pair.second.status == TaskStatus::IN_PROGRESS) {
                    count++;
                }
            }

            return count;
        }

        size_t getCompletedTaskCount() {
            std::lock_guard<std::mutex> lock(tasks_mutex_);

            size_t count = 0;
            for (const auto& pair : tasks_) {
                if (pair.second.status == TaskStatus::COMPLETED) {
                    count++;
                }
            }

            return count;
        }

        size_t getFailedTaskCount() {
            std::lock_guard<std::mutex> lock(tasks_mutex_);

            size_t count = 0;
            for (const auto& pair : tasks_) {
                if (pair.second.status == TaskStatus::FAILED) {
                    count++;
                }
            }

            return count;
        }

        double getOverallProgress() {
            std::lock_guard<std::mutex> lock(tasks_mutex_);

            if (tasks_.empty()) {
                return 1.0;
            }

            size_t completed = 0;
            for (const auto& [task_id, task_info] : tasks_) {
                if (task_info.status == TaskStatus::COMPLETED) {
                    completed++;
                }
            }

            return static_cast<double>(completed) / tasks_.size();
        }

        /// 특정 task_id의 현재 상태를 조회한다. 이 TaskManager가 그 task_id를 전혀 모르면
        /// (다른 세대의 task이거나 잘못된 id) std::nullopt를 반환한다. 호출부(예:
        /// processIcpJob의 분산 fine 정합 대기 루프)가 "이 task가 재시도까지 다 소진하고
        /// 완전히 끝났는지" 실제 상태를 근거로 판단할 수 있게 한다 -- 고정된 배수로 시간을
        /// 추측하는 대신 FAILED/TIMEOUT(재시도 소진, failTaskInternalNoCallback 참고)에
        /// 도달했는지를 직접 확인할 수 있다.
        std::optional<TaskStatus> getTaskStatus(uint32_t task_id) {
            std::lock_guard<std::mutex> lock(tasks_mutex_);
            auto it = tasks_.find(task_id);
            if (it == tasks_.end()) {
                return std::nullopt;
            }
            return it->second.status;
        }

        /// task_id가 더 이상 진행될 여지가 없는 "종료 상태"(재시도까지 모두 소진한
        /// FAILED/TIMEOUT, 또는 이미 성공한 COMPLETED)에 도달했는지를 확인한다.
        /// PENDING/ASSIGNED/IN_PROGRESS는 아직 재시도 여지가 있으므로 false. 이 TaskManager가
        /// 그 task_id를 모르면(다른 세대의 task) true를 반환한다 -- 더 기다려 봐야 아무
        /// 의미가 없기 때문이다.
        bool isTaskTerminal(uint32_t task_id) {
            auto status = getTaskStatus(task_id);
            if (!status.has_value()) {
                return true;
            }
            return *status == TaskStatus::COMPLETED ||
                *status == TaskStatus::FAILED ||
                *status == TaskStatus::TIMEOUT;
        }

        void setTaskTimeout(uint32_t timeout_seconds) {
            task_timeout_seconds_ = timeout_seconds;
            std::cout << "Task timeout set to " << timeout_seconds << " seconds" << std::endl;
        }

        void checkTimeouts() {
            std::vector<PendingFailCallback> pending_callbacks;

            {
                std::scoped_lock lock(tasks_mutex_, slaves_mutex_);
                auto now = std::chrono::steady_clock::now();

                int timed_out_count = 0;
                for (auto& [task_id, task_info] : tasks_) {
                    if (task_info.status == TaskStatus::ASSIGNED ||
                        task_info.status == TaskStatus::IN_PROGRESS) {

                        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                            now - task_info.assigned_time).count();

                        if (elapsed > static_cast<int64_t>(task_timeout_seconds_)) {
                            std::cout << "Task " << task_id << " timed out after "
                                << elapsed << " seconds" << std::endl;

                            timed_out_count++;

                            auto pending = failTaskInternalNoCallback(task_id, "Task timeout",
                                                                       TaskStatus::TIMEOUT);
                            if (pending.should_invoke) {
                                pending_callbacks.push_back(std::move(pending));
                            }
                        }
                    }
                }

                if (timed_out_count > 0) {
                    std::cout << "Found " << timed_out_count << " timed out tasks" << std::endl;
                }
            }

            for (const auto& pending : pending_callbacks) {
                if (task_failed_callback_) {
                    task_failed_callback_(pending.task_info, pending.error_message);
                }
            }
        }

        std::vector<ProcessingResult> getCompletedResults() {
            std::lock_guard<std::mutex> lock(results_mutex_);
            return completed_results_;
        }

        void clearCompletedResults() {
            std::lock_guard<std::mutex> lock(results_mutex_);
            size_t count = completed_results_.size();
            completed_results_.clear();
            std::cout << "Cleared " << count << " completed results" << std::endl;
        }

        void addBimPcResult(const BimPcResult& result) {
            std::lock_guard<std::mutex> lock(results_mutex_);
            bimpc_results_.push_back(result);
        }

        std::vector<BimPcResult> getBimPcResults() const {
            std::lock_guard<std::mutex> lock(results_mutex_);
            return bimpc_results_;
        }

        void clearBimPcResults() {
            std::lock_guard<std::mutex> lock(results_mutex_);
            bimpc_results_.clear();
        }

        void setTaskCompletedCallback(TaskCompletedCallback callback) {
            task_completed_callback_ = callback;
        }

        void setTaskFailedCallback(TaskFailedCallback callback) {
            task_failed_callback_ = callback;
        }

    private:
        PendingFailCallback failTaskInternalNoCallback(uint32_t task_id, const std::string& error_message,
                                                       TaskStatus terminal_status = TaskStatus::FAILED) {
            PendingFailCallback result;
            result.should_invoke = false;
            result.error_message = error_message;

            auto task_it = tasks_.find(task_id);
            if (task_it == tasks_.end()) {
                return result;
            }

            TaskInfo& task_info = task_it->second;
            task_info.retry_count++;

            if (!task_info.assigned_slave.empty()) {
                auto slave_it = slaves_.find(task_info.assigned_slave);
                if (slave_it != slaves_.end()) {
                    slave_it->second.is_busy = false;
                    slave_it->second.current_task_id.clear();
                    slave_it->second.failed_tasks++;

                    auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - task_info.assigned_time).count() / 1000.0;

                    updateSlaveStatistics(task_info.assigned_slave, false, processing_time);
                }
            }

            if (task_info.retry_count < task_info.max_retries) {
                task_info.status = TaskStatus::PENDING;
                task_info.assigned_slave.clear();
                std::cout << "Task " << task_id << " failed, retrying..." << std::endl;
            }
            else {
                task_info.status = terminal_status;
                std::cout << "Task " << task_id << " failed permanently." << std::endl;

                if (task_failed_callback_) {
                    result.should_invoke = true;
                    result.task_info = task_info;
                }
            }

            return result;
        }

        std::string selectSlaveForTask() {
            std::vector<std::string> available_slaves;
            for (auto& [slave_id, slave_info] : slaves_) {
                if (slave_info.is_active && !slave_info.is_busy) {
                    available_slaves.push_back(slave_id);
                }
            }

            if (available_slaves.empty()) {
                return "";
            }

            std::string selected = available_slaves[current_slave_index_ % available_slaves.size()];
            current_slave_index_++;

            return selected;
        }

        void updateSlaveStatistics(const std::string& slave_id, bool success, double processing_time) {
            auto slave_it = slaves_.find(slave_id);
            if (slave_it != slaves_.end()) {
                SlaveWorkerInfo& slave_info = slave_it->second;

                uint32_t total_tasks = slave_info.completed_tasks + slave_info.failed_tasks;
                if (total_tasks > 0) {
                    slave_info.avg_processing_time =
                        (slave_info.avg_processing_time * (total_tasks - 1) + processing_time) / total_tasks;
                }
                else {
                    slave_info.avg_processing_time = processing_time;
                }
            }
        }
    };

    /**
     * @brief 스레드 안전(thread-safe) 통계 추적기
     */
    class ProcessingStats {
    public:
        ProcessingStats() : processed_count_(0), failed_count_(0), total_time_(0.0) {}

        void recordSuccess(double processing_time) {
            std::lock_guard<std::mutex> lock(mutex_);
            processed_count_++;
            total_time_ += processing_time;
        }

        void recordFailure() {
            std::lock_guard<std::mutex> lock(mutex_);
            failed_count_++;
        }

        uint32_t getProcessedCount() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return processed_count_;
        }

        uint32_t getFailedCount() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return failed_count_;
        }

        double getAverageProcessingTime() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return processed_count_ > 0 ? total_time_ / processed_count_ : 0.0;
        }

    private:
        mutable std::mutex mutex_;
        uint32_t processed_count_;
        uint32_t failed_count_;
        double total_time_;
    };

} // namespace DPApp