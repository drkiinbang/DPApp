#pragma once

#include <vector>
#include <queue>
#include <map>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <future>
#include <functional>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <random>
#include <cmath>

#include "PointCloudTypes.h"
#include "NetworkManager.h"

namespace DPApp {

// 작업 상태 정의
enum class TaskStatus {
    PENDING,        // 대기 중
    ASSIGNED,       // 할당됨
    IN_PROGRESS,    // 진행 중
    COMPLETED,      // 완료됨
    FAILED,         // 실패
    TIMEOUT         // 타임아웃
};

// 작업 우선순위
enum class TaskPriority {
    LOW = 0,
    NORMAL = 1,
    HIGH = 2,
    CRITICAL = 3
};

// 확장된 작업 정보
struct TaskInfo {
    uint32_t task_id;
    uint32_t chunk_id;
    std::string task_type;
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
    
    TaskInfo() : task_id(0), chunk_id(0), priority(TaskPriority::NORMAL), 
                status(TaskStatus::PENDING), retry_count(0), max_retries(3),
                created_time(std::chrono::steady_clock::now()) {}
};

// 슬레이브 워커 정보
struct SlaveWorkerInfo {
    std::string slave_id;
    bool is_active;
    bool is_busy;
    std::string current_task_id;
    std::chrono::steady_clock::time_point last_heartbeat;
    uint32_t completed_tasks;
    uint32_t failed_tasks;
    double avg_processing_time;  // 초 단위
    
    SlaveWorkerInfo() : is_active(false), is_busy(false), completed_tasks(0), 
                       failed_tasks(0), avg_processing_time(0.0),
                       last_heartbeat(std::chrono::steady_clock::now()) {}
};

// 작업 분배 전략
enum class TaskDistributionStrategy {
    ROUND_ROBIN,        // 순차 분배
    LOAD_BALANCED,      // 부하 균형
    PERFORMANCE_BASED,  // 성능 기반
};

// 작업 관리자 (Master용)
class TaskManager {
private:
    std::atomic<bool> stopping_{ false };

public:
    explicit TaskManager(const uint32_t timeout_seconds)
        : next_task_id_(1), distribution_strategy_(TaskDistributionStrategy::LOAD_BALANCED),
        task_timeout_seconds_(timeout_seconds), current_slave_index_(0) {
    }

    ~TaskManager() {
        clearAllTasks();
    }

    void requestStop() {
        stopping_ = true;
    }

    bool isStopping() const {
        return stopping_;
    }
    
    // 작업 관리
    uint32_t addTask(const std::string& task_type, 
                    std::shared_ptr<PointCloudChunk> chunk,
                    const std::vector<uint8_t>& parameters,
                    TaskPriority priority = TaskPriority::NORMAL) {
        std::lock_guard<std::mutex> lock(tasks_mutex_);

        uint32_t task_id = next_task_id_++;

        TaskInfo task_info;
        task_info.task_id = task_id;
        task_info.chunk_id = chunk->chunk_id;
        task_info.task_type = task_type;
        task_info.priority = priority;
        task_info.parameters = parameters;
        task_info.chunk_data = chunk;

        tasks_[task_id] = std::move(task_info);

        std::cout << "Task added: " << task_id << " (" << task_type << ")" << std::endl;

        return task_id;
    }
                    
    bool removeTask(uint32_t task_id) {
        std::lock_guard<std::mutex> lock(tasks_mutex_);

        auto it = tasks_.find(task_id);
        if (it == tasks_.end()) {
            return false;
        }

        tasks_.erase(it);
        return true;
    }

    void clearAllTasks() {
        std::lock_guard<std::mutex> lock(tasks_mutex_);
        tasks_.clear();
    }
    
    // 작업 분배
    void setDistributionStrategy(TaskDistributionStrategy strategy) { distribution_strategy_ = strategy; }

    bool assignTasksToSlaves() {
        if (stopping_) {
            return false;  // 종료 요청시 새 작업 할당 중지
        }

        std::lock_guard<std::mutex> tasks_lock(tasks_mutex_);
        std::lock_guard<std::mutex> slaves_lock(slaves_mutex_);

        // 대기 중인 작업들을 우선순위 순으로 정렬
        std::vector<uint32_t> pending_tasks;
        for (auto& [task_id, task_info] : tasks_) {
            if (task_info.status == TaskStatus::PENDING) {
                pending_tasks.push_back(task_id);
            }
        }

        // 우선순위 기준으로 정렬
        std::sort(pending_tasks.begin(), pending_tasks.end(),
            [this](uint32_t a, uint32_t b) {
                return tasks_[a].priority > tasks_[b].priority;
            });

        int assigned_count = 0;

        for (uint32_t task_id : pending_tasks) {
            TaskInfo& task_info = tasks_[task_id];

            std::string selected_slave = selectSlaveForTask(task_info);
            if (!selected_slave.empty()) {
                task_info.status = TaskStatus::ASSIGNED;
                task_info.assigned_slave = selected_slave;
                task_info.assigned_time = std::chrono::steady_clock::now();

                // 슬레이브를 바쁨 상태로 설정
                slaves_[selected_slave].is_busy = true;
                slaves_[selected_slave].current_task_id = std::to_string(task_id);

                assigned_count++;
                std::cout << "Task " << task_id << " assigned to " << selected_slave << std::endl;
            }
        }

        return assigned_count > 0;
    }
    
    // 작업 상태 관리
    bool updateTaskStatus(uint32_t task_id, TaskStatus status);
    bool completeTask(uint32_t task_id, const ProcessingResult& result);
    bool failTask(uint32_t task_id, const std::string& error_message);
    
    // 슬레이브 관리
    void registerSlave(const std::string& slave_id);
    void unregisterSlave(const std::string& slave_id);
    void updateSlaveHeartbeat(const std::string& slave_id);
    
    // 상태 조회
    std::vector<TaskInfo> getTasksByStatus(TaskStatus status);
    std::vector<SlaveWorkerInfo> getAllSlaves();
    
    // 통계
    size_t getPendingTaskCount();
    size_t getActiveTaskCount();
    size_t getCompletedTaskCount();
    size_t getFailedTaskCount();
    double getOverallProgress();
    
    // 타임아웃 처리
    void setTaskTimeout(uint32_t timeout_seconds) { task_timeout_seconds_ = timeout_seconds; }
    void checkTimeouts();
    
    // 결과 수집
    std::vector<ProcessingResult> getCompletedResults();
    void clearCompletedResults();
    
    // 콜백 설정
    using TaskCompletedCallback = std::function<void(const TaskInfo&, const ProcessingResult&)>;
    using TaskFailedCallback = std::function<void(const TaskInfo&, const std::string&)>;
    
    void setTaskCompletedCallback(TaskCompletedCallback callback) { task_completed_callback_ = callback; }
    void setTaskFailedCallback(TaskFailedCallback callback) { task_failed_callback_ = callback; }

private:
    // 작업 분배 전략 구현
    std::string selectSlaveForTask(const TaskInfo& task) {
        switch (distribution_strategy_) {
        case TaskDistributionStrategy::ROUND_ROBIN:
            return selectSlaveRoundRobin();
        case TaskDistributionStrategy::LOAD_BALANCED:
            return selectSlaveLoadBalanced();
        case TaskDistributionStrategy::PERFORMANCE_BASED:
            return selectSlavePerformanceBased();
        default:
            return selectSlaveRoundRobin();
        }
    }

    std::string selectSlaveRoundRobin() {
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

    std::string selectSlaveLoadBalanced() {
        std::string best_slave;
        uint32_t min_load = UINT32_MAX;

        for (auto& [slave_id, slave_info] : slaves_) {
            if (slave_info.is_active && !slave_info.is_busy) {
                uint32_t load = slave_info.completed_tasks + slave_info.failed_tasks;
                if (load < min_load) {
                    min_load = load;
                    best_slave = slave_id;
                }
            }
        }

        return best_slave;
    }

    std::string selectSlavePerformanceBased() {
        std::string best_slave;
        double best_performance = 0.0;

        for (auto& [slave_id, slave_info] : slaves_) {
            if (slave_info.is_active && !slave_info.is_busy) {
                // 성공률과 처리 속도를 고려한 점수 계산
                double success_rate = 1.0;
                if (slave_info.completed_tasks + slave_info.failed_tasks > 0) {
                    success_rate = static_cast<double>(slave_info.completed_tasks) /
                        (slave_info.completed_tasks + slave_info.failed_tasks);
                }

                double speed_factor = 1.0;
                if (slave_info.avg_processing_time > 0) {
                    speed_factor = 1.0 / slave_info.avg_processing_time;
                }

                double performance = success_rate * speed_factor;

                if (performance > best_performance) {
                    best_performance = performance;
                    best_slave = slave_id;
                }
            }
        }

        return best_slave;
    }
    
    void updateSlaveStatistics(const std::string& slave_id, bool success, double processing_time);
    
    std::map<uint32_t, TaskInfo> tasks_;
    std::map<std::string, SlaveWorkerInfo> slaves_;
    std::vector<ProcessingResult> completed_results_;
    
    std::mutex tasks_mutex_;
    std::mutex slaves_mutex_;
    std::mutex results_mutex_;
    
    std::atomic<uint32_t> next_task_id_;
    TaskDistributionStrategy distribution_strategy_;
    uint32_t task_timeout_seconds_;
    size_t current_slave_index_;  // Round-robin용
    
    TaskCompletedCallback task_completed_callback_;
    TaskFailedCallback task_failed_callback_;
};

// 작업 처리기 (Slave용)
class TaskProcessor {
public:
    TaskProcessor();
    ~TaskProcessor();
    
    // 처리기 등록
    using ProcessorFunction = std::function<ProcessingResult(const ProcessingTask&, const PointCloudChunk&)>;
    
    void registerProcessor(const std::string& task_type, ProcessorFunction processor);
    void unregisterProcessor(const std::string& task_type);
    
    // 작업 처리
    ProcessingResult processTask(const ProcessingTask& task, const PointCloudChunk& chunk);
    
    // 지원되는 작업 타입 조회
    std::vector<std::string> getSupportedTaskTypes();
    bool supportsTaskType(const std::string& task_type);
    
    // 통계
    uint32_t getProcessedTaskCount() const { return processed_task_count_; }
    uint32_t getFailedTaskCount() const { return failed_task_count_; }
    double getAverageProcessingTime() const { return avg_processing_time_; }

private:
    std::map<std::string, ProcessorFunction> processors_;
    std::mutex processors_mutex_;
    
    std::atomic<uint32_t> processed_task_count_;
    std::atomic<uint32_t> failed_task_count_;
    std::atomic<double> avg_processing_time_;
};

// 기본 포인트클라우드 처리 함수들
namespace PointCloudProcessors {
    ProcessingResult filterPoints(const ProcessingTask& task, const PointCloudChunk& chunk);
    ProcessingResult classifyPoints(const ProcessingTask& task, const PointCloudChunk& chunk);
    ProcessingResult estimateNormals(const ProcessingTask& task, const PointCloudChunk& chunk);
    ProcessingResult removeOutliers(const ProcessingTask& task, const PointCloudChunk& chunk);
    ProcessingResult downsample(const ProcessingTask& task, const PointCloudChunk& chunk);
}

///
/// TaskManager Implementation
///
bool TaskManager::completeTask(uint32_t task_id, const ProcessingResult& result) {
    std::lock_guard<std::mutex> tasks_lock(tasks_mutex_);
    std::lock_guard<std::mutex> slaves_lock(slaves_mutex_);
    std::lock_guard<std::mutex> results_lock(results_mutex_);

    auto task_it = tasks_.find(task_id);
    if (task_it == tasks_.end()) {
        return false;
    }

    TaskInfo& task_info = task_it->second;

    if (result.success) {
        task_info.status = TaskStatus::COMPLETED;
        completed_results_.push_back(result);

        // 슬레이브 통계 업데이트
        if (!task_info.assigned_slave.empty()) {
            auto slave_it = slaves_.find(task_info.assigned_slave);
            if (slave_it != slaves_.end()) {
                slave_it->second.is_busy = false;
                slave_it->second.current_task_id.clear();
                slave_it->second.completed_tasks++;

                // 처리 시간 계산
                auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - task_info.assigned_time).count() / 1000.0;

                updateSlaveStatistics(task_info.assigned_slave, true, processing_time);
            }
        }

        task_info.completed_time = std::chrono::steady_clock::now();

        if (task_completed_callback_) {
            task_completed_callback_(task_info, result);
        }

        std::cout << "Task " << task_id << " completed successfully" << std::endl;
    }
    else {
        failTask(task_id, result.error_message);
    }

    return true;
}

bool TaskManager::failTask(uint32_t task_id, const std::string& error_message) {
    std::lock_guard<std::mutex> tasks_lock(tasks_mutex_);
    std::lock_guard<std::mutex> slaves_lock(slaves_mutex_);

    auto task_it = tasks_.find(task_id);
    if (task_it == tasks_.end()) {
        return false;
    }

    TaskInfo& task_info = task_it->second;
    task_info.retry_count++;

    // 슬레이브 해제
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

    // 재시도 가능한지 확인
    if (task_info.retry_count < task_info.max_retries) {
        task_info.status = TaskStatus::PENDING;
        task_info.assigned_slave.clear();
        std::cout << "Task " << task_id << " failed, retrying ("
            << task_info.retry_count << "/" << task_info.max_retries << ")" << std::endl;
    }
    else {
        task_info.status = TaskStatus::FAILED;
        if (task_failed_callback_) {
            task_failed_callback_(task_info, error_message);
        }
        std::cout << "Task " << task_id << " failed permanently: " << error_message << std::endl;
    }

    return true;
}

void TaskManager::registerSlave(const std::string& slave_id) {
    std::lock_guard<std::mutex> lock(slaves_mutex_);

    SlaveWorkerInfo slave_info;
    slave_info.slave_id = slave_id;
    slave_info.is_active = true;

    slaves_[slave_id] = slave_info;

    std::cout << "Slave registered: " << slave_id << std::endl;
}

void TaskManager::unregisterSlave(const std::string& slave_id) {
    std::lock_guard<std::mutex> slaves_lock(slaves_mutex_);
    std::lock_guard<std::mutex> tasks_lock(tasks_mutex_);

    auto slave_it = slaves_.find(slave_id);
    if (slave_it != slaves_.end()) {
        // 해당 슬레이브에 할당된 작업들을 다시 대기 상태로 변경
        for (auto& [task_id, task_info] : tasks_) {
            if (task_info.assigned_slave == slave_id &&
                (task_info.status == TaskStatus::ASSIGNED || task_info.status == TaskStatus::IN_PROGRESS)) {
                task_info.status = TaskStatus::PENDING;
                task_info.assigned_slave.clear();
            }
        }

        slaves_.erase(slave_it);
        std::cout << "Slave unregistered: " << slave_id << std::endl;
    }
}

void TaskManager::updateSlaveStatistics(const std::string& slave_id, bool success, double processing_time) {
    auto slave_it = slaves_.find(slave_id);
    if (slave_it != slaves_.end()) {
        SlaveWorkerInfo& slave_info = slave_it->second;

        // 평균 처리 시간 업데이트
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

void TaskManager::checkTimeouts() {
    std::lock_guard<std::mutex> tasks_lock(tasks_mutex_);

    auto now = std::chrono::steady_clock::now();

    for (auto& [task_id, task_info] : tasks_) {
        if (task_info.status == TaskStatus::ASSIGNED || task_info.status == TaskStatus::IN_PROGRESS) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                now - task_info.assigned_time).count();

            if (elapsed > task_timeout_seconds_) {
                std::cout << "Task " << task_id << " timed out" << std::endl;
                task_info.status = TaskStatus::TIMEOUT;

                // 슬레이브 해제
                if (!task_info.assigned_slave.empty()) {
                    std::lock_guard<std::mutex> slaves_lock(slaves_mutex_);
                    auto slave_it = slaves_.find(task_info.assigned_slave);
                    if (slave_it != slaves_.end()) {
                        slave_it->second.is_busy = false;
                        slave_it->second.current_task_id.clear();
                    }
                }

                // 재시도를 위해 다시 대기 상태로 변경
                failTask(task_id, "Task timeout");
            }
        }
    }
}

double TaskManager::getOverallProgress() {
    std::lock_guard<std::mutex> lock(tasks_mutex_);

    if (tasks_.empty()) {
        return 1.0; /// If task is empty, return 100%
    }

    std::vector<TaskStatus> statuses;
    statuses.reserve(tasks_.size());
    for (const auto& [task_id, task_info] : tasks_) {
        statuses.push_back(task_info.status);
    }

    size_t completed = std::count(statuses.begin(), statuses.end(), TaskStatus::COMPLETED);
    return static_cast<double>(completed) / statuses.size();
}

std::vector<ProcessingResult> TaskManager::getCompletedResults() {
    std::lock_guard<std::mutex> lock(results_mutex_);
    return completed_results_;
}

void TaskManager::clearCompletedResults() {
    std::lock_guard<std::mutex> lock(results_mutex_);
    completed_results_.clear();
}

bool TaskManager::updateTaskStatus(uint32_t task_id, TaskStatus status) {
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

void TaskManager::updateSlaveHeartbeat(const std::string& slave_id) {
    std::lock_guard<std::mutex> lock(slaves_mutex_);

    auto it = slaves_.find(slave_id);
    if (it != slaves_.end()) {
        it->second.last_heartbeat = std::chrono::steady_clock::now();
    }
}

std::vector<TaskInfo> TaskManager::getTasksByStatus(TaskStatus status) {
    std::lock_guard<std::mutex> lock(tasks_mutex_);

    std::vector<TaskInfo> result;
    for (const auto& pair : tasks_) {
        if (pair.second.status == status) {
            result.push_back(pair.second);
        }
    }

    return result;
}

std::vector<SlaveWorkerInfo> TaskManager::getAllSlaves() {
    std::lock_guard<std::mutex> lock(slaves_mutex_);

    std::vector<SlaveWorkerInfo> result;
    for (const auto& pair : slaves_) {
        result.push_back(pair.second);
    }

    return result;
}

size_t TaskManager::getPendingTaskCount() {
    std::lock_guard<std::mutex> lock(tasks_mutex_);

    size_t count = 0;
    for (const auto& pair : tasks_) {
        if (pair.second.status == TaskStatus::PENDING) {
            count++;
        }
    }

    return count;
}

size_t TaskManager::getActiveTaskCount() {
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

size_t TaskManager::getCompletedTaskCount() {
    std::lock_guard<std::mutex> lock(tasks_mutex_);

    size_t count = 0;
    for (const auto& pair : tasks_) {
        if (pair.second.status == TaskStatus::COMPLETED) {
            count++;
        }
    }

    return count;
}

size_t TaskManager::getFailedTaskCount() {
    std::lock_guard<std::mutex> lock(tasks_mutex_);

    size_t count = 0;
    for (const auto& pair : tasks_) {
        if (pair.second.status == TaskStatus::FAILED) {
            count++;
        }
    }

    return count;
}

// TaskProcessor 구현
TaskProcessor::TaskProcessor()
    : processed_task_count_(0), failed_task_count_(0), avg_processing_time_(0.0) {

    // 기본 프로세서들 등록
    registerProcessor("filter", PointCloudProcessors::filterPoints);
    registerProcessor("classify", PointCloudProcessors::classifyPoints);
    registerProcessor("estimate_normals", PointCloudProcessors::estimateNormals);
    registerProcessor("remove_outliers", PointCloudProcessors::removeOutliers);
    registerProcessor("downsample", PointCloudProcessors::downsample);
}

TaskProcessor::~TaskProcessor() {
    std::lock_guard<std::mutex> lock(processors_mutex_);
    processors_.clear();
}

void TaskProcessor::registerProcessor(const std::string& task_type, ProcessorFunction processor) {
    std::lock_guard<std::mutex> lock(processors_mutex_);
    processors_[task_type] = processor;
    std::cout << "Processor registered for task type: " << task_type << std::endl;
}

void TaskProcessor::unregisterProcessor(const std::string& task_type) {
    std::lock_guard<std::mutex> lock(processors_mutex_);
    processors_.erase(task_type);
}

ProcessingResult TaskProcessor::processTask(const ProcessingTask& task, const PointCloudChunk& chunk) {
    auto start_time = std::chrono::steady_clock::now();

    ProcessingResult result;
    result.task_id = task.task_id;
    result.chunk_id = task.chunk_id;

    try {
        std::lock_guard<std::mutex> lock(processors_mutex_);

        auto it = processors_.find(task.task_type);
        if (it == processors_.end()) {
            result.success = false;
            result.error_message = "Unsupported task type: " + task.task_type;
            failed_task_count_++;
            return result;
        }

        // 프로세서 실행
        result = it->second(task, chunk);

        if (result.success) {
            processed_task_count_++;
        }
        else {
            failed_task_count_++;
        }

    }
    catch (const std::exception& e) {
        result.success = false;
        result.error_message = "Processing error: " + std::string(e.what());
        failed_task_count_++;
    }

    // 처리 시간 업데이트
    auto end_time = std::chrono::steady_clock::now();
    auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time).count() / 1000.0;

    uint32_t total_tasks = processed_task_count_ + failed_task_count_;
    if (total_tasks > 0) {
        avg_processing_time_ = (avg_processing_time_ * (total_tasks - 1) + processing_time) / total_tasks;
    }

    return result;
}

std::vector<std::string> TaskProcessor::getSupportedTaskTypes() {
    std::lock_guard<std::mutex> lock(processors_mutex_);

    std::vector<std::string> types;
    for (const auto& [type, processor] : processors_) {
        types.push_back(type);
    }

    return types;
}

bool TaskProcessor::supportsTaskType(const std::string& task_type) {
    std::lock_guard<std::mutex> lock(processors_mutex_);
    return processors_.find(task_type) != processors_.end();
}

/// 기본 포인트클라우드 처리 함수들 구현
/// PointCloudProcessors는 이런 경우에 :
///
/// 모든 슬레이브에서 공통으로 사용하는 표준 알고리즘
/// 라이브러리 형태로 제공하고 싶은 기능
/// 다른 모듈에서도 직접 사용할 가능성이 있는 기능
/// Compare to registerCustomProcessors function in SlaveApplication.cpp

namespace PointCloudProcessors {

    ProcessingResult filterPoints(const ProcessingTask& task, const PointCloudChunk& chunk) {
        ProcessingResult result;
        result.task_id = task.task_id;
        result.chunk_id = task.chunk_id;
        result.success = true;

        try {
            // 파라미터 파싱 (간단한 예제)
            // 실제로는 더 복잡한 파라미터 구조를 사용할 것
            double min_z = -100.0;
            double max_z = 100.0;

            if (task.parameters.size() >= 16) {
                std::memcpy(&min_z, task.parameters.data(), sizeof(double));
                std::memcpy(&max_z, task.parameters.data() + sizeof(double), sizeof(double));
            }

            // Z 좌표 필터링
            for (const auto& point : chunk.points) {
                if (point.z >= min_z && point.z <= max_z) {
                    result.processed_points.push_back(point);
                }
            }

            std::cout << "Filtered " << chunk.points.size() << " points to "
                << result.processed_points.size() << " points" << std::endl;

        }
        catch (const std::exception& e) {
            result.success = false;
            result.error_message = "Filter error: " + std::string(e.what());
        }

        return result;
    }

    ProcessingResult classifyPoints(const ProcessingTask& task, const PointCloudChunk& chunk) {
        ProcessingResult result;
        result.task_id = task.task_id;
        result.chunk_id = task.chunk_id;
        result.success = true;

        try {
            // 간단한 높이 기반 분류 예제
            result.processed_points = chunk.points;

            for (auto& point : result.processed_points) {
                if (point.z < 0) {
                    point.classification = 2; // Ground
                }
                else if (point.z < 5) {
                    point.classification = 1; // Low vegetation
                }
                else if (point.z < 20) {
                    point.classification = 4; // Medium vegetation
                }
                else {
                    point.classification = 5; // High vegetation
                }
            }

            std::cout << "Classified " << result.processed_points.size() << " points" << std::endl;

        }
        catch (const std::exception& e) {
            result.success = false;
            result.error_message = "Classification error: " + std::string(e.what());
        }

        return result;
    }

    ProcessingResult estimateNormals(const ProcessingTask& task, const PointCloudChunk& chunk) {
        ProcessingResult result;
        result.task_id = task.task_id;
        result.chunk_id = task.chunk_id;
        result.success = true;

        try {
            // 간단한 법선 벡터 추정 (실제로는 더 복잡한 알고리즘 필요)
            result.processed_points = chunk.points;

            // 각 포인트에 대해 간단한 법선 벡터 계산
            // 실제로는 k-nearest neighbors나 radius search 사용
            for (size_t i = 0; i < result.processed_points.size(); ++i) {
                // 임시로 Z축을 법선으로 설정
                // 실제로는 주변 점들을 이용한 평면 피팅 필요
                auto& point = result.processed_points[i];

                // 법선 정보를 RGB에 저장 (시각화용)
                point.r = 127; // Normal X
                point.g = 127; // Normal Y
                point.b = 255; // Normal Z (위쪽 향함)
            }

            std::cout << "Estimated normals for " << result.processed_points.size() << " points" << std::endl;

        }
        catch (const std::exception& e) {
            result.success = false;
            result.error_message = "Normal estimation error: " + std::string(e.what());
        }

        return result;
    }

    ProcessingResult removeOutliers(const ProcessingTask& task, const PointCloudChunk& chunk) {
        ProcessingResult result;
        result.task_id = task.task_id;
        result.chunk_id = task.chunk_id;
        result.success = true;

        try {
            // 통계적 아웃라이어 제거 (간단한 버전)
            if (chunk.points.empty()) {
                result.success = false;
                result.error_message = "Empty point cloud";
                return result;
            }

            // Z 좌표의 평균과 표준편차 계산
            double sum_z = 0.0;
            for (const auto& point : chunk.points) {
                sum_z += point.z;
            }
            double mean_z = sum_z / chunk.points.size();

            double sum_sq_diff = 0.0;
            for (const auto& point : chunk.points) {
                double diff = point.z - mean_z;
                sum_sq_diff += diff * diff;
            }
            double std_dev = std::sqrt(sum_sq_diff / chunk.points.size());

            // 2 표준편차 범위 내의 점들만 유지
            double threshold = 2.0 * std_dev;

            for (const auto& point : chunk.points) {
                if (std::abs(point.z - mean_z) <= threshold) {
                    result.processed_points.push_back(point);
                }
            }

            std::cout << "Removed outliers: " << chunk.points.size() << " -> "
                << result.processed_points.size() << " points" << std::endl;

        }
        catch (const std::exception& e) {
            result.success = false;
            result.error_message = "Outlier removal error: " + std::string(e.what());
        }

        return result;
    }

    ProcessingResult downsample(const ProcessingTask& task, const PointCloudChunk& chunk) {
        ProcessingResult result;
        result.task_id = task.task_id;
        result.chunk_id = task.chunk_id;
        result.success = true;

        try {
            // 파라미터에서 다운샘플링 비율 읽기
            double sampling_ratio = 0.1; // 기본값: 10%

            if (task.parameters.size() >= sizeof(double)) {
                std::memcpy(&sampling_ratio, task.parameters.data(), sizeof(double));
            }

            // 무작위 다운샘플링
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0.0, 1.0);

            for (const auto& point : chunk.points) {
                if (dis(gen) < sampling_ratio) {
                    result.processed_points.push_back(point);
                }
            }

            std::cout << "Downsampled " << chunk.points.size() << " points to "
                << result.processed_points.size() << " points (ratio: "
                << sampling_ratio << ")" << std::endl;

        }
        catch (const std::exception& e) {
            result.success = false;
            result.error_message = "Downsampling error: " + std::string(e.what());
        }

        return result;
    }

} // namespace PointCloudProcessors

} // namespace DPApp