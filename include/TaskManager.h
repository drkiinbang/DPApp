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
    PRIORITY_FIRST      // 우선순위 우선
};

// 작업 관리자 (Master용)
class TaskManager {
public:
    TaskManager();
    ~TaskManager();
    
    // 작업 관리
    uint32_t addTask(const std::string& task_type, 
                    std::shared_ptr<PointCloudChunk> chunk,
                    const std::vector<uint8_t>& parameters,
                    TaskPriority priority = TaskPriority::NORMAL);
                    
    bool removeTask(uint32_t task_id);
    void clearAllTasks();
    
    // 작업 분배
    void setDistributionStrategy(TaskDistributionStrategy strategy) { distribution_strategy_ = strategy; }
    bool assignTasksToSlaves();
    bool assignTaskToSlave(uint32_t task_id, const std::string& slave_id);
    
    // 작업 상태 관리
    bool updateTaskStatus(uint32_t task_id, TaskStatus status);
    bool completeTask(uint32_t task_id, const ProcessingResult& result);
    bool failTask(uint32_t task_id, const std::string& error_message);
    
    // 슬레이브 관리
    void registerSlave(const std::string& slave_id);
    void unregisterSlave(const std::string& slave_id);
    void updateSlaveHeartbeat(const std::string& slave_id);
    bool isSlaveActive(const std::string& slave_id);
    
    // 상태 조회
    TaskStatus getTaskStatus(uint32_t task_id);
    std::vector<TaskInfo> getAllTasks();
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
    std::string selectSlaveForTask(const TaskInfo& task);
    std::string selectSlaveRoundRobin();
    std::string selectSlaveLoadBalanced();
    std::string selectSlavePerformanceBased();
    
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

} // namespace DPApp