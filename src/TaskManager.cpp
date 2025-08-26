#include "../include/TaskManager.h"
#include <iostream>
#include <algorithm>
#include <random>

namespace DPApp {

// TaskManager 구현
TaskManager::TaskManager() 
    : next_task_id_(1), distribution_strategy_(TaskDistributionStrategy::LOAD_BALANCED),
      task_timeout_seconds_(300), current_slave_index_(0) {
}

TaskManager::~TaskManager() {
    clearAllTasks();
}

uint32_t TaskManager::addTask(const std::string& task_type, 
                             std::shared_ptr<PointCloudChunk> chunk,
                             const std::vector<uint8_t>& parameters,
                             TaskPriority priority) {
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

bool TaskManager::removeTask(uint32_t task_id) {
    std::lock_guard<std::mutex> lock(tasks_mutex_);
    
    auto it = tasks_.find(task_id);
    if (it == tasks_.end()) {
        return false;
    }
    
    tasks_.erase(it);
    return true;
}

void TaskManager::clearAllTasks() {
    std::lock_guard<std::mutex> lock(tasks_mutex_);
    tasks_.clear();
}

bool TaskManager::assignTasksToSlaves() {
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

std::string TaskManager::selectSlaveForTask(const TaskInfo& task) {
    switch (distribution_strategy_) {
        case TaskDistributionStrategy::ROUND_ROBIN:
            return selectSlaveRoundRobin();
        case TaskDistributionStrategy::LOAD_BALANCED:
            return selectSlaveLoadBalanced();
        case TaskDistributionStrategy::PERFORMANCE_BASED:
            return selectSlavePerformanceBased();
        case TaskDistributionStrategy::PRIORITY_FIRST:
        default:
            return selectSlaveLoadBalanced(); // 기본값
    }
}

std::string TaskManager::selectSlaveRoundRobin() {
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

std::string TaskManager::selectSlaveLoadBalanced() {
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

std::string TaskManager::selectSlavePerformanceBased() {
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
    } else {
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
    } else {
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
        } else {
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
        return 1.0; // 작업이 없으면 100% 완료
    }
    
    size_t completed = 0;
    for (const auto& [task_id, task_info] : tasks_) {
        if (task_info.status == TaskStatus::COMPLETED) {
            completed++;
        }
    }
    
    return static_cast<double>(completed) / tasks_.size();
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

} // namespace DPApp