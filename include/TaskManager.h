#pragma once

// Prevent Windows min/max macros from interfering with std::min/max
#ifdef _WIN32
#define NOMINMAX
#endif

#define MAX_NUM_PTS_CHUNK 100000

#include <vector>
#include <queue>
#include <map>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <thread>
#include <exception>

#include "PointCloudTypes.h"
#include "NetworkManager.h"
#include "ReadPointFile.h"
#include "bim/gltf.h"
#include "bim/mbr.h"
#include "bim/BimInfo.h"

namespace fs = std::filesystem;

namespace DPApp {

    /**
     * @brief Task execution status enumeration
     */
    enum class TaskStatus {
        PENDING,        // Waiting for assignment
        ASSIGNED,       // Assigned to a worker
        IN_PROGRESS,    // Currently processing
        COMPLETED,      // Successfully completed
        FAILED,         // Failed execution
        TIMEOUT         // Timed out
    };

    /**
     * @brief Task priority levels
     */
    enum class TaskPriority {
        LOW = 0,
        NORMAL = 1,
        HIGH = 2
    };

    /**
     * @brief General task information structure
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

        TaskInfo() : task_id(0), chunk_id(0), task_type(TaskType::CONVERT_PTS),
            priority(TaskPriority::NORMAL), status(TaskStatus::PENDING),
            retry_count(0), max_retries(3), created_time(std::chrono::steady_clock::now()) {
        }

        /**
         * @brief Calculate task execution time in seconds
         */
        double getExecutionTime() const {
            if (status != TaskStatus::COMPLETED) return 0.0;

            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                completed_time - assigned_time);
            return duration.count() / 1000.0;
        }
    };

    /**
     * @brief Slave worker information and statistics
     */
    struct SlaveWorkerInfo {
        std::string slave_id;
        bool is_active;
        bool is_busy;
        std::string current_task_id;
        std::chrono::steady_clock::time_point last_heartbeat;
        uint32_t completed_tasks;
        uint32_t failed_tasks;
        double avg_processing_time;  // Average processing time in seconds

        SlaveWorkerInfo() : is_active(false), is_busy(false), completed_tasks(0),
            failed_tasks(0), avg_processing_time(0.0),
            last_heartbeat(std::chrono::steady_clock::now()) {
        }

        /**
         * @brief Calculate success rate percentage
         */
        double getSuccessRate() const {
            uint32_t total = completed_tasks + failed_tasks;
            return total > 0 ? static_cast<double>(completed_tasks) / total : 1.0;
        }
    };

    /**
     * @brief Point cloud loading utilities
     */
    namespace PointCloudLoader {
        std::vector<std::shared_ptr<PointCloudChunk>> loadPointCloudFileInChunks(
            const std::string& file_path,
            uint32_t max_points_per_chunk = MAX_NUM_PTS_CHUNK);

        std::shared_ptr<PointCloudChunk> loadXYZFile(const std::string& file_path);

        std::vector<std::shared_ptr<PointCloudChunk>> loadXYZFileStreaming(
            const std::string& file_path,
            uint32_t max_points_per_chunk);
    }

    /**
     * @brief Task Manager class for distributed processing (Master node)
     * Handles one task type per execution session
     */
    class TaskManager {
    private:
        std::atomic<bool> stopping_{ false };
        TaskType current_task_type_;        // Fixed task type for this session
        std::vector<uint8_t> task_parameters_; // Fixed parameters for this session

    public:
        explicit TaskManager(const uint32_t timeout_seconds, TaskType task_type, const std::vector<uint8_t>& parameters)
            : next_task_id_(1), task_timeout_seconds_(timeout_seconds), current_slave_index_(0),
            current_task_type_(task_type), task_parameters_(parameters) {
            std::cout << "TaskManager initialized for task type: " << taskStr(task_type)
                << " with " << timeout_seconds << "s timeout" << std::endl;
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

        /**
         * @brief Add a new task to the processing queue (simplified - no task type parameter)
         */
        uint32_t addTask(std::shared_ptr<PointCloudChunk> chunk) {
            std::lock_guard<std::mutex> lock(tasks_mutex_);

            uint32_t task_id = next_task_id_++;

            TaskInfo task_info;
            task_info.task_id = task_id;
            task_info.chunk_id = chunk->chunk_id;
            task_info.task_type = current_task_type_;    // Use session task type
            task_info.priority = TaskPriority::NORMAL;   // All tasks same priority
            task_info.parameters = task_parameters_;     // Use session parameters
            task_info.chunk_data = chunk;

            tasks_[task_id] = std::move(task_info);

            std::cout << "Task added: " << task_id << " (" << taskStr(current_task_type_) << ")" << std::endl;

            return task_id;
        }

        /**
         * @brief Clear all tasks from the manager
         */
        void clearAllTasks() {
            std::lock_guard<std::mutex> lock(tasks_mutex_);
            size_t count = tasks_.size();
            tasks_.clear();
            std::cout << "Cleared " << count << " tasks" << std::endl;
        }

        /**
         * @brief Assign pending tasks to available slave workers
         */
        bool assignTasksToSlaves() {
            if (stopping_) {
                return false;  // Don't assign tasks when stopping
            }

            std::lock_guard<std::mutex> tasks_lock(tasks_mutex_);
            std::lock_guard<std::mutex> slaves_lock(slaves_mutex_);

            // Find pending tasks and sort by priority
            std::vector<uint32_t> pending_tasks;
            for (auto& [task_id, task_info] : tasks_) {
                if (task_info.status == TaskStatus::PENDING) {
                    pending_tasks.push_back(task_id);
                }
            }

            if (pending_tasks.empty()) {
                return false;
            }

            // Sort by priority (highest first)
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

                    // Update slave status
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

        /**
         * @brief Update task status
         */
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

        /**
         * @brief Complete a task with results
         */
        bool completeTask(uint32_t task_id, const ProcessingResult& result) {
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

                // Update slave statistics
                if (!task_info.assigned_slave.empty()) {
                    auto slave_it = slaves_.find(task_info.assigned_slave);
                    if (slave_it != slaves_.end()) {
                        slave_it->second.is_busy = false;
                        slave_it->second.current_task_id.clear();
                        slave_it->second.completed_tasks++;

                        // Calculate processing time
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
                failTaskInternal(task_id, result.error_message);
            }

            return true;
        }

        /**
         * @brief Mark a task as failed
         */
        bool failTask(uint32_t task_id, const std::string& error_message) {
            std::lock_guard<std::mutex> tasks_lock(tasks_mutex_);
            std::lock_guard<std::mutex> slaves_lock(slaves_mutex_);

            failTaskInternal(task_id, error_message);
            return true;
        }

        /**
         * @brief Register a new slave worker
         */
        void registerSlave(const std::string& slave_id) {
            std::lock_guard<std::mutex> lock(slaves_mutex_);

            SlaveWorkerInfo slave_info;
            slave_info.slave_id = slave_id;
            slave_info.is_active = true;

            slaves_[slave_id] = slave_info;

            std::cout << "Slave registered: " << slave_id << std::endl;
        }

        /**
         * @brief Unregister a slave worker and reassign its tasks
         */
        void unregisterSlave(const std::string& slave_id) {
            std::lock_guard<std::mutex> slaves_lock(slaves_mutex_);
            std::lock_guard<std::mutex> tasks_lock(tasks_mutex_);

            auto slave_it = slaves_.find(slave_id);
            if (slave_it != slaves_.end()) {
                // Reassign tasks from the unregistered slave back to pending
                int reassigned_tasks = 0;
                for (auto& [task_id, task_info] : tasks_) {
                    if (task_info.assigned_slave == slave_id &&
                        (task_info.status == TaskStatus::ASSIGNED || task_info.status == TaskStatus::IN_PROGRESS)) {
                        task_info.status = TaskStatus::PENDING;
                        task_info.assigned_slave.clear();
                        reassigned_tasks++;
                    }
                }

                slaves_.erase(slave_it);
                std::cout << "Slave unregistered: " << slave_id << " (" << reassigned_tasks << " tasks reassigned)" << std::endl;
            }
        }

        /**
         * @brief Update slave heartbeat timestamp
         */
        void updateSlaveHeartbeat(const std::string& slave_id) {
            std::lock_guard<std::mutex> lock(slaves_mutex_);

            auto it = slaves_.find(slave_id);
            if (it != slaves_.end()) {
                it->second.last_heartbeat = std::chrono::steady_clock::now();
            }
        }

        // Status query methods
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

        std::vector<SlaveWorkerInfo> getAllSlaves() {
            std::lock_guard<std::mutex> lock(slaves_mutex_);

            std::vector<SlaveWorkerInfo> result;
            for (const auto& pair : slaves_) {
                result.push_back(pair.second);
            }

            return result;
        }

        // Statistics methods
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

        /**
         * @brief Calculate overall processing progress
         */
        double getOverallProgress() {
            std::lock_guard<std::mutex> lock(tasks_mutex_);

            if (tasks_.empty()) {
                return 1.0; // 100% if no tasks
            }

            std::vector<TaskStatus> statuses;
            statuses.reserve(tasks_.size());
            for (const auto& [task_id, task_info] : tasks_) {
                statuses.push_back(task_info.status);
            }

            size_t completed = std::count(statuses.begin(), statuses.end(), TaskStatus::COMPLETED);
            return static_cast<double>(completed) / statuses.size();
        }

        /**
         * @brief Set task timeout in seconds
         */
        void setTaskTimeout(uint32_t timeout_seconds) {
            task_timeout_seconds_ = timeout_seconds;
            std::cout << "Task timeout set to " << timeout_seconds << " seconds" << std::endl;
        }

        /**
         * @brief Check for timed out tasks and slaves
         */
        void checkTimeouts() {
            std::lock_guard<std::mutex> tasks_lock(tasks_mutex_);

            auto now = std::chrono::steady_clock::now();

            // Check task timeouts
            int timed_out_count = 0;
            for (auto& [task_id, task_info] : tasks_) {
                if (task_info.status == TaskStatus::ASSIGNED || task_info.status == TaskStatus::IN_PROGRESS) {
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        now - task_info.assigned_time).count();

                    if (elapsed > static_cast<int64_t>(task_timeout_seconds_)) {
                        std::cout << "Task " << task_id << " timed out after " << elapsed << " seconds" << std::endl;
                        task_info.status = TaskStatus::TIMEOUT;
                        timed_out_count++;

                        // Release slave
                        if (!task_info.assigned_slave.empty()) {
                            std::lock_guard<std::mutex> slaves_lock(slaves_mutex_);
                            auto slave_it = slaves_.find(task_info.assigned_slave);
                            if (slave_it != slaves_.end()) {
                                slave_it->second.is_busy = false;
                                slave_it->second.current_task_id.clear();
                            }
                        }

                        // Retry or fail permanently
                        failTask(task_id, "Task timeout");
                    }
                }
            }

            if (timed_out_count > 0) {
                std::cout << "Found " << timed_out_count << " timed out tasks" << std::endl;
            }
        }

        // Result management
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

        // Callback setup
        using TaskCompletedCallback = std::function<void(const TaskInfo&, const ProcessingResult&)>;
        using TaskFailedCallback = std::function<void(const TaskInfo&, const std::string&)>;

        void setTaskCompletedCallback(TaskCompletedCallback callback) {
            task_completed_callback_ = callback;
        }

        void setTaskFailedCallback(TaskFailedCallback callback) {
            task_failed_callback_ = callback;
        }

    private:
        /**
         * @brief Internal task failure handling with retry logic
         */
        void failTaskInternal(uint32_t task_id, const std::string& error_message) {
            auto task_it = tasks_.find(task_id);
            if (task_it == tasks_.end()) {
                return;
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
                task_info.status = TaskStatus::FAILED;
                if (task_failed_callback_) {
                    task_failed_callback_(task_info, error_message);
                }
                std::cout << "Task " << task_id << " failed permanently." << std::endl;
            }
        }

        // Simple round-robin slave selection
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

                // Update average processing time (exponential moving average)
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

        // Member variables
        std::map<uint32_t, TaskInfo> tasks_;
        std::map<std::string, SlaveWorkerInfo> slaves_;
        std::vector<ProcessingResult> completed_results_;

        std::mutex tasks_mutex_;
        std::mutex slaves_mutex_;
        std::mutex results_mutex_;

        std::atomic<uint32_t> next_task_id_;
        uint32_t task_timeout_seconds_;
        size_t current_slave_index_;  // For round-robin

        TaskCompletedCallback task_completed_callback_;
        TaskFailedCallback task_failed_callback_;
    };

    /**
     * @brief Task Processor class for individual workers (Slave nodes)
     * Fixed 2 task types: CONVERT_PTS and BIM_DISTANCE_CALCULATION
     */
    class TaskProcessor {
    public:
        TaskProcessor();
        ~TaskProcessor();

        // Task processing - handles both supported task types internally
        ProcessingResult processTask(const ProcessingTask& task, const PointCloudChunk& chunk);

        // Supported task type queries
        std::vector<TaskType> getSupportedTaskTypes() const;
        bool supportsTaskType(const TaskType task_type) const;

        // Statistics
        uint32_t getProcessedTaskCount() const { return processed_task_count_; }
        uint32_t getFailedTaskCount() const { return failed_task_count_; }
        double getAverageProcessingTime() const { return avg_processing_time_; }

    private:
        std::atomic<uint32_t> processed_task_count_;
        std::atomic<uint32_t> failed_task_count_;
        std::atomic<double> avg_processing_time_;
    };

    /**
     * @brief Simple point cloud processing functions
     */
    namespace PointCloudProcessors {
        /// Convert pts format
        ProcessingResult convertPts(const ProcessingTask& task, const PointCloudChunk& chunk);

        /// Simple distance calculation (simplified BIM functionality)
        ProcessingResult calculateDistance(const ProcessingTask& task, const PointCloudChunk& chunk);
    }

    ///
    /// Implementation Section
    ///

    // Point cloud loading utilities implementation
    namespace PointCloudLoader {

        std::vector<std::shared_ptr<PointCloudChunk>> loadPointCloudFileInChunks(
            const std::string& file_path, uint32_t max_points_per_chunk) {

            std::vector<std::shared_ptr<PointCloudChunk>> chunks;

            try {
                std::cout << "Loading point cloud file: " << file_path << std::endl;

                // Check file extension
                std::string extension = std::filesystem::path(file_path).extension().string();
                std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

                if (extension == ".xyz" || extension == ".pts") {
                    chunks = loadXYZFileStreaming(file_path, max_points_per_chunk);
                }
                else {
                    std::cerr << "Unsupported file format: " << extension << std::endl;
                    return chunks;
                }

                std::cout << "Created " << chunks.size() << " chunks" << std::endl;

            }
            catch (const std::exception& e) {
                std::cerr << "Error loading file: " << e.what() << std::endl;
            }

            return chunks;
        }

        std::shared_ptr<PointCloudChunk> loadXYZFile(const std::string& file_path) {
            auto chunk = std::make_shared<PointCloudChunk>();
            chunk->chunk_id = 0;

            try {
                std::ifstream file(file_path);
                if (!file.is_open()) {
                    std::cerr << "Cannot open XYZ file: " << file_path << std::endl;
                    return chunk;
                }

                std::string line;
                size_t line_count = 0;

                while (std::getline(file, line)) {
                    line_count++;

                    // Skip empty lines and comments
                    if (line.empty() || line[0] == '#') continue;

                    std::istringstream iss(line);
                    Point3D point;

                    if (iss >> point.x >> point.y >> point.z) {
                        // Try to read additional fields (intensity, colors)
                        float temp_intensity = 0;
                        int temp_r = 128, temp_g = 128, temp_b = 128;

                        iss >> temp_intensity >> temp_r >> temp_g >> temp_b;

                        point.intensity = static_cast<uint16_t>(temp_intensity);
                        point.r = static_cast<uint8_t>(temp_r);
                        point.g = static_cast<uint8_t>(temp_g);
                        point.b = static_cast<uint8_t>(temp_b);
                        point.classification = 0;

                        chunk->points.push_back(point);
                    }

                    // Progress indicator for large files
                    if (line_count % 100000 == 0) {
                        std::cout << "Loaded " << chunk->points.size() << " points..." << std::endl;
                    }
                }

                file.close();
                std::cout << "Loaded " << chunk->points.size() << " points from XYZ file" << std::endl;

            }
            catch (const std::exception& e) {
                std::cerr << "Error reading XYZ file " << file_path << ": " << e.what() << std::endl;
            }

            return chunk;
        }

        std::vector<std::shared_ptr<PointCloudChunk>> loadXYZFileStreaming(
            const std::string& file_path, uint32_t max_points_per_chunk) {

            std::vector<std::shared_ptr<PointCloudChunk>> chunks;

            try {
                std::ifstream file(file_path);
                if (!file.is_open()) {
                    std::cerr << "Cannot open XYZ file: " << file_path << std::endl;
                    return chunks;
                }

                auto current_chunk = std::make_shared<PointCloudChunk>();
                current_chunk->chunk_id = 0;
                current_chunk->points.reserve(max_points_per_chunk);

                std::string line;
                size_t line_count = 0;
                size_t points_in_current_chunk = 0;
                size_t total_points_processed = 0;
                uint32_t chunk_counter = 0;

                while (std::getline(file, line)) {
                    line_count++;

                    // Skip empty line or comment line
                    if (line.empty() || line[0] == '#') continue;

                    std::istringstream iss(line);
                    Point3D point;

                    if (iss >> point.x >> point.y >> point.z) {
                        // Attempt to read additional fields
                        float temp_intensity = 0;
                        int temp_r = 128, temp_g = 128, temp_b = 128;

                        iss >> temp_intensity >> temp_r >> temp_g >> temp_b;

                        point.intensity = static_cast<uint16_t>(temp_intensity);
                        point.r = static_cast<uint8_t>(temp_r);
                        point.g = static_cast<uint8_t>(temp_g);
                        point.b = static_cast<uint8_t>(temp_b);
                        point.classification = 0;

                        current_chunk->points.push_back(point);
                        points_in_current_chunk++;
                        total_points_processed++;

                        // Check chunk size limit
                        if (points_in_current_chunk >= max_points_per_chunk) {
                            // Complete chunk
                            chunks.push_back(current_chunk);

                            // Start new chunk
                            chunk_counter++;
                            current_chunk = std::make_shared<PointCloudChunk>();
                            current_chunk->chunk_id = chunk_counter;
                            current_chunk->points.reserve(max_points_per_chunk);
                            points_in_current_chunk = 0;
                        }
                    }

                    // Progress (every 100k lines)
                    if (line_count % 100000 == 0) {
                        std::cout << "Processing line: " << line_count << std::endl;
                    }
                }

                // Last chunk
                if (points_in_current_chunk > 0) {
                    chunks.push_back(current_chunk);
                }

                file.close();

                std::cout << "Streaming load completed: " << chunks.size() << " chunks, "
                    << total_points_processed << " total points" << std::endl;

            }
            catch (const std::exception& e) {
                std::cerr << "Error in XYZ streaming: " << e.what() << std::endl;
            }

            return chunks;
        }

    } // namespace PointCloudLoader

    /// TaskProcessor Implementation
    TaskProcessor::TaskProcessor()
        : processed_task_count_(0), failed_task_count_(0), avg_processing_time_(0.0) {
        std::cout << "TaskProcessor initialized with 2 fixed task types (CONVERT_PTS, BIM_DISTANCE_CALCULATION)" << std::endl;
    }

    TaskProcessor::~TaskProcessor() {
        std::cout << "TaskProcessor destroyed" << std::endl;
    }

    ProcessingResult TaskProcessor::processTask(const ProcessingTask& task, const PointCloudChunk& chunk) {
        auto start_time = std::chrono::steady_clock::now();

        ProcessingResult result;
        result.task_id = task.task_id;
        result.chunk_id = task.chunk_id;

        try {
            // Direct switch on task type - no dynamic registration needed
            switch (task.task_type) {
            case TaskType::CONVERT_PTS:
                result = PointCloudProcessors::convertPts(task, chunk);
                break;

            case TaskType::BIM_DISTANCE_CALCULATION:
                result = PointCloudProcessors::calculateDistance(task, chunk);
                break;

            default:
                result.success = false;
                result.error_message = "Unsupported task type: " + std::string(taskStr(task.task_type));
                failed_task_count_++;
                return result;
            }

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

        // Update processing time statistics
        auto end_time = std::chrono::steady_clock::now();
        auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time).count() / 1000.0;

        uint32_t total_tasks = processed_task_count_ + failed_task_count_;
        if (total_tasks > 0) {
            avg_processing_time_ = (avg_processing_time_ * (total_tasks - 1) + processing_time) / total_tasks;
        }

        return result;
    }

    std::vector<TaskType> TaskProcessor::getSupportedTaskTypes() const {
        return { TaskType::CONVERT_PTS, TaskType::BIM_DISTANCE_CALCULATION };
    }

    bool TaskProcessor::supportsTaskType(const TaskType task_type) const {
        return task_type == TaskType::CONVERT_PTS || task_type == TaskType::BIM_DISTANCE_CALCULATION;
    }

    /// PointCloudProcessors Implementation
    namespace PointCloudProcessors {

        ProcessingResult convertPts(const ProcessingTask& task, const PointCloudChunk& chunk) {
            ProcessingResult result;
            result.task_id = task.task_id;
            result.chunk_id = task.chunk_id;
            result.success = true;

            try {
                std::cout << "Converting PTS chunk " << chunk.chunk_id
                    << " with " << chunk.points.size() << " points" << std::endl;

                // Simple PTS conversion - just copy points with potential format adjustments
                result.processed_points = chunk.points;

                // Example: Apply coordinate offset if specified in parameters
                if (!task.parameters.empty() && task.parameters.size() >= sizeof(double) * 3) {
                    double offset_x, offset_y, offset_z;
                    std::memcpy(&offset_x, task.parameters.data(), sizeof(double));
                    std::memcpy(&offset_y, task.parameters.data() + sizeof(double), sizeof(double));
                    std::memcpy(&offset_z, task.parameters.data() + sizeof(double) * 2, sizeof(double));

                    for (auto& point : result.processed_points) {
                        point.x += static_cast<float>(offset_x);
                        point.y += static_cast<float>(offset_y);
                        point.z += static_cast<float>(offset_z);
                    }

                    std::cout << "Applied offset: (" << offset_x << ", " << offset_y << ", " << offset_z << ")" << std::endl;
                }

                std::cout << "PTS conversion completed for " << result.processed_points.size() << " points" << std::endl;

            }
            catch (const std::exception& e) {
                result.success = false;
                result.error_message = "PTS conversion error: " + std::string(e.what());
            }

            return result;
        }

        ProcessingResult calculateDistance(const ProcessingTask& task, const PointCloudChunk& chunk) {
            ProcessingResult result;
            result.task_id = task.task_id;
            result.chunk_id = task.chunk_id;
            result.success = true;

            try {
                std::cout << "Calculating distances for chunk " << chunk.chunk_id
                    << " with " << chunk.points.size() << " points" << std::endl;

                // Simple distance calculation - calculate distance from origin for each point
                result.processed_points = chunk.points;

                // Calculate distance from origin and store in intensity field
                for (auto& point : result.processed_points) {
                    float distance = static_cast<float>(std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z));

                    // Store distance in intensity field (convert to millimeters, clamped to uint16_t range)
                    point.intensity = static_cast<uint16_t>((std::min)(distance * 1000, 65535.0f));

                    // Color code by distance (blue = close, red = far)
                    float normalized_distance = (std::min)(distance / 100.0f, 1.0f); // Normalize to 100m max

                    point.r = static_cast<uint8_t>(normalized_distance * 255);
                    point.g = static_cast<uint8_t>((1.0f - normalized_distance) * 128);
                    point.b = static_cast<uint8_t>((1.0f - normalized_distance) * 255);
                }

                std::cout << "Distance calculation completed for " << result.processed_points.size() << " points" << std::endl;

            }
            catch (const std::exception& e) {
                result.success = false;
                result.error_message = "Distance calculation error: " + std::string(e.what());
            }

            return result;
        }

    } // namespace PointCloudProcessors

    namespace MeshProcessors {
        std::string wstring_to_utf8(const std::wstring& wstr) {
            int size_needed = WideCharToMultiByte(CP_UTF8, 0, wstr.data(), (int)wstr.size(), NULL, 0, NULL, NULL);
            std::string utf8str(size_needed, 0);
            WideCharToMultiByte(CP_UTF8, 0, wstr.data(), (int)wstr.size(), &utf8str[0], size_needed, NULL, NULL);
            return utf8str;
        }

        bool convert_gltf_to_nodes2(const std::string& dataset_name,
            const std::string& bim_folder,
            const std::string& nodes2_folder,
            const bool is_offset_applied,
            const float* offset,
            const std::string& compressed_bim_file_path) {
            std::filesystem::path input_path(bim_folder); // glb 파일이 저장된 폴더 경로
            std::filesystem::path output_path(nodes2_folder); // .nodes2 파일을 저장할 폴더 경로

            if (std::filesystem::exists(input_path) == false)
            {
                std::cout << "input_folder '" << bim_folder << "' is not exist" << std::endl;
                return false;
            }

            size_t total_gltf_files = std::distance(fs::directory_iterator(input_path), fs::directory_iterator());
            int processed_gltf_files = 0;

            std::string output_file_name = dataset_name + ".nodes2"; // binary file name
            auto full_path = output_path / output_file_name;
            output_file_name = full_path.string();
            
            //float global_bbox_min[3] = { NUM_MAX, NUM_MAX, NUM_MAX }; // 최대값으로 초기화
            //float global_bbox_max[3] = { NUM_MIN, NUM_MIN, NUM_MIN }; // 최소값으로 초기화
            float global_bbox_min[3] = { (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::max)() }; // 최대값으로 초기화
            float global_bbox_max[3] = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() }; // 최소값으로 초기화

            // .nodes2 파일로 geometry 정보 출력하기 위한 outputfilestream 생성
            std::ofstream* filestream = nullptr;
            filestream = new std::ofstream(output_file_name, std::ios::out | std::ios::binary);

            if (!filestream->good())
            {
                std::cerr << "file open failure : " << output_file_name << std::endl;
                return false;
            }
            // Iterate through GLTF files in the directory
            int idx = 1000; // bim_id의 초기값 - 파일명에 Revit Element ID가 없는 경우 사용됨    
            for (const auto& entry : std::filesystem::directory_iterator(input_path))
            {
                std::wcout << entry.path().wstring() << std::endl; // unicode string 
                std::cout << entry.path().string() << std::endl; // utf8 string // 영문 windows에서는 주석처리 해야 함
                if (!entry.is_regular_file())
                {
                    std::wcout << entry.path().wstring() << " not regular file" << std::endl;
                    std::cout << entry.path().string() << " not regular file" << std::endl; // utf8 string // 영문 windows에서는 주석처리 해야 함
                    continue;
                }

                auto& data_file = entry.path();
                if (data_file.extension().compare(".gltf") != 0 && data_file.extension().compare(".glb") != 0)
                {
                    if (data_file.extension().compare(".bin") != 0)
                        std::cout << data_file << " not gltf(glb) file" << std::endl;
                    continue;
                }

                // UTF-16을 UTF-8로 변환
                const std::string utf8FileName = wstring_to_utf8(data_file);
                std::string gltf_file_name = utf8FileName;

                // create new gltf_file object
                gltf gltf_file;
                BimInfo bim_info = gltf_file.read(gltf_file_name);  // read gltf file and get BimInfo object

                // bim_id, node_name 설정
                int bim_id = idx;
                std::string node_name;
                try
                {
                    // 파일 이름에서 확장자를 제외하고 '_'로 분할하여 마지막 문자열을 bim_id로 설정합니다.
                    std::wstring filenameW = data_file.stem().wstring(); // 파일 이름 - convert to unicode   
                    std::string filename = wstring_to_utf8(filenameW);

                    std::vector<std::string> parts;
                    std::stringstream ss(filename);
                    std::string item;
                    while (std::getline(ss, item, '_')) {// '_'로 분할
                        parts.push_back(item);
                    }

                    // 문자열의 마지막 부분(Revit Element ID)를 bim_id로 변환합니다.
                    if (!parts.empty()) {
                        //bim_id = boost::lexical_cast<unsigned long>(parts.back());
                        std::string lastPart = parts.back();
                        // 숫자만 남기기 위해 문자 제거
                        lastPart.erase(std::remove_if(lastPart.begin(), lastPart.end(), [](char c) { return !std::isdigit(c); }), lastPart.end());
                        if (!lastPart.empty()) {
                            try {
                                bim_id = std::stoul(lastPart); // boost::lexical_cast 대체
                            }
                            catch (const std::exception& e) {
                                std::cerr << "Error: Failed to convert to number: " << e.what() << std::endl;
                                bim_id = ++idx;
                            }
                        }
                        else {
                            std::cerr << "Error: No numeric part found in the last part of filename" << std::endl;
                            bim_id = ++idx; // 기본값으로 idx를 증가시킴
                        }
                    }
                    else {
                        std::cerr << "Error: Filename does not contain valid parts" << std::endl;
                        bim_id = ++idx; // 기본값으로 idx를 증가시킴
                    }

                    // 마지막 '_'의 위치를 찾고 앞부분(node name)을 가져옵니다.
                    size_t lastUnderscorePos = filename.find_last_of("_");
                    node_name = filename.substr(0, lastUnderscorePos); // node name 
                }
                catch (const std::exception& e)
                {
                    std::cerr << data_file.stem() << std::endl;
                    std::cerr << e.what() << std::endl;
                    bim_id = ++idx;
                }

                // utf8FileName의 파일명만 남기고 나머지는 지우기.
                std::string node_name_utf8 = utf8FileName;
                std::size_t lastSlashPos = node_name_utf8.find_last_of("\\");
                node_name_utf8 = node_name_utf8.substr(lastSlashPos + 1);
                // 확장자도 지우기
                std::size_t lastDotPos = node_name_utf8.find_last_of(".");
                node_name_utf8 = node_name_utf8.substr(0, lastDotPos);
                // 마지막 '_'의 위치를 찾고 앞부분(node name)을 가져옵니다.
                size_t lastUnderscorePos = node_name_utf8.find_last_of("_");
                node_name_utf8 = node_name_utf8.substr(0, lastUnderscorePos);
                // node_name 맨 마지막 문자가 '_'이면 ''로 변경
                if (node_name_utf8.back() == '_') { node_name_utf8.pop_back(); }

                //std::cout << bim_id << ": " << node_name << " " << node_name_utf8 << std::endl;

                // vertex buffer(geometry)를 가져옴 (offset 적용 (translation))
                std::size_t buffer_size = 0;
                char* vertex_buffer = bim_info.get_vertex_buffer(is_offset_applied, offset, buffer_size);

                // bounding box, MBR 정보를 가져옴
                MBR bbox = bim_info.get_mbr(is_offset_applied, offset);
                auto bbox_min = bbox.get_min(); // bbox의 최소값을 복사합니다.
                auto bbox_max = bbox.get_max(); // bbox의 최대값을 복사합니다.
                // 전역 bounding box 정보 업데이트
                for (int i = 0; i < 3; ++i) {
                    global_bbox_min[i] = (std::min)(global_bbox_min[i], bbox_min[i]);
                    global_bbox_max[i] = (std::max)(global_bbox_max[i], bbox_max[i]);
                }

                delete[] vertex_buffer;

                // .nodes2 파일에 bim_info 저장
                bim_info.save(bim_id, node_name, is_offset_applied, offset, *filestream);

                // Progress 출력
                processed_gltf_files++;
                int progress = (int)((float)processed_gltf_files / total_gltf_files * 100);
                if (processed_gltf_files % 100 == 0 || progress == 100)
                    std::cout << "Processed " << processed_gltf_files << " / " << total_gltf_files << " (" << progress << "%)" << std::endl;
            }

            filestream->close();

            return true;
        }
    }

} // namespace DPApp