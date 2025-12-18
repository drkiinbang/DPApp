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
#include "bim/gltf_mesh.h"
#include "bim/mbr.h"
#include "bim/BimInfo.h"
#include "DefinePointType.hpp"
#include "LaslibReader.hpp"
#include "Pc2Reader.hpp"
#include "bimtree/nanoflann.hpp"

namespace fs = std::filesystem;

const std::size_t CHUNK_POINTS = 1000000;

const double _PI_ = acos(-1.0);

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

        std::vector<std::shared_ptr<PointCloudChunk>> loadXYZFileStreaming(const std::string& file_path, uint32_t max_points_per_chunk);

        std::vector<std::shared_ptr<PointCloudChunk>> loadLasFileStreaming(const std::string& file_path, uint32_t max_points_per_chunk);
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
            std::lock_guard<std::mutex> tasks_lock(tasks_mutex_);
            std::lock_guard<std::mutex> slaves_lock(slaves_mutex_);
            
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

                /// Check file extension
                std::string extension = std::filesystem::path(file_path).extension().string();
                std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

                if (extension == ".xyz" || extension == ".pts") {
                    chunks = loadXYZFileStreaming(file_path, max_points_per_chunk);
                }
                else if (extension == ".las" || extension == ".laz") {
                    chunks = loadLasFileStreaming(file_path, max_points_per_chunk);
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

        std::vector<std::shared_ptr<PointCloudChunk>> loadXYZFileStreaming(const std::string& file_path, const uint32_t max_points_per_chunk) {

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

                    if (iss >> point[0] >> point[1] >> point[2]) {
                        // 추가 필드가 있으면 읽지만 사용하지 않음 (파싱 오류 방지)
                        // float temp_intensity = 0;
                        // int temp_r = 128, temp_g = 128, temp_b = 128;
                        // iss >> temp_intensity >> temp_r >> temp_g >> temp_b;

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

        std::vector<std::shared_ptr<PointCloudChunk>> loadLasFileStreaming(const std::string& file_path, const uint32_t max_points_per_chunk)
        {
            std::vector<std::shared_ptr<PointCloudChunk>> chunks;

            try {
                las::LASToolsReader lasReader;

                if (!lasReader.open(file_path)) {
                    std::cerr << "Failed to open input file: " << file_path << std::endl;
                    return chunks;
                }

                size_t total_num_points = lasReader.getPointCount();
                chunks.reserve(size_t(total_num_points / max_points_per_chunk) + 1);
                size_t total_points_loaded = 0;
                uint32_t chunk_counter = 0;

                for (size_t start = 0; start < total_num_points; start += max_points_per_chunk) {
                    std::size_t end = (std::min)(start + max_points_per_chunk, total_num_points);
                    if (!lasReader.loadPointRange(start, end)) {
                        std::cerr << "Failed in loadPointRange(" << start << ", " << end << ")\n";
                        break;
                    }

                    auto current_chunk = std::make_shared<PointCloudChunk>();
                    current_chunk->chunk_id = chunk_counter;
                    current_chunk->points.reserve(lasReader.getLoadedPointCount());

                    for (const auto& p : lasReader.getLoadedPoints()) {
                        current_chunk->points.emplace_back(p.x, p.y, p.z);
                    }

                    total_points_loaded += current_chunk->points.size();
                    chunks.push_back(current_chunk);
                    chunk_counter++;

                    // Progress reporting (every chunk)
                    if (chunk_counter % 10 == 0) {
                        std::cout << "Loaded chunks: " << chunk_counter
                            << ", points: " << total_points_loaded << std::endl;
                    }
                }

                lasReader.close();

                std::cout << "Streaming load completed: " << chunks.size() << " chunks, "
                    << total_points_loaded << " total points" << std::endl;
            }
            catch (const std::exception& e) {
                std::cerr << "Error in Las streaming: " << e.what() << std::endl;
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
        return { 
            TaskType::CONVERT_PTS, 
            TaskType::BIM_DISTANCE_CALCULATION
        };
    }

    bool TaskProcessor::supportsTaskType(const TaskType task_type) const {
        return task_type == TaskType::CONVERT_PTS || 
               task_type == TaskType::BIM_DISTANCE_CALCULATION;
    }

    namespace util {
        std::string wstring_to_utf8(const std::wstring& wstr) {
            int size_needed = WideCharToMultiByte(CP_UTF8, 0, wstr.data(), (int)wstr.size(), NULL, 0, NULL, NULL);
            std::string utf8str(size_needed, 0);
            WideCharToMultiByte(CP_UTF8, 0, wstr.data(), (int)wstr.size(), &utf8str[0], size_needed, NULL, NULL);
            return utf8str;
        }
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
                        point[0] += static_cast<float>(offset_x);
                        point[1] += static_cast<float>(offset_y);
                        point[2] += static_cast<float>(offset_z);
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
                    float distance = static_cast<float>(std::sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]));
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

    namespace Pt2MeshProcessors {
        bool convert_gltf2nodes2(const std::string& dataset_name,
            const std::string& bim_folder,
            const std::string& nodes2_folder,
            std::vector<BimMeshInfo>& bimData,
            const bool is_offset_applied,
            const float* offset) {
            std::filesystem::path input_path(bim_folder); // glb 파일이 저장된 폴더 경로
            std::filesystem::path output_path(nodes2_folder); // .nodes2 파일을 저장할 폴더 경로

            if (std::filesystem::exists(input_path) == false) {
                std::cout << "input_folder '" << bim_folder << "' is not exist" << std::endl;
                return false;
            }

            size_t total_gltf_files = std::distance(fs::directory_iterator(input_path), fs::directory_iterator());
            int processed_gltf_files = 0;

            std::string output_file_name = dataset_name + ".nodes2"; // binary file name
            auto full_path = output_path / output_file_name;
            output_file_name = full_path.string();
            
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

            auto numEntries = std::distance(fs::directory_iterator(input_path), fs::directory_iterator{});
            bimData.clear();
            bimData.reserve(numEntries);

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
                const std::string utf8FileName = util::wstring_to_utf8(data_file);
                std::string gltf_file_name = utf8FileName;

                // create new gltf_file object
                GltfMesh gltf_file;
                bimData.push_back(BimMeshInfo());
                BimMeshInfo& singleFile = bimData.back();
                // read gltf file and get BimInfo object1
                if (!gltf_file.read(gltf_file_name, singleFile, is_offset_applied, offset)) {
                    bimData.pop_back();
                    continue;
                }

                auto& bim_info = singleFile;
                
                // bim_id, node_name 설정
                int bim_id = idx;
                std::string node_name;
                try
                {
                    // 파일 이름에서 확장자를 제외하고 '_'로 분할하여 마지막 문자열을 bim_id로 설정합니다.
                    std::wstring filenameW = data_file.stem().wstring(); // 파일 이름 - convert to unicode   
                    std::string filename = util::wstring_to_utf8(filenameW);

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

       std::vector<std::string> split(const std::string& str, const std::string& delims = "\t, ") {
            std::vector<std::string> tokens;
            size_t start = str.find_first_not_of(delims), end = 0;

            while (start != std::string::npos) {
                end = str.find_first_of(delims, start);
                if (end == std::string::npos) {
                    tokens.push_back(str.substr(start));
                    break;
                }
                else {
                    tokens.push_back(str.substr(start, end - start));
                    start = str.find_first_not_of(delims, end);
                }
            }
            return tokens;
        }

        bool convert_pts2pc2(const std::string& input_file_path,
            const std::string& output_file_path,
            const bool is_offset_applied,
            const float* offset)
        {
            auto start_t = std::chrono::system_clock::now();

            std::ifstream pts_file(input_file_path);
            if (!pts_file.is_open()) {
                std::cerr << "Failed to open input file: " << input_file_path << std::endl;
                return false;
            }

            /// Check the output folder path
            std::error_code ec;
            std::string* err = nullptr;
            std::filesystem::path output_path(output_file_path);
            fs::path dir = output_path.parent_path();
            if (!dir.empty() && !fs::exists(dir)) {
                if (!fs::create_directories(dir, ec)) {
                    std::cerr << "Failed in creating directories: " + dir.string();
                    return false;
                }
                else
                    std::cerr << "Create a directory: " + dir.string();
            }

            auto file_open_mode = std::ios::out | std::ios::binary;
            std::ofstream result_point_cloud_file(output_file_path, file_open_mode);
            if (!result_point_cloud_file.is_open()) {
                std::cerr << "Failed to open output binary file: " << output_file_path << std::endl;
                pts_file.close();
                return false;
            }

            /// Write offset
            if (is_offset_applied)
                result_point_cloud_file.write(reinterpret_cast<const char*>(offset), sizeof(float) * 3);
            else {
                float zeroOffset[3] = { 0.f, 0.f, 0.f };
                result_point_cloud_file.write(reinterpret_cast<const char*>(zeroOffset), sizeof(float) * 3);
            }

            std::vector<float> buffer(3 * CHUNK_POINTS);
            std::size_t idx = 0;
            unsigned long long numPts = 0;
            cnvrt::BOUNDING_BOX bb;
            auto prev_t = start_t;

            std::string line_contents;

            /// Read first line
            if (!std::getline(pts_file, line_contents)) {
                std::cerr << "Failed to read the first line from the PTS file." << std::endl;
                pts_file.close();
                result_point_cloud_file.close();
                return false;
            }

            /// Check if first line is point count or data
            std::vector<std::string> token = split(line_contents, "\t, ");
            unsigned long long total_num_points = 0;

            if (token.size() == 1) {
                /// First line is point count
                total_num_points = std::stoull(token[0]);
                std::cout << "Total number of points recored in the first line in the file: " << total_num_points << "\n";
            }
            else if (token.size() >= 3) {
                /// First line is data - process it
                for (int i = 0; i < 3; i++) {
                    double value = std::stod(token[i]);
                    buffer[i] = static_cast<float>(value);
                }
                idx = 1;
            }

            /// Process remaining lines
            while (std::getline(pts_file, line_contents))
            {
                std::vector<std::string> result = split(line_contents, "\t, ");
                if (result.size() < 3) {
                    continue;
                }

                /// Just store raw values
                for (unsigned int i = 0; i < 3; i++) {
                    double value = std::stod(result[i]);
                    buffer[idx * 3 + i] = static_cast<float>(value);
                }

                /// Increase the index of loaded points
                idx++;

                if (idx == CHUNK_POINTS) {
                    /// Apply offset in batch
                    if (is_offset_applied) {
                        for (size_t i = 0; i < idx; ++i) {
                            for (size_t j = 0; j < 3; ++j) {
                                buffer[i * 3 + j] -= offset[j];
                            }
                        }
                    }

                    /// Update bounding box in batch
                    cnvrt::POINT_DOUBLE point_d;
                    for (size_t i = 0; i < idx; ++i) {
                        point_d.x = buffer[i * 3 + 0];
                        point_d.y = buffer[i * 3 + 1];
                        point_d.z = buffer[i * 3 + 2];
                        bb.merge_point(point_d);
                    }

                    numPts += static_cast<unsigned long long>(idx);
                    result_point_cloud_file.write(reinterpret_cast<const char*>(buffer.data()), 12 * CHUNK_POINTS);
                    
					auto curr_t = std::chrono::system_clock::now();
					if ((curr_t - prev_t) > std::chrono::seconds(2)) {
						std::cout << "Processing " << numPts << " points.\n";
						prev_t = curr_t;
					}

					idx = 0;
				}

                
            }

            /// Process remaining data
            if (idx > 0)
            {
                /// Apply offset in batch
                if (is_offset_applied) {
                    for (size_t i = 0; i < idx; ++i) {
                        for (size_t j = 0; j < 3; ++j) {
                            buffer[i * 3 + j] -= offset[j];
                        }
                    }
                }

                /// Update bounding box in batch
                cnvrt::POINT_DOUBLE point_d;
                for (size_t i = 0; i < idx; ++i) {
                    point_d.x = buffer[i * 3 + 0];
                    point_d.y = buffer[i * 3 + 1];
                    point_d.z = buffer[i * 3 + 2];
                    bb.merge_point(point_d);
                }

                numPts += static_cast<unsigned long long>(idx);
                result_point_cloud_file.write(reinterpret_cast<const char*>(buffer.data()), 12 * idx);
            }

            std::cout << "Processing " << numPts << " points. All points are processed.\n";

            pts_file.close();
            result_point_cloud_file.close();

            auto end_t = std::chrono::system_clock::now();
            auto exec_t = end_t - start_t;
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(exec_t).count();
            std::cout << "Writing " << output_file_path << " is finished.\n";
            std::cout << "Converting time: " << seconds << " seconds\nTotal number of points: " << numPts << ".\n";

            return true;
        }

        bool convert_las2pc2(const std::string& input_file_path,
            const std::string& output_file_path,
            const bool is_offset_applied,
            const float* offset)
        {
            auto start_t = std::chrono::system_clock::now();

            las::LASToolsReader lasReader;
            
            if (!lasReader.open(input_file_path)) {
                std::cerr << "Failed to open input file: " << input_file_path << std::endl;
                return false;
            }

            /// Check the output folder path
            std::error_code ec;
            std::string* err = nullptr;
            std::filesystem::path output_path(output_file_path);
            fs::path dir = output_path.parent_path();
            if (!dir.empty() && !fs::exists(dir)) {
                if (!fs::create_directories(dir, ec)) {
                    std::cerr << "Failed in creating directories: " + dir.string();
                    return false;
                }
                else
                    std::cerr << "Create a directory: " + dir.string();
            }

            auto file_open_mode = std::ios::out | std::ios::binary;
            std::ofstream result_point_cloud_file(output_file_path, file_open_mode);
            if (result_point_cloud_file.is_open()) {
                lasReader.printHeader();
            }
            else {
                std::cerr << "Failed to open output binary file: " << output_file_path << std::endl;
                lasReader.close();
                return false;
            }

            /// Write offset
            if(is_offset_applied)
                result_point_cloud_file.write(reinterpret_cast<const char*>(offset), sizeof(float) * 3);
            else {
                float zeroOffset[3] = { 0.f, 0.f, 0.f };
                result_point_cloud_file.write(reinterpret_cast<const char*>(zeroOffset), sizeof(float) * 3);
            }

            std::vector<float> buffer(3 * CHUNK_POINTS);
            unsigned long long numPts = 0;
            cnvrt::BOUNDING_BOX bb;
            auto prev_t = start_t;

            size_t total_num_points = lasReader.getPointCount();
            const std::size_t N = (std::min)(CHUNK_POINTS, total_num_points);

            for (size_t start = 0; start < total_num_points; start += N) {
                std::size_t end = (std::min)(start + N, total_num_points);
                if (!lasReader.loadPointRange(start, end)) {
                    std::cerr << "Failed in loadPointRandge(" << start << ", " << end << ")\n";
                    break;
                }

                if (is_offset_applied) {
                    size_t idx = 0;
                    for (const auto& p : lasReader.getLoadedPoints()) {
                        buffer[idx * 3 + 0] = static_cast<float>(p.x) - offset[0];
                        buffer[idx * 3 + 1] = static_cast<float>(p.y) - offset[1];
                        buffer[idx * 3 + 2] = static_cast<float>(p.z) - offset[2];

                        /// Update bounding box in batch
                        cnvrt::POINT_DOUBLE point_d;
                        for (size_t i = 0; i < idx; ++i) {
                            point_d.x = buffer[i * 3 + 0];
                            point_d.y = buffer[i * 3 + 1];
                            point_d.z = buffer[i * 3 + 2];
                            bb.merge_point(point_d);
                        }

                        ++idx;
                    }
                }
                else {
                    size_t idx = 0;
                    for (const auto& p : lasReader.getLoadedPoints()) {
                        buffer[idx * 3 + 0] = static_cast<float>(p.x);
                        buffer[idx * 3 + 1] = static_cast<float>(p.y);
                        buffer[idx * 3 + 2] = static_cast<float>(p.z);

                        /// Update bounding box in batch
                        cnvrt::POINT_DOUBLE point_d;
                        for (size_t i = 0; i < idx; ++i) {
                            point_d.x = buffer[i * 3 + 0];
                            point_d.y = buffer[i * 3 + 1];
                            point_d.z = buffer[i * 3 + 2];
                            bb.merge_point(point_d);
                        }

                        ++idx;
                    }

                }
                
                numPts += static_cast<unsigned long long>(end - start);
                result_point_cloud_file.write(reinterpret_cast<const char*>(buffer.data()), 12 * (end - start));

                auto curr_t = std::chrono::system_clock::now();
                if ((curr_t - prev_t) > std::chrono::seconds(2)) {
                    std::cout << "Processing " << numPts << " points / " << total_num_points << "\n";
                    prev_t = curr_t;
                }
            }

            lasReader.close();
            result_point_cloud_file.close();

            std::cout << "Processing " << numPts << " points. All points are processed.\n";

            auto end_t = std::chrono::system_clock::now();
            auto exec_t = end_t - start_t;
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(exec_t).count();
            std::cout << "Writing " << output_file_path << " is finished.\n";
            std::cout << "Converting time: " << seconds << " seconds\nTotal number of points: " << numPts << ".\n";

            return true;
        }
    }

	namespace mesh {
        /// Extract node name from a full path
        void extractNodeInfo(const std::filesystem::path& file_path,
            std::string& nodeName,
            int& bimId,
            int& fallback_idx)
        {
            // 기본값 설정
            nodeName = "unknown";
            bimId = fallback_idx;

            try {
                // UTF-16을 UTF-8로 변환
                std::wstring filenameW = file_path.stem().wstring();
                std::string filename = util::wstring_to_utf8(filenameW);

                if (filename.empty()) {
                    std::cerr << "Error: Empty filename" << std::endl;
                    bimId = ++fallback_idx;
                    return;
                }

                // 마지막 '_' 위치 찾기
                size_t lastUnderscorePos = filename.find_last_of('_');

                if (lastUnderscorePos == std::string::npos) {
                    // '_'가 없는 경우
                    nodeName = filename;
                    bimId = ++fallback_idx;
                    return;
                }

                // 노드 이름 추출 (마지막 '_' 이전)
                nodeName = filename.substr(0, lastUnderscorePos);

                // 노드 이름 마지막 문자가 '_'이면 제거
                if (!nodeName.empty() && nodeName.back() == '_') {
                    nodeName.pop_back();
                }

                // BIM ID 추출 (마지막 '_' 이후)
                std::string idPart = filename.substr(lastUnderscorePos + 1);

                // 숫자만 추출
                idPart.erase(
                    std::remove_if(idPart.begin(), idPart.end(),
                        [](unsigned char c) { return !std::isdigit(c); }
                    ),
                    idPart.end()
                );

                if (idPart.empty()) {
                    std::cerr << "Error: No numeric part found in filename" << std::endl;
                    bimId = ++fallback_idx;
                    return;
                }

                // 문자열을 숫자로 변환
                try {
                    unsigned long temp = std::stoul(idPart);
                    if (temp > static_cast<unsigned long>((std::numeric_limits<int>::max)())) {
                        std::cerr << "Warning: BIM ID too large: " << temp << std::endl;
                        bimId = ++fallback_idx;
                    }
                    else {
                        bimId = static_cast<int>(temp);
                    }
                }
                catch (const std::exception& e) {
                    std::cerr << "Error: Failed to convert to number: " << e.what() << std::endl;
                    bimId = ++fallback_idx;
                }
            }
            catch (const std::exception& e) {
                std::cerr << "Exception in extractNodeInfo: " << e.what() << std::endl;
                bimId = ++fallback_idx;
            }
        }

        /// Geometry and boundingbox
        struct GeometryInfo {
            MBR bbox;
            size_t buffer_size;
            bool success;
        };

        GeometryInfo processGeometryAndBoundingBox(BimMeshInfo& bim_info,
            std::array<float, 3>& bbox_min,
            std::array<float, 3>& bbox_max)
        {
            GeometryInfo geo_info;
            geo_info.success = false;
            geo_info.buffer_size = 0;

            try {
                bool is_offset_applied = false;
                float offset[3] = { 0.f, 0.f, 0.f };

                /// compute bbox using a function in bim_info
                geo_info.bbox = bim_info.get_mbr(is_offset_applied, offset);
                auto local_bbox_min = geo_info.bbox.get_min();
                auto local_bbox_max = geo_info.bbox.get_max();

                /// 전역 바운딩 박스 업데이트 (파라미터)
                for (int i = 0; i < 3; ++i) {
                    bbox_min[i] = (std::min)(bbox_min[i], local_bbox_min[i]);
                    bbox_max[i] = (std::max)(bbox_max[i], local_bbox_max[i]);
                }

                geo_info.success = true;
            }
            catch (const std::exception& e) {
                std::cerr << "Error in processGeometryAndBoundingBox: " << e.what() << std::endl;
            }

            return geo_info;
        }

        bool loadGltf(const std::string& bim_folder, std::vector<BimMeshInfo>& bimData)
        {
            std::filesystem::path input_path(bim_folder);
            
            // 경로 검증
            if (!std::filesystem::exists(input_path)) {
                std::cout << "input_folder '" << bim_folder << "' does not exist" << std::endl;
                return false;
            }

            // 진행 상황 추적
            size_t total_gltf_files = std::distance(
                fs::directory_iterator(input_path),
                fs::directory_iterator()
            );
            int processed_gltf_files = 0;
            int fallback_idx = 1000; // Revit Element ID가 없을 때 사용할 기본 ID

            // 디렉토리 순회
            for (const auto& entry : std::filesystem::directory_iterator(input_path))
            {
                // 디버그 출력
                std::wcout << entry.path().wstring() << std::endl;
                std::cout << entry.path().string() << std::endl; // 영문 Windows에서는 주석 처리

                // 일반 파일 확인
                if (!entry.is_regular_file()) {
                    std::wcout << entry.path().wstring() << " not regular file" << std::endl;
                    continue;
                }

                // GLTF/GLB 파일 확인
                auto& data_file = entry.path();
                if (data_file.extension() != ".gltf" && data_file.extension() != ".glb") {
                    if (data_file.extension() != ".bin") {
                        std::cout << data_file << " not gltf(glb) file" << std::endl;
                    }
                    continue;
                }

                // UTF-16을 UTF-8로 변환
                const std::string utf8FileName = util::wstring_to_utf8(data_file);

                // GLTF 파일 읽기
                bimData.push_back(BimMeshInfo());
                BimMeshInfo& currentGltf = bimData.back();

                bool is_offset_applied = false;
                float offset[3] = { 0.f, 0.f, 0.f };

                GltfMesh gltf_file;
                /// read gltf
                if (!gltf_file.read(utf8FileName, currentGltf, is_offset_applied, offset)) {
                    std::cerr << "Failed to read GLTF file: " << utf8FileName << std::endl;
                    continue;
                }

                extractNodeInfo(utf8FileName, currentGltf.node_name, currentGltf.bim_id, fallback_idx);

                /// Bounding-box
                for (auto& min : currentGltf.bbox_min)
                    min = (std::numeric_limits<float>::max)();
                for (auto& max : currentGltf.bbox_max)
                    max = std::numeric_limits<float>::lowest();

                GeometryInfo geometry = processGeometryAndBoundingBox(currentGltf, currentGltf.bbox_min, currentGltf.bbox_max);

                // 진행 상황 출력
                processed_gltf_files++;
                int progress = (int)((float)processed_gltf_files / total_gltf_files * 100);
                if (processed_gltf_files % 100 == 0 || progress == 100) {
                    std::cout << "Processed " << processed_gltf_files
                        << " / " << total_gltf_files
                        << " (" << progress << "%)" << std::endl;
                }
            }

            return true;
        }

        namespace tree {
            /// Point3D를 위한 PointCloud 어댑터
            struct PointCloud3DAdaptor
            {
                const std::vector<Point3D>& points;

                PointCloud3DAdaptor(const std::vector<Point3D>& pts) : points(pts) {}

                /// nanoflann에서 필요한 인터페이스 구현
                inline size_t kdtree_get_point_count() const { return points.size(); }

                /// 특정 포인트의 특정 차원 값 반환
                inline double kdtree_get_pt(const size_t idx, const size_t dim) const
                {
                    if (dim == 0) return points[idx][0];
                    else if (dim == 1) return points[idx][1];
                    else return points[idx][2];
                }

                /// 바운딩 박스 계산 (빠른 빌드를 위해)
                template <class BBOX>
                bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
            };

            struct Point {
                float x, y, z;
            };

            /// Face 포인트 정보를 담는 구조체
            struct FacePointInfo {
                Point point;
                size_t bimIdx;
                size_t faceIdx;
                bool isPseudo;

                FacePointInfo(const Point& p, size_t bIdx, size_t fIdx, bool pseudo = false)
                    : point(p), bimIdx(bIdx), faceIdx(fIdx), isPseudo(pseudo) {
                }
            };

            // nanoflann용 PointCloud 어댑터
            struct FacePointCloudAdaptor {
                const std::vector<FacePointInfo>& points;

                FacePointCloudAdaptor(const std::vector<FacePointInfo>& pts) : points(pts) {}

                // 필수: 포인트 개수 반환
                inline size_t kdtree_get_point_count() const { return points.size(); }

                // 필수: 특정 포인트의 특정 차원 값 반환
                inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
                    const Point& p = points[idx].point;
                    if (dim == 0) return p.x;
                    else if (dim == 1) return p.y;
                    else return p.z;
                }

                // 선택: 바운딩 박스
                template <class BBOX>
                bool kdtree_get_bbox(BBOX&) const { return false; }
            };

            /// KD-tree 타입 정의: KDTree3D
            typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud3DAdaptor>, PointCloud3DAdaptor, 3> KDTree3D;

            /// KD-tree 타입 정의: KDTreeFace
            typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, FacePointCloudAdaptor>, FacePointCloudAdaptor, 3, size_t> KDTreeFace;

            /// KD-tree 빌드 함수
            bool buildKdtree3D(const std::vector<Point3D>& points,
                std::shared_ptr<PointCloud3DAdaptor>& adaptor,
                std::shared_ptr<KDTree3D>& tree,
                int maxLeaf = 10)
            {
                if (points.empty()) {
                    std::cerr << "Error: Point cloud is empty" << std::endl;
                    return false;
                }

                try {
                    /// 어댑터 생성
                    adaptor.reset(new PointCloud3DAdaptor(points));

                    /// KD-tree 생성 및 빌드
                    tree.reset(new KDTree3D(3, *adaptor,
                        nanoflann::KDTreeSingleIndexAdaptorParams(maxLeaf)));
                    tree->buildIndex();
                }
                catch (const std::exception& e) {
                    std::cerr << "Error in buildKdtree3D: " << e.what() << std::endl;
                    return false;
                }
                catch (...) {
                    std::cerr << "Unknown error in buildKdtree3D" << std::endl;
                    return false;
                }

                return true;
            }

            void findNearest3D() {
                std::vector<Point3D> points;
                std::shared_ptr<PointCloud3DAdaptor> adaptor;
                std::shared_ptr<KDTree3D> tree;
                if (!buildKdtree3D(points, adaptor, tree))
                    return;

                size_t numClosest = 2;
                std::vector<size_t> indices(numClosest);
                std::vector<double> distances(numClosest);
                Point3D query(2.0, 3.0, 4.0);
                double queryPt[3] = { 2.0, 3.0, 4.0 };
                //size_t numFound = tree->knnSearch(queryPt, 1, indices.data(), distances.data());

                /*
                size_t num_results = 1;
                std::vector<size_t> ret_index(num_results);
                std::vector<double> out_dist_sqr(num_results);

                nanoflann::KNNResultSet<double> resultSet(num_results);
                resultSet.init(&ret_index[0], &out_dist_sqr[0]);

                double query_pt[3] = { query.x, query.y, query.z };
                tree->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));

                std::cout << "Nearest point index: " << ret_index[0] << std::endl;
                std::cout << "Distance squared: " << out_dist_sqr[0] << std::endl;
                */
            }

            /// Face KD-tree 관리 클래스
            class FaceFinder {
            public:
                FaceFinder() = default;
                ~FaceFinder() = default;

                /*
                /// BimMeshInfo 데이터로부터 KD-tree 구축
                bool buildKdTree(std::vector<BimMeshInfo>& bimData, double grid_size = 0.1) {
                    points.clear();
                    points.reserve(bimData.size() * 1000);

                    std::cout << "Building KD-tree for face search..." << std::endl;

                    /// 모든 BimMeshInfo 순회
                    for (size_t bimIdx = 0; bimIdx < bimData.size(); ++bimIdx) {
                        auto& bimInfo = bimData[bimIdx];
                        auto& faces = bimInfo.getFaces();

                        /// 각 face 처리
                        for (size_t faceIdx = 0; faceIdx < faces.size(); ++faceIdx) {
                            auto& face = faces[faceIdx];

                            // 1. Vertex 포인트 추가
                            points.emplace_back(face.getv1(), bimIdx, faceIdx, false);
                            points.emplace_back(face.getv2(), bimIdx, faceIdx, false);
                            points.emplace_back(face.getv3(), bimIdx, faceIdx, false);

                            // 2. Pseudo 포인트 생성 및 추가
                            std::vector<Point> pseudoPts = generatePseudoPoints(face, grid_size);
                            for (const auto& pt : pseudoPts) {
                                points.emplace_back(pt, bimIdx, faceIdx, true);
                            }
                        }

                        if ((bimIdx + 1) % 100 == 0) {
                            std::cout << "Processed " << (bimIdx + 1) << " / "
                                << bimData.size() << " BIM objects" << std::endl;
                        }
                    }

                    std::cout << "Total points collected: " << points.size() << std::endl;

                    if (points.empty()) {
                        std::cerr << "Error: No points collected for KD-tree" << std::endl;
                        return false;
                    }

                    try {
                        /// KD-tree 빌드
                        adaptor.reset(new FacePointCloudAdaptor(points));
                        kdtree.reset(new KDTreeFace(
                            3,  // 차원
                            *adaptor,
                            nanoflann::KDTreeSingleIndexAdaptorParams(10)  // max leaf
                        ));
                        kdtree->buildIndex();

                        std::cout << "KD-tree built successfully!" << std::endl;
                        return true;
                    }
                    catch (const std::exception& e) {
                        std::cerr << "Error building KD-tree: " << e.what() << std::endl;
                        return false;
                    }
                    catch (...) {
                        std::cerr << "Unknown error building KD-tree" << std::endl;
                        return false;
                    }
                }
                */

                /*
                // 가장 가까운 face 찾기
                bool findNearestFace(const Point& queryPoint,
                    size_t& bimIdx,
                    size_t& faceIdx,
                    double& distance) const {
                    if (!kdtree) {
                        std::cerr << "KD-tree not built yet" << std::endl;
                        return false;
                    }

                    const size_t num_results = 1;
                    size_t ret_index;
                    double out_dist_sqr;

                    nanoflann::KNNResultSet<double> resultSet(num_results);
                    resultSet.init(&ret_index, &out_dist_sqr);

                    double query_pt[3] = { queryPoint.x, queryPoint.y, queryPoint.z };
                    kdtree->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));

                    if (ret_index < points.size()) {
                        const auto& nearestPoint = points[ret_index];
                        bimIdx = nearestPoint.bimIdx;
                        faceIdx = nearestPoint.faceIdx;
                        distance = std::sqrt(out_dist_sqr);
                        return true;
                    }

                    return false;
                }
                */

                /*
                // K개의 가장 가까운 face들 찾기 (중복 제거)
                std::vector<std::tuple<size_t, size_t, double>> findNearestFaces(
                    const Point& queryPoint,
                    size_t k = 5) const {

                    if (!kdtree) {
                        std::cerr << "KD-tree not built yet" << std::endl;
                        return {};
                    }

                    // K개보다 많이 검색 (같은 face의 여러 포인트를 걸러내기 위해)
                    const size_t search_k = (std::min)(k * 10, points.size());
                    std::vector<size_t> ret_indices(search_k);
                    std::vector<double> out_dists_sqr(search_k);

                    nanoflann::KNNResultSet<double> resultSet(search_k);
                    resultSet.init(&ret_indices[0], &out_dists_sqr[0]);

                    double query_pt[3] = { queryPoint.x, queryPoint.y, queryPoint.z };
                    kdtree->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));

                    // Face별로 가장 가까운 거리만 유지 (중복 제거)
                    std::map<std::pair<size_t, size_t>, double> uniqueFaces;

                    for (size_t i = 0; i < search_k && i < ret_indices.size(); ++i) {
                        const auto& pt = points[ret_indices[i]];
                        auto faceKey = std::make_pair(pt.bimIdx, pt.faceIdx);
                        double dist = std::sqrt(out_dists_sqr[i]);

                        if (uniqueFaces.find(faceKey) == uniqueFaces.end() ||
                            uniqueFaces[faceKey] > dist) {
                            uniqueFaces[faceKey] = dist;
                        }

                        if (uniqueFaces.size() >= k) break;
                    }

                    // 결과를 vector로 변환
                    std::vector<std::tuple<size_t, size_t, double>> results;
                    for (const auto& [key, dist] : uniqueFaces) {
                        results.emplace_back(key.first, key.second, dist);
                    }

                    // 거리순 정렬
                    std::sort(results.begin(), results.end(),
                        [](const auto& a, const auto& b) {
                            return std::get<2>(a) < std::get<2>(b);
                        });

                    return results;
                }
                */

                /*
                // 반경 내 face들 찾기
                std::vector<std::tuple<size_t, size_t, double>> findFacesWithinRadius(
                    const Point& queryPoint,
                    double radius) const {

                    if (!kdtree) {
                        std::cerr << "KD-tree not built yet" << std::endl;
                        return {};
                    }

                    std::vector<std::pair<size_t, double>> ret_matches;
                    nanoflann::SearchParams params;
                    params.sorted = true;

                    double query_pt[3] = { queryPoint.x, queryPoint.y, queryPoint.z };
                    const double search_radius_sqr = radius * radius;

                    size_t nMatches = kdtree->radiusSearch(query_pt, search_radius_sqr, ret_matches, params);

                    // Face별로 중복 제거
                    std::map<std::pair<size_t, size_t>, double> uniqueFaces;

                    for (const auto& match : ret_matches) {
                        const auto& pt = points[match.first];
                        auto faceKey = std::make_pair(pt.bimIdx, pt.faceIdx);
                        double dist = std::sqrt(match.second);

                        if (uniqueFaces.find(faceKey) == uniqueFaces.end() ||
                            uniqueFaces[faceKey] > dist) {
                            uniqueFaces[faceKey] = dist;
                        }
                    }

                    std::vector<std::tuple<size_t, size_t, double>> results;
                    for (const auto& [key, dist] : uniqueFaces) {
                        results.emplace_back(key.first, key.second, dist);
                    }

                    std::sort(results.begin(), results.end(),
                        [](const auto& a, const auto& b) {
                            return std::get<2>(a) < std::get<2>(b);
                        });

                    return results;
                }
                */

                size_t getPointCount() const { return points.size(); }

            private:
                // Pseudo 포인트 생성 (Face의 generate_pseudo_points 버그 수정 버전)
                std::vector<Point> generatePseudoPoints(const Face& face, double grid_size) const {
                    std::vector<Point> pseudoPts;

                    if (grid_size <= 0.0) return pseudoPts;

                    Point v1, v2, v3;
                    v1.x = face.getv1().getX();
                    v1.y = face.getv1().getY();
                    v1.z = face.getv1().getZ();
                    v2.x = face.getv2().getX();
                    v2.y = face.getv2().getY();
                    v2.z = face.getv2().getZ();
                    v3.x = face.getv3().getX();
                    v3.y = face.getv3().getY();
                    v3.z = face.getv3().getZ();

                    // 두 edge 벡터 정의 (v2를 기준점으로)
                    Point edge1(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
                    Point edge2(v3.x - v2.x, v3.y - v2.y, v3.z - v2.z);  // 버그 수정

                    // Edge 길이 계산
                    double len1 = std::sqrt(edge1.x * edge1.x + edge1.y * edge1.y + edge1.z * edge1.z);
                    double len2 = std::sqrt(edge2.x * edge2.x + edge2.y * edge2.y + edge2.z * edge2.z);

                    if (len1 < std::numeric_limits<double>::epsilon() ||
                        len2 < std::numeric_limits<double>::epsilon())
                        return pseudoPts;

                    if (len1 < grid_size && len2 < grid_size)
                        return pseudoPts;

                    // 각도 계산
                    double dotproduct = edge1.x * edge2.x + edge1.y * edge2.y + edge1.z * edge2.z;
                    double cosAngle = dotproduct / (len1 * len2);
                    // clamp to [-1, 1]
                    if (cosAngle > 1.0) cosAngle = 1.0;
                    if (cosAngle < -1.0) cosAngle = -1.0;

                    double angle = std::abs(std::acos(cosAngle));
                    double sinAngle = std::sin(angle);

                    double slant_grid_size = grid_size;
                    if (sinAngle > std::numeric_limits<double>::epsilon())
                        slant_grid_size = grid_size / sinAngle;

                    // 샘플링 수 계산
                    int num_steps1 = (std::max)(1, static_cast<int>(len1 / grid_size));
                    int num_steps2 = (std::max)(1, static_cast<int>(len2 / slant_grid_size));

                    const double inv_steps1 = 1.0 / static_cast<double>(num_steps1);
                    const double inv_steps2 = 1.0 / static_cast<double>(num_steps2);

                    // Pseudo 포인트 생성
                    pseudoPts.reserve(num_steps1 * num_steps2 / 2);

                    for (int i = 0; i <= num_steps1; ++i) {
                        double u = static_cast<double>(i) * inv_steps1;

                        for (int j = 0; j <= num_steps2; ++j) {
                            double v = static_cast<double>(j) * inv_steps2;

                            // 삼각형 내부 점만 추가 (Barycentric coordinates)
                            if (u + v <= 1.0) {
                                pseudoPts.emplace_back(
                                    v2.x + u * edge1.x + v * edge2.x,
                                    v2.y + u * edge1.y + v * edge2.y,
                                    v2.z + u * edge1.z + v * edge2.z
                             );
                            }
                        }
                    }

                    return pseudoPts;
                }

                std::vector<FacePointInfo> points;
                std::shared_ptr<FacePointCloudAdaptor> adaptor;
                std::shared_ptr<KDTreeFace> kdtree;
            };
        }

        /*
        /// Load a las file
        bool loadLasFileStreaming(const std::string& file_path, std::shared_ptr<tree::KDTree3D>& tree)
        {
            try {
                las::LASToolsReader lasReader;

                if (!lasReader.open(file_path)) {
                    std::cerr << "Failed to open input file: " << file_path << std::endl;
                    return false;
                }

                if (!lasReader.loadAllPoints()) {
                    std::cerr << "Failed in loadAllPoints\n";
                    return false;
                }

                std::vector<Point3D> points;
                points.reserve(lasReader.getLoadedPointCount());

                for (const auto& p : lasReader.getLoadedPoints()) {
                    points.emplace_back(p.x, p.y, p.z);
                }

                /// Build KD-tree
                std::shared_ptr<tree::PointCloud3DAdaptor> adaptor;
                
                if (!buildKdtree3D(points, adaptor, tree, 20)) {
                    std::cerr << "Failed in building kd-tree\n";
                    return false;
                }

                lasReader.close();

                std::cout << "Loading pointcloud completed: " << points.size() << " points\n";
            }
            catch (const std::exception& e) {
                std::cerr << "Error in Las loading: " << e.what() << std::endl;
            }

            return true;
        }

        bool loadInputData(const std::string& file_path, const std::string& bim_folder) {
            
            std::shared_ptr<tree::KDTree3D> tree;

            if (!loadLasFileStreaming(file_path, tree)) {
                return false;
            }
                        
            std::vector<BimMeshInfo> bimData;
            if (!loadGltf(bim_folder, bimData)) {
                return false;
            }



            return true;
        }

        */

	}/// namespace mesh

    namespace mesh2 {
        /// Vertex3D: 3D vertex data
        class Vertex3D
        {
        public:
            Vertex3D()
            {
                xyz[0] = xyz[1] = xyz[2] = 0.0;
            }

            Vertex3D(const double x, const double y, const double z)
            {
                xyz[0] = x;
                xyz[1] = y;
                xyz[2] = z;
            }

            Vertex3D(const Vertex3D& val)
            {
                copy(val);
                faceId = val.faceId;
            }

            inline Vertex3D& operator=(const Vertex3D& val)
            {
                copy(val);
                return *this;
            }

            inline Vertex3D operator+=(const Vertex3D& val)
            {
                this->xyz[0] += val.xyz[0];
                this->xyz[1] += val.xyz[1];
                this->xyz[2] += val.xyz[2];
                return *this;
            }

            inline Vertex3D operator/=(const double& val)
            {
                try
                {
                    this->xyz[0] /= val;
                    this->xyz[1] /= val;
                    this->xyz[2] /= val;
                }
                catch (...)
                {
                    throw std::runtime_error("Error from Vertex3D operator /=");
                }
                return *this;
            }

            inline Vertex3D operator-(const Vertex3D& val) const
            {
                Vertex3D retVal;
                retVal.xyz[0] = this->xyz[0] - val.xyz[0];
                retVal.xyz[1] = this->xyz[1] - val.xyz[1];
                retVal.xyz[2] = this->xyz[2] - val.xyz[2];
                return retVal;
            }

            inline Vertex3D operator+(const Vertex3D& val) const
            {
                Vertex3D retVal;
                retVal.xyz[0] = this->xyz[0] + val.xyz[0];
                retVal.xyz[1] = this->xyz[1] + val.xyz[1];
                retVal.xyz[2] = this->xyz[2] + val.xyz[2];
                return retVal;
            }

            inline Vertex3D operator/(const double val) const
            {
                Vertex3D retVal;

                try {
                    retVal.xyz[0] = this->xyz[0] / val;
                    retVal.xyz[1] = this->xyz[1] / val;
                    retVal.xyz[2] = this->xyz[2] / val;
                }
                catch (...)
                {
                    throw std::runtime_error("Fatal error from Vertex3D operator ""/""");
                }

                return retVal;
            }

            inline Vertex3D operator*(const double val) const
            {
                Vertex3D retVal;

                retVal.xyz[0] = this->xyz[0] * val;
                retVal.xyz[1] = this->xyz[1] * val;
                retVal.xyz[2] = this->xyz[2] * val;

                return retVal;
            }

            inline void copy(const Vertex3D& val)
            {
                ::memcpy(xyz, val.xyz, 3 * sizeof(double));
                faceId = val.faceId;
            }

            inline bool operator==(const Vertex3D& pt) const
            {
                return pt.x() == xyz[0] && pt.y() == xyz[1] && pt.z() == xyz[2];
            }

            inline const double& x() const { return xyz[0]; }
            inline const double& y() const { return xyz[1]; }
            inline const double& z() const { return xyz[2]; }

            inline double& x() { return xyz[0]; }
            inline double& y() { return xyz[1]; }
            inline double& z() { return xyz[2]; }

            inline double& operator[] (const std::size_t idx)
            {
                assert(idx < 3);
                return xyz[idx];
            }

            inline const double& operator[] (const std::size_t idx) const
            {
                try
                {
                    return xyz[idx];
                }
                catch (...)
                {
                    throw std::runtime_error("Error in Vertex3D::operator[]");
                }
            }

            inline void set(const double newX, const double newY, const double newZ)
            {
                xyz[0] = newX;
                xyz[1] = newY;
                xyz[2] = newZ;
            }

            inline const std::vector<int>& getFaceId() const { return faceId; }
            inline std::vector<int>& getFaceId() { return faceId; }
            void setFaceId(const std::vector<int>& newFaceId) { faceId = newFaceId; }
            inline void addFaceId(const int idx)
            {
                faceId.push_back(idx);
            }

        private:
            double xyz[3];
            std::vector<int> faceId;
        };

        ///  MeshData: structure for mesh data
        template<class T> struct MeshData
        {
        public:
            typedef T CoordValueType;

            /// Must return the number of data points
            inline std::size_t kdtree_get_point_count() const { return vertices.size(); }

            /// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
            inline T kdtree_distance(const T* p1, const size_t idx_p2, size_t size) const
            {
                T s = 0;
                for (size_t i = 0; i < size; ++i)
                {
                    const T d = p1[i] - vertices[idx_p2][i];
                    s += d * d;
                }
                return s;
            }

            /// Returns the dim'th component of the idx'th point in the class:
            inline T kdtree_get_pt(const size_t idx, int dim) const
            {
                return vertices[idx][dim];
            }

            /// Optional bounding-box computation: return false to default to a standard bbox computation loop.
            /// Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
            /// Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
            template <class BBOX> bool kdtree_get_bbox(BBOX& /*bb*/) const
            {
                return false;
            }

        public:
            std::vector<Vertex3D> vertices;
            std::vector<std::vector<unsigned int>> faces;
            std::vector<Vertex3D> fNormals;
        };

        /// Mesh data type
        using MeshDataType = MeshData<double>;

        /// KD Tree type to use for the mesh node data
        using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<MeshDataType::CoordValueType, MeshDataType>, MeshDataType, 3>;

        /// data dimension
        const std::size_t dimension = 3;

        /// Pseudo point type
        enum class PSEUDOTYPE { NONE = 0, CENTROID = 1, MIDPOINT = 2 };

        class SSMFlann
        {
        public:
            SSMFlann(const double maxLength = 0.0, const PSEUDOTYPE pType = PSEUDOTYPE::MIDPOINT);

            /**setNumberOfPoints
            *@brief resize vertex data vector
            *@param size : vertex data size (number of vertices)
            */
            void setNumberOfPoints(const unsigned int size);

            /**setVertex
            *@brief add a vertex without checking duplicated points
            *@param index : vertex index
            *@param x
            *@param y
            *@param z
            *@return bool
            */
            bool setVertex(const unsigned int index, const double x, const double y, const double z);

            /**setNumberOfFaces
            *@brief resize face data vector
            *@param size : number of faces
            */
            void setNumberOfFaces(const unsigned int size);

            /**setPseudoType
            *@brief set the type of generating pseudo points
            *@param pType: type
            */
            void setPseudoType(const PSEUDOTYPE pType);

            /**setFace
            *@brief add a face
            *@param faceIndex : face index
            *@param vtxIndex : indices of vertices consisting a face
            *@param fnx : face normal vector x coordinate
            *@param fny : face normal vector y coordinate
            *@param fnz : face normal vector z coordinate
            *@return bool
            */
            bool setFace(const unsigned int faceIndex, const std::vector<unsigned int>& vtxIndex);

            /**setFaceNormal
            *@brief add a face normal vector
            *@param faceIndex : face index
            *@param fnx : face normal vector x coordinate
            *@param fny : face normal vector y coordinate
            *@param fnz : face normal vector z coordinate
            *@return bool
            */
            bool setFaceNormal(const unsigned int faceIndex, const double fnx, const double fny, const double fnz);

            /**getFaceNormal
            *@brief get a face normal vector
            *@param faceIndex : face index
            *@return Vertex3D
            */
            Vertex3D getFaceNormal(const unsigned int faceIndex) const;

            /**clearVertexData
            *@brief clear vertex data
            */
            void clearVertexData();

            /**buildTree
            *@brief buildTree
            *@param maxLeaf max number of leaves of kdtree
            */
            void buildTree(const unsigned int maxLeaf = 20);

            /**buildTree
            *@brief import index info from a file
            *@param maxLeaf max number of leaves of kdtree
            *@return bool
            */
            bool buildTree(const std::string& idxFilePath, const unsigned int maxLeaf = 20);

            /**search
            *@brief search closest vertices
            */
            bool search(const double x0, const double y0, const double z0,
                std::vector<double>& distances,
                std::vector<std::size_t>& indices,
                const unsigned int numClosest,
                const double distThreshold = 0.0) const;

            /**raytrace
            *@brief find the closest point along the given ray
            *@param sx: x coordinate of a start point of a ray
            *@param sy: y coordinate of a start point of a ray
            *@param sz: z coordinate of a start point of a ray
            *@param ex: x coordinate of a end point of a ray
            *@param ey: y coordinate of a end point of a ray
            *@param ez: z coordinate of a end point of a ray
            *@param radius: search radius
            *@param foundPts: point indices and distances, result values
            *@return bool
            */
            bool raytrace(const double sx, const double sy, const double sz,
                const double ex, const double ey, const double ez,
                const double radius,
                std::vector<std::pair<size_t, double>>& foundPts);

            /**getVertex
            *@param vtxIndex : vertex index
            */
            Vertex3D getVertex(const std::size_t vtxIndex) const;

            /**getDistThreshold
            */
            double getDistThreshold() const { return this->maxDist; }

            /**getVertices
            */
            const std::vector<Vertex3D>& getVertices() const { return this->meshData.vertices; }

            /**getPseudoPts
            */
            const std::vector<Vertex3D>& getPseudoPts() const { return this->pseudoPts; }

            /**getFaces
            */
            const std::vector<std::vector<unsigned int>>& getFaces() const { return this->meshData.faces; }

            /**setDistThreshold
            */
            void setDistThreshold(const double newDistTh) { this->maxDist = newDistTh; }

            /**exportIndex
            *@param idxFilePath : index file path
            *@return bool
            */
            bool exportIndex(const std::string& idxFilePath);

        private:
            /**defineFace
            *@brief define face with vertex indices
            *@param faceIndex face index
            *@param vertexIndices vertex indices in a face
            *@return bool
            */
            bool defineFace(const unsigned int faceIndex, const std::vector<unsigned int>& vertexIndices);

            /**computeCentroid
            *@param vertices : vertices consisting a face
            *@return Vertex3D : centroid 3D coordinates
            */
            Vertex3D computeCentroid(const std::vector<Vertex3D>& vertices);

            /**checkDist2Center
            *@param vertices : vertices consisting a face
            *@param centroid : centroid of a polygon
            *@return bool : [false] there is a too long side (>maxSide) in the vertices defining a face
            */
            bool checkDist2Center(const std::vector<Vertex3D>& vertices, const Vertex3D& centroid);

            /**defineSubTriangles
            *@brief define sub-polygons (triangles)
            *@param vertices : vertices consisting a face
            *@param centroid : centroid of a polygon
            */
            std::vector< std::vector<Vertex3D>> defineSubTriangles(const std::vector<Vertex3D>& vertices, const Vertex3D& centroid);

            /**insertPseudoPoint: generate pseudo points in a face with centroid points
            *@param vertexIndices : vertex indices consisting a face
            *@param faceIndex : face index
            */
            void insertPseudoPoint(const std::vector<unsigned int>& vertexIndices, const unsigned int faceIndex);

            /**insertPseudoPointMid: generate pseudo points in a face with centroid points
            *@param vertexIndices : vertex indices consisting a face
            *@param faceIndex : face index
            */
            void insertPseudoPointMid(const std::vector<unsigned int>& vertexIndices, const unsigned int faceIndex);

            /**addMidPts: add mid points to a given line
            *@param p0 : the start point of a line
            *@param p1 : the end point of a line
            *@param maxLength : maximum length threshold
            *@param pseudoPts : bucket for storing pseudo point (midPts)
            *@return bool: if true, the line is large, else, short
            */
            bool addMidPts(const Vertex3D& p0, const Vertex3D& p1, const double maxLength, std::vector<Vertex3D>& pseudoPts);

            /**divideTriangle: divide a triangle into three sub-triangles
            *@param triVtx : three vertices
            *@param maxLength : maximum length threshold
            *@param pseudoPts : bucket for storing pseudo point (midPts)
            *@param centroid : center of a triangle
            *@return bool: if true, triangle is large, else, small
            */
            bool divideTriangle(const std::vector<Vertex3D>& triVtx, const double maxLength, std::vector<Vertex3D>& pseudoPts, Vertex3D& centroid);

            /**divideFace: divide a face (polygon)
            *@param faceVtx : face vertices
            *@param maxLength : maximum length threshold
            *@param pseudoPts : bucket for storing pseudo point (midPts)
            */
            void divideFace(const std::vector<Vertex3D>& faceVtx, const double maxLength, std::vector<Vertex3D>& pseudoPts);

        private:
            MeshDataType meshData;
            std::vector<Vertex3D> pseudoPts;
            std::shared_ptr<KDTree> tree;
            double maxDist;
            PSEUDOTYPE pseudoType;
        };

        SSMFlann::SSMFlann(const double maxLength, const PSEUDOTYPE pType)
        {
            this->maxDist = maxLength;
            pseudoType = pType;
        };

        void SSMFlann::setNumberOfPoints(const unsigned int size)
        {
            meshData.vertices.resize(size);
        }

        bool SSMFlann::setVertex(const unsigned int index, const double x, const double y, const double z)
        {
            if (!(index < meshData.vertices.size()))
                return false;

            try
            {
                meshData.vertices[index].set(x, y, z);
                return true;
            }
            catch (...)
            {
                std::cerr << "Error in setVertex of SSMFlann" << std::endl;
                return false;
            }
        }

        void SSMFlann::setNumberOfFaces(const unsigned int size)
        {
            meshData.faces.resize(size);
            meshData.fNormals.resize(size);
        }

        /**setPseudoType
        *@brief set the type of generating pseudo points
        *@param pType: type
        */
        void SSMFlann::setPseudoType(const PSEUDOTYPE pType)
        {
            pseudoType = pType;
        }

        bool SSMFlann::setFace(const unsigned int faceIndex, const std::vector<unsigned int>& vtxIndex)
        {
            if (!(faceIndex < meshData.faces.size()))
            {
                std::cerr << "Error in setFace of SSMFlann: check the index of a face" << std::endl;
                return false;
            }

            if (vtxIndex.size() < 3)
            {
                std::cerr << "Error in setFace of SSMFlann: check the number of vertices consisting a face (should be greater than 2)" << std::endl;
                return false;
            }

            try
            {
                meshData.faces[faceIndex] = vtxIndex;
                return defineFace(faceIndex, vtxIndex);
            }
            catch (...)
            {
                std::cerr << "Error in setFace of SSMFlann" << std::endl;
                return false;
            }
        }

        bool SSMFlann::setFaceNormal(const unsigned int faceIndex, const double fnx, const double fny, const double fnz)
        {
            if (!(faceIndex < meshData.fNormals.size()))
            {
                std::cerr << "Error in setFaceNormal of SSMFlann: check the index of a face-normal" << std::endl;
                return false;
            }

            try
            {
                meshData.fNormals[faceIndex][0] = fnx;
                meshData.fNormals[faceIndex][1] = fny;
                meshData.fNormals[faceIndex][2] = fnz;
            }
            catch (...)
            {
                std::cerr << "Error in setFaceNormal of SSMFlann" << std::endl;
                return false;
            }

            return true;
        }

        Vertex3D SSMFlann::getFaceNormal(const unsigned int faceIndex) const
        {
            try
            {
                return this->meshData.fNormals[faceIndex];
            }
            catch (...)
            {

                std::cerr << "Error in getting a face normal vector" << std::endl;
                Vertex3D temp;
                return temp;
            }
        }

        void SSMFlann::clearVertexData()
        {
            meshData.vertices.clear();
        }

        bool SSMFlann::defineFace(const unsigned int faceIndex, const std::vector<unsigned int>& vertexIndices)
        {
            try
            {
                for (const unsigned int index : vertexIndices)
                {
                    meshData.vertices[index].addFaceId(faceIndex);
                }

                /// divide a face into sub-triangles
                switch (this->pseudoType)
                {
                case PSEUDOTYPE::CENTROID:
                    insertPseudoPoint(vertexIndices, faceIndex);
                    break;
                case PSEUDOTYPE::MIDPOINT:
                    insertPseudoPointMid(vertexIndices, faceIndex);
                    break;
                default:
                    break;
                }

                return true;
            }
            catch (...)
            {

                std::cerr << "Error in defineFace of SSMFlann" << std::endl;
                return false;
            }
        }

        void SSMFlann::buildTree(const unsigned int maxLeaf)
        {
            try {
                /// Add additional pseudo points
                size_t totalSize = meshData.vertices.size() + pseudoPts.size();
                meshData.vertices.reserve(totalSize);
                meshData.vertices.insert(meshData.vertices.end(), pseudoPts.begin(), pseudoPts.end());

                /// Larger maxLeaf result in fast search, but takes long time for tree construction
                size_t dimension = 3;
                tree.reset(new KDTree(dimension, meshData, nanoflann::KDTreeSingleIndexAdaptorParams(maxLeaf)));
                tree->buildIndex();
            }
            catch (...) {
                std::cerr << "Error in buildTree of SSMFlann" << std::endl;
            }
        }

        bool SSMFlann::buildTree(const std::string& idxFilePath, const unsigned int maxLeaf)
        {
            try {
                /// Add additional pseudo points
                meshData.vertices.insert(meshData.vertices.end(), pseudoPts.begin(), pseudoPts.end());

                /// Larger maxLeaf result in fast search, but takes long time for tree construction
                size_t dimension = 3;
                tree.reset(new KDTree(dimension, meshData, nanoflann::KDTreeSingleIndexAdaptorParams(maxLeaf)));

                FILE* idxFile = fopen(idxFilePath.c_str(), "rb");
                if (!idxFile) return false;
                tree->loadIndex(idxFile);
                fclose(idxFile);
            }
            catch (...) {
                std::cerr << "Error in buildTree(import index info from a file) of SSMFlann" << std::endl;
            }

            return true;
        }

        bool SSMFlann::search(const double x0, const double y0, const double z0,
            std::vector<double>& distances,
            std::vector<std::size_t>& indices,
            const unsigned int numClosest,
            const double distThreshold) const
        {
            MeshDataType::CoordValueType searchPt[] = { x0, y0, z0 };
            indices.resize(numClosest, 0);
            distances.resize(numClosest);

            std::vector<double> squareDist(numClosest);
            size_t numFound = tree->knnSearch(searchPt, numClosest, indices.data(), squareDist.data());

            for (size_t i = 0; i < numFound; ++i) {
                distances[i] = std::sqrt(squareDist[i]);
            }

            if (numFound < numClosest) {
                indices.resize(numFound);
                distances.resize(numFound);
            }

            if (distThreshold == 0.0)
                return true;

            const double thresholdSq = distThreshold * distThreshold;
            return (squareDist[0] < thresholdSq);
        }

        /**raytrace
        *@brief find the closest point along the given ray
        *@param sx: x coordinate of a start point of a ray
        *@param sy: y coordinate of a start point of a ray
        *@param sz: z coordinate of a start point of a ray
        *@param ex: x coordinate of a end point of a ray
        *@param ey: y coordinate of a end point of a ray
        *@param ez: z coordinate of a end point of a ray
        *@param radius: search radius
        *@param foundPts: point indices and distances, result values
        *@return bool
        */
        bool SSMFlann::raytrace(const double sx, const double sy, const double sz,
            const double ex, const double ey, const double ez,
            const double radius,
            std::vector<std::pair<size_t, double>>& foundPts)
        {
            double dx = ex - sx;
            double dy = ey - sy;
            double dz = ez - sz;
            double maxDist = sqrt(dx * dx + dy * dy + dz * dz);
            if (maxDist < std::numeric_limits<double>::epsilon())
                return false;
            double nx = dx / maxDist;
            double ny = dy / maxDist;
            double nz = dz / maxDist;

            unsigned int count = 0;
            do
            {
                ++count;
                double fwdDist = radius * count;

                if (fwdDist > maxDist)
                    break;

                MeshDataType::CoordValueType searchPt[] = { sx + nx * fwdDist, sy + ny * fwdDist, sz + nz * fwdDist };
                nanoflann::SearchParams params;
                /// Search nearest point in radius
                const size_t nMatches = tree->radiusSearch(&searchPt[0], radius, foundPts, params);
                if (nMatches > 0)
                    return true;

            } while (1);

            return false;
        }

        Vertex3D SSMFlann::getVertex(const std::size_t vtxIndex) const
        {
            return meshData.vertices[vtxIndex];
        }

        /**computeCentroid
        *@param vertices : vertices consisting a face
        *@return Vertex3D : centroid 3D coordinates
        */
        Vertex3D SSMFlann::computeCentroid(const std::vector<Vertex3D>& vertices)
        {
            double sumX = 0.0;
            double sumY = 0.0;
            double sumZ = 0.0;
            for (const auto& vtx : vertices)
            {
                sumX += vtx.x();
                sumY += vtx.y();
                sumZ += vtx.z();
            }

            return Vertex3D(sumX / vertices.size(), sumY / vertices.size(), sumZ / vertices.size());
        }

        /**checkDist2Center
        *@param vertices : vertices consisting a face
        *@param centroid : centroid of a polygon
        *@return bool : [false] there is a too long side (>maxSide) in the vertices defining a face
        */
        bool SSMFlann::checkDist2Center(const std::vector<Vertex3D>& vertices, const Vertex3D& centroid)
        {
            const double maxDistSq = maxDist * maxDist;
            for (unsigned int i = 0; i < vertices.size(); ++i)
            {
                auto dx = vertices[i].x() - centroid.x();
                auto dy = vertices[i].y() - centroid.y();
                auto dz = vertices[i].z() - centroid.z();
                auto distSq = dx * dx + dy * dy + dz * dz;
                if (distSq < maxDistSq) return true;
            }

            return false;
        }

        /**defineSubTriangles
        *@brief define sub-polygons (triangles)
        *@param vertices : vertices consisting a face
        *@param centroid : centroid of a polygon
        */
        std::vector< std::vector<Vertex3D>> SSMFlann::defineSubTriangles(const std::vector<Vertex3D>& vertices, const Vertex3D& centroid)
        {
            std::vector< std::vector<Vertex3D>> triangles(vertices.size());

            for (unsigned int i = 0; i < vertices.size(); ++i)
            {
                unsigned int j = i + 1;
                if (j == vertices.size()) j = 0;

                triangles[i].resize(3);
                triangles[i][0] = centroid;
                triangles[i][1] = vertices[i];
                triangles[i][2] = vertices[j];
            }

            return triangles;
        }

        /**insertPseudoPoint: generate pseudo points in a face with centroid points
        *@param vertexIndices : vertex indices consisting a face
        *@param faceIndex : face index
        */
        void SSMFlann::insertPseudoPoint(const std::vector<unsigned int>& vertexIndices, const unsigned int faceIndex)
        {
            if (this->maxDist < std::numeric_limits<double>::epsilon())
                return;

            /// Collect vertices using indices
            std::vector<std::vector<Vertex3D>> verticesGroup(1);
            verticesGroup[0].reserve(vertexIndices.size());
            for (auto idx : vertexIndices)
                verticesGroup[0].push_back(meshData.vertices[idx]);

            std::vector<std::vector<Vertex3D>> newCollectedVertices;

            /// Check size of a face and put centroid points if need
            do
            {
                /// New polygons defined by centroid points
                newCollectedVertices.clear();

                for (auto vertices : verticesGroup)
                {
                    /// compute a centroid
                    Vertex3D centroid = computeCentroid(vertices);

                    /// check if there is a long side
                    if (checkDist2Center(vertices, centroid))
                        continue;

                    /// Assign face index and save the centroid (pseudoPts)
                    centroid.addFaceId(faceIndex);
                    pseudoPts.push_back(centroid);

                    /// Define new sub-triangles
                    std::vector<std::vector<Vertex3D>> triangles = defineSubTriangles(vertices, centroid);

                    /// Collect newly defined triangle
                    newCollectedVertices.insert(newCollectedVertices.end(), triangles.begin(), triangles.end());
                }

                if (newCollectedVertices.size() < 1)
                    break;
                else
                    verticesGroup = newCollectedVertices;

            } while (1);
        }

        /**insertPseudoPointMid: generate pseudo points
        *@param vertexIndices : vertex indices consisting a face
        *@param faceIndex : face index
        */
        void SSMFlann::insertPseudoPointMid(const std::vector<unsigned int>& vertexIndices, const unsigned int faceIndex)
        {
            if (this->maxDist < std::numeric_limits<double>::epsilon())
                return;

            /// Collect vertices for a face
            std::vector<Vertex3D> faceVtx;
            faceVtx.reserve(vertexIndices.size());
            for (auto idx : vertexIndices)
                faceVtx.push_back(meshData.vertices[idx]);

            /// Generate pseudo points by dividing a face into sub-triangles
            std::vector<Vertex3D> midPts;
            midPts.reserve(vertexIndices.size() * 10);
            divideFace(faceVtx, this->maxDist, midPts);
            midPts.shrink_to_fit();
            for (auto& pt : midPts)
                pt.addFaceId(faceIndex);
            pseudoPts.insert(pseudoPts.end(), midPts.begin(), midPts.end());
        }

        /**addMidPts: add mid points to a given line
        *@param p0 : the start point of a line
        *@param p1 : the end point of a line
        *@param maxLength : maximum length threshold
        *@param pseudoPts : bucket for storing pseudo point (midPts)
        *@param faceId: face Id
        *@return bool: if true, the line is large, else, short
        */
        bool SSMFlann::addMidPts(const Vertex3D& p0, const Vertex3D& p1, const double maxLength, std::vector<Vertex3D>& pseudoPts)
        {
            std::vector<Vertex3D> addedPts;

            /// check distances
            Vertex3D v = p1 - p0;
            auto distSq = v.x() * v.x() + v.y() * v.y() + v.z() * v.z();
            auto maxLengthSq = maxLength * maxLength;

            if (distSq > maxLengthSq)
            {
                auto dist = std::sqrt(distSq);
                Vertex3D dir = v / dist;
                unsigned int num = static_cast<int>(dist / maxLength + 1.0);
                double dLength = dist / static_cast<double>(num);
                addedPts.reserve(num - 1);
                for (unsigned int i = 1; i < num; ++i)
                {
                    Vertex3D tempPt = dir * (dLength * static_cast<double>(i)) + p0;
                    addedPts.push_back(tempPt);
                }
            }

            if (addedPts.size() > 0)
            {
                pseudoPts.insert(pseudoPts.end(), addedPts.begin(), addedPts.end());
                return true;
            }
            else
                return false;
        }

        /**divideTriangle: divide a triangle into three sub-triangles
        *@param triVtx : three vertices
        *@param maxLength : maximum length threshold
        *@param pseudoPts : bucket for storing pseudo point (midPts)
        *@param centroid : center of a triangle
        *@return bool: if true, triangle is large, else, small
        */
        bool SSMFlann::divideTriangle(const std::vector<Vertex3D>& triVtx, const double maxLength, std::vector<Vertex3D>& pseudoPts, Vertex3D& centroid)
        {
            /// compute a centroid
            centroid = Vertex3D((triVtx[0].x() + triVtx[1].x() + triVtx[2].x()) / 3.,
                (triVtx[0].y() + triVtx[1].y() + triVtx[2].y()) / 3.,
                (triVtx[0].z() + triVtx[1].z() + triVtx[2].z()) / 3.);

            std::vector<Vertex3D> midPts;
            bool longSide = true;

            /// Mid-points
            if (!addMidPts(triVtx[0], centroid, maxLength, midPts))
                longSide = false;
            if (!addMidPts(triVtx[1], centroid, maxLength, midPts))
                longSide = false;
            if (!addMidPts(triVtx[2], centroid, maxLength, midPts))
                longSide = false;

            pseudoPts.push_back(centroid);
            pseudoPts.insert(pseudoPts.end(), midPts.begin(), midPts.end());

            return longSide;
        }

        /**divideFace: divide a face (polygon)
        *@param faceVtx : face vertices
        *@param maxLength : maximum length threshold
        *@param pseudoPts : bucket for storing pseudo point (midPts)
        */
        void SSMFlann::divideFace(const std::vector<Vertex3D>& faceVtx, const double maxLength, std::vector<Vertex3D>& pseudoPts)
        {
            if (faceVtx.size() < 3)
                return;

            Vertex3D centroid; /// Centroid

            /// Divide sides
            for (unsigned int i = 0; i < faceVtx.size(); ++i)
            {
                unsigned int j;
                if (i == faceVtx.size() - 1) j = 0;
                else j = i + 1;

                /// Add pseudo points to the inside
                addMidPts(faceVtx[i], faceVtx[j], maxLength, pseudoPts);

                centroid += faceVtx[i];
            }

            if (pseudoPts.size() < 1)
                return;

            centroid /= static_cast<double>(faceVtx.size());

            bool bigSize = false;

            for (unsigned int i = 0; i < faceVtx.size(); ++i)
            {
                auto dP = centroid - faceVtx[i];
                auto dist = sqrt(dP.x() * dP.y() + dP.x() * dP.y() + dP.z() * dP.z());
                if (dist > maxLength)
                {
                    pseudoPts.push_back(centroid);
                    bigSize = true;
                    break;
                }
            }

            if (!bigSize)
                return;

            /// First sub-triangles from a face
            std::vector<std::vector<Vertex3D>> subTri(faceVtx.size());

            for (unsigned int i = 0; i < faceVtx.size(); ++i)
            {
                /// Add pseudo points to the inside
                addMidPts(centroid, faceVtx[i], maxLength, pseudoPts);

                /// Add pseudo point to the sides
                Vertex3D side;
                unsigned int j;
                if (i == faceVtx.size() - 1) j = 0;
                else j = i + 1;

                std::vector<Vertex3D> tri(3);
                tri[0] = faceVtx[i];
                tri[1] = faceVtx[j];
                tri[2] = centroid;

                subTri[i] = tri;
            }

            /// Check sub-triangles
            unsigned int count = 0;
            do
            {
                ++count;

                auto subTriCopy = subTri;
                subTri.clear();

                for (const auto& sub : subTriCopy)
                {
                    Vertex3D triCenter;
                    if (divideTriangle(sub, maxLength, pseudoPts, triCenter))
                    {
                        std::vector<Vertex3D> dividedTri(3);
                        dividedTri[0] = sub[0];
                        dividedTri[1] = sub[1];
                        dividedTri[2] = triCenter;
                        subTri.push_back(dividedTri);
                        dividedTri[0] = sub[1];
                        dividedTri[1] = sub[2];
                        dividedTri[2] = triCenter;
                        subTri.push_back(dividedTri);
                        dividedTri[0] = sub[2];
                        dividedTri[1] = sub[0];
                        dividedTri[2] = triCenter;
                        subTri.push_back(dividedTri);
                    }
                }

                if (subTri.size() < 1)
                    break;

            } while (1);
        }

        /**exportIndex
        */
        bool SSMFlann::exportIndex(const std::string& idxFilePath)
        {
            if (!tree)
                return false;

            FILE* idxFile = fopen(idxFilePath.c_str(), "wb");
            if (!idxFile) return false;

            tree->saveIndex(idxFile);

            fclose(idxFile);

            return true;
        }

        bool loadGltf(const std::string& bim_folder, std::vector<BimMeshInfo>& bimData)
        {
            std::filesystem::path input_path(bim_folder);

            // 경로 검증
            if (!std::filesystem::exists(input_path)) {
                std::cout << "input_folder '" << bim_folder << "' does not exist" << std::endl;
                return false;
            }

            // 진행 상황 추적
            size_t total_gltf_files = std::distance(
                fs::directory_iterator(input_path),
                fs::directory_iterator()
            );
            int processed_gltf_files = 0;
            int fallback_idx = 1000; // Revit Element ID가 없을 때 사용할 기본 ID

            // 디렉토리 순회
            for (const auto& entry : std::filesystem::directory_iterator(input_path))
            {
                // 디버그 출력
                std::wcout << entry.path().wstring() << std::endl;
                std::cout << entry.path().string() << std::endl; // 영문 Windows에서는 주석 처리

                // 일반 파일 확인
                if (!entry.is_regular_file()) {
                    std::wcout << entry.path().wstring() << " not regular file" << std::endl;
                    continue;
                }

                // GLTF/GLB 파일 확인
                auto& data_file = entry.path();
                if (data_file.extension() != ".gltf" && data_file.extension() != ".glb") {
                    if (data_file.extension() != ".bin") {
                        std::cout << data_file << " not gltf(glb) file" << std::endl;
                    }
                    continue;
                }

                // UTF-16을 UTF-8로 변환
                const std::string utf8FileName = util::wstring_to_utf8(data_file);

                // GLTF 파일 읽기
                bimData.push_back(BimMeshInfo());
                BimMeshInfo& currentGltf = bimData.back();

                bool is_offset_applied = false;
                float offset[3] = { 0.f, 0.f, 0.f };

                GltfMesh gltf_file;
                /// read gltf
                if (!gltf_file.read(utf8FileName, currentGltf, is_offset_applied, offset)) {
                    std::cerr << "Failed to read GLTF file: " << utf8FileName << std::endl;
                    continue;
                }

                mesh::extractNodeInfo(utf8FileName, currentGltf.node_name, currentGltf.bim_id, fallback_idx);

                /// Bounding-box
                for (auto& min : currentGltf.bbox_min)
                    min = (std::numeric_limits<float>::max)();
                for (auto& max : currentGltf.bbox_max)
                    max = std::numeric_limits<float>::lowest();

                mesh::GeometryInfo geometry = mesh::processGeometryAndBoundingBox(currentGltf, currentGltf.bbox_min, currentGltf.bbox_max);

                /// 진행 상황 출력
                processed_gltf_files++;
                int progress = (int)((float)processed_gltf_files / total_gltf_files * 100);
                if (processed_gltf_files % 100 == 0 || progress == 100) {
                    std::cout << "Processed " << processed_gltf_files
                        << " / " << total_gltf_files
                        << " (" << progress << "%)" << std::endl;
                }
            }

            return true;
        }
    }
    
} /// namespace DPApp