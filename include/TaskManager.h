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
#include <future>
#include <functional>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <random>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <thread>

#include "PointCloudTypes.h"
#include "NetworkManager.h"
#include "ReadPointFile.h"

namespace DPApp {

    /**
     * @brief 3D mesh vertex structure with basic vector operations
     */
    struct MeshVertex {
        float x, y, z;

        MeshVertex() : x(0.0f), y(0.0f), z(0.0f) {}

        MeshVertex(float x_, float y_, float z_)
            : x(x_), y(y_), z(z_) {
        }

        MeshVertex(double x_, double y_, double z_)
            : x(static_cast<float>(x_)), y(static_cast<float>(y_)), z(static_cast<float>(z_)) {
        }

        MeshVertex(int x_, int y_, int z_)
            : x(static_cast<float>(x_)), y(static_cast<float>(y_)), z(static_cast<float>(z_)) {
        }

        // Vector operations
        MeshVertex operator-(const MeshVertex& other) const {
            return MeshVertex(x - other.x, y - other.y, z - other.z);
        }

        MeshVertex operator+(const MeshVertex& other) const {
            return MeshVertex(x + other.x, y + other.y, z + other.z);
        }

        MeshVertex operator*(float scalar) const {
            return MeshVertex(x * scalar, y * scalar, z * scalar);
        }

        float dot(const MeshVertex& other) const {
            return x * other.x + y * other.y + z * other.z;
        }

        float length() const {
            return std::sqrt(x * x + y * y + z * z);
        }
    };

    /**
     * @brief Triangle mesh element with utility functions
     */
    struct MeshTriangle {
        MeshVertex v0, v1, v2;

        MeshTriangle() : v0(), v1(), v2() {}

        MeshTriangle(const MeshVertex& vertex0, const MeshVertex& vertex1, const MeshVertex& vertex2)
            : v0(vertex0), v1(vertex1), v2(vertex2) {
        }

        /**
         * @brief Calculate triangle center point
         */
        MeshVertex center() const {
            return MeshVertex(
                (v0.x + v1.x + v2.x) / 3.0f,
                (v0.y + v1.y + v2.y) / 3.0f,
                (v0.z + v1.z + v2.z) / 3.0f
            );
        }

        /**
         * @brief Calculate triangle normal vector
         */
        MeshVertex normal() const {
            MeshVertex edge1 = v1 - v0;
            MeshVertex edge2 = v2 - v0;

            MeshVertex cross(
                edge1.y * edge2.z - edge1.z * edge2.y,
                edge1.z * edge2.x - edge1.x * edge2.z,
                edge1.x * edge2.y - edge1.y * edge2.x
            );

            float len = cross.length();
            if (len > 0.0f) {
                return cross * (1.0f / len);
            }

            return MeshVertex(0.0f, 0.0f, 1.0f);
        }
    };

    /**
     * @brief BIM (Building Information Model) data container
     */
    struct BIMData {
        std::vector<MeshTriangle> triangles;
        std::string source_file;
        std::string model_name;

        BIMData() = default;
        BIMData(const std::string& file) : source_file(file) {
            model_name = std::filesystem::path(file).stem().string();
        }

        /**
         * @brief Calculate bounding box of the BIM model
         */
        void calculateBounds(MeshVertex& min_bounds, MeshVertex& max_bounds) const {
            if (triangles.empty()) return;

            min_bounds = max_bounds = triangles[0].v0;

            for (const auto& triangle : triangles) {
                for (const auto& vertex : { triangle.v0, triangle.v1, triangle.v2 }) {
                    min_bounds.x = (std::min)(min_bounds.x, vertex.x);
                    min_bounds.y = (std::min)(min_bounds.y, vertex.y);
                    min_bounds.z = (std::min)(min_bounds.z, vertex.z);

                    max_bounds.x = (std::max)(max_bounds.x, vertex.x);
                    max_bounds.y = (std::max)(max_bounds.y, vertex.y);
                    max_bounds.z = (std::max)(max_bounds.z, vertex.z);
                }
            }
        }
    };

    /**
     * @brief Result structure for distance calculations
     */
    struct DistanceResult {
        uint32_t point_index;
        float distance_to_mesh;
        MeshTriangle closest_triangle;
        MeshVertex closest_point_on_mesh;
        std::string source_point_file;
        std::string closest_bim_file;

        DistanceResult() : point_index(0), distance_to_mesh((std::numeric_limits<float>::max)()) {}
    };

    /**
     * @brief Point cloud file information structure
     */
    struct PointCloudFileInfo {
        std::string file_path;
        std::string file_name;
        size_t point_count;
        bool is_loaded;
        MeshVertex min_bounds, max_bounds;

        PointCloudFileInfo() : point_count(0), is_loaded(false) {}
        PointCloudFileInfo(const std::string& path) : file_path(path), point_count(0), is_loaded(false) {
            file_name = std::filesystem::path(path).filename().string();
        }
    };

    /**
     * @brief Parameters for BIM-PointCloud comparison tasks
     */
    struct BIMComparisonParams {
        std::string bim_folder_path;
        std::string pointcloud_folder_path;
        std::string bim_file_pattern;        // Default: "*.gltf"
        std::string pointcloud_file_pattern; // Default: "*.las"
        uint32_t max_points_per_chunk;       // Maximum points per processing chunk
        float distance_threshold;            // Distance threshold in meters
        bool enable_color_coding;            // Enable distance-based color coding

        BIMComparisonParams()
            : bim_file_pattern("*.gltf"), pointcloud_file_pattern("*.las"),
            max_points_per_chunk(100000), distance_threshold(10.0f),
            enable_color_coding(true) {
        }
    };

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
        HIGH = 2,
        CRITICAL = 3
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

        TaskInfo() : task_id(0), chunk_id(0), priority(TaskPriority::NORMAL),
            status(TaskStatus::PENDING), retry_count(0), max_retries(3),
            created_time(std::chrono::steady_clock::now()) {
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
        std::vector<std::string> supported_task_types;

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

        /**
         * @brief Calculate overall performance score
         */
        double getPerformanceScore() const {
            double success_rate = getSuccessRate();
            double speed_factor = avg_processing_time > 0 ? 1.0 / avg_processing_time : 1.0;
            return success_rate * speed_factor;
        }
    };

    /**
     * @brief Task distribution strategies
     */
    enum class TaskDistributionStrategy {
        ROUND_ROBIN,        // Sequential distribution
        LOAD_BALANCED,      // Load-based distribution
        PERFORMANCE_BASED,  // Performance-based distribution
        TASK_TYPE_AWARE     // Task type aware distribution
    };

    /**
     * @brief Point cloud loading utilities
     */
    namespace PointCloudLoader {
        std::vector<std::shared_ptr<PointCloudChunk>> loadPointCloudFileInChunks(const std::string& file_path, uint32_t max_points_per_chunk = MAX_NUM_PTS_CHUNK);

        std::shared_ptr<PointCloudChunk> loadLASFile(const std::string& file_path);

        std::shared_ptr<PointCloudChunk> loadXYZFile(const std::string& file_path);

        PointCloudFileInfo getPointCloudFileInfo(const std::string& file_path);

        // Chunk streaming function
        std::vector<std::shared_ptr<PointCloudChunk>> loadPointCloudFileInChunksStreaming(
            const std::string& file_path,
            uint32_t max_points_per_chunk = 100000);

        // File streaming function
        std::vector<std::shared_ptr<PointCloudChunk>> loadXYZFileStreaming(
            const std::string& file_path,
            uint32_t max_points_per_chunk);

        std::vector<std::shared_ptr<PointCloudChunk>> loadLASFileStreaming(
            const std::string& file_path,
            uint32_t max_points_per_chunk);
    }

    /**
     * @brief Task Manager class for distributed processing (Master node)
     */
    class TaskManager {
    private:
        std::atomic<bool> stopping_{ false };

    public:
        explicit TaskManager(const uint32_t timeout_seconds)
            : next_task_id_(1), distribution_strategy_(TaskDistributionStrategy::LOAD_BALANCED),
            task_timeout_seconds_(timeout_seconds), current_slave_index_(0) {
            std::cout << "TaskManager initialized with " << timeout_seconds << "s timeout" << std::endl;
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

        /**
         * @brief Add a new task to the processing queue
         */
        uint32_t addTask(TaskType task_type,
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

            std::cout << "Task added: " << task_id << " (" << taskStr(task_type) << ")" << std::endl;

            return task_id;
        }

        /**
         * @brief Add BIM comparison tasks for point cloud processing
         */
        std::vector<uint32_t> addBIMComparisonTasks(
            const BIMComparisonParams& params, 
            TaskPriority priority = TaskPriority::NORMAL) {

            std::vector<uint32_t> task_ids;

            try {
                std::cout << "Starting BIM-PointCloud comparison setup..." << std::endl;
                std::cout << "BIM folder: " << params.bim_folder_path << std::endl;
                std::cout << "PointCloud folder: " << params.pointcloud_folder_path << std::endl;

                if (!std::filesystem::exists(params.bim_folder_path) ||
                    !std::filesystem::is_directory(params.bim_folder_path)) {
                    std::cerr << "BIM folder not found: " << params.bim_folder_path << std::endl;
                    return task_ids;
                }

                if (!std::filesystem::exists(params.pointcloud_folder_path) ||
                    !std::filesystem::is_directory(params.pointcloud_folder_path)) {
                    std::cerr << "PointCloud folder not found: " << params.pointcloud_folder_path << std::endl;
                    return task_ids;
                }

                std::vector<PointCloudFileInfo> pc_files = findPointCloudFiles(
                    params.pointcloud_folder_path, params.pointcloud_file_pattern);

                if (pc_files.empty()) {
                    std::cerr << "No point cloud files found in " << params.pointcloud_folder_path << std::endl;
                    return task_ids;
                }

                std::cout << "Found " << pc_files.size() << " point cloud files" << std::endl;

                // Create tasks for each point cloud file
                for (const auto& pc_file : pc_files) {
                    // Create multiple tasks by chunking the file
                    std::vector<uint32_t> file_task_ids = createTasksForPointCloudFile(
                        pc_file, params, priority);

                    task_ids.insert(task_ids.end(), file_task_ids.begin(), file_task_ids.end());
                }

                std::cout << "Created " << task_ids.size() << " BIM comparison tasks" << std::endl;

            }
            catch (const std::exception& e) {
                std::cerr << "Error creating BIM comparison tasks: " << e.what() << std::endl;
            }

            return task_ids;
        }

        /**
         * @brief Simplified BIM comparison task creation (overload)
         */
        std::vector<uint32_t> addBIMComparisonTasks(const std::string& bim_folder_path,
            const std::string& pointcloud_folder_path,
            TaskPriority priority = TaskPriority::NORMAL) {
            BIMComparisonParams params;
            params.bim_folder_path = bim_folder_path;
            params.pointcloud_folder_path = pointcloud_folder_path;

            return addBIMComparisonTasks(params, priority);
        }

        /**
         * @brief Remove a task from the manager
         */
        bool removeTask(uint32_t task_id) {
            std::lock_guard<std::mutex> lock(tasks_mutex_);

            auto it = tasks_.find(task_id);
            if (it == tasks_.end()) {
                return false;
            }

            std::cout << "Task removed: " << task_id << std::endl;
            tasks_.erase(it);
            return true;
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
         * @brief Set task distribution strategy
         */
        void setDistributionStrategy(TaskDistributionStrategy strategy) {
            distribution_strategy_ = strategy;
            std::cout << "Distribution strategy changed to: " << static_cast<int>(strategy) << std::endl;
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
            (std::sort)(pending_tasks.begin(), pending_tasks.end(),
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

                // Store BIM comparison results if applicable
                if (task_info.task_type == TaskType::BIM_DISTANCE_CALCULATION) {
                    std::lock_guard<std::mutex> bim_lock(bim_results_mutex_);
                    // Additional BIM result storage logic would go here
                }

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

            size_t completed = (std::count)(statuses.begin(), statuses.end(), TaskStatus::COMPLETED);
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

            // Check slave timeouts
            int timed_out_slaves = 0;
            for (auto& [slave_id, slave_info] : slaves_) {
                if (slave_info.is_active) {
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        now - slave_info.last_heartbeat).count();

                    if (elapsed > static_cast<int64_t>(task_timeout_seconds_)) {
                        slave_info.is_active = false;  // Deactivate
                        timed_out_slaves++;
                        std::cout << "Slave " << slave_id << " timed out after "
                            << elapsed << " seconds" << std::endl;
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

        /**
         * @brief Get BIM comparison results
         */
        std::vector<DistanceResult> getBIMComparisonResults() {
            std::lock_guard<std::mutex> lock(bim_results_mutex_);
            return bim_comparison_results_;
        }

        /**
         * @brief Save BIM comparison results to CSV file
         */
        void saveBIMComparisonResults(const std::string& output_file_path) {
            std::lock_guard<std::mutex> lock(bim_results_mutex_);

            try {
                std::ofstream output_file(output_file_path);
                if (!output_file.is_open()) {
                    std::cerr << "Failed to open output file: " << output_file_path << std::endl;
                    return;
                }

                // CSV header
                output_file << "PointIndex,Distance(m),SourceFile,ClosestBIMFile,"
                    << "ClosestTriangleV0X,ClosestTriangleV0Y,ClosestTriangleV0Z,"
                    << "ClosestTriangleV1X,ClosestTriangleV1Y,ClosestTriangleV1Z,"
                    << "ClosestTriangleV2X,ClosestTriangleV2Y,ClosestTriangleV2Z,"
                    << "ClosestPointX,ClosestPointY,ClosestPointZ\n";

                // Write data rows
                for (const auto& result : bim_comparison_results_) {
                    output_file << result.point_index << ","
                        << result.distance_to_mesh << ","
                        << "\"" << result.source_point_file << "\","
                        << "\"" << result.closest_bim_file << "\","
                        << result.closest_triangle.v0.x << "," << result.closest_triangle.v0.y << "," << result.closest_triangle.v0.z << ","
                        << result.closest_triangle.v1.x << "," << result.closest_triangle.v1.y << "," << result.closest_triangle.v1.z << ","
                        << result.closest_triangle.v2.x << "," << result.closest_triangle.v2.y << "," << result.closest_triangle.v2.z << ","
                        << result.closest_point_on_mesh.x << "," << result.closest_point_on_mesh.y << "," << result.closest_point_on_mesh.z << "\n";
                }

                output_file.close();
                std::cout << "BIM comparison results saved to: " << output_file_path << std::endl;
                std::cout << "Total results: " << bim_comparison_results_.size() << std::endl;

            }
            catch (const std::exception& e) {
                std::cerr << "Error saving BIM comparison results: " << e.what() << std::endl;
            }
        }

        /**
         * @brief Print comprehensive statistics
         */
        void printStatistics() {
            std::lock_guard<std::mutex> tasks_lock(tasks_mutex_);
            std::lock_guard<std::mutex> slaves_lock(slaves_mutex_);

            std::cout << "\n=== TaskManager Statistics ===" << std::endl;
            std::cout << "Total Tasks: " << tasks_.size() << std::endl;
            std::cout << "Pending: " << getPendingTaskCount() << std::endl;
            std::cout << "Active: " << getActiveTaskCount() << std::endl;
            std::cout << "Completed: " << getCompletedTaskCount() << std::endl;
            std::cout << "Failed: " << getFailedTaskCount() << std::endl;
            std::cout << "Progress: " << (getOverallProgress() * 100.0) << "%" << std::endl;

            std::cout << "\nSlaves: " << slaves_.size() << std::endl;
            for (const auto& [slave_id, slave_info] : slaves_) {
                std::cout << "  " << slave_id << ": "
                    << (slave_info.is_active ? "Active" : "Inactive")
                    << (slave_info.is_busy ? " (Busy)" : " (Idle)")
                    << " | Completed: " << slave_info.completed_tasks
                    << " | Failed: " << slave_info.failed_tasks
                    << " | Avg Time: " << slave_info.avg_processing_time << "s"
                    << " | Success Rate: " << (slave_info.getSuccessRate() * 100.0) << "%"
                    << std::endl;
            }
            std::cout << "==============================\n" << std::endl;
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

        /**
         * @brief Find point cloud files in directory
         */
        std::vector<PointCloudFileInfo> findPointCloudFiles(const std::string& folder_path,
            const std::string& file_pattern) {
            std::vector<PointCloudFileInfo> files;

            // Supported point cloud file extensions
            std::vector<std::string> supported_extensions = { ".pts", ".xyz" };

            try {
                for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
                    if (entry.is_regular_file()) {
                        std::string file_path = entry.path().string();
                        std::string file_extension = entry.path().extension().string();

                        // Convert to lowercase for comparison
                        (std::transform)(file_extension.begin(), file_extension.end(),
                            file_extension.begin(), ::tolower);

                        // Check if file extension is supported
                        if ((std::find)(supported_extensions.begin(), supported_extensions.end(),
                            file_extension) != supported_extensions.end()) {
                            PointCloudFileInfo file_info(file_path);

                            // Get file information (simplified, should use actual file readers)
                            file_info = PointCloudLoader::getPointCloudFileInfo(file_path);

                            files.push_back(file_info);
                            std::cout << "Found point cloud file: " << entry.path().filename().string()
                                << " (" << file_info.point_count << " points estimated)" << std::endl;
                        }
                    }
                }
            }
            catch (const std::exception& e) {
                std::cerr << "Error scanning point cloud folder: " << e.what() << std::endl;
            }

            return files;
        }

        /**
         * @brief Create tasks for a single point cloud file
         */
        std::vector<uint32_t> createTasksForPointCloudFile(const PointCloudFileInfo& pc_file,
            const BIMComparisonParams& params,
            TaskPriority priority) {
            std::vector<uint32_t> task_ids;

            try {
                // Load point cloud file in chunks
                std::vector<std::shared_ptr<PointCloudChunk>> chunks =
                    PointCloudLoader::loadPointCloudFileInChunks(pc_file.file_path, params.max_points_per_chunk);

                if (chunks.empty()) {
                    std::cerr << "Failed to load point cloud file: " << pc_file.file_path << std::endl;
                    return task_ids;
                }

                std::cout << "Loaded " << pc_file.file_path << " into " << chunks.size() << " chunks" << std::endl;

                // Create BIM comparison task for each chunk
                for (auto& chunk : chunks) {
                    // Encode task parameters
                    std::vector<uint8_t> parameters;

                    // Encode BIM folder path
                    uint32_t bim_path_len = static_cast<uint32_t>(params.bim_folder_path.length());
                    parameters.insert(parameters.end(),
                        reinterpret_cast<const uint8_t*>(&bim_path_len),
                        reinterpret_cast<const uint8_t*>(&bim_path_len) + sizeof(uint32_t));
                    parameters.insert(parameters.end(),
                        params.bim_folder_path.begin(), params.bim_folder_path.end());

                    // Encode BIM file pattern
                    uint32_t bim_pattern_len = static_cast<uint32_t>(params.bim_file_pattern.length());
                    parameters.insert(parameters.end(),
                        reinterpret_cast<const uint8_t*>(&bim_pattern_len),
                        reinterpret_cast<const uint8_t*>(&bim_pattern_len) + sizeof(uint32_t));
                    parameters.insert(parameters.end(),
                        params.bim_file_pattern.begin(), params.bim_file_pattern.end());

                    // Encode source point cloud file path
                    uint32_t pc_path_len = static_cast<uint32_t>(pc_file.file_path.length());
                    parameters.insert(parameters.end(),
                        reinterpret_cast<const uint8_t*>(&pc_path_len),
                        reinterpret_cast<const uint8_t*>(&pc_path_len) + sizeof(uint32_t));
                    parameters.insert(parameters.end(),
                        pc_file.file_path.begin(), pc_file.file_path.end());

                    // Encode additional parameters (distance threshold, color coding flag)
                    parameters.insert(parameters.end(),
                        reinterpret_cast<const uint8_t*>(&params.distance_threshold),
                        reinterpret_cast<const uint8_t*>(&params.distance_threshold) + sizeof(float));

                    uint8_t color_coding = params.enable_color_coding ? 1 : 0;
                    parameters.push_back(color_coding);

                    // Add task
                    uint32_t task_id = addTask(TaskType::BIM_DISTANCE_CALCULATION, chunk, parameters, priority);
                    task_ids.push_back(task_id);
                }

            }
            catch (const std::exception& e) {
                std::cerr << "Error creating tasks for point cloud file " << pc_file.file_path
                    << ": " << e.what() << std::endl;
            }

            return task_ids;
        }

        // Task distribution strategy implementations
        std::string selectSlaveForTask(const TaskInfo& task) {
            switch (distribution_strategy_) {
            case TaskDistributionStrategy::ROUND_ROBIN:
                return selectSlaveRoundRobin();
            case TaskDistributionStrategy::LOAD_BALANCED:
                return selectSlaveLoadBalanced();
            case TaskDistributionStrategy::PERFORMANCE_BASED:
                return selectSlavePerformanceBased();
            case TaskDistributionStrategy::TASK_TYPE_AWARE:
                return selectSlaveTaskTypeAware(task);
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
                    double performance = slave_info.getPerformanceScore();

                    if (performance > best_performance) {
                        best_performance = performance;
                        best_slave = slave_id;
                    }
                }
            }

            return best_slave;
        }

        std::string selectSlaveTaskTypeAware(const TaskInfo& task) {
            std::string best_slave;
            double best_score = 0.0;

            for (auto& [slave_id, slave_info] : slaves_) {
                if (slave_info.is_active && !slave_info.is_busy) {
                    bool supports_task = (std::find)(slave_info.supported_task_types.begin(),
                        slave_info.supported_task_types.end(),
                        taskStr(task.task_type)) != slave_info.supported_task_types.end();

                    if (supports_task) {
                        double score = slave_info.getPerformanceScore();
                        if (score > best_score) {
                            best_score = score;
                            best_slave = slave_id;
                        }
                    }
                }
            }

            // Fallback to performance-based if no matching slave found
            return best_slave.empty() ? selectSlavePerformanceBased() : best_slave;
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

        // BIM comparison results storage
        std::vector<DistanceResult> bim_comparison_results_;

        std::mutex tasks_mutex_;
        std::mutex slaves_mutex_;
        std::mutex results_mutex_;
        std::mutex bim_results_mutex_;

        std::atomic<uint32_t> next_task_id_;
        TaskDistributionStrategy distribution_strategy_;
        uint32_t task_timeout_seconds_;
        size_t current_slave_index_;  // For round-robin

        TaskCompletedCallback task_completed_callback_;
        TaskFailedCallback task_failed_callback_;
    };

    /**
     * @brief Task Processor class for individual workers (Slave nodes)
     */
    class TaskProcessor {
    public:
        TaskProcessor();
        ~TaskProcessor();

        // Processor function type definition
        using ProcessorFunction = std::function<ProcessingResult(const ProcessingTask&, const PointCloudChunk&)>;

        void registerProcessor(const TaskType task_type, ProcessorFunction processor);
        void unregisterProcessor(const TaskType task_type);

        // Task processing
        ProcessingResult processTask(const ProcessingTask& task, const PointCloudChunk& chunk);

        // Supported task type queries
        std::vector<TaskType> getSupportedTaskTypes();
        bool supportsTaskType(const TaskType task_type);

        // Statistics
        uint32_t getProcessedTaskCount() const { return processed_task_count_; }
        uint32_t getFailedTaskCount() const { return failed_task_count_; }
        double getAverageProcessingTime() const { return avg_processing_time_; }

        /**
         * @brief Get processing statistics by task type
         */
        std::map<TaskType, uint32_t> getTaskTypeStatistics() const {
            std::lock_guard<std::mutex> lock(processors_mutex_);
            return task_type_stats_;
        }

    private:
        std::map<TaskType, ProcessorFunction> processors_;
        mutable std::mutex processors_mutex_;
        std::map<TaskType, uint32_t> task_type_stats_;

        std::atomic<uint32_t> processed_task_count_;
        std::atomic<uint32_t> failed_task_count_;
        std::atomic<double> avg_processing_time_;
    };

    /**
     * @brief Basic point cloud processing functions
     */
    namespace PointCloudProcessors {
        /// Distance calculation between point cloud and BIM models
        ProcessingResult calculateBIMDistance(const ProcessingTask& task, const PointCloudChunk& chunk);
        /// Convert pts
        ProcessingResult convertPts(const ProcessingTask& task, const PointCloudChunk& chunk);
    }

    /**
     * @brief BIM file loading utilities
     */
    namespace BIMLoader {
        // Load BIM data from GLTF/GLB files (would require tinygltf or assimp)
        BIMData loadBIMData(const std::string& file_path);

        // Load all BIM files from a folder
        std::vector<BIMData> loadBIMDataFromFolder(const std::string& folder_path,
            const std::string& file_pattern = "*.gltf");

        // Find BIM files (without loading, for metadata only)
        std::vector<std::string> findBIMFiles(const std::string& folder_path,
            const std::string& file_pattern = "*.gltf");
    }

    /**
     * @brief Distance calculation utilities
     */
    namespace DistanceCalculator {
        // Calculate distance from point to triangle
        float pointToTriangleDistance(const MeshVertex& point, const MeshTriangle& triangle, MeshVertex& closest_point);

        // Calculate distance from point to mesh
        DistanceResult pointToMeshDistance(const MeshVertex& point, const BIMData& bim_data);

        // Calculate distances between point cloud and multiple BIM models
        std::vector<DistanceResult> calculateDistances(const PointCloudChunk& chunk,
            const std::vector<BIMData>& bim_data_list,
            const std::string& source_file = "");

        /**
         * @brief Spatial indexing for fast distance queries
         */
        class SpatialIndex {
        public:
            void buildIndex(const std::vector<BIMData>& bim_data_list);
            std::vector<size_t> queryNearbyTriangles(const MeshVertex& point, float radius);
        private:
            // Could implement octree, kd-tree, or other spatial data structures
            std::vector<MeshTriangle> all_triangles_;
            std::vector<size_t> triangle_indices_;
        };
    }

    ///
    /// Implementation Section
    ///

    /// TaskProcessor Implementation
    TaskProcessor::TaskProcessor()
        : processed_task_count_(0), failed_task_count_(0), avg_processing_time_(0.0) {

        /// BIM distance calculation
        registerProcessor(TaskType::BIM_DISTANCE_CALCULATION, PointCloudProcessors::calculateBIMDistance);
        /// PTS converting
        registerProcessor(TaskType::CONVERT_PTS, PointCloudProcessors::convertPts);

        std::cout << "TaskProcessor initialized with " << processors_.size() << " processors" << std::endl;
    }

    TaskProcessor::~TaskProcessor() {
        std::lock_guard<std::mutex> lock(processors_mutex_);
        processors_.clear();
        std::cout << "TaskProcessor destroyed" << std::endl;
    }

    void TaskProcessor::registerProcessor(const TaskType task_type, ProcessorFunction processor) {
        std::lock_guard<std::mutex> lock(processors_mutex_);
        processors_[task_type] = processor;
        task_type_stats_[task_type] = 0;
        std::cout << "Processor registered for task type: " << taskStr(task_type) << std::endl;
    }

    void TaskProcessor::unregisterProcessor(const TaskType task_type) {
        std::lock_guard<std::mutex> lock(processors_mutex_);
        processors_.erase(task_type);
        task_type_stats_.erase(task_type);
        std::cout << "Processor unregistered for task type: " << taskStr(task_type) << std::endl;
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
                result.error_message = "Unsupported task type: " + std::string(taskStr(task.task_type));
                failed_task_count_++;
                return result;
            }

            // Execute processor
            result = it->second(task, chunk);

            if (result.success) {
                processed_task_count_++;
                task_type_stats_[task.task_type]++;
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

    std::vector<TaskType> TaskProcessor::getSupportedTaskTypes() {
        std::lock_guard<std::mutex> lock(processors_mutex_);

        std::vector<TaskType> types;
        for (const auto& [type, processor] : processors_) {
            types.push_back(type);
        }

        return types;
    }

    bool TaskProcessor::supportsTaskType(const TaskType task_type) {
        std::lock_guard<std::mutex> lock(processors_mutex_);
        return processors_.find(task_type) != processors_.end();
    }

    /// PointCloudProcessors Implementation
    namespace PointCloudProcessors {

        ProcessingResult calculateBIMDistance(const ProcessingTask& task, const PointCloudChunk& chunk) {
            ProcessingResult result;
            result.task_id = task.task_id;
            result.chunk_id = task.chunk_id;
            result.success = true;

            try {
                // Parameter parsing
                if (task.parameters.size() < sizeof(uint32_t) * 3) {
                    result.success = false;
                    result.error_message = "Invalid parameters for BIM distance calculation";
                    return result;
                }

                size_t offset = 0;

                // Extract BIM folder path
                uint32_t bim_path_len;
                std::memcpy(&bim_path_len, task.parameters.data() + offset, sizeof(uint32_t));
                offset += sizeof(uint32_t);

                std::string bim_folder_path(
                    reinterpret_cast<const char*>(task.parameters.data() + offset), bim_path_len);
                offset += bim_path_len;

                // Extract BIM file pattern
                uint32_t bim_pattern_len;
                std::memcpy(&bim_pattern_len, task.parameters.data() + offset, sizeof(uint32_t));
                offset += sizeof(uint32_t);

                std::string bim_file_pattern(
                    reinterpret_cast<const char*>(task.parameters.data() + offset), bim_pattern_len);
                offset += bim_pattern_len;

                // Extract source point cloud file path
                uint32_t pc_path_len;
                std::memcpy(&pc_path_len, task.parameters.data() + offset, sizeof(uint32_t));
                offset += sizeof(uint32_t);

                std::string source_pc_file(
                    reinterpret_cast<const char*>(task.parameters.data() + offset), pc_path_len);
                offset += pc_path_len;

                // Extract additional parameters
                float distance_threshold = 10.0f;
                bool enable_color_coding = true;

                if (task.parameters.size() >= offset + sizeof(float) + sizeof(uint8_t)) {
                    std::memcpy(&distance_threshold, task.parameters.data() + offset, sizeof(float));
                    offset += sizeof(float);

                    uint8_t color_flag = task.parameters[offset];
                    enable_color_coding = (color_flag != 0);
                }

                std::cout << "Processing BIM distance calculation:" << std::endl;
                std::cout << "  BIM folder: " << bim_folder_path << std::endl;
                std::cout << "  Pattern: " << bim_file_pattern << std::endl;
                std::cout << "  Source PC file: " << std::filesystem::path(source_pc_file).filename().string() << std::endl;
                std::cout << "  Distance threshold: " << distance_threshold << "m" << std::endl;

                // Load BIM data
                std::vector<BIMData> bim_data_list = BIMLoader::loadBIMDataFromFolder(
                    bim_folder_path, bim_file_pattern);

                if (bim_data_list.empty()) {
                    std::cout << "Warning: No BIM files found in " << bim_folder_path << std::endl;
                    result.processed_points = chunk.points;
                    return result;
                }

                std::cout << "  Loaded " << bim_data_list.size() << " BIM files" << std::endl;

                // Calculate distances
                std::vector<DistanceResult> distance_results =
                    DistanceCalculator::calculateDistances(chunk, bim_data_list, source_pc_file);

                // Copy input points and update with distance information
                result.processed_points = chunk.points;

                for (size_t i = 0; i < distance_results.size() && i < result.processed_points.size(); ++i) {
                    const auto& dist_result = distance_results[i];
                    auto& point = result.processed_points[i];

                    // Store distance in intensity field (convert meters to millimeters, clamped to uint16_t range)
                    point.intensity = static_cast<uint16_t>((std::min)(dist_result.distance_to_mesh * 1000, 65535.0f));

                    // Distance-based color coding
                    if (enable_color_coding) {
                        float normalized_distance = (std::min)(dist_result.distance_to_mesh / distance_threshold, 1.0f);

                        // Color gradient: Blue (close) -> Green (medium) -> Red (far)
                        if (normalized_distance < 0.5f) {
                            // Close distance: Blue -> Green
                            point.r = 255;
                            point.g = static_cast<uint8_t>(normalized_distance * 2.0f * 255);
                            point.b = 0;
                        }
                        else {
                            // Far distance: Green -> Red
                            point.r = static_cast<uint8_t>((2.0f - normalized_distance * 2.0f) * 255);
                            point.g = static_cast<uint8_t>((2.0f - normalized_distance * 2.0f) * 255);
                            point.b = static_cast<uint8_t>((normalized_distance * 2.0f - 1.0f) * 255);
                        }
                    }
                }

                std::cout << "  Calculated distances for " << distance_results.size() << " points" << std::endl;

            }
            catch (const std::exception& e) {
                result.success = false;
                result.error_message = "BIM distance calculation error: " + std::string(e.what());
            }

            return result;
        }

        ProcessingResult convertPts(const ProcessingTask& task, const PointCloudChunk& chunk) {
            ProcessingResult result;
            result.task_id = task.task_id;
            result.chunk_id = task.chunk_id;
            result.success = true;



            /*NOT completed yet*/



            return result;
        }

    } // namespace PointCloudProcessors

    // PointCloudLoader Implementation
    namespace PointCloudLoader {

        std::vector<std::shared_ptr<PointCloudChunk>> loadPointCloudFileInChunks(const std::string& file_path, uint32_t max_points_per_chunk) {

            // Use streaming version
            return loadPointCloudFileInChunksStreaming(file_path, max_points_per_chunk);
        }

        /// Simple memory usage estimation function
        inline size_t estimateMemoryUsageMB(const std::vector<std::shared_ptr<PointCloudChunk>>& chunks) {
            size_t total_points = 0;
            for (const auto& chunk : chunks) {
                if (chunk) {
                    total_points += chunk->points.size();
                }
            }

            // Point3D size + overhead estimation
            size_t bytes_per_point = sizeof(Point3D) + 8; /// Including some overhead
            size_t total_bytes = total_points * bytes_per_point;

            return total_bytes / (1024 * 1024); /// Convert to MB
        }

        /// Function to display progress and memory usage together
        void printProgressWithMemory(size_t current_chunks, size_t current_points,
            const std::string& operation) {
            size_t estimated_mb = (current_points * sizeof(Point3D)) / (1024 * 1024);

            std::cout << operation << " - Chunks: " << current_chunks
                << ", Points: " << current_points
                << ", Est. Memory: ~" << estimated_mb << "MB" << std::endl;
        }

        PointCloudFileInfo getPointCloudFileInfo(const std::string& file_path) {
            PointCloudFileInfo info(file_path);

            try {
                std::string extension = std::filesystem::path(file_path).extension().string();
                (std::transform)(extension.begin(), extension.end(), extension.begin(), ::tolower);

                auto file_size = std::filesystem::file_size(file_path);

                if (extension == ".pts" || extension == ".xyz") {
                    info.point_count = ptsfile::countPointsInFile(file_path);
                }
                else {
                    throw std::runtime_error("It's not a *.pts file: " + file_path);
                }

                std::cout << "File info collected: " << info.file_name
                    << " (estimated " << info.point_count << " points)" << std::endl;

            }
            catch (const std::exception& e) {
                std::cerr << "Error getting file info for " << file_path << ": " << e.what() << std::endl;
                info.point_count = 0;
            }

            return info;
        }

        // File format specific loaders (simplified implementations)
        std::shared_ptr<PointCloudChunk> loadLASFile(const std::string& file_path) {
            auto chunk = std::make_shared<PointCloudChunk>();
            chunk->chunk_id = 0;

            // TODO: Implement using libLAS or PDAL
            std::cout << "Loading LAS file: " << file_path << " (dummy implementation)" << std::endl;

            // Dummy data generation (replace with actual LAS reader)
            for (int i = 0; i < 1000; ++i) {
                Point3D point;
                point.x = static_cast<float>(i) * 0.1f;
                point.y = static_cast<float>(i % 100) * 0.1f;
                point.z = static_cast<float>(i % 10);
                point.intensity = static_cast<uint16_t>(i % 65535);
                point.r = static_cast<uint8_t>(i % 255);
                point.g = static_cast<uint8_t>((i * 2) % 255);
                point.b = static_cast<uint8_t>((i * 3) % 255);
                chunk->points.push_back(point);
            }

            return chunk;
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

        std::vector<std::shared_ptr<PointCloudChunk>> loadPointCloudFileInChunksStreaming(
            const std::string& file_path, uint32_t max_points_per_chunk) {

            std::vector<std::shared_ptr<PointCloudChunk>> chunks;

            try {
                std::cout << "Loading point cloud file (streaming): " << file_path << std::endl;

                // Check file extension
                std::string extension = std::filesystem::path(file_path).extension().string();
                std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

                if (extension == ".las" || extension == ".laz") {
                    chunks = loadLASFileStreaming(file_path, max_points_per_chunk);
                }
                else if (extension == ".xyz" || extension == ".pts") {
                    chunks = loadXYZFileStreaming(file_path, max_points_per_chunk);
                }
                else {
                    std::cerr << "Unsupported file format: " << extension << std::endl;
                    return chunks;
                }

                std::cout << "Created " << chunks.size() << " chunks via streaming" << std::endl;

            }
            catch (const std::exception& e) {
                std::cerr << "Error in streaming load: " << e.what() << std::endl;
            }

            return chunks;
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

                /// Initialize a cuttent chunk
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

                    /// Skip empty line or comment line
                    if (line.empty() || line[0] == '#') continue;

                    std::istringstream iss(line);
                    Point3D point;

                    if (iss >> point.x >> point.y >> point.z) {
                        /// Attempt to read additional fields
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

                        /// Check chunk size limit
                        if (points_in_current_chunk >= max_points_per_chunk) {
                            /// Compute bounding box
                            current_chunk->calculateBounds();

                            /// Complete chunk
                            chunks.push_back(current_chunk);

                            /// Progress and memory usage
                            printProgressWithMemory(chunks.size(), total_points_processed, "Streaming");

                            /// Start new chunk
                            chunk_counter++;
                            current_chunk = std::make_shared<PointCloudChunk>();
                            current_chunk->chunk_id = chunk_counter;
                            current_chunk->points.reserve(max_points_per_chunk);
                            points_in_current_chunk = 0;
                        }
                    }

                    /// Progress (every 100k lines)
                    if (line_count % 100000 == 0) {
                        printProgressWithMemory(chunks.size(), total_points_processed, "Processing");
                    }
                }

                /// Last chunk
                if (points_in_current_chunk > 0) {
                    current_chunk->calculateBounds();
                    chunks.push_back(current_chunk);

                    std::cout << "Final chunk " << chunk_counter
                        << " completed (" << points_in_current_chunk << " points)" << std::endl;
                }

                file.close();

                /// Final result
                size_t total_estimated_mb = estimateMemoryUsageMB(chunks);
                std::cout << "Streaming load completed: " << chunks.size() << " chunks, "
                    << total_points_processed << " total points, "
                    << "~" << total_estimated_mb << "MB estimated usage" << std::endl;

            }
            catch (const std::exception& e) {
                std::cerr << "Error in XYZ streaming: " << e.what() << std::endl;
            }

            return chunks;
        }

        std::vector<std::shared_ptr<PointCloudChunk>> loadLASFileStreaming(
            const std::string& file_path, uint32_t max_points_per_chunk) {

            std::vector<std::shared_ptr<PointCloudChunk>> chunks;

            // TODO: LAS streaming implementation (currently dummy)
            std::cout << "LAS streaming not implemented yet, using fallback" << std::endl;

            // Temporary: use existing method
            auto full_chunk = loadLASFile(file_path);
            if (full_chunk && !full_chunk->points.empty()) {

                if (full_chunk->points.size() <= max_points_per_chunk) {
                    chunks.push_back(full_chunk);
                }
                else {
                    // Split in memory (until LAS implementation is complete)
                    size_t total_points = full_chunk->points.size();
                    size_t chunk_count = (total_points + max_points_per_chunk - 1) / max_points_per_chunk;

                    for (size_t i = 0; i < chunk_count; ++i) {
                        size_t start_idx = i * max_points_per_chunk;
                        size_t end_idx = (std::min)(start_idx + max_points_per_chunk, total_points);

                        auto chunk = std::make_shared<PointCloudChunk>();
                        chunk->chunk_id = static_cast<uint32_t>(i);
                        chunk->points.assign(
                            full_chunk->points.begin() + start_idx,
                            full_chunk->points.begin() + end_idx
                        );
                        chunk->calculateBounds();

                        chunks.push_back(chunk);
                    }
                }
            }

            return chunks;
        }

    } // namespace PointCloudLoader

    // BIMLoader Implementation
    namespace BIMLoader {

        BIMData loadBIMData(const std::string& file_path) {
            BIMData bim_data(file_path);

            try {
                std::cout << "Loading BIM file: " << file_path << std::endl;

                // TODO: Implement actual GLTF/GLB loading using tinygltf, assimp, etc.

                // Dummy implementation (replace with actual file parsing)
                if (std::filesystem::exists(file_path)) {
                    // Create a simple test geometry (box)

                    // Floor
                    MeshVertex v1(0.0f, 0.0f, 0.0f);
                    MeshVertex v2(10.0f, 0.0f, 0.0f);
                    MeshVertex v3(10.0f, 10.0f, 0.0f);
                    MeshVertex v4(0.0f, 10.0f, 0.0f);

                    bim_data.triangles.emplace_back(v1, v2, v3);
                    bim_data.triangles.emplace_back(v1, v3, v4);

                    // Ceiling
                    MeshVertex v5(0.0f, 0.0f, 3.0f);
                    MeshVertex v6(10.0f, 0.0f, 3.0f);
                    MeshVertex v7(10.0f, 10.0f, 3.0f);
                    MeshVertex v8(0.0f, 10.0f, 3.0f);

                    bim_data.triangles.emplace_back(v5, v7, v6);
                    bim_data.triangles.emplace_back(v5, v8, v7);

                    // Additional walls could be added here...

                    std::cout << "Loaded " << bim_data.triangles.size() << " triangles from " << file_path << std::endl;
                }
                else {
                    std::cout << "BIM file not found: " << file_path << std::endl;
                }

            }
            catch (const std::exception& e) {
                std::cerr << "Error loading BIM file " << file_path << ": " << e.what() << std::endl;
            }

            return bim_data;
        }

        std::vector<BIMData> loadBIMDataFromFolder(const std::string& folder_path,
            const std::string& file_pattern) {
            std::vector<BIMData> bim_data_list;

            try {
                if (!std::filesystem::exists(folder_path) || !std::filesystem::is_directory(folder_path)) {
                    std::cerr << "BIM folder not found: " << folder_path << std::endl;
                    return bim_data_list;
                }

                // Determine file extension from pattern
                std::string extension = ".gltf";
                if (file_pattern.find(".glb") != std::string::npos) {
                    extension = ".glb";
                }

                for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
                    if (entry.is_regular_file()) {
                        std::string file_path = entry.path().string();
                        std::string file_extension = entry.path().extension().string();

                        (std::transform)(file_extension.begin(), file_extension.end(),
                            file_extension.begin(), ::tolower);

                        if (file_extension == extension ||
                            (extension == ".gltf" && file_extension == ".glb")) {
                            BIMData bim_data = loadBIMData(file_path);
                            if (!bim_data.triangles.empty()) {
                                bim_data_list.push_back(std::move(bim_data));
                            }
                        }
                    }
                }

                std::cout << "Found and loaded " << bim_data_list.size()
                    << " BIM files from " << folder_path << std::endl;

            }
            catch (const std::exception& e) {
                std::cerr << "Error loading BIM data from folder " << folder_path
                    << ": " << e.what() << std::endl;
            }

            return bim_data_list;
        }

        std::vector<std::string> findBIMFiles(const std::string& folder_path,
            const std::string& file_pattern) {
            std::vector<std::string> file_paths;

            try {
                if (!std::filesystem::exists(folder_path) || !std::filesystem::is_directory(folder_path)) {
                    return file_paths;
                }

                std::string extension = ".gltf";
                if (file_pattern.find(".glb") != std::string::npos) {
                    extension = ".glb";
                }

                for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
                    if (entry.is_regular_file()) {
                        std::string file_extension = entry.path().extension().string();
                        (std::transform)(file_extension.begin(), file_extension.end(),
                            file_extension.begin(), ::tolower);

                        if (file_extension == extension ||
                            (extension == ".gltf" && file_extension == ".glb")) {
                            file_paths.push_back(entry.path().string());
                        }
                    }
                }

            }
            catch (const std::exception& e) {
                std::cerr << "Error finding BIM files in " << folder_path << ": " << e.what() << std::endl;
            }

            return file_paths;
        }

    } // namespace BIMLoader

    // DistanceCalculator Implementation
    namespace DistanceCalculator {

        float pointToTriangleDistance(const MeshVertex& point, const MeshTriangle& triangle, MeshVertex& closest_point) {
            // Calculate distance from point to triangle in 3D space
            // This is a simplified implementation - a complete version would implement
            // proper barycentric coordinate calculations and edge/vertex distance handling

            const MeshVertex& v0 = triangle.v0;
            const MeshVertex& v1 = triangle.v1;
            const MeshVertex& v2 = triangle.v2;

            // Calculate triangle normal
            MeshVertex edge1 = v1 - v0;
            MeshVertex edge2 = v2 - v0;
            MeshVertex normal = MeshVertex(
                edge1.y * edge2.z - edge1.z * edge2.y,
                edge1.z * edge2.x - edge1.x * edge2.z,
                edge1.x * edge2.y - edge1.y * edge2.x
            );

            float normal_length = normal.length();
            if (normal_length > 0.0f) {
                normal = normal * (1.0f / normal_length);
            }

            // Calculate distance from point to triangle plane
            MeshVertex to_point = point - v0;
            float plane_distance = std::abs(to_point.dot(normal));

            // Project point onto triangle plane
            MeshVertex projected_point = point - normal * (to_point.dot(normal));

            // Simplified: use triangle center as closest point
            // TODO: Implement proper barycentric coordinate calculation
            // 1. Project point onto triangle plane
            // 2. Check if projected point is inside triangle using barycentric coordinates
            // 3. If inside, distance is plane distance; if outside, find closest edge/vertex
            closest_point = triangle.center();

            float dx = point.x - closest_point.x;
            float dy = point.y - closest_point.y;
            float dz = point.z - closest_point.z;

            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        DistanceResult pointToMeshDistance(const MeshVertex& point, const BIMData& bim_data) {
            DistanceResult result;
            result.distance_to_mesh = (std::numeric_limits<float>::max)();
            result.closest_bim_file = bim_data.source_file;

            MeshVertex temp_closest_point;

            for (const auto& triangle : bim_data.triangles) {
                MeshVertex closest_point_on_triangle;
                float distance = pointToTriangleDistance(point, triangle, closest_point_on_triangle);

                if (distance < result.distance_to_mesh) {
                    result.distance_to_mesh = distance;
                    result.closest_triangle = triangle;
                    result.closest_point_on_mesh = closest_point_on_triangle;
                }
            }

            return result;
        }

        std::vector<DistanceResult> calculateDistances(const PointCloudChunk& chunk,
            const std::vector<BIMData>& bim_data_list,
            const std::string& source_file) {
            std::vector<DistanceResult> results;
            results.reserve(chunk.points.size());

            std::cout << "Calculating distances for " << chunk.points.size()
                << " points against " << bim_data_list.size() << " BIM models" << std::endl;

            // Progress tracking
            size_t processed_points = 0;
            size_t total_points = chunk.points.size();
            auto start_time = std::chrono::steady_clock::now();

            for (size_t i = 0; i < chunk.points.size(); ++i) {
                const auto& point = chunk.points[i];

                // Convert point cloud point to mesh vertex
                MeshVertex mesh_point(point.x, point.y, point.z);

                DistanceResult point_result;
                point_result.point_index = static_cast<uint32_t>(i);
                point_result.distance_to_mesh = (std::numeric_limits<float>::max)();
                point_result.source_point_file = source_file;

                // Find minimum distance to all BIM models
                for (const auto& bim_data : bim_data_list) {
                    DistanceResult temp_result = pointToMeshDistance(mesh_point, bim_data);

                    if (temp_result.distance_to_mesh < point_result.distance_to_mesh) {
                        point_result.distance_to_mesh = temp_result.distance_to_mesh;
                        point_result.closest_triangle = temp_result.closest_triangle;
                        point_result.closest_point_on_mesh = temp_result.closest_point_on_mesh;
                        point_result.closest_bim_file = temp_result.closest_bim_file;
                    }
                }

                results.push_back(point_result);
                processed_points++;

                // Progress indicator (every 10%)
                if (processed_points % (total_points / 10 + 1) == 0) {
                    auto current_time = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
                    double progress = static_cast<double>(processed_points) / total_points * 100.0;

                    std::cout << "  Progress: " << progress << "% (" << processed_points
                        << "/" << total_points << ") - Elapsed: " << elapsed << "s" << std::endl;
                }
            }

            // Final statistics
            if (!results.empty()) {
                auto min_max = (std::minmax_element)(results.begin(), results.end(),
                    [](const DistanceResult& a, const DistanceResult& b) {
                        return a.distance_to_mesh < b.distance_to_mesh;
                    });

                // Calculate average distance
                double sum_distance = 0.0;
                for (const auto& result : results) {
                    sum_distance += result.distance_to_mesh;
                }
                double avg_distance = sum_distance / results.size();

                auto end_time = std::chrono::steady_clock::now();
                auto total_time = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();

                std::cout << "Distance calculation completed:" << std::endl;
                std::cout << "  Min distance: " << min_max.first->distance_to_mesh << "m" << std::endl;
                std::cout << "  Max distance: " << min_max.second->distance_to_mesh << "m" << std::endl;
                std::cout << "  Average distance: " << avg_distance << "m" << std::endl;
                std::cout << "  Total processing time: " << total_time << "s" << std::endl;
                std::cout << "  Points per second: " << (results.size() / (std::max)(static_cast<size_t>(total_time), size_t(1))) << std::endl;
            }

            return results;
        }

        // SpatialIndex Implementation (simplified version)
        void SpatialIndex::buildIndex(const std::vector<BIMData>& bim_data_list) {
            all_triangles_.clear();
            triangle_indices_.clear();

            size_t triangle_count = 0;
            for (const auto& bim_data : bim_data_list) {
                triangle_count += bim_data.triangles.size();
            }

            all_triangles_.reserve(triangle_count);
            triangle_indices_.reserve(triangle_count);

            size_t index = 0;
            for (const auto& bim_data : bim_data_list) {
                for (const auto& triangle : bim_data.triangles) {
                    all_triangles_.push_back(triangle);
                    triangle_indices_.push_back(index++);
                }
            }

            std::cout << "Spatial index built with " << all_triangles_.size() << " triangles" << std::endl;
        }

        std::vector<size_t> SpatialIndex::queryNearbyTriangles(const MeshVertex& point, float radius) {
            std::vector<size_t> nearby_triangles;

            // Simplified spatial query (actual implementation would use octree, kd-tree, etc.)
            for (size_t i = 0; i < all_triangles_.size(); ++i) {
                const auto& triangle = all_triangles_[i];
                MeshVertex center = triangle.center();

                float dx = point.x - center.x;
                float dy = point.y - center.y;
                float dz = point.z - center.z;
                float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

                if (distance <= radius) {
                    nearby_triangles.push_back(i);
                }
            }

            return nearby_triangles;
        }

    } // namespace DistanceCalculator

} // namespace DPApp