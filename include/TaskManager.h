#pragma once

// Windowsì˜ min/max ë§¤í¬ë¡œì™€ std::min/max ì¶©ëŒ ë°©ì§€
#ifdef _WIN32
#define NOMINMAX
#endif

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

namespace DPApp {

    // BIM ë©"ì‹œ ì •ì  êµ¬ì¡°ì²´
    struct MeshVertex {
        float x, y, z;

        // 기본 생성자를 명시적으로 정의
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

        // ë²¡í„° ì—°ì‚°ì„ ìœ„í•œ ì—°ì‚°ìž ì˜¤ë²„ë¡œë"©
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

    // BIM ë©"ì‹œ ì‚¼ê°í˜• êµ¬ì¡°ì²´
    struct MeshTriangle {
        MeshVertex v0, v1, v2;

        // 기본 생성자를 명시적으로 정의
        MeshTriangle() : v0(), v1(), v2() {}

        MeshTriangle(const MeshVertex& vertex0, const MeshVertex& vertex1, const MeshVertex& vertex2)
            : v0(vertex0), v1(vertex1), v2(vertex2) {
        }

        // ì‚¼ê°í˜•ì˜ ì¤'ì‹¬ì  ê³„ì‚°
        MeshVertex center() const {
            return MeshVertex(
                (v0.x + v1.x + v2.x) / 3.0f,
                (v0.y + v1.y + v2.y) / 3.0f,
                (v0.z + v1.z + v2.z) / 3.0f
            );
        }

        // ì‚¼ê°í˜•ì˜ ë²•ì„  ë²¡í„° ê³„ì‚°
        MeshVertex normal() const {
            MeshVertex edge1 = v1 - v0;
            MeshVertex edge2 = v2 - v0;

            // ì™¸ì  ê³„ì‚°
            MeshVertex cross(
                edge1.y * edge2.z - edge1.z * edge2.y,
                edge1.z * edge2.x - edge1.x * edge2.z,
                edge1.x * edge2.y - edge1.y * edge2.x
            );

            float len = cross.length();
            if (len > 0.0f) {
                return cross * (1.0f / len);
            }

            return MeshVertex(0.0f, 0.0f, 1.0f); // ê¸°ë³¸ ë²•ì„ 
        }
    };

    // BIM ë©"ì‹œ ë°ì´í„° êµ¬ì¡°ì²´
    struct BIMData {
        std::vector<MeshTriangle> triangles;
        std::string source_file;
        std::string model_name;

        BIMData() = default;
        BIMData(const std::string& file) : source_file(file) {
            model_name = std::filesystem::path(file).stem().string();
        }

        // ë°"ìš´ë"© ë°•ìŠ¤ ê³„ì‚°
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

    // ê±°ë¦¬ ê³„ì‚° ê²°ê³¼ êµ¬ì¡°ì²´
    struct DistanceResult {
        uint32_t point_index;
        float distance_to_mesh;
        MeshTriangle closest_triangle;
        MeshVertex closest_point_on_mesh;
        std::string source_point_file;
        std::string closest_bim_file;

        DistanceResult() : point_index(0), distance_to_mesh((std::numeric_limits<float>::max)()) {}
    };

    // í¬ì¸íŠ¸í´ë¼ìš°ë"œ íŒŒì¼ ì •ë³´
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

    // BIM-í¬ì¸íŠ¸í´ë¼ìš°ë"œ ë¹„êµ ìž'ì—… íŒŒë¼ë¯¸í„°
    struct BIMComparisonParams {
        std::string bim_folder_path;
        std::string pointcloud_folder_path;
        std::string bim_file_pattern;        // ê¸°ë³¸ê°': "*.gltf"
        std::string pointcloud_file_pattern; // ê¸°ë³¸ê°': "*.las"
        uint32_t max_points_per_chunk;       // ì²­í‚¹ì„ ìœ„í•œ ìµœëŒ€ í¬ì¸íŠ¸ ìˆ˜
        float distance_threshold;            // ìµœëŒ€ ê±°ë¦¬ ìž„ê³„ê°' (ë¯¸í„°)
        bool enable_color_coding;            // ê±°ë¦¬ ê¸°ë°˜ ìƒ‰ìƒ ì½"ë"© í™œì„±í™"

        BIMComparisonParams()
            : bim_file_pattern("*.gltf"), pointcloud_file_pattern("*.las"),
            max_points_per_chunk(100000), distance_threshold(10.0f),
            enable_color_coding(true) {
        }
    };

    // ìž'ì—… ìƒíƒœ ì •ì˜
    enum class TaskStatus {
        PENDING,        // ëŒ€ê¸° ì¤'
        ASSIGNED,       // í• ë‹¹ë¨
        IN_PROGRESS,    // ì§„í–‰ ì¤'
        COMPLETED,      // ì™„ë£Œë¨
        FAILED,         // ì‹¤íŒ¨
        TIMEOUT         // íƒ€ìž„ì•„ì›ƒ
    };

    // ìž'ì—… ìš°ì„ ìˆœìœ„
    enum class TaskPriority {
        LOW = 0,
        NORMAL = 1,
        HIGH = 2,
        CRITICAL = 3
    };

    // í™•ìž¥ëœ ìž'ì—… ì •ë³´
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

        // ìž'ì—… ì‹¤í–‰ ì‹œê°„ ê³„ì‚°
        double getExecutionTime() const {
            if (status != TaskStatus::COMPLETED) return 0.0;

            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                completed_time - assigned_time);
            return duration.count() / 1000.0;
        }
    };

    // ìŠ¬ë ˆì´ë¸Œ ì›Œì»¤ ì •ë³´
    struct SlaveWorkerInfo {
        std::string slave_id;
        bool is_active;
        bool is_busy;
        std::string current_task_id;
        std::chrono::steady_clock::time_point last_heartbeat;
        uint32_t completed_tasks;
        uint32_t failed_tasks;
        double avg_processing_time;  // ì´ˆ ë‹¨ìœ„
        std::vector<std::string> supported_task_types;

        SlaveWorkerInfo() : is_active(false), is_busy(false), completed_tasks(0),
            failed_tasks(0), avg_processing_time(0.0),
            last_heartbeat(std::chrono::steady_clock::now()) {
        }

        // ì„±ê³µë¥  ê³„ì‚°
        double getSuccessRate() const {
            uint32_t total = completed_tasks + failed_tasks;
            return total > 0 ? static_cast<double>(completed_tasks) / total : 1.0;
        }

        // ì„±ëŠ¥ ì ìˆ˜ ê³„ì‚°
        double getPerformanceScore() const {
            double success_rate = getSuccessRate();
            double speed_factor = avg_processing_time > 0 ? 1.0 / avg_processing_time : 1.0;
            return success_rate * speed_factor;
        }
    };

    // ìž'ì—… ë¶„ë°° ì „ëžµ
    enum class TaskDistributionStrategy {
        ROUND_ROBIN,        // ìˆœì°¨ ë¶„ë°°
        LOAD_BALANCED,      // ë¶€í•˜ ê· í˜•
        PERFORMANCE_BASED,  // ì„±ëŠ¥ ê¸°ë°˜
        TASK_TYPE_AWARE     // ìž'ì—… íƒ€ìž… ì¸ì‹
    };

    // í¬ì¸íŠ¸í´ë¼ìš°ë"œ íŒŒì¼ ë¡œë" ìœ í‹¸ë¦¬í‹° (ì „ë°© ì„ ì–¸)
    namespace PointCloudLoader {
        std::vector<std::shared_ptr<PointCloudChunk>> loadPointCloudFileInChunks(
            const std::string& file_path, uint32_t max_points_per_chunk = 100000);
        std::shared_ptr<PointCloudChunk> loadPointCloudFile(const std::string& file_path);
        std::shared_ptr<PointCloudChunk> loadLASFile(const std::string& file_path);
        std::shared_ptr<PointCloudChunk> loadPLYFile(const std::string& file_path);
        std::shared_ptr<PointCloudChunk> loadPCDFile(const std::string& file_path);
        std::shared_ptr<PointCloudChunk> loadXYZFile(const std::string& file_path);
        PointCloudFileInfo getPointCloudFileInfo(const std::string& file_path);
    }

    // ìž'ì—… ê´€ë¦¬ìž (Masterìš©)
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

        // ìž'ì—… ê´€ë¦¬
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

        // BIMê³¼ í¬ì¸íŠ¸í´ë¼ìš°ë"œ í´ë"ë¥¼ ìž…ë ¥ìœ¼ë¡œ ë°›ì•„ ê±°ë¦¬ ê³„ì‚° ìž'ì—…ë"¤ì„ ìƒì„±í•˜ëŠ" ë©"ì†Œë"œ
        std::vector<uint32_t> addBIMComparisonTasks(const BIMComparisonParams& params,
            TaskPriority priority = TaskPriority::NORMAL) {
            std::vector<uint32_t> task_ids;

            try {
                std::cout << "Starting BIM-PointCloud comparison setup..." << std::endl;
                std::cout << "BIM folder: " << params.bim_folder_path << std::endl;
                std::cout << "PointCloud folder: " << params.pointcloud_folder_path << std::endl;

                // 1. í´ë" ìœ íš¨ì„± ê²€ì‚¬
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

                // 2. í¬ì¸íŠ¸í´ë¼ìš°ë"œ íŒŒì¼ë"¤ ì°¾ê¸° ë° ì •ë³´ ìˆ˜ì§'
                std::vector<PointCloudFileInfo> pc_files = findPointCloudFiles(
                    params.pointcloud_folder_path, params.pointcloud_file_pattern);

                if (pc_files.empty()) {
                    std::cerr << "No point cloud files found in " << params.pointcloud_folder_path << std::endl;
                    return task_ids;
                }

                std::cout << "Found " << pc_files.size() << " point cloud files" << std::endl;

                // 3. ê° í¬ì¸íŠ¸í´ë¼ìš°ë"œ íŒŒì¼ì— ëŒ€í•´ ìž'ì—… ìƒì„±
                for (const auto& pc_file : pc_files) {
                    // íŒŒì¼ì„ ì²­í¬ë¡œ ë¶„í• í•˜ì—¬ ìž'ì—… ìƒì„±
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

        // ê°„ë‹¨í•œ BIM ë¹„êµ ìž'ì—… ì¶"ê°€ (í´ë" ê²½ë¡œë§Œìœ¼ë¡œ)
        std::vector<uint32_t> addBIMComparisonTasks(const std::string& bim_folder_path,
            const std::string& pointcloud_folder_path,
            TaskPriority priority = TaskPriority::NORMAL) {
            BIMComparisonParams params;
            params.bim_folder_path = bim_folder_path;
            params.pointcloud_folder_path = pointcloud_folder_path;

            return addBIMComparisonTasks(params, priority);
        }

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

        void clearAllTasks() {
            std::lock_guard<std::mutex> lock(tasks_mutex_);
            size_t count = tasks_.size();
            tasks_.clear();
            std::cout << "Cleared " << count << " tasks" << std::endl;
        }

        // ìž'ì—… ë¶„ë°°
        void setDistributionStrategy(TaskDistributionStrategy strategy) {
            distribution_strategy_ = strategy;
            std::cout << "Distribution strategy changed to: " << static_cast<int>(strategy) << std::endl;
        }

        bool assignTasksToSlaves() {
            if (stopping_) {
                return false;  // ì¢…ë£Œ ìš"ì²­ì‹œ ìƒˆ ìž'ì—… í• ë‹¹ ì¤'ì§€
            }

            std::lock_guard<std::mutex> tasks_lock(tasks_mutex_);
            std::lock_guard<std::mutex> slaves_lock(slaves_mutex_);

            // ëŒ€ê¸° ì¤'ì¸ ìž'ì—…ë"¤ì„ ìš°ì„ ìˆœìœ„ ìˆœìœ¼ë¡œ ì •ë ¬
            std::vector<uint32_t> pending_tasks;
            for (auto& [task_id, task_info] : tasks_) {
                if (task_info.status == TaskStatus::PENDING) {
                    pending_tasks.push_back(task_id);
                }
            }

            if (pending_tasks.empty()) {
                return false;
            }

            // ìš°ì„ ìˆœìœ„ ê¸°ì¤€ìœ¼ë¡œ ì •ë ¬
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

                    // ìŠ¬ë ˆì´ë¸Œë¥¼ ë°"ìœ ìƒíƒœë¡œ ì„¤ì •
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

        // ìž'ì—… ìƒíƒœ ê´€ë¦¬
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

                // BIM ë¹„êµ ê²°ê³¼ì¸ ê²½ìš° ë³„ë„ë¡œ ì €ìž¥
                if (task_info.task_type == TaskType::BIM_DISTANCE_CALCULATION) {
                    std::lock_guard<std::mutex> bim_lock(bim_results_mutex_);
                    // BIM ê²°ê³¼ë¥¼ ë³„ë„ ì €ìž¥í•˜ëŠ" ë¡œì§ì€ í•„ìš"ì‹œ êµ¬í˜„
                }

                // ìŠ¬ë ˆì´ë¸Œ í†µê³„ ì—…ë°ì´íŠ¸
                if (!task_info.assigned_slave.empty()) {
                    auto slave_it = slaves_.find(task_info.assigned_slave);
                    if (slave_it != slaves_.end()) {
                        slave_it->second.is_busy = false;
                        slave_it->second.current_task_id.clear();
                        slave_it->second.completed_tasks++;

                        // ì²˜ë¦¬ ì‹œê°„ ê³„ì‚°
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
                //failTask(task_id, result.error_message);
                failTaskInternal(task_id, result.error_message);
            }

            return true;
        }

        bool failTask(uint32_t task_id, const std::string& error_message) {
            std::lock_guard<std::mutex> tasks_lock(tasks_mutex_);
            std::lock_guard<std::mutex> slaves_lock(slaves_mutex_);

            failTaskInternal(task_id, error_message);
            return true;
        }

        // ìŠ¬ë ˆì´ë¸Œ ê´€ë¦¬
        void registerSlave(const std::string& slave_id) {
            std::lock_guard<std::mutex> lock(slaves_mutex_);

            SlaveWorkerInfo slave_info;
            slave_info.slave_id = slave_id;
            slave_info.is_active = true;

            slaves_[slave_id] = slave_info;

            std::cout << "Slave registered: " << slave_id << std::endl;
        }

        void unregisterSlave(const std::string& slave_id) {
            std::lock_guard<std::mutex> slaves_lock(slaves_mutex_);
            std::lock_guard<std::mutex> tasks_lock(tasks_mutex_);

            auto slave_it = slaves_.find(slave_id);
            if (slave_it != slaves_.end()) {
                // í•´ë‹¹ ìŠ¬ë ˆì´ë¸Œì— í• ë‹¹ëœ ìž'ì—…ë"¤ì„ ë‹¤ì‹œ ëŒ€ê¸° ìƒíƒœë¡œ ë³€ê²½
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

        void updateSlaveHeartbeat(const std::string& slave_id) {
            std::lock_guard<std::mutex> lock(slaves_mutex_);

            auto it = slaves_.find(slave_id);
            if (it != slaves_.end()) {
                it->second.last_heartbeat = std::chrono::steady_clock::now();
            }
        }

        // ìƒíƒœ ì¡°íšŒ
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

        // í†µê³„
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
                return 1.0; // ìž'ì—…ì´ ì—†ìœ¼ë©´ 100%
            }

            std::vector<TaskStatus> statuses;
            statuses.reserve(tasks_.size());
            for (const auto& [task_id, task_info] : tasks_) {
                statuses.push_back(task_info.status);
            }

            size_t completed = (std::count)(statuses.begin(), statuses.end(), TaskStatus::COMPLETED);
            return static_cast<double>(completed) / statuses.size();
        }

        // íƒ€ìž„ì•„ì›ƒ ì²˜ë¦¬
        void setTaskTimeout(uint32_t timeout_seconds) {
            task_timeout_seconds_ = timeout_seconds;
            std::cout << "Task timeout set to " << timeout_seconds << " seconds" << std::endl;
        }

        void checkTimeouts() {
            std::lock_guard<std::mutex> tasks_lock(tasks_mutex_);

            auto now = std::chrono::steady_clock::now();
            int timed_out_count = 0;

            for (auto& [task_id, task_info] : tasks_) {
                if (task_info.status == TaskStatus::ASSIGNED || task_info.status == TaskStatus::IN_PROGRESS) {
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        now - task_info.assigned_time).count();

                    if (elapsed > static_cast<int64_t>(task_timeout_seconds_)) {
                        std::cout << "Task " << task_id << " timed out after " << elapsed << " seconds" << std::endl;
                        task_info.status = TaskStatus::TIMEOUT;
                        timed_out_count++;

                        // ìŠ¬ë ˆì´ë¸Œ í•´ì œ
                        if (!task_info.assigned_slave.empty()) {
                            std::lock_guard<std::mutex> slaves_lock(slaves_mutex_);
                            auto slave_it = slaves_.find(task_info.assigned_slave);
                            if (slave_it != slaves_.end()) {
                                slave_it->second.is_busy = false;
                                slave_it->second.current_task_id.clear();
                            }
                        }

                        // ìž¬ì‹œë„ë¥¼ ìœ„í•´ ë‹¤ì‹œ ëŒ€ê¸° ìƒíƒœë¡œ ë³€ê²½
                        failTask(task_id, "Task timeout");
                    }
                }
            }

            if (timed_out_count > 0) {
                std::cout << "Found " << timed_out_count << " timed out tasks" << std::endl;
            }
        }

        // ê²°ê³¼ ìˆ˜ì§'
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

        // BIM ë¹„êµ ê²°ê³¼ ì „ìš© ë©"ì†Œë"œë"¤
        std::vector<DistanceResult> getBIMComparisonResults() {
            std::lock_guard<std::mutex> lock(bim_results_mutex_);
            return bim_comparison_results_;
        }

        void saveBIMComparisonResults(const std::string& output_file_path) {
            std::lock_guard<std::mutex> lock(bim_results_mutex_);

            try {
                std::ofstream output_file(output_file_path);
                if (!output_file.is_open()) {
                    std::cerr << "Failed to open output file: " << output_file_path << std::endl;
                    return;
                }

                // CSV í—¤ë" ìž'ì„±
                output_file << "PointIndex,Distance(m),SourceFile,ClosestBIMFile,"
                    << "ClosestTriangleV0X,ClosestTriangleV0Y,ClosestTriangleV0Z,"
                    << "ClosestTriangleV1X,ClosestTriangleV1Y,ClosestTriangleV1Z,"
                    << "ClosestTriangleV2X,ClosestTriangleV2Y,ClosestTriangleV2Z,"
                    << "ClosestPointX,ClosestPointY,ClosestPointZ\n";

                // ê²°ê³¼ ë°ì´í„° ìž'ì„±
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

        // í†µê³„ ë¦¬í¬íŠ¸ ìƒì„±
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

        // ì½œë°± ì„¤ì •
        using TaskCompletedCallback = std::function<void(const TaskInfo&, const ProcessingResult&)>;
        using TaskFailedCallback = std::function<void(const TaskInfo&, const std::string&)>;

        void setTaskCompletedCallback(TaskCompletedCallback callback) {
            task_completed_callback_ = callback;
        }

        void setTaskFailedCallback(TaskFailedCallback callback) {
            task_failed_callback_ = callback;
        }

    private:
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

        // í¬ì¸íŠ¸í´ë¼ìš°ë"œ íŒŒì¼ ì°¾ê¸°
        std::vector<PointCloudFileInfo> findPointCloudFiles(const std::string& folder_path,
            const std::string& file_pattern) {
            std::vector<PointCloudFileInfo> files;

            // ì§€ì›í•˜ëŠ" í¬ì¸íŠ¸í´ë¼ìš°ë"œ íŒŒì¼ í™•ìž¥ìžë"¤
            std::vector<std::string> supported_extensions = { ".las", ".laz", ".ply", ".pcd", ".xyz", ".pts" };

            try {
                for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
                    if (entry.is_regular_file()) {
                        std::string file_path = entry.path().string();
                        std::string file_extension = entry.path().extension().string();

                        // í™•ìž¥ìžë¥¼ ì†Œë¬¸ìžë¡œ ë³€í™˜
                        (std::transform)(file_extension.begin(), file_extension.end(),
                            file_extension.begin(), ::tolower);

                        // ì§€ì›í•˜ëŠ" í™•ìž¥ìžì¸ì§€ í™•ì¸
                        if ((std::find)(supported_extensions.begin(), supported_extensions.end(),
                            file_extension) != supported_extensions.end()) {
                            PointCloudFileInfo file_info(file_path);

                            // íŒŒì¼ ì •ë³´ ìˆ˜ì§' (ì‹¤ì œ ë¡œë"œ ì—†ì´ ë©"íƒ€ë°ì´í„°ë§Œ)
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

        // í¬ì¸íŠ¸í´ë¼ìš°ë"œ íŒŒì¼ì— ëŒ€í•œ ìž'ì—…ë"¤ ìƒì„±
        std::vector<uint32_t> createTasksForPointCloudFile(const PointCloudFileInfo& pc_file,
            const BIMComparisonParams& params,
            TaskPriority priority) {
            std::vector<uint32_t> task_ids;

            try {
                // í¬ì¸íŠ¸í´ë¼ìš°ë"œ íŒŒì¼ì„ ì²­í¬ë¡œ ë¡œë"œ
                std::vector<std::shared_ptr<PointCloudChunk>> chunks =
                    PointCloudLoader::loadPointCloudFileInChunks(pc_file.file_path, params.max_points_per_chunk);

                if (chunks.empty()) {
                    std::cerr << "Failed to load point cloud file: " << pc_file.file_path << std::endl;
                    return task_ids;
                }

                std::cout << "Loaded " << pc_file.file_path << " into " << chunks.size() << " chunks" << std::endl;

                // ê° ì²­í¬ì— ëŒ€í•´ BIM ë¹„êµ ìž'ì—… ìƒì„±
                for (auto& chunk : chunks) {
                    // BIM í´ë" ê²½ë¡œì™€ íŒŒì¼ íŒ¨í„´ì„ íŒŒë¼ë¯¸í„°ë¡œ íŒ¨í‚¹
                    std::vector<uint8_t> parameters;

                    // BIM í´ë" ê²½ë¡œ íŒ¨í‚¹
                    uint32_t bim_path_len = static_cast<uint32_t>(params.bim_folder_path.length());
                    parameters.insert(parameters.end(),
                        reinterpret_cast<const uint8_t*>(&bim_path_len),
                        reinterpret_cast<const uint8_t*>(&bim_path_len) + sizeof(uint32_t));
                    parameters.insert(parameters.end(),
                        params.bim_folder_path.begin(), params.bim_folder_path.end());

                    // BIM íŒŒì¼ íŒ¨í„´ íŒ¨í‚¹
                    uint32_t bim_pattern_len = static_cast<uint32_t>(params.bim_file_pattern.length());
                    parameters.insert(parameters.end(),
                        reinterpret_cast<const uint8_t*>(&bim_pattern_len),
                        reinterpret_cast<const uint8_t*>(&bim_pattern_len) + sizeof(uint32_t));
                    parameters.insert(parameters.end(),
                        params.bim_file_pattern.begin(), params.bim_file_pattern.end());

                    // ì›ë³¸ í¬ì¸íŠ¸í´ë¼ìš°ë"œ íŒŒì¼ ê²½ë¡œ íŒ¨í‚¹
                    uint32_t pc_path_len = static_cast<uint32_t>(pc_file.file_path.length());
                    parameters.insert(parameters.end(),
                        reinterpret_cast<const uint8_t*>(&pc_path_len),
                        reinterpret_cast<const uint8_t*>(&pc_path_len) + sizeof(uint32_t));
                    parameters.insert(parameters.end(),
                        pc_file.file_path.begin(), pc_file.file_path.end());

                    // ì¶"ê°€ íŒŒë¼ë¯¸í„° (ê±°ë¦¬ ìž„ê³„ê°', ìƒ‰ìƒ ì½"ë"© ë"±)
                    parameters.insert(parameters.end(),
                        reinterpret_cast<const uint8_t*>(&params.distance_threshold),
                        reinterpret_cast<const uint8_t*>(&params.distance_threshold) + sizeof(float));

                    uint8_t color_coding = params.enable_color_coding ? 1 : 0;
                    parameters.push_back(color_coding);

                    // ìž'ì—… ì¶"ê°€
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

        // ìž'ì—… ë¶„ë°° ì „ëžµ êµ¬í˜„
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

            // ì§€ì›í•˜ëŠ" ìŠ¬ë ˆì´ë¸Œê°€ ì—†ìœ¼ë©´ ì¼ë°˜ì ì¸ ì„±ëŠ¥ ê¸°ë°˜ ì„ íƒ
            return best_slave.empty() ? selectSlavePerformanceBased() : best_slave;
        }

        void updateSlaveStatistics(const std::string& slave_id, bool success, double processing_time) {
            auto slave_it = slaves_.find(slave_id);
            if (slave_it != slaves_.end()) {
                SlaveWorkerInfo& slave_info = slave_it->second;

                // í‰ê·  ì²˜ë¦¬ ì‹œê°„ ì—…ë°ì´íŠ¸ (ì´ë™í‰ê· )
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

        std::map<uint32_t, TaskInfo> tasks_;
        std::map<std::string, SlaveWorkerInfo> slaves_;
        std::vector<ProcessingResult> completed_results_;

        // BIM ë¹„êµ ê²°ê³¼ë¥¼ ë³„ë„ë¡œ ì €ìž¥
        std::vector<DistanceResult> bim_comparison_results_;

        std::mutex tasks_mutex_;
        std::mutex slaves_mutex_;
        std::mutex results_mutex_;
        std::mutex bim_results_mutex_;

        std::atomic<uint32_t> next_task_id_;
        TaskDistributionStrategy distribution_strategy_;
        uint32_t task_timeout_seconds_;
        size_t current_slave_index_;  // Round-robinìš©

        TaskCompletedCallback task_completed_callback_;
        TaskFailedCallback task_failed_callback_;
    };

    // ìž'ì—… ì²˜ë¦¬ê¸° (Slaveìš©)
    class TaskProcessor {
    public:
        TaskProcessor();
        ~TaskProcessor();

        // ì²˜ë¦¬ê¸° ë"±ë¡
        using ProcessorFunction = std::function<ProcessingResult(const ProcessingTask&, const PointCloudChunk&)>;

        void registerProcessor(const TaskType task_type, ProcessorFunction processor);
        void unregisterProcessor(const TaskType task_type);

        // ìž'ì—… ì²˜ë¦¬
        ProcessingResult processTask(const ProcessingTask& task, const PointCloudChunk& chunk);

        // ì§€ì›ë˜ëŠ" ìž'ì—… íƒ€ìž… ì¡°íšŒ
        std::vector<TaskType> getSupportedTaskTypes();
        bool supportsTaskType(const TaskType task_type);

        // í†µê³„
        uint32_t getProcessedTaskCount() const { return processed_task_count_; }
        uint32_t getFailedTaskCount() const { return failed_task_count_; }
        double getAverageProcessingTime() const { return avg_processing_time_; }

        // í"„ë¡œì„¸ì„œ ì„±ëŠ¥ í†µê³„
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

    // ê¸°ë³¸ í¬ì¸íŠ¸í´ë¼ìš°ë"œ ì²˜ë¦¬ í•¨ìˆ˜ë"¤
    namespace PointCloudProcessors {
        /// [Smaple] Filtering based on heights
        ProcessingResult filterPoints(const ProcessingTask& task, const PointCloudChunk& chunk);
        /// [Smaple] Classification based on heights
        ProcessingResult classifyPoints(const ProcessingTask& task, const PointCloudChunk& chunk);
        /// [Smaple] Estimating normal vectors
        ProcessingResult estimateNormals(const ProcessingTask& task, const PointCloudChunk& chunk);
        /// [Smaple] Statitical filtering ( mean_z - sigma < inlier < min_z + sigma)
        ProcessingResult removeOutliers(const ProcessingTask& task, const PointCloudChunk& chunk);
        /// [Sample] Random down sampling
        ProcessingResult downsample(const ProcessingTask& task, const PointCloudChunk& chunk);

        /// Distances between point cloud and BIM 
        ProcessingResult calculateBIMDistance(const ProcessingTask& task, const PointCloudChunk& chunk);
    }

    // BIM íŒŒì¼ ë¡œë" ìœ í‹¸ë¦¬í‹°
    namespace BIMLoader {
        // GLTF/GLB íŒŒì¼ì—ì„œ ë©"ì‹œ ë°ì´í„°ë¥¼ ë¡œë"œí•˜ëŠ" í•¨ìˆ˜
        BIMData loadBIMData(const std::string& file_path);

        // í´ë"ì—ì„œ BIM íŒŒì¼ë"¤ì„ ì°¾ì•„ ë¡œë"œí•˜ëŠ" í•¨ìˆ˜
        std::vector<BIMData> loadBIMDataFromFolder(const std::string& folder_path,
            const std::string& file_pattern = "*.gltf");

        // BIM íŒŒì¼ ì •ë³´ë§Œ ìˆ˜ì§' (ì‹¤ì œ ë¡œë"© ì—†ì´)
        std::vector<std::string> findBIMFiles(const std::string& folder_path,
            const std::string& file_pattern = "*.gltf");
    }

    // ê±°ë¦¬ ê³„ì‚° ìœ í‹¸ë¦¬í‹°
    namespace DistanceCalculator {
        // ì ì—ì„œ ì‚¼ê°í˜•ê¹Œì§€ì˜ ìµœë‹¨ ê±°ë¦¬ ê³„ì‚°
        float pointToTriangleDistance(const MeshVertex& point, const MeshTriangle& triangle, MeshVertex& closest_point);

        // ì ì—ì„œ ë©"ì‹œê¹Œì§€ì˜ ìµœë‹¨ ê±°ë¦¬ ê³„ì‚°
        DistanceResult pointToMeshDistance(const MeshVertex& point, const BIMData& bim_data);

        // í¬ì¸íŠ¸ í´ë¼ìš°ë"œì™€ BIM ë°ì´í„° ê°„ì˜ ê±°ë¦¬ ê³„ì‚°
        std::vector<DistanceResult> calculateDistances(const PointCloudChunk& chunk,
            const std::vector<BIMData>& bim_data_list,
            const std::string& source_file = "");

        // ë¹ ë¥¸ ê±°ë¦¬ ê³„ì‚°ì„ ìœ„í•œ ê³µê°„ ë¶„í•  êµ¬ì¡°
        class SpatialIndex {
        public:
            void buildIndex(const std::vector<BIMData>& bim_data_list);
            std::vector<size_t> queryNearbyTriangles(const MeshVertex& point, float radius);
        private:
            // ì‹¤ì œ êµ¬í˜„ì—ì„œëŠ" octree, kd-tree ë"± ì‚¬ìš©
            std::vector<MeshTriangle> all_triangles_;
            std::vector<size_t> triangle_indices_;
        };
    }

    ///
    /// Implementation (ë‚˜ë¨¸ì§€ êµ¬í˜„ë¶€ëŠ" ê°™ì•„ì„œ ìƒëžµ)
    ///

    /// TaskProcessor Implementation
    TaskProcessor::TaskProcessor()
        : processed_task_count_(0), failed_task_count_(0), avg_processing_time_(0.0) {

        registerProcessor(TaskType::FILTER, PointCloudProcessors::filterPoints);
        /// BIM to PTS
        registerProcessor(TaskType::BIM_DISTANCE_CALCULATION, PointCloudProcessors::calculateBIMDistance);

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

            // í"„ë¡œì„¸ì„œ ì‹¤í–‰
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

        // ì²˜ë¦¬ ì‹œê°„ ì—…ë°ì´íŠ¸
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

        ProcessingResult filterPoints(const ProcessingTask& task, const PointCloudChunk& chunk) {
            ProcessingResult result;
            result.task_id = task.task_id;
            result.chunk_id = task.chunk_id;
            result.success = true;

            try {
                // íŒŒë¼ë¯¸í„° íŒŒì‹± (ê°„ë‹¨í•œ ì˜ˆì œ)
                double min_z = -100.0;
                double max_z = 100.0;

                if (task.parameters.size() >= 16) {
                    std::memcpy(&min_z, task.parameters.data(), sizeof(double));
                    std::memcpy(&max_z, task.parameters.data() + sizeof(double), sizeof(double));
                }

                // Z ì¢Œí'œ í•„í„°ë§
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
                // ê°„ë‹¨í•œ ë†'ì´ ê¸°ë°˜ ë¶„ë¥˜ ì˜ˆì œ
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
                // ê°„ë‹¨í•œ ë²•ì„  ë²¡í„° ì¶"ì • (ì‹¤ì œë¡œëŠ" ë" ë³µìž¡í•œ ì•Œê³ ë¦¬ì¦˜ í•„ìš")
                result.processed_points = chunk.points;

                // ê° í¬ì¸íŠ¸ì— ëŒ€í•´ ê°„ë‹¨í•œ ë²•ì„  ë²¡í„° ê³„ì‚°
                // ì‹¤ì œë¡œëŠ" k-nearest neighborsë‚˜ radius search ì‚¬ìš©
                for (size_t i = 0; i < result.processed_points.size(); ++i) {
                    auto& point = result.processed_points[i];

                    // ë²•ì„  ì •ë³´ë¥¼ RGBì— ì €ìž¥ (ì‹œê°í™"ìš©)
                    point.r = 127; // Normal X
                    point.g = 127; // Normal Y
                    point.b = 255; // Normal Z (ìœ„ìª½ í–¥í•¨)
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
                // í†µê³„ì  ì•„ì›ƒë¼ì´ì–´ ì œê±° (ê°„ë‹¨í•œ ë²„ì „)
                if (chunk.points.empty()) {
                    result.success = false;
                    result.error_message = "Empty point cloud";
                    return result;
                }

                // Z ì¢Œí'œì˜ í‰ê· ê³¼ í'œì¤€íŽ¸ì°¨ ê³„ì‚°
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

                // 2 í'œì¤€íŽ¸ì°¨ ë²"ìœ„ ë‚´ì˜ ì ë"¤ë§Œ ìœ ì§€
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
                // íŒŒë¼ë¯¸í„°ì—ì„œ ë‹¤ìš´ìƒ˜í"Œë§ ë¹„ìœ¨ ì½ê¸°
                double sampling_ratio = 0.1; // ê¸°ë³¸ê°': 10%

                if (task.parameters.size() >= sizeof(double)) {
                    std::memcpy(&sampling_ratio, task.parameters.data(), sizeof(double));
                }

                // ë¬´ìž'ìœ„ ë‹¤ìš´ìƒ˜í"Œë§
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

        // BIM ê±°ë¦¬ ê³„ì‚° í"„ë¡œì„¸ì„œ (ìˆ˜ì •ëœ ë²„ì „)
        ProcessingResult calculateBIMDistance(const ProcessingTask& task, const PointCloudChunk& chunk) {
            ProcessingResult result;
            result.task_id = task.task_id;
            result.chunk_id = task.chunk_id;
            result.success = true;

            try {
                // íŒŒë¼ë¯¸í„° íŒŒì‹±
                if (task.parameters.size() < sizeof(uint32_t) * 3) {
                    result.success = false;
                    result.error_message = "Invalid parameters for BIM distance calculation";
                    return result;
                }

                size_t offset = 0;

                // BIM í´ë" ê²½ë¡œ ì¶"ì¶œ
                uint32_t bim_path_len;
                std::memcpy(&bim_path_len, task.parameters.data() + offset, sizeof(uint32_t));
                offset += sizeof(uint32_t);

                std::string bim_folder_path(
                    reinterpret_cast<const char*>(task.parameters.data() + offset), bim_path_len);
                offset += bim_path_len;

                // BIM íŒŒì¼ íŒ¨í„´ ì¶"ì¶œ
                uint32_t bim_pattern_len;
                std::memcpy(&bim_pattern_len, task.parameters.data() + offset, sizeof(uint32_t));
                offset += sizeof(uint32_t);

                std::string bim_file_pattern(
                    reinterpret_cast<const char*>(task.parameters.data() + offset), bim_pattern_len);
                offset += bim_pattern_len;

                // ì›ë³¸ í¬ì¸íŠ¸í´ë¼ìš°ë"œ íŒŒì¼ ê²½ë¡œ ì¶"ì¶œ
                uint32_t pc_path_len;
                std::memcpy(&pc_path_len, task.parameters.data() + offset, sizeof(uint32_t));
                offset += sizeof(uint32_t);

                std::string source_pc_file(
                    reinterpret_cast<const char*>(task.parameters.data() + offset), pc_path_len);
                offset += pc_path_len;

                // ì¶"ê°€ íŒŒë¼ë¯¸í„°ë"¤
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

                // BIM ë°ì´í„° ë¡œë"œ
                std::vector<BIMData> bim_data_list = BIMLoader::loadBIMDataFromFolder(
                    bim_folder_path, bim_file_pattern);

                if (bim_data_list.empty()) {
                    std::cout << "Warning: No BIM files found in " << bim_folder_path << std::endl;
                    result.processed_points = chunk.points;
                    return result;
                }

                std::cout << "  Loaded " << bim_data_list.size() << " BIM files" << std::endl;

                // ê±°ë¦¬ ê³„ì‚°
                std::vector<DistanceResult> distance_results =
                    DistanceCalculator::calculateDistances(chunk, bim_data_list, source_pc_file);

                // ê²°ê³¼ë¥¼ ì²˜ë¦¬ëœ í¬ì¸íŠ¸ì— ë°˜ì˜
                result.processed_points = chunk.points;

                for (size_t i = 0; i < distance_results.size() && i < result.processed_points.size(); ++i) {
                    const auto& dist_result = distance_results[i];
                    auto& point = result.processed_points[i];

                    // ê±°ë¦¬ ì •ë³´ë¥¼ intensity í•„ë"œì— ì €ìž¥ (ë¯¸í„° ë‹¨ìœ„ë¥¼ ë°€ë¦¬ë¯¸í„°ë¡œ ë³€í™˜)
                    point.intensity = static_cast<uint16_t>((std::min)(dist_result.distance_to_mesh * 1000, 65535.0f));

                    // ê±°ë¦¬ ê¸°ë°˜ ìƒ‰ìƒ ì½"ë"©
                    if (enable_color_coding) {
                        float normalized_distance = (std::min)(dist_result.distance_to_mesh / distance_threshold, 1.0f);

                        // ìƒ‰ìƒ ê·¸ë¼ë°ì´ì…˜: íŒŒëž'(ë©€ìŒ) -> ì´ˆë¡(ì¤'ê°„) -> ë¹¨ê°•(ê°€ê¹Œì›€)
                        if (normalized_distance < 0.5f) {
                            // ê°€ê¹Œìš´ ê±°ë¦¬: ë¹¨ê°• -> ë…¸ëž'
                            point.r = 255;
                            point.g = static_cast<uint8_t>(normalized_distance * 2.0f * 255);
                            point.b = 0;
                        }
                        else {
                            // ë¨¼ ê±°ë¦¬: ë…¸ëž' -> íŒŒëž'
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

    } // namespace PointCloudProcessors

    // PointCloudLoader Implementation
    namespace PointCloudLoader {

        std::vector<std::shared_ptr<PointCloudChunk>> loadPointCloudFileInChunks(
            const std::string& file_path, uint32_t max_points_per_chunk) {

            std::vector<std::shared_ptr<PointCloudChunk>> chunks;

            try {
                std::cout << "Loading point cloud file: " << file_path << std::endl;

                // íŒŒì¼ í™•ìž¥ìžì— ë"°ë¥¸ ë¡œë" ì„ íƒ
                std::string extension = std::filesystem::path(file_path).extension().string();
                (std::transform)(extension.begin(), extension.end(), extension.begin(), ::tolower);

                std::shared_ptr<PointCloudChunk> full_chunk;

                if (extension == ".las" || extension == ".laz") {
                    full_chunk = loadLASFile(file_path);
                }
                else if (extension == ".ply") {
                    full_chunk = loadPLYFile(file_path);
                }
                else if (extension == ".pcd") {
                    full_chunk = loadPCDFile(file_path);
                }
                else if (extension == ".xyz" || extension == ".pts") {
                    full_chunk = loadXYZFile(file_path);
                }
                else {
                    std::cerr << "Unsupported file format: " << extension << std::endl;
                    return chunks;
                }

                if (!full_chunk || full_chunk->points.empty()) {
                    std::cerr << "Failed to load point cloud file: " << file_path << std::endl;
                    return chunks;
                }

                std::cout << "Loaded " << full_chunk->points.size() << " points" << std::endl;

                // ì²­í‚¹ ìˆ˜í–‰
                if (full_chunk->points.size() <= max_points_per_chunk) {
                    // ì²­í‚¹ì´ í•„ìš"ì—†ëŠ" ê²½ìš°
                    chunks.push_back(full_chunk);
                }
                else {
                    // ì²­í‚¹ ìˆ˜í–‰
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

                        chunks.push_back(chunk);
                    }

                    std::cout << "Split into " << chunks.size() << " chunks" << std::endl;
                }

            }
            catch (const std::exception& e) {
                std::cerr << "Error loading point cloud file " << file_path << ": " << e.what() << std::endl;
            }

            return chunks;
        }

        std::shared_ptr<PointCloudChunk> loadPointCloudFile(const std::string& file_path) {
            auto chunks = loadPointCloudFileInChunks(file_path, UINT32_MAX);
            return chunks.empty() ? nullptr : chunks[0];
        }

        PointCloudFileInfo getPointCloudFileInfo(const std::string& file_path) {
            PointCloudFileInfo info(file_path);

            try {
                // ë¹ ë¥¸ ë©"íƒ€ë°ì´í„° ìˆ˜ì§' (ì‹¤ì œ ë¡œë"© ì—†ì´)
                std::string extension = std::filesystem::path(file_path).extension().string();
                (std::transform)(extension.begin(), extension.end(), extension.begin(), ::tolower);

                // íŒŒì¼ í¬ê¸° ê¸°ë°˜ ì¶"ì • (ì‹¤ì œë¡œëŠ" í—¤ë" íŒŒì‹±)
                auto file_size = std::filesystem::file_size(file_path);

                if (extension == ".las" || extension == ".laz") {
                    // LAS íŒŒì¼ì˜ ëŒ€ëžµì ì¸ í¬ì¸íŠ¸ ìˆ˜ ì¶"ì •
                    info.point_count = file_size / 28; // ëŒ€ëžµì ì¸ í¬ì¸íŠ¸ë‹¹ ë°"ì´íŠ¸ ìˆ˜
                }
                else if (extension == ".xyz" || extension == ".pts") {
                    // í…ìŠ¤íŠ¸ íŒŒì¼ì˜ ê²½ìš° ë¼ì¸ ìˆ˜ ê¸°ë°˜ ì¶"ì •
                    std::ifstream file(file_path);
                    info.point_count = (std::count)(std::istreambuf_iterator<char>(file),
                        std::istreambuf_iterator<char>(), '\n');
                }
                else {
                    info.point_count = file_size / 20; // ê¸°ë³¸ ì¶"ì •ê°'
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

        // íŒŒì¼ í˜•ì‹ë³„ ë¡œë"ë"¤ (ê°„ë‹¨í•œ êµ¬í˜„, ì‹¤ì œë¡œëŠ" í•´ë‹¹ ë¼ì´ë¸ŒëŸ¬ë¦¬ ì‚¬ìš©)
        std::shared_ptr<PointCloudChunk> loadLASFile(const std::string& file_path) {
            auto chunk = std::make_shared<PointCloudChunk>();
            chunk->chunk_id = 0;

            // TODO: libLAS ë˜ëŠ" PDAL ë¼ì´ë¸ŒëŸ¬ë¦¬ ì‚¬ìš©
            // í˜„ìž¬ëŠ" ë"ë¯¸ êµ¬í˜„
            std::cout << "Loading LAS file: " << file_path << " (dummy implementation)" << std::endl;

            // ë"ë¯¸ ë°ì´í„° ìƒì„± (ì‹¤ì œ êµ¬í˜„ì‹œ ì œê±°)
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

        std::shared_ptr<PointCloudChunk> loadPLYFile(const std::string& file_path) {
            auto chunk = std::make_shared<PointCloudChunk>();
            chunk->chunk_id = 0;

            // TODO: miniply ë˜ëŠ" tinyply ë¼ì´ë¸ŒëŸ¬ë¦¬ ì‚¬ìš©
            std::cout << "Loading PLY file: " << file_path << " (dummy implementation)" << std::endl;

            return chunk;
        }

        std::shared_ptr<PointCloudChunk> loadPCDFile(const std::string& file_path) {
            auto chunk = std::make_shared<PointCloudChunk>();
            chunk->chunk_id = 0;

            // TODO: PCL ë¼ì´ë¸ŒëŸ¬ë¦¬ ì‚¬ìš©
            std::cout << "Loading PCD file: " << file_path << " (dummy implementation)" << std::endl;

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

                    // ë¹ˆ ì¤„ì´ë‚˜ ì£¼ì„ ê±´ë„ˆë›°ê¸°
                    if (line.empty() || line[0] == '#') continue;

                    std::istringstream iss(line);
                    Point3D point;

                    if (iss >> point.x >> point.y >> point.z) {
                        // ì¶"ê°€ ì •ë³´ê°€ ìžˆë‹¤ë©´ ì²˜ë¦¬ (ê°•ë„, ìƒ‰ìƒ ë"±)
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

                    // ì§„í–‰ìƒí™© ì¶œë ¥ (í° íŒŒì¼ì˜ ê²½ìš°)
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

    } // namespace PointCloudLoader

    // BIMLoader Implementation
    namespace BIMLoader {

        BIMData loadBIMData(const std::string& file_path) {
            BIMData bim_data(file_path);

            try {
                std::cout << "Loading BIM file: " << file_path << std::endl;

                // ì‹¤ì œ êµ¬í˜„ì—ì„œëŠ" tinygltf, assimp ë"±ì˜ ë¼ì´ë¸ŒëŸ¬ë¦¬ ì‚¬ìš© í•„ìš"
                // TODO: GLTF/GLB íŒŒì¼ íŒŒì‹± êµ¬í˜„

                // ì˜ˆì œìš© ë"ë¯¸ ë°ì´í„° (ì‹¤ì œë¡œëŠ" íŒŒì¼ì—ì„œ ë¡œë"œ)
                if (std::filesystem::exists(file_path)) {
                    // ê°„ë‹¨í•œ í…ŒìŠ¤íŠ¸ìš© ì‚¬ê°í˜• ë' ê°œ (ë°•ìŠ¤ í˜•íƒœ)

                    // ë°"ë‹¥ë©´
                    MeshVertex v1(0.0f, 0.0f, 0.0f);
                    MeshVertex v2(10.0f, 0.0f, 0.0f);
                    MeshVertex v3(10.0f, 10.0f, 0.0f);
                    MeshVertex v4(0.0f, 10.0f, 0.0f);

                    bim_data.triangles.emplace_back(v1, v2, v3);
                    bim_data.triangles.emplace_back(v1, v3, v4);

                    // ìœ—ë©´
                    MeshVertex v5(0.0f, 0.0f, 3.0f);
                    MeshVertex v6(10.0f, 0.0f, 3.0f);
                    MeshVertex v7(10.0f, 10.0f, 3.0f);
                    MeshVertex v8(0.0f, 10.0f, 3.0f);

                    bim_data.triangles.emplace_back(v5, v7, v6);
                    bim_data.triangles.emplace_back(v5, v8, v7);

                    // ì¸¡ë©´ë"¤ë„ ì¶"ê°€ ê°€ëŠ¥...

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

                // íŒ¨í„´ì— ë§žëŠ" íŒŒì¼ë"¤ ì°¾ê¸°
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
            // ì ì—ì„œ ì‚¼ê°í˜•ê¹Œì§€ì˜ ìµœë‹¨ê±°ë¦¬ ê³„ì‚° (3D)
            // ì´ëŠ" ë³µìž¡í•œ ê¸°í•˜í•™ì  ê³„ì‚°ì´ë¯€ë¡œ ê°„ë‹¨í•œ êµ¬í˜„ë§Œ ì œì‹œ

            const MeshVertex& v0 = triangle.v0;
            const MeshVertex& v1 = triangle.v1;
            const MeshVertex& v2 = triangle.v2;

            // ì‚¼ê°í˜• í‰ë©´ì˜ ë²•ì„  ë²¡í„°
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

            // ì ì—ì„œ ì‚¼ê°í˜• í‰ë©´ê¹Œì§€ì˜ ìˆ˜ì§ ê±°ë¦¬
            MeshVertex to_point = point - v0;
            float plane_distance = std::abs(to_point.dot(normal));

            // ì ì„ ì‚¼ê°í˜• í‰ë©´ì— íˆ¬ì˜
            MeshVertex projected_point = point - normal * (to_point.dot(normal));

            // íˆ¬ì˜ëœ ì ì´ ì‚¼ê°í˜• ë‚´ë¶€ì— ìžˆëŠ"ì§€ í™•ì¸ (barycentric coordinates)
            // ê°„ë‹¨í™"ëœ êµ¬í˜„: ì‚¼ê°í˜•ì˜ ì¤'ì‹¬ì  ì‚¬ìš©
            closest_point = triangle.center();

            float dx = point.x - closest_point.x;
            float dy = point.y - closest_point.y;
            float dz = point.z - closest_point.z;

            return std::sqrt(dx * dx + dy * dy + dz * dz);

            // TODO: ì‹¤ì œ êµ¬í˜„ì—ì„œëŠ" ë‹¤ìŒê³¼ ê°™ì€ ì •í™•í•œ ê³„ì‚° í•„ìš":
            // 1. ì ì„ ì‚¼ê°í˜• í‰ë©´ì— íˆ¬ì˜
            // 2. íˆ¬ì˜ëœ ì ì´ ì‚¼ê°í˜• ë‚´ë¶€ì— ìžˆëŠ"ì§€ í™•ì¸ (barycentric coordinates)
            // 3. ë‚´ë¶€ì— ìžˆìœ¼ë©´ í‰ë©´ê¹Œì§€ì˜ ê±°ë¦¬, ì™¸ë¶€ì— ìžˆìœ¼ë©´ ê°€ìž¥ ê°€ê¹Œìš´ ëª¨ì„œë¦¬/ì •ì ê¹Œì§€ì˜ ê±°ë¦¬
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

            // ì§„í–‰ìƒí™© ì¶"ì ì„ ìœ„í•œ ë³€ìˆ˜ë"¤
            size_t processed_points = 0;
            size_t total_points = chunk.points.size();
            auto start_time = std::chrono::steady_clock::now();

            for (size_t i = 0; i < chunk.points.size(); ++i) {
                const auto& point = chunk.points[i];

                // í¬ì¸íŠ¸í´ë¼ìš°ë"œ ì ì„ ë©"ì‹œ ì •ì ìœ¼ë¡œ ë³€í™˜
                MeshVertex mesh_point(point.x, point.y, point.z);

                DistanceResult point_result;
                point_result.point_index = static_cast<uint32_t>(i);
                point_result.distance_to_mesh = (std::numeric_limits<float>::max)();
                point_result.source_point_file = source_file;

                // ëª¨ë"  BIM ë°ì´í„°ì— ëŒ€í•´ ìµœë‹¨ ê±°ë¦¬ ì°¾ê¸°
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

                // ì§„í–‰ìƒí™© ì¶œë ¥ (10% ê°„ê²©)
                if (processed_points % (total_points / 10 + 1) == 0) {
                    auto current_time = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
                    double progress = static_cast<double>(processed_points) / total_points * 100.0;

                    std::cout << "  Progress: " << progress << "% (" << processed_points
                        << "/" << total_points << ") - Elapsed: " << elapsed << "s" << std::endl;
                }
            }

            // í†µê³„ ì¶œë ¥
            if (!results.empty()) {
                auto min_max = (std::minmax_element)(results.begin(), results.end(),
                    [](const DistanceResult& a, const DistanceResult& b) {
                        return a.distance_to_mesh < b.distance_to_mesh;
                    });

                // í‰ê·  ê±°ë¦¬ ê³„ì‚°
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

        // SpatialIndex Implementation (ê°„ë‹¨í•œ ë²„ì „)
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

            // ê°„ë‹¨í•œ ì„ í˜• ê²€ìƒ‰ (ì‹¤ì œë¡œëŠ" octree, kd-tree ë"± ì‚¬ìš©)
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