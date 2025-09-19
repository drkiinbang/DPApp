#pragma once

#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <sstream>

#undef min
#undef max

namespace DPApp {

enum class TaskType : uint8_t {
    UNKNOWN = 0,
    CONVERT_PTS,
    BIM_DISTANCE_CALCULATION
    };

/// TaskType to string
inline const char* taskStr(TaskType type) {
    switch (type) {
    case TaskType::CONVERT_PTS: return "convert_pts";
    case TaskType::BIM_DISTANCE_CALCULATION: return "bim_distance_calculation";
    default: return "unknown";
    }
}

/// String to TaskType
inline TaskType strTask(const std::string& str) {
    if (str == "convert_pts") return TaskType::CONVERT_PTS;
    if (str == "bim_distance_calculation") return TaskType::BIM_DISTANCE_CALCULATION;
    return TaskType::UNKNOWN;
}

/// 3D point struct
struct Point3D {
    double x, y, z;
    float intensity; /// intensity (for las)
    uint8_t r, g, b; /// RGB
    uint16_t classification; /// classification info
    
    Point3D() : x(0), y(0), z(0), intensity(0), r(0), g(0), b(0), classification(0) {}
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_), intensity(0), r(0), g(0), b(0), classification(0) {}
    Point3D(double x_, double y_, double z_, float intensity_, uint8_t r_, uint8_t g_, uint8_t b_) 
        : x(x_), y(y_), z(z_), intensity(intensity_), r(r_), g(g_), b(b_), classification(0) {}
};

/// Pointcloud heade info
struct PointCloudHeader {
    std::string filename;
    uint64_t point_count;
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    std::string coordinate_system;
    
    PointCloudHeader() : point_count(0), min_x(0), min_y(0), min_z(0), max_x(0), max_y(0), max_z(0) {}
};

/// Pointcloud chunk
struct PointCloudChunk {
    uint32_t chunk_id;
    std::vector<Point3D> points;
    PointCloudHeader header;
    
    /// Bounding box
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    
    PointCloudChunk() : chunk_id(0), min_x(0), min_y(0), min_z(0), max_x(0), max_y(0), max_z(0) {}
    
    /// Compute bounding box
    void calculateBounds() {
        if (points.empty()) {
            min_x = min_y = min_z = 0;
            max_x = max_y = max_z = 0;
            return;
        }

        min_x = max_x = points[0].x;
        min_y = max_y = points[0].y;
        min_z = max_z = points[0].z;

        for (const auto& point : points) {
            min_x = std::min(min_x, point.x);
            min_y = std::min(min_y, point.y);
            min_z = std::min(min_z, point.z);
            max_x = std::max(max_x, point.x);
            max_y = std::max(max_y, point.y);
            max_z = std::max(max_z, point.z);
        }
    }
    
    /// Get chunk size in byte
    size_t getSize() const {
        return sizeof(PointCloudChunk) + points.size() * sizeof(Point3D);
    }
};

/// Processing task definition
struct ProcessingTask {
    uint32_t task_id;
    uint32_t chunk_id;
    TaskType task_type;
    std::vector<uint8_t> parameters;
    
    ProcessingTask() : task_id(0), chunk_id(0), task_type(TaskType::UNKNOWN) {}
};

/// Processing result
struct ProcessingResult {
    uint32_t task_id;
    uint32_t chunk_id;
    bool success;
    std::string error_message;
    std::vector<Point3D> processed_points; /// Processed points
    std::vector<uint8_t> result_data; /// Additional result data
    
    ProcessingResult() : task_id(0), chunk_id(0), success(false) {}
};

/// Pointcloud class
class PointCloud {
public:
    PointCloud() { clear(); }
    ~PointCloud() {}
    
    /// Load a pointcloud file
    bool loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return false;
        }

        clear();
        header_.filename = filename;

        /// Load a simple xyz format file
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::istringstream iss(line);
            double x, y, z;
            float intensity = 0;
            uint8_t r = 255, g = 255, b = 255;

            if (iss >> x >> y >> z) {
                /// It additional info available
                iss >> intensity >> r >> g >> b;

                Point3D point(x, y, z, intensity, r, g, b);
                points_.push_back(point);
            }
        }

        header_.point_count = points_.size();
        updateBounds();

        file.close();
        return true;
    }

    bool saveToFile(const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }

        /// Create header info
        file << "# Point Cloud Data\n";
        file << "# Points: " << points_.size() << "\n";
        file << "# Bounds: [" << header_.min_x << "," << header_.min_y << "," << header_.min_z
            << "] to [" << header_.max_x << "," << header_.max_y << "," << header_.max_z << "]\n";
        file << "# Format: X Y Z Intensity R G B\n";

        /// Store point data
        for (const auto& point : points_) {
            file << point.x << " " << point.y << " " << point.z << " "
                << point.intensity << " "
                << static_cast<int>(point.r) << " "
                << static_cast<int>(point.g) << " "
                << static_cast<int>(point.b) << "\n";
        }

        file.close();
        return true;
    }
    
    /// Chunk split
    std::vector<PointCloudChunk> splitIntoChunks(size_t max_points_per_chunk) {
        std::vector<PointCloudChunk> chunks;

        if (points_.empty()) return chunks;

        size_t total_points = points_.size();
        size_t num_chunks = (total_points + max_points_per_chunk - 1) / max_points_per_chunk;

        for (size_t i = 0; i < num_chunks; ++i) {
            PointCloudChunk chunk;
            chunk.chunk_id = static_cast<uint32_t>(i);
            chunk.header = header_;

            size_t start_idx = i * max_points_per_chunk;
            size_t end_idx = std::min(start_idx + max_points_per_chunk, total_points);

            chunk.points.assign(points_.begin() + start_idx, points_.begin() + end_idx);
            chunk.calculateBounds();

            chunks.push_back(std::move(chunk));
        }

        return chunks;
    }
    
    /// Merge chunks
    void mergeChunks(const std::vector<PointCloudChunk>& chunks) {
        clear();

        for (const auto& chunk : chunks) {
            addPoints(chunk.points);
        }

        updateBounds();
    }
    
    /// Add a point
    void addPoint(const Point3D& point) {
        points_.push_back(point);
    }

    /// Add points
    void addPoints(const std::vector<Point3D>& points) {
        points_.insert(points_.end(), points.begin(), points.end());
    }
    
    /// Clear points
    void clear() {
        points_.clear();
        header_ = PointCloudHeader();
    }
    
    ///  Get pointcloud info
    size_t getPointCount() const { return points_.size(); }
    const PointCloudHeader& getHeader() const { return header_; }
    const std::vector<Point3D>& getPoints() const { return points_; }
    
    /// Update bounding box
    void updateBounds() {
        if (points_.empty()) {
            header_.min_x = header_.min_y = header_.min_z = 0;
            header_.max_x = header_.max_y = header_.max_z = 0;
            return;
        }

        header_.min_x = header_.max_x = points_[0].x;
        header_.min_y = header_.max_y = points_[0].y;
        header_.min_z = header_.max_z = points_[0].z;

        for (const auto& point : points_) {
            header_.min_x = std::min(header_.min_x, point.x);
            header_.min_y = std::min(header_.min_y, point.y);
            header_.min_z = std::min(header_.min_z, point.z);
            header_.max_x = std::max(header_.max_x, point.x);
            header_.max_y = std::max(header_.max_y, point.y);
            header_.max_z = std::max(header_.max_z, point.z);
        }

        header_.point_count = points_.size();
    }

private:
    std::vector<Point3D> points_;
    PointCloudHeader header_;
};

using PointCloudPtr = std::shared_ptr<PointCloud>;

} // namespace DPApp