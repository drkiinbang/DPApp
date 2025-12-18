#pragma once

#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <sstream>

#include "bim/BimMeshInfo.h"
#include "bim/MeshChunk.h"
#include "bimtree/xyzpoint.hpp"

#undef min
#undef max

namespace DPApp {

    /// Task type enumeration
    enum class TaskType : uint8_t {
        UNKNOWN = 0,
        CONVERT_PTS = 1,
        BIM_DISTANCE_CALCULATION = 2,
        BIM_PC2_DIST = 3
    };

    /// Convert TaskType to string
    inline const char* taskStr(TaskType type) {
        switch (type) {
        case TaskType::CONVERT_PTS: return "convert_pts";
        case TaskType::BIM_DISTANCE_CALCULATION: return "bim_distance_calculation";
        case TaskType::BIM_PC2_DIST: return "bim_pc2_distance";
        default: return "unknown";
        }
    }

    /// Convert string to TaskType
    inline TaskType strTask(const std::string& str) {
        if (str == "convert_pts") return TaskType::CONVERT_PTS;
        if (str == "bim_distance_calculation") return TaskType::BIM_DISTANCE_CALCULATION;
        if (str == "bim_pc2_distance") return TaskType::BIM_PC2_DIST;
        if (str == "bim_pc_process") return TaskType::BIM_PC2_DIST;
        return TaskType::UNKNOWN;
    }

    /// 3D point type definition
    using Point3D = pctree::XYZPoint;

    /// Alternative Point3D implementations (deprecated / experimental)
    /// struct Point3D {
    ///     double x, y, z;
    /// };

    /// Detailed Point3D class (legacy implementation)
    /// class Point3D { ... };

    /// Point cloud header information
    struct PointCloudHeader {
        std::string filename;
        uint64_t point_count;
        float min_x, min_y, min_z;
        float max_x, max_y, max_z;
        std::string coordinate_system;

        PointCloudHeader()
            : point_count(0),
            min_x(0), min_y(0), min_z(0),
            max_x(0), max_y(0), max_z(0) {
        }
    };

    /// Point cloud chunk structure
    struct PointCloudChunk {
        uint32_t chunk_id;
        std::vector<Point3D> points;
        PointCloudHeader header;

        /// Bounding box of the chunk
        float min_x, min_y, min_z;
        float max_x, max_y, max_z;

        PointCloudChunk()
            : chunk_id(0),
            min_x(0), min_y(0), min_z(0),
            max_x(0), max_y(0), max_z(0) {
        }

        /// Compute bounding box of the chunk
        void calculateBounds() {
            if (points.empty()) {
                min_x = min_y = min_z = 0;
                max_x = max_y = max_z = 0;
                return;
            }

            min_x = max_x = points[0][0];
            min_y = max_y = points[0][1];
            min_z = max_z = points[0][2];

            for (const auto& point : points) {
                min_x = std::min(min_x, point[0]);
                min_y = std::min(min_y, point[1]);
                min_z = std::min(min_z, point[2]);
                max_x = std::max(max_x, point[0]);
                max_y = std::max(max_y, point[1]);
                max_z = std::max(max_z, point[2]);
            }
        }

        /// Get estimated chunk size in bytes
        size_t getSize() const {
            return sizeof(PointCloudChunk) + points.size() * sizeof(Point3D);
        }
    };

    /// BIM and PointCloud combined chunk
    struct BimPcChunk {
        uint32_t chunk_id;
        chunkbim::MeshChunk bim;          /// BIM mesh chunk
        std::vector<Point3D> points;      /// Point cloud chunk

        /// Bounding box of point cloud
        float min_x, min_y, min_z;
        float max_x, max_y, max_z;

        BimPcChunk()
            : chunk_id(0),
            min_x(0), min_y(0), min_z(0),
            max_x(0), max_y(0), max_z(0) {
        }

        /// Estimate memory usage
        size_t estimateMemoryUsage() const {
            return sizeof(BimMeshInfo) + points.size() * sizeof(Point3D);
        }

        /// Compute bounding box for BIM and point cloud
        void calculateBounds() {
            bim.calculateBounds();

            if (points.empty()) {
                min_x = min_y = min_z = 0;
                max_x = max_y = max_z = 0;
                return;
            }

            min_x = max_x = points[0][0];
            min_y = max_y = points[0][1];
            min_z = max_z = points[0][2];

            for (const auto& point : points) {
                min_x = std::min(min_x, point[0]);
                min_y = std::min(min_y, point[1]);
                min_z = std::min(min_z, point[2]);
                max_x = std::max(max_x, point[0]);
                max_y = std::max(max_y, point[1]);
                max_z = std::max(max_z, point[2]);
            }
        }

        /// Get estimated chunk size in bytes
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

        ProcessingTask()
            : task_id(0),
            chunk_id(0),
            task_type(TaskType::UNKNOWN) {
        }
    };

    /// Generic processing result
    struct ProcessingResult {
        uint32_t task_id;
        uint32_t chunk_id;
        bool success;
        std::string error_message;
        std::vector<Point3D> processed_points;   /// Output points
        std::vector<uint8_t> result_data;         /// Additional binary data

        ProcessingResult()
            : task_id(0),
            chunk_id(0),
            success(false) {
        }
    };

    /// BIM–PointCloud processing result
    struct BimPcResult {
        uint32_t task_id;
        uint32_t chunk_id;
        bool success;
        std::string error_message;

        /// Processing statistics
        uint32_t total_points_processed;
        uint32_t total_faces_processed;
        double processing_time_ms;

        /// Distance statistics
        double min_distance;
        double max_distance;
        double avg_distance;
        double std_deviation;

        /// Per-point results
        std::vector<double> point_distances;
        std::vector<int32_t> nearest_face_ids;

        /// Classification statistics
        uint32_t points_within_threshold;
        uint32_t points_outside_threshold;

        /// Additional extensible data
        std::vector<uint8_t> extra_data;

        BimPcResult()
            : task_id(0),
            chunk_id(0),
            success(false),
            total_points_processed(0),
            total_faces_processed(0),
            processing_time_ms(0),
            min_distance(0),
            max_distance(0),
            avg_distance(0),
            std_deviation(0),
            points_within_threshold(0),
            points_outside_threshold(0) {
        }
    };

    /// Point cloud container class
    class PointCloud {
    public:
        PointCloud() { clear(); }
        ~PointCloud() {}

        /// Load point cloud from file (XYZ format)
        bool loadFromFile(const std::string& filename) {
            std::ifstream file(filename);
            if (!file.is_open()) {
                return false;
            }

            clear();
            header_.filename = filename;

            std::string line;
            while (std::getline(file, line)) {
                if (line.empty() || line[0] == '#') continue;

                std::istringstream iss(line);
                float x, y, z;
                float intensity = 0;
                uint8_t r = 255, g = 255, b = 255;

                if (iss >> x >> y >> z) {
                    iss >> intensity >> r >> g >> b;
                    points_.emplace_back(x, y, z);
                }
            }

            header_.point_count = points_.size();
            updateBounds();
            return true;
        }

        /// Save point cloud to file
        bool saveToFile(const std::string& filename) {
            std::ofstream file(filename);
            if (!file.is_open()) {
                return false;
            }

            file << "# Point Cloud Data\n";
            file << "# Points: " << points_.size() << "\n";
            file << "# Bounds: [" << header_.min_x << "," << header_.min_y << "," << header_.min_z
                << "] to [" << header_.max_x << "," << header_.max_y << "," << header_.max_z << "]\n";
            file << "# Format: X Y Z\n";

            for (const auto& point : points_) {
                file << point[0] << " " << point[1] << " " << point[2] << "\n";
            }

            return true;
        }

        /// Split point cloud into chunks
        std::vector<PointCloudChunk> splitIntoChunks(size_t max_points_per_chunk) {
            std::vector<PointCloudChunk> chunks;
            if (points_.empty()) return chunks;

            size_t total_points = points_.size();
            size_t num_chunks = (total_points + max_points_per_chunk - 1) / max_points_per_chunk;

            for (size_t i = 0; i < num_chunks; ++i) {
                PointCloudChunk chunk;
                chunk.chunk_id = static_cast<uint32_t>(i);
                chunk.header = header_;

                size_t start = i * max_points_per_chunk;
                size_t end = std::min(start + max_points_per_chunk, total_points);

                chunk.points.assign(points_.begin() + start, points_.begin() + end);
                chunk.calculateBounds();
                chunks.push_back(std::move(chunk));
            }

            return chunks;
        }

        /// Merge chunks into a single point cloud
        void mergeChunks(const std::vector<PointCloudChunk>& chunks) {
            clear();
            for (const auto& chunk : chunks) {
                addPoints(chunk.points);
            }
            updateBounds();
        }

        /// Add a single point
        void addPoint(const Point3D& point) {
            points_.push_back(point);
        }

        /// Add multiple points
        void addPoints(const std::vector<Point3D>& points) {
            points_.insert(points_.end(), points.begin(), points.end());
        }

        /// Clear all data
        void clear() {
            points_.clear();
            header_ = PointCloudHeader();
        }

        /// Accessors
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

            header_.min_x = header_.max_x = points_[0][0];
            header_.min_y = header_.max_y = points_[0][1];
            header_.min_z = header_.max_z = points_[0][2];

            for (const auto& point : points_) {
                header_.min_x = std::min(header_.min_x, point[0]);
                header_.min_y = std::min(header_.min_y, point[1]);
                header_.min_z = std::min(header_.min_z, point[2]);
                header_.max_x = std::max(header_.max_x, point[0]);
                header_.max_y = std::max(header_.max_y, point[1]);
                header_.max_z = std::max(header_.max_z, point[2]);
            }

            header_.point_count = points_.size();
        }

    private:
        std::vector<Point3D> points_;
        PointCloudHeader header_;
    };

    /// Shared pointer alias for PointCloud
    using PointCloudPtr = std::shared_ptr<PointCloud>;

} /// namespace DPApp
