#pragma once

#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <sstream>

#include "bim/BimMeshInfo.h"
#include "bimtree/xyzpoint.hpp"

#undef min
#undef max

namespace DPApp {

enum class TaskType : uint8_t { UNKNOWN = 0
    , CONVERT_PTS = 1
    , BIM_DISTANCE_CALCULATION = 2
    , BIM_PC2_DIST = 3
    , TEST_INTEGER_SUM
    };

/// TaskType to string
inline const char* taskStr(TaskType type) {
    switch (type) {
    case TaskType::CONVERT_PTS: return "convert_pts";
    case TaskType::BIM_DISTANCE_CALCULATION: return "bim_distance_calculation";
    case TaskType::BIM_PC2_DIST: return "bim_pc2_distance";
    case TaskType::TEST_INTEGER_SUM: return "TEST_INTEGER_SUM";
    default: return "unknown";
    }
}

/// String to TaskType
inline TaskType strTask(const std::string& str) {
    if (str == "convert_pts") return TaskType::CONVERT_PTS;
    if (str == "bim_distance_calculation") return TaskType::BIM_DISTANCE_CALCULATION;
    if (str == "bim_pc2_distance") return TaskType::BIM_PC2_DIST;
    return TaskType::UNKNOWN;
}

/// 3D point struct
using Point3D = pctree::XYZPoint;
/*
struct Point3D {
    double x, y, z;
    
    Point3D() : x(0), y(0), z(0) {}
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};
*/

/*
class Point3D
{
public:

    Point3D()
    {
        xyz[0] = xyz[1] = xyz[2] = 0.0;
    }

    Point3D(const double x, const double y, const double z)
    {
        xyz[0] = x;
        xyz[1] = y;
        xyz[2] = z;
    }

    Point3D(const Point3D& val)
    {
        copy(val);
        faceId = val.faceId;
    }

    inline Point3D operator=(const Point3D& val)
    {
        copy(val);
        return *this;
    }

    inline Point3D operator+=(const Point3D& val)
    {
        this->xyz[0] += val.xyz[0];
        this->xyz[1] += val.xyz[1];
        this->xyz[2] += val.xyz[2];
        return *this;
    }

    inline Point3D operator/=(const double& val)
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

    inline Point3D operator-(const Point3D& val) const
    {
        Point3D retVal;
        retVal.xyz[0] = this->xyz[0] - val.xyz[0];
        retVal.xyz[1] = this->xyz[1] - val.xyz[1];
        retVal.xyz[2] = this->xyz[2] - val.xyz[2];
        return retVal;
    }

    inline Point3D operator+(const Point3D& val) const
    {
        Point3D retVal;
        retVal.xyz[0] = this->xyz[0] + val.xyz[0];
        retVal.xyz[1] = this->xyz[1] + val.xyz[1];
        retVal.xyz[2] = this->xyz[2] + val.xyz[2];
        return retVal;
    }

    inline Point3D operator/(const double val) const
    {
        Point3D retVal;

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

    inline Point3D operator*(const double val) const
    {
        Point3D retVal;

        retVal.xyz[0] = this->xyz[0] * val;
        retVal.xyz[1] = this->xyz[1] * val;
        retVal.xyz[2] = this->xyz[2] * val;

        return retVal;
    }

    inline void copy(const Point3D& val)
    {
        ::memcpy(xyz, val.xyz, 3 * sizeof(double));
        faceId = val.faceId;
    }

    inline bool operator==(const Point3D& pt) const
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
        try
        {
            return xyz[idx];
        }
        catch (...)
        {
            throw std::runtime_error("Error in Vertex3D::operator[]");
        }
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
*/

/// Pointcloud heade info
struct PointCloudHeader {
    std::string filename;
    uint64_t point_count;
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    std::string coordinate_system;
    
    PointCloudHeader() : point_count(0), min_x(0), min_y(0), min_z(0), max_x(0), max_y(0), max_z(0) {}
};

/// Pointcloud chunk
struct PointCloudChunk {
    uint32_t chunk_id;
    std::vector<Point3D> points;
    PointCloudHeader header;
    
    /// Bounding box
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    
    PointCloudChunk() : chunk_id(0), min_x(0), min_y(0), min_z(0), max_x(0), max_y(0), max_z(0) {}
    
    /// Compute bounding box
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
            min_x = (std::min)(min_x, point[0]);
            min_y = (std::min)(min_y, point[1]);
            min_z = (std::min)(min_z, point[2]);
            max_x = (std::max)(max_x, point[0]);
            max_y = (std::max)(max_y, point[1]);
            max_z = (std::max)(max_z, point[2]);
        }
    }
    
    /// Get chunk size in byte
    size_t getSize() const {
        return sizeof(PointCloudChunk) + points.size() * sizeof(Point3D);
    }
};

/// BimPc chunk
struct BimPcChunk {
    uint32_t chunk_id;
    BimMeshInfo bim;
    std::vector<Point3D> points;

    /// Bounding box (pc)
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;

    /// Bounding box (mesh)
    float msh_min_x, msh_min_y, msh_min_z;
    float msh_max_x, msh_max_y, msh_max_z;

    BimPcChunk() : chunk_id(0), min_x(0), min_y(0), min_z(0), max_x(0), max_y(0), max_z(0) {}

    /// Estimate memory usage
    size_t estimateMemoryUsage() const {
        return sizeof(BimMeshInfo) + points.size() * sizeof(Point3D);
    }

    void calculateBounds() {
        calculatePcBounds();
        calculateMeshBounds();
    }

    /// Get chunk size in byte
    size_t getSize() const {
        return sizeof(PointCloudChunk) + points.size() * sizeof(Point3D);
    }

private:
    /// Compute bounding box (pc)
    void calculatePcBounds() {
        if (points.empty()) {
            min_x = min_y = min_z = 0;
            max_x = max_y = max_z = 0;
            return;
        }

        min_x = max_x = points[0][0];
        min_y = max_y = points[0][1];
        min_z = max_z = points[0][2];

        for (const auto& point : points) {
            min_x = (std::min)(min_x, point[0]);
            min_y = (std::min)(min_y, point[1]);
            min_z = (std::min)(min_z, point[2]);
            max_x = (std::max)(max_x, point[0]);
            max_y = (std::max)(max_y, point[1]);
            max_z = (std::max)(max_z, point[2]);
        }
    }

    /// Compute bounding box (mesh)
    void calculateMeshBounds()
    {
        bool is_offset_applied = false;
        float offset[3] = { 0.f, 0.f, 0.f };

        auto bbox = bim.get_mbr(is_offset_applied, offset);
        auto bbox_min = bbox.get_min();
        auto bbox_max = bbox.get_max();

        msh_min_x = bbox_min[0];
        msh_min_y = bbox_min[1];
        msh_min_z = bbox_min[2];

        msh_max_x = bbox_max[0];
        msh_max_y = bbox_max[1];
        msh_max_z = bbox_max[2];
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

                Point3D point(x, y, z);
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
            file << point[0] << " " << point[1] << " " << point[2] << "\n";
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

        header_.min_x = header_.max_x = points_[0][0];
        header_.min_y = header_.max_y = points_[0][1];
        header_.min_z = header_.max_z = points_[0][2];

        for (const auto& point : points_) {
            header_.min_x = (std::min)(header_.min_x, point[0]);
            header_.min_y = (std::min)(header_.min_y, point[1]);
            header_.min_z = (std::min)(header_.min_z, point[2]);
            header_.max_x = (std::max)(header_.max_x, point[0]);
            header_.max_y = (std::max)(header_.max_y, point[1]);
            header_.max_z = (std::max)(header_.max_z, point[2]);
        }

        header_.point_count = points_.size();
    }

private:
    std::vector<Point3D> points_;
    PointCloudHeader header_;
};

using PointCloudPtr = std::shared_ptr<PointCloud>;

} // namespace DPApp