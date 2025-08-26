#include "../include/PointCloudTypes.h"
#include <algorithm>
#include <limits>
#include <fstream>
#include <sstream>

namespace DPApp {

// PointCloudChunk 구현
void PointCloudChunk::calculateBounds() {
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

size_t PointCloudChunk::getSize() const {
    return sizeof(PointCloudChunk) + points.size() * sizeof(Point3D);
}

// PointCloud 구현
PointCloud::PointCloud() {
    clear();
}

PointCloud::~PointCloud() {
}

bool PointCloud::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    clear();
    header_.filename = filename;
    
    // 간단한 XYZ 파일 형식으로 로드
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::istringstream iss(line);
        double x, y, z;
        float intensity = 0;
        uint8_t r = 255, g = 255, b = 255;
        
        if (iss >> x >> y >> z) {
            // 추가 데이터가 있으면 읽기
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

bool PointCloud::saveToFile(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    // 헤더 정보 작성
    file << "# Point Cloud Data\n";
    file << "# Points: " << points_.size() << "\n";
    file << "# Bounds: [" << header_.min_x << "," << header_.min_y << "," << header_.min_z 
         << "] to [" << header_.max_x << "," << header_.max_y << "," << header_.max_z << "]\n";
    file << "# Format: X Y Z Intensity R G B\n";
    
    // 포인트 데이터 저장
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

std::vector<PointCloudChunk> PointCloud::splitIntoChunks(size_t max_points_per_chunk) {
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

void PointCloud::mergeChunks(const std::vector<PointCloudChunk>& chunks) {
    clear();
    
    for (const auto& chunk : chunks) {
        addPoints(chunk.points);
    }
    
    updateBounds();
}

void PointCloud::addPoint(const Point3D& point) {
    points_.push_back(point);
}

void PointCloud::addPoints(const std::vector<Point3D>& points) {
    points_.insert(points_.end(), points.begin(), points.end());
}

void PointCloud::clear() {
    points_.clear();
    header_ = PointCloudHeader();
}

void PointCloud::updateBounds() {
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

} // namespace DPApp