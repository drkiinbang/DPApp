#pragma once

#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <sstream>

#undef min
#undef max

namespace DPApp {

// 3D 포인트 구조체
struct Point3D {
    double x, y, z;
    float intensity;        // 강도값 (LAS 파일용)
    uint8_t r, g, b;       // RGB 색상값
    uint16_t classification; // 분류 정보
    
    Point3D() : x(0), y(0), z(0), intensity(0), r(0), g(0), b(0), classification(0) {}
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_), intensity(0), r(0), g(0), b(0), classification(0) {}
    Point3D(double x_, double y_, double z_, float intensity_, uint8_t r_, uint8_t g_, uint8_t b_) 
        : x(x_), y(y_), z(z_), intensity(intensity_), r(r_), g(g_), b(b_), classification(0) {}
};

// 포인트클라우드 헤더 정보
struct PointCloudHeader {
    std::string filename;
    uint64_t point_count;
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    std::string coordinate_system;
    
    PointCloudHeader() : point_count(0), min_x(0), min_y(0), min_z(0), max_x(0), max_y(0), max_z(0) {}
};

// 포인트클라우드 청크 (작업 단위)
struct PointCloudChunk {
    uint32_t chunk_id;
    std::vector<Point3D> points;
    PointCloudHeader header;
    
    // 바운딩 박스
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    
    PointCloudChunk() : chunk_id(0), min_x(0), min_y(0), min_z(0), max_x(0), max_y(0), max_z(0) {}
    
    /// 바운딩 박스 계산
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
    
    // 청크 크기 반환 (바이트)
    size_t getSize() const {
        return sizeof(PointCloudChunk) + points.size() * sizeof(Point3D);
    }
};

// 처리 작업 정의
struct ProcessingTask {
    uint32_t task_id;
    uint32_t chunk_id;
    std::string task_type;      // "filter", "classification", "normal_estimation" 등
    std::vector<uint8_t> parameters; // 작업별 파라미터 (직렬화됨)
    
    ProcessingTask() : task_id(0), chunk_id(0) {}
};

// 처리 결과
struct ProcessingResult {
    uint32_t task_id;
    uint32_t chunk_id;
    bool success;
    std::string error_message;
    std::vector<Point3D> processed_points;  // 처리된 포인트들
    std::vector<uint8_t> result_data;       // 추가 결과 데이터
    
    ProcessingResult() : task_id(0), chunk_id(0), success(false) {}
};

// 전체 포인트클라우드 관리
class PointCloud {
public:
    PointCloud() { clear(); }
    ~PointCloud() {}
    
    // 파일 로드/저장
    bool loadFromFile(const std::string& filename) {
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

    bool saveToFile(const std::string& filename) {
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
    
    // 청크 분할
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
    
    // 청크 병합
    void mergeChunks(const std::vector<PointCloudChunk>& chunks) {
        clear();

        for (const auto& chunk : chunks) {
            addPoints(chunk.points);
        }

        updateBounds();
    }
    
    // 포인트 추가/삭제
    void addPoint(const Point3D& point) {
        points_.push_back(point);
    }

    void addPoints(const std::vector<Point3D>& points) {
        points_.insert(points_.end(), points.begin(), points.end());
    }
    
    void clear() {
        points_.clear();
        header_ = PointCloudHeader();
    }
    
    // 정보 조회
    size_t getPointCount() const { return points_.size(); }
    const PointCloudHeader& getHeader() const { return header_; }
    const std::vector<Point3D>& getPoints() const { return points_; }
    
    // 바운딩 박스 업데이트
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