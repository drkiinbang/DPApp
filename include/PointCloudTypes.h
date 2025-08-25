#pragma once

#include <vector>
#include <string>
#include <memory>

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
    
    // 바운딩 박스 계산
    void calculateBounds();
    
    // 청크 크기 반환 (바이트)
    size_t getSize() const;
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
    PointCloud();
    ~PointCloud();
    
    // 파일 로드/저장
    bool loadFromFile(const std::string& filename);
    bool saveToFile(const std::string& filename);
    
    // 청크 분할
    std::vector<PointCloudChunk> splitIntoChunks(size_t max_points_per_chunk);
    
    // 청크 병합
    void mergeChunks(const std::vector<PointCloudChunk>& chunks);
    
    // 포인트 추가/삭제
    void addPoint(const Point3D& point);
    void addPoints(const std::vector<Point3D>& points);
    void clear();
    
    // 정보 조회
    size_t getPointCount() const { return points_.size(); }
    const PointCloudHeader& getHeader() const { return header_; }
    const std::vector<Point3D>& getPoints() const { return points_; }
    
    // 바운딩 박스 업데이트
    void updateBounds();

private:
    std::vector<Point3D> points_;
    PointCloudHeader header_;
};

using PointCloudPtr = std::shared_ptr<PointCloud>;

} // namespace DPApp