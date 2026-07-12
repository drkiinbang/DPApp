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

    /// 태스크 타입 열거형
    enum class TaskType : uint8_t {
        UNKNOWN = 0,
        CONVERT_PTS = 1,
        BIM_DISTANCE_CALCULATION = 2,
        BIM_PC2_DIST = 3,

        // 테스트용 타입 (Master/Slave 시스템 검증용)
        TEST_ECHO = 10,      // 단순 데이터 왕복
        TEST_COMPUTE = 11,   // 계산 검증 (제곱 연산)
        TEST_DELAY = 12,     // 의도적 지연 (타임아웃 테스트)
        TEST_FAIL = 13,      // 의도적 실패 (재시도 테스트)

        // ICP 정합 타입
        ICP_COARSE_ALIGNMENT = 20,  // Coarse alignment (Master에서 수행)
        ICP_FINE_ALIGNMENT = 21,    // Fine alignment (Slave 분산 처리)
        ICP_APPLY_TRANSFORM = 22,   // Transform 적용 (결과 저장)
    };

    /// TaskType을 문자열로 변환
    inline const char* taskStr(TaskType type) {
        switch (type) {
        case TaskType::CONVERT_PTS: return "convert_pts";
        case TaskType::BIM_DISTANCE_CALCULATION: return "bim_distance_calculation";
        case TaskType::BIM_PC2_DIST: return "bim_pc2_distance";
        case TaskType::TEST_ECHO: return "test_echo";
        case TaskType::TEST_COMPUTE: return "test_compute";
        case TaskType::TEST_DELAY: return "test_delay";
        case TaskType::TEST_FAIL: return "test_fail";
        case TaskType::ICP_COARSE_ALIGNMENT: return "icp_coarse";
        case TaskType::ICP_FINE_ALIGNMENT: return "icp_fine";
        case TaskType::ICP_APPLY_TRANSFORM: return "icp_apply";
        default: return "unknown";
        }
    }

    /// 문자열을 TaskType으로 변환
    inline TaskType strTask(const std::string& str) {
        if (str == "convert_pts") return TaskType::CONVERT_PTS;
        if (str == "bim_distance_calculation") return TaskType::BIM_DISTANCE_CALCULATION;
        if (str == "bim_pc2_distance") return TaskType::BIM_PC2_DIST;
        if (str == "bim_pc_process") return TaskType::BIM_PC2_DIST;
        if (str == "test_echo") return TaskType::TEST_ECHO;
        if (str == "test_compute") return TaskType::TEST_COMPUTE;
        if (str == "test_delay") return TaskType::TEST_DELAY;
        if (str == "test_fail") return TaskType::TEST_FAIL;
        if (str == "icp_coarse") return TaskType::ICP_COARSE_ALIGNMENT;
        if (str == "icp_fine") return TaskType::ICP_FINE_ALIGNMENT;
        if (str == "icp_apply") return TaskType::ICP_APPLY_TRANSFORM;
        if (str == "icp") return TaskType::ICP_FINE_ALIGNMENT;  // 기본값
        return TaskType::UNKNOWN;
    }

    /// 3차원 점 타입 정의
    using Point3D = pctree::XYZPoint;

    /// Point3D의 다른 구현들 (사용 중단 / 실험적)
    /// struct Point3D {
    ///     double x, y, z;
    /// };

    /// Point3D 상세 클래스 (레거시 구현)
    /// class Point3D { ... };

    /// 포인트클라우드 헤더 정보
    struct PointCloudHeader {
        std::string filename;
        uint64_t point_count;
        double min_x, min_y, min_z;
        double max_x, max_y, max_z;
        std::string coordinate_system;

        PointCloudHeader()
            : point_count(0),
            min_x(0), min_y(0), min_z(0),
            max_x(0), max_y(0), max_z(0) {
        }
    };

    /// 포인트클라우드 청크 구조체
    struct PointCloudChunk {
        uint32_t chunk_id;
        std::vector<Point3D> points;
        PointCloudHeader header;

        /// 청크의 바운딩 박스
        double min_x, min_y, min_z;
        double max_x, max_y, max_z;

        PointCloudChunk()
            : chunk_id(0),
            min_x(0), min_y(0), min_z(0),
            max_x(0), max_y(0), max_z(0) {
        }

        /// 청크의 바운딩 박스 계산
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

        /// 청크의 대략적인 바이트 크기 계산
        size_t getSize() const {
            return sizeof(PointCloudChunk) + points.size() * sizeof(Point3D);
        }
    };

    /// BIM과 포인트클라우드를 결합한 청크. `points`는 절대좌표 대비
    /// (offsetX, offsetY, offsetZ)만큼 이동된 `float`로 저장된다 -- `bim`(메시)은
    /// double/절대좌표 그대로 유지된다. 이를 사용하는 쪽(processBimPc)은 함수 맨
    /// 앞에서 딱 한 번 `points`를 double로 승격시키고 offset을 다시 더한 뒤 `bim`과
    /// 비교한다.
    struct BimPcChunk {
        uint32_t chunk_id;
        chunkbim::MeshChunk bim;               /// BIM 메시 청크 (double, 절대좌표)
        std::vector<std::array<float, 3>> points; /// 포인트클라우드 청크, 이동됨, float

        /// `points`에 적용된 재배치(rebase) offset (double, 정확한 값)
        double offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;

        /// 포인트클라우드의 바운딩 박스
        double min_x, min_y, min_z;
        double max_x, max_y, max_z;

        BimPcChunk()
            : chunk_id(0),
            min_x(0), min_y(0), min_z(0),
            max_x(0), max_y(0), max_z(0) {
        }

        /// 메모리 사용량 추정
        size_t estimateMemoryUsage() const {
            return sizeof(BimMeshInfo) + points.size() * sizeof(std::array<float, 3>);
        }

        /// BIM과 포인트클라우드의 바운딩 박스 계산
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
                min_x = std::min(min_x, static_cast<double>(point[0]));
                min_y = std::min(min_y, static_cast<double>(point[1]));
                min_z = std::min(min_z, static_cast<double>(point[2]));
                max_x = std::max(max_x, static_cast<double>(point[0]));
                max_y = std::max(max_y, static_cast<double>(point[1]));
                max_z = std::max(max_z, static_cast<double>(point[2]));
            }
        }

        /// 청크의 대략적인 바이트 크기 계산
        size_t getSize() const {
            return sizeof(BimPcChunk) + points.size() * sizeof(std::array<float, 3>);
        }
    };

    /// 처리 태스크 정의
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

    /// 범용 처리 결과
    struct ProcessingResult {
        uint32_t task_id;
        uint32_t chunk_id;
        bool success;
        std::string error_message;
        std::vector<Point3D> processed_points;   /// 출력 점들
        std::vector<uint8_t> result_data;         /// 추가 바이너리 데이터

        ProcessingResult()
            : task_id(0),
            chunk_id(0),
            success(false) {
        }
    };

    /// BIM–포인트클라우드 처리 결과
    struct BimPcResult {
        uint32_t task_id;
        uint32_t chunk_id;
        bool success;
        std::string error_message;

        /// 처리 통계
        uint32_t total_points_processed;
        uint32_t total_faces_processed;
        double processing_time_ms;

        /// 거리 통계
        double min_distance;
        double max_distance;
        double avg_distance;
        double std_deviation;

        /// 점별 결과
        std::vector<double> point_distances;
        std::vector<int32_t> nearest_face_ids;

        /// 분류 통계
        uint32_t points_within_threshold;
        uint32_t points_outside_threshold;

        /// 추가 확장 데이터
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

    /// 포인트클라우드 컨테이너 클래스
    class PointCloud {
    public:
        PointCloud() { clear(); }
        ~PointCloud() {}

        /// 파일로부터 포인트클라우드 로딩 (XYZ 포맷)
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
                double x, y, z;
                double intensity = 0;
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

        /// 포인트클라우드를 파일로 저장
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

        /// 포인트클라우드를 청크 단위로 분할
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

        /// 청크들을 하나의 포인트클라우드로 병합
        void mergeChunks(const std::vector<PointCloudChunk>& chunks) {
            clear();
            for (const auto& chunk : chunks) {
                addPoints(chunk.points);
            }
            updateBounds();
        }

        /// 점 하나 추가
        void addPoint(const Point3D& point) {
            points_.push_back(point);
        }

        /// 여러 점 추가
        void addPoints(const std::vector<Point3D>& points) {
            points_.insert(points_.end(), points.begin(), points.end());
        }

        /// 모든 데이터 지우기
        void clear() {
            points_.clear();
            header_ = PointCloudHeader();
        }

        /// 접근자
        size_t getPointCount() const { return points_.size(); }
        const PointCloudHeader& getHeader() const { return header_; }
        const std::vector<Point3D>& getPoints() const { return points_; }

        /// 바운딩 박스 갱신
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

    /// PointCloud용 shared_ptr 별칭
    using PointCloudPtr = std::shared_ptr<PointCloud>;

} /// namespace DPApp
