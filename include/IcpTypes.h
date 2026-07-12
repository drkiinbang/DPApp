#pragma once

/// ============================================================================
/// IcpTypes.h - ICP(Iterative Closest Point, 반복 최근접점) 정합 관련 타입 정의
/// ============================================================================
///
/// 이 헤더는 포인트클라우드(LAS)와 메시(GLTF) 간의 분산 ICP 정합에 사용되는
/// 데이터 구조체들을 정의한다.
///
/// 위치: D:\repository\DPApp_II\include\IcpTypes.h
/// ============================================================================

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace icp {

    //=========================================================================
    // Transform4x4 - 4x4 변환 행렬 (Column-Major, 열 우선)
    //=========================================================================

    /// 강체 변환(rigid body transformation)을 표현하는 4x4 변환 행렬.
    /// 메모리 배치(column-major, OpenGL 방식):
    /// | m[0]  m[4]  m[8]   m[12] |   | R00 R01 R02 Tx |
    /// | m[1]  m[5]  m[9]   m[13] | = | R10 R11 R12 Ty |
    /// | m[2]  m[6]  m[10]  m[14] |   | R20 R21 R22 Tz |
    /// | m[3]  m[7]  m[11]  m[15] |   | 0   0   0   1  |
    struct Transform4x4 {
        double m[16];

        /// 단위 행렬(identity matrix) 생성
        static Transform4x4 identity() {
            Transform4x4 result;
            std::memset(result.m, 0, sizeof(result.m));
            result.m[0] = result.m[5] = result.m[10] = result.m[15] = 1.0;
            return result;
        }

        /// 이동(translation) 전용 행렬 생성
        static Transform4x4 translation(double tx, double ty, double tz) {
            Transform4x4 result = identity();
            result.m[12] = tx;
            result.m[13] = ty;
            result.m[14] = tz;
            return result;
        }

        /// 3x3 회전 행렬(row-major)과 이동 벡터로부터 4x4 변환 행렬 생성
        static Transform4x4 fromRotationTranslation(const double R[9], double tx, double ty, double tz) {
            Transform4x4 result;
            std::memset(result.m, 0, sizeof(result.m));

            /// 회전 성분 (row-major R을 column-major m으로 옮겨 담음)
            result.m[0] = R[0];  result.m[4] = R[1];  result.m[8] = R[2];
            result.m[1] = R[3];  result.m[5] = R[4];  result.m[9] = R[5];
            result.m[2] = R[6];  result.m[6] = R[7];  result.m[10] = R[8];

            /// 이동 성분
            result.m[12] = tx;
            result.m[13] = ty;
            result.m[14] = tz;

            /// 동차좌표(homogeneous coordinate) 성분
            result.m[15] = 1.0;

            return result;
        }

        /// 행렬 곱셈: this * other
        Transform4x4 operator*(const Transform4x4& other) const {
            Transform4x4 result;
            std::memset(result.m, 0, sizeof(result.m));

            for (int col = 0; col < 4; ++col) {
                for (int row = 0; row < 4; ++row) {
                    for (int k = 0; k < 4; ++k) {
                        result.m[col * 4 + row] += m[k * 4 + row] * other.m[col * 4 + k];
                    }
                }
            }
            return result;
        }

        /// 3차원 점 변환: result = R * p + t
        void transformPoint(double x, double y, double z, double& ox, double& oy, double& oz) const {
            ox = m[0] * x + m[4] * y + m[8] * z + m[12];
            oy = m[1] * x + m[5] * y + m[9] * z + m[13];
            oz = m[2] * x + m[6] * y + m[10] * z + m[14];
        }

        /// 3차원 점 변환 (제자리(in-place) 버전)
        void transformPointInPlace(double& x, double& y, double& z) const {
            double ox, oy, oz;
            transformPoint(x, y, z, ox, oy, oz);
            x = ox; y = oy; z = oz;
        }

        /// 회전 행렬(3x3, row-major) 추출
        void getRotation(double R[9]) const {
            R[0] = m[0];  R[1] = m[4];  R[2] = m[8];
            R[3] = m[1];  R[4] = m[5];  R[5] = m[9];
            R[6] = m[2];  R[7] = m[6];  R[8] = m[10];
        }

        /// 이동 벡터 추출
        void getTranslation(double& tx, double& ty, double& tz) const {
            tx = m[12];
            ty = m[13];
            tz = m[14];
        }

        /// 역변환 계산 (강체 변환이라는 가정 하에: R^T, -R^T * t 방식으로 계산.
        /// 일반적인 행렬 역행렬 계산(가우스 소거 등)보다 훨씬 빠르며, 회전 행렬이
        /// 직교(orthogonal)하다는 성질을 이용한다 -- 스케일/전단이 섞인 일반 4x4
        /// 행렬에는 사용할 수 없다.)
        Transform4x4 inverse() const {
            Transform4x4 result;

            /// 회전 성분 전치(transpose) -- 직교 행렬의 역행렬은 전치와 같다
            result.m[0] = m[0];   result.m[4] = m[1];   result.m[8] = m[2];
            result.m[1] = m[4];   result.m[5] = m[5];   result.m[9] = m[6];
            result.m[2] = m[8];   result.m[6] = m[9];   result.m[10] = m[10];

            /// 이동 성분: -R^T * t
            result.m[12] = -(result.m[0] * m[12] + result.m[4] * m[13] + result.m[8] * m[14]);
            result.m[13] = -(result.m[1] * m[12] + result.m[5] * m[13] + result.m[9] * m[14]);
            result.m[14] = -(result.m[2] * m[12] + result.m[6] * m[13] + result.m[10] * m[14]);

            /// 동차좌표 성분
            result.m[3] = result.m[7] = result.m[11] = 0.0;
            result.m[15] = 1.0;

            return result;
        }

        /// 두 변환 행렬 차이의 Frobenius norm 계산 (수렴 여부 판정에 사용)
        double distanceTo(const Transform4x4& other) const {
            double sum = 0.0;
            for (int i = 0; i < 16; ++i) {
                double diff = m[i] - other.m[i];
                sum += diff * diff;
            }
            return std::sqrt(sum);
        }
    };

    /// 절대좌표계 점에 대해 유효한 강체 변환(R*p_abs + t ~= q_abs)이 주어졌을 때,
    /// 재배치(rebase) offset만큼 이미 이동되어 있는 점(p_shifted = p_abs - offset)에
    /// 그대로 적용해도 같은 이동된 좌표계에서의 결과(q_shifted = q_abs - offset)를
    /// 얻을 수 있는 등가 변환을 유도한다:
    ///   t_shifted = t_absolute - (I - R) * offset = t_absolute - offset + R * offset
    /// 회전(R)은 공통 이동 offset에 영향받지 않으므로 그대로 유지된다. 이 함수는
    /// 이미 계산이 끝난(절대좌표 기준) ICP 변환을, Master가 들고 있는 대용량의
    /// offset-재배치된 float 포인트 배열 전체에 double로 승격시키지 않고 바로
    /// 적용하기 위해서만 사용한다 -- ICP 연산 자체는 항상 절대좌표(이동되지 않은)
    /// 청크 데이터를 대상으로 동작한다.
    inline Transform4x4 toShiftedFrame(const Transform4x4& absoluteTransform,
        double offsetX, double offsetY, double offsetZ) {
        Transform4x4 result = absoluteTransform;
        double R[9];
        absoluteTransform.getRotation(R);
        double rOffsetX = R[0] * offsetX + R[1] * offsetY + R[2] * offsetZ;
        double rOffsetY = R[3] * offsetX + R[4] * offsetY + R[5] * offsetZ;
        double rOffsetZ = R[6] * offsetX + R[7] * offsetY + R[8] * offsetZ;
        result.m[12] = absoluteTransform.m[12] - offsetX + rOffsetX;
        result.m[13] = absoluteTransform.m[13] - offsetY + rOffsetY;
        result.m[14] = absoluteTransform.m[14] - offsetZ + rOffsetZ;
        return result;
    }

    //=========================================================================
    // IcpConfig - ICP 알고리즘 설정값
    //=========================================================================

    /// ICP 알고리즘의 동작을 조절하는 설정 매개변수 모음
    struct IcpConfig {
        /// 최대 반복 횟수
        int maxIterations = 50;

        /// 수렴 판정 임계값 (변환 행렬 변화량 기준)
        double convergenceThreshold = 1e-6;

        /// 대응점(correspondence)으로 인정할 최대 거리 (미터)
        double maxCorrespondenceDistance = 1.0;

        /// 이상치(outlier) 제거 임계값 (MAD의 배수)
        double outlierRejectionThreshold = 2.5;

        /// coarse(전역) 정합용 다운샘플링 비율 (1 = 다운샘플링 없음)
        int downsampleRatio = 10;

        /// 대응점 탐색 시 normal(법선) 기반 필터링 사용 여부
        bool useNormalFiltering = true;

        /// 필터링에 사용할 normal 각도 임계값 (도 단위)
        double normalAngleThreshold = 45.0;

        /// 정합에 필요한 최소 대응점 개수
        int minCorrespondences = 100;

        /// 메시 가상점(pseudo-point) 생성 시 사용할 격자 간격 (미터)
        double pseudoPointGridSize = 0.1;

        /// 상세 로그 출력 여부
        bool verbose = false;

        /// 설정값 유효성 검사
        bool isValid() const {
            return maxIterations > 0 &&
                convergenceThreshold > 0.0 &&
                maxCorrespondenceDistance > 0.0 &&
                outlierRejectionThreshold > 0.0 &&
                downsampleRatio >= 1 &&
                normalAngleThreshold > 0.0 && normalAngleThreshold < 180.0 &&
                minCorrespondences > 0 &&
                pseudoPointGridSize > 0.0;
        }
    };

    //=========================================================================
    // IcpChunk - 분산 ICP 처리를 위한 데이터 청크
    //=========================================================================

    /// Slave에서의 로컬 ICP 처리를 위해 전송되는 청크 데이터. 점 배열은 절대좌표
    /// 대비 (offsetX, offsetY, offsetZ)만큼 이동된 `float`로 저장한다 -- 이렇게
    /// 하면 밀리미터 이하의 정밀도를 유지하면서도 전송/저장 크기를 절반으로
    /// 줄일 수 있다 (offset을 어떻게 결정하는지는 PseudoPointGenerator.hpp의
    /// icp::computeBimRebaseOffset() 참고). 이 청크를 실제로 사용하는 쪽(runIcp)은
    /// 함수 맨 앞에서 딱 한 번만 double로 승격시키고 offset을 다시 더한 뒤에
    /// 실제 연산을 시작한다.
    struct IcpChunk {
        /// 청크 고유 식별자
        uint32_t chunk_id = 0;

        /// source 점들(포인트클라우드 유래) - [x, y, z], 이동됨(shifted), float
        std::vector<std::array<float, 3>> sourcePoints;

        /// target 점들(메시 유래 가상점) - [x, y, z], 이동됨(shifted), float
        std::vector<std::array<float, 3>> targetPoints;

        /// 각 점이 속한 face를 가리키는 인덱스
        std::vector<size_t> faceIndices;

        /// face normal 벡터 - [nx, ny, nz]. 방향 벡터이므로 offset 이동의 대상이 아니다.
        std::vector<std::array<float, 3>> faceNormals;

        /// face별 대표점 (face 하나당 정점 1개) - [x, y, z], 이동됨(shifted), float
        std::vector<std::array<float, 3>> facePts;

        /// 초기 변환 (coarse 정합 결과로부터 전달됨)
        Transform4x4 initialTransform;

        /// 이 청크에 적용할 ICP 설정값
        IcpConfig config;

        /// sourcePoints/targetPoints/facePts에 적용된 재배치(rebase) offset (double, 정확한 값)
        /// -- double로 승격시킨 뒤 이 값을 다시 더하면 절대좌표로 복원된다.
        double offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;

        /// source 점들의 바운딩 박스
        double source_min_x = 0.0, source_min_y = 0.0, source_min_z = 0.0;
        double source_max_x = 0.0, source_max_y = 0.0, source_max_z = 0.0;

        /// target 점들의 바운딩 박스
        double target_min_x = 0.0, target_min_y = 0.0, target_min_z = 0.0;
        double target_max_x = 0.0, target_max_y = 0.0, target_max_z = 0.0;

        /// 기본 생성자
        IcpChunk() : initialTransform(Transform4x4::identity()) {}

        /// source 점들로부터 바운딩 박스 계산
        void calculateSourceBounds() {
            if (sourcePoints.empty()) return;

            source_min_x = source_max_x = sourcePoints[0][0];
            source_min_y = source_max_y = sourcePoints[0][1];
            source_min_z = source_max_z = sourcePoints[0][2];

            for (const auto& p : sourcePoints) {
                if (p[0] < source_min_x) source_min_x = p[0];
                if (p[1] < source_min_y) source_min_y = p[1];
                if (p[2] < source_min_z) source_min_z = p[2];
                if (p[0] > source_max_x) source_max_x = p[0];
                if (p[1] > source_max_y) source_max_y = p[1];
                if (p[2] > source_max_z) source_max_z = p[2];
            }
        }

        /// target 점들로부터 바운딩 박스 계산
        void calculateTargetBounds() {
            if (targetPoints.empty()) return;

            target_min_x = target_max_x = targetPoints[0][0];
            target_min_y = target_max_y = targetPoints[0][1];
            target_min_z = target_max_z = targetPoints[0][2];

            for (const auto& p : targetPoints) {
                if (p[0] < target_min_x) target_min_x = p[0];
                if (p[1] < target_min_y) target_min_y = p[1];
                if (p[2] < target_min_z) target_min_z = p[2];
                if (p[0] > target_max_x) target_max_x = p[0];
                if (p[1] > target_max_y) target_max_y = p[1];
                if (p[2] > target_max_z) target_max_z = p[2];
            }
        }

        /// 대략적인 메모리 크기(바이트) 계산
        size_t getApproximateSize() const {
            return sizeof(IcpChunk) +
                sourcePoints.size() * sizeof(std::array<float, 3>) +
                targetPoints.size() * sizeof(std::array<float, 3>) +
                faceIndices.size() * sizeof(size_t) +
                faceNormals.size() * sizeof(std::array<float, 3>);
        }

        /// 처리 가능한 유효 데이터를 갖고 있는지 확인
        bool isValid() const {
            return !sourcePoints.empty() &&
                !targetPoints.empty() &&
                config.isValid();
        }
    };

    //=========================================================================
    // IcpResult - ICP 처리 결과
    //=========================================================================

    /// 청크 하나에 대한 ICP 처리 결과
    struct IcpResult {
        /// 청크 식별자 (IcpChunk::chunk_id와 대응)
        uint32_t chunk_id = 0;

        /// 최종 변환 행렬 (initial * 로컬 미세조정을 합성한 값)
        Transform4x4 transform;

        /// 로컬 미세조정 변환만 (initial 성분 제외)
        Transform4x4 localTransform;

        /// 최종 평균제곱근오차(RMSE)
        double finalRMSE = 0.0;

        /// 초기 RMSE (ICP 반복 시작 전)
        double initialRMSE = 0.0;

        /// 실제로 수행된 반복 횟수
        int actualIterations = 0;

        /// 마지막 반복에서의 유효 대응점 개수
        int correspondenceCount = 0;

        /// 처리된 source 점 개수
        int sourcePointCount = 0;

        /// 사용된 target 점 개수
        int targetPointCount = 0;

        /// ICP가 임계값 이내로 수렴했는지 여부
        bool converged = false;

        /// 처리가 성공했는지 여부
        bool success = false;

        /// 오류 메시지 (실패 시)
        std::string errorMessage;

        /// 처리 소요 시간 (밀리초)
        double processingTimeMs = 0.0;

        /// 기본 생성자
        IcpResult() : transform(Transform4x4::identity()),
            localTransform(Transform4x4::identity()) {
        }

        /// RMSE 개선 비율 계산 (0.0 ~ 1.0)
        double getImprovementRatio() const {
            if (initialRMSE <= 0.0) return 0.0;
            return (initialRMSE - finalRMSE) / initialRMSE;
        }

        /// 결과 품질이 양호한지 확인
        bool isGood(double maxRMSE = 0.1) const {
            return success && converged && finalRMSE < maxRMSE;
        }
    };

    //=========================================================================
    // IcpJobStatus - 전체 ICP 작업(job)의 상태
    //=========================================================================

    /// ICP 작업 생명주기를 나타내는 상태 열거형
    enum class IcpJobStatus {
        PENDING,            ///< 작업이 생성되어 시작을 대기 중
        LOADING_DATA,       ///< LAS와 GLTF 데이터를 로딩 중
        GENERATING_PSEUDO,  ///< 메시로부터 가상점(pseudo-point) 생성 중
        COARSE_ALIGNMENT,   ///< Master에서 coarse(전역) 정합 실행 중
        DISTRIBUTING,       ///< Slave들에게 청크를 분배하는 중
        FINE_ALIGNMENT,     ///< Slave들에서 fine(정밀) 정합 실행 중
        AGGREGATING,        ///< Slave들의 결과를 통합하는 중
        APPLYING_TRANSFORM, ///< 최종 변환을 적용하는 중
        SAVING_RESULT,      ///< 정합된 포인트클라우드를 저장하는 중
        COMPLETED,          ///< 작업이 성공적으로 완료됨
        FAILED,             ///< 작업이 오류로 실패함
        CANCELLED           ///< 사용자에 의해 작업이 취소됨
    };

    /// IcpJobStatus를 문자열로 변환
    inline const char* statusToString(IcpJobStatus status) {
        switch (status) {
        case IcpJobStatus::PENDING:            return "PENDING";
        case IcpJobStatus::LOADING_DATA:       return "LOADING_DATA";
        case IcpJobStatus::GENERATING_PSEUDO:  return "GENERATING_PSEUDO";
        case IcpJobStatus::COARSE_ALIGNMENT:   return "COARSE_ALIGNMENT";
        case IcpJobStatus::DISTRIBUTING:       return "DISTRIBUTING";
        case IcpJobStatus::FINE_ALIGNMENT:     return "FINE_ALIGNMENT";
        case IcpJobStatus::AGGREGATING:        return "AGGREGATING";
        case IcpJobStatus::APPLYING_TRANSFORM: return "APPLYING_TRANSFORM";
        case IcpJobStatus::SAVING_RESULT:      return "SAVING_RESULT";
        case IcpJobStatus::COMPLETED:          return "COMPLETED";
        case IcpJobStatus::FAILED:             return "FAILED";
        case IcpJobStatus::CANCELLED:          return "CANCELLED";
        default:                               return "UNKNOWN";
        }
    }

    //=========================================================================
    // IcpElementAlignment - BIM 부재(element) 단위 정합 결과
    //=========================================================================

    /// BIM 부재 하나에 대해 개별적으로 추정된 정합 결과. 작업 전체의 통합 결과와
    /// 함께 보관되어, 호출자가 부재별 품질을 따로 확인할 수 있게 한다(예: 특정
    /// 부재의 fine 정합이 잘 안 됐을 때, 통합 RMSE 하나만 보고 추측할 필요 없이
    /// 바로 원인 부재를 짚어낼 수 있다). 여기서의 `transform`은 그 청크의 *로컬*
    /// 미세조정 결과만을 담고 있으며(IcpResult::transform), aggregateWeightedTransforms가
    /// 모든 청크를 통합하는 것과 동일한 방식으로 작업의 coarse 변환과 합성된
    /// 값이다 -- 통합 수식의 상세한 유도는 CHANGELOG/doc 문서 참고.
    struct IcpElementAlignment {
        uint32_t chunk_id = 0;
        std::string elementName;
        int elementRevitId = 0;
        int sourcePointCount = 0;
        int targetPointCount = 0;
        double rmse = 0.0;
        bool converged = false;
        bool success = false;
        std::string errorMessage;
        Transform4x4 transform;

        IcpElementAlignment() : transform(Transform4x4::identity()) {}
    };

    //=========================================================================
    // IcpJob - Master 측 작업(job) 관리 구조체
    //=========================================================================

    /// ICP 작업의 전체 정보 (Master가 관리)
    struct IcpJob {
        /// 작업 고유 식별자
        std::string jobId;

        /// 입력 파일 경로
        std::string lasFilePath;
        std::string bimFolderPath;

        /// LAS offset (사용자가 직접 지정하는 수동 사전 이동값, 로딩 직후 적용됨)
        double offsetX = 0.0;
        double offsetY = 0.0;
        double offsetZ = 0.0;

        /// 재배치(rebase) offset (BIM 중심점으로부터 자동으로 계산됨, 위의 수동
        /// offsetX/Y/Z와는 별개). 포인트클라우드 좌표는 원점 부근에서의 밀리미터
        /// 이하 정밀도를 지키기 위해 이 값만큼 이동시킨(float) 상태로 저장/전송
        /// 되며, 최종 변환은 작업이 완료될 때 딱 한 번 원래 좌표계로 보정된다
        /// (processIcpJob에서의 aggregateWeightedTransforms 사용 부분 참고).
        double rebaseOffsetX = 0.0;
        double rebaseOffsetY = 0.0;
        double rebaseOffsetZ = 0.0;

        /// 정합된 포인트클라우드를 저장할 출력 파일 경로
        std::string outputPath;

        /// 작업 설정값
        IcpConfig config;

        /// 현재 상태
        IcpJobStatus status = IcpJobStatus::PENDING;

        /// 오류 메시지 (status == FAILED일 때)
        std::string errorMessage;

        /// coarse 정합 결과 (1단계)
        Transform4x4 coarseTransform;
        double coarseRMSE = 0.0;
        bool coarseConverged = false;

        /// 청크별 결과 (2단계)
        std::vector<IcpResult> chunkResults;
        int totalChunks = 0;
        int completedChunks = 0;
        int failedChunks = 0;

        /// 최종 통합 결과 (3단계)
        Transform4x4 finalTransform;
        double finalRMSE = 0.0;

        /// BIM 부재(부재)별 정합 결과, 성공적으로 처리된 청크마다 하나씩 (fine
        /// 정합이 부재별 청크가 아니라 Master 단일 패스로 실행된 경우에는 비어
        /// 있음). 통합 결과와 함께 디스크에 저장된다 -- MasterApplication_ICP.cpp의
        /// alignmentOutputPath/saveAlignmentResults() 참고.
        std::vector<IcpElementAlignment> elementResults;

        /// 저장된 부재별+통합 정합 결과 JSON 파일의 경로 (파일이 실제로 쓰여진 뒤 설정됨)
        std::string alignmentOutputPath;

        /// 통계
        size_t totalPointCount = 0;
        size_t totalMeshTriangles = 0;
        size_t totalPseudoPoints = 0;

        /// 시간 정보 (epoch 이후 경과 밀리초)
        double startTimeMs = 0.0;
        double endTimeMs = 0.0;
        double coarseAlignmentTimeMs = 0.0;
        double fineAlignmentTimeMs = 0.0;

        /// 취소 요청 플래그 (스레드 안전을 위해 atomic 사용)
        std::atomic<bool> cancelRequested{ false };

        /// 기본 생성자
        IcpJob() : coarseTransform(Transform4x4::identity()),
            finalTransform(Transform4x4::identity()) {
        }

        /// 복사 생성자 (atomic 멤버를 직접 다뤄야 하므로 컴파일러가 자동 생성하지 못함)
        IcpJob(const IcpJob& other)
            : jobId(other.jobId)
            , lasFilePath(other.lasFilePath)
            , bimFolderPath(other.bimFolderPath)
            , offsetX(other.offsetX)
            , offsetY(other.offsetY)
            , offsetZ(other.offsetZ)
            , rebaseOffsetX(other.rebaseOffsetX)
            , rebaseOffsetY(other.rebaseOffsetY)
            , rebaseOffsetZ(other.rebaseOffsetZ)
            , outputPath(other.outputPath)
            , config(other.config)
            , status(other.status)
            , errorMessage(other.errorMessage)
            , coarseTransform(other.coarseTransform)
            , coarseRMSE(other.coarseRMSE)
            , coarseConverged(other.coarseConverged)
            , chunkResults(other.chunkResults)
            , totalChunks(other.totalChunks)
            , completedChunks(other.completedChunks)
            , failedChunks(other.failedChunks)
            , finalTransform(other.finalTransform)
            , finalRMSE(other.finalRMSE)
            , elementResults(other.elementResults)
            , alignmentOutputPath(other.alignmentOutputPath)
            , totalPointCount(other.totalPointCount)
            , totalMeshTriangles(other.totalMeshTriangles)
            , totalPseudoPoints(other.totalPseudoPoints)
            , startTimeMs(other.startTimeMs)
            , endTimeMs(other.endTimeMs)
            , coarseAlignmentTimeMs(other.coarseAlignmentTimeMs)
            , fineAlignmentTimeMs(other.fineAlignmentTimeMs)
            , cancelRequested(other.cancelRequested.load())
        {
        }

        /// 진행률 계산 (0~100)
        double getProgress() const {
            switch (status) {
            case IcpJobStatus::PENDING:           return 0.0;
            case IcpJobStatus::LOADING_DATA:      return 5.0;
            case IcpJobStatus::GENERATING_PSEUDO: return 10.0;
            case IcpJobStatus::COARSE_ALIGNMENT:  return 20.0;
            case IcpJobStatus::DISTRIBUTING:      return 25.0;
            case IcpJobStatus::FINE_ALIGNMENT: {
                if (totalChunks == 0) return 25.0;
                double chunkProgress = static_cast<double>(completedChunks) / totalChunks;
                return 25.0 + chunkProgress * 60.0; // 25% ~ 85% 구간을 청크 진행률에 비례 배분
            }
            case IcpJobStatus::AGGREGATING:        return 90.0;
            case IcpJobStatus::APPLYING_TRANSFORM: return 95.0;
            case IcpJobStatus::SAVING_RESULT:      return 98.0;
            case IcpJobStatus::COMPLETED:          return 100.0;
            case IcpJobStatus::FAILED:             return 0.0;
            case IcpJobStatus::CANCELLED:          return 0.0;
            default:                               return 0.0;
            }
        }

        /// 경과 시간(초) 계산 (작업이 아직 진행 중이면 현재 시각 기준으로 계산)
        double getElapsedTimeSec() const {
            if (startTimeMs <= 0.0) return 0.0;
            double end = (endTimeMs > 0.0) ? endTimeMs
                : static_cast<double>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count());
            return (end - startTimeMs) / 1000.0;
        }

        /// 작업이 종료 상태(성공/실패/취소)인지 확인
        bool isFinished() const {
            return status == IcpJobStatus::COMPLETED ||
                status == IcpJobStatus::FAILED ||
                status == IcpJobStatus::CANCELLED;
        }

        /// 작업이 현재 진행 중인지 확인
        bool isRunning() const {
            return !isFinished() && status != IcpJobStatus::PENDING;
        }
    };

    //=========================================================================
    // IcpStatistics - 보고용으로 집계한 통계 정보
    //=========================================================================

    /// ICP 작업 결과에 대한 통계치
    struct IcpStatistics {
        /// 포인트클라우드 통계
        size_t totalSourcePoints = 0;
        size_t processedSourcePoints = 0;

        /// 메시 통계
        size_t totalMeshTriangles = 0;
        size_t totalPseudoPoints = 0;

        /// 정합 통계
        double initialRMSE = 0.0;
        double coarseRMSE = 0.0;
        double finalRMSE = 0.0;
        double rmseImprovement = 0.0; ///< 백분율(%)

        /// 반복 횟수 통계
        int coarseIterations = 0;
        int totalFineIterations = 0;
        double averageFineIterations = 0.0;

        /// 대응점 통계
        int totalCorrespondences = 0;
        double averageCorrespondenceDistance = 0.0;

        /// 청크 통계
        int totalChunks = 0;
        int successfulChunks = 0;
        int failedChunks = 0;
        int convergedChunks = 0;

        /// 처리 시간 (초)
        double totalProcessingTimeSec = 0.0;
        double coarseAlignmentTimeSec = 0.0;
        double fineAlignmentTimeSec = 0.0;
        double dataLoadingTimeSec = 0.0;

        /// IcpJob으로부터 통계치 계산
        static IcpStatistics fromJob(const IcpJob& job) {
            IcpStatistics stats;

            stats.totalSourcePoints = job.totalPointCount;
            stats.totalMeshTriangles = job.totalMeshTriangles;
            stats.totalPseudoPoints = job.totalPseudoPoints;
            stats.coarseRMSE = job.coarseRMSE;
            stats.finalRMSE = job.finalRMSE;
            stats.totalChunks = job.totalChunks;

            int successCount = 0;
            int convergedCount = 0;
            int totalIter = 0;
            int totalCorr = 0;

            for (const auto& result : job.chunkResults) {
                if (result.success) {
                    successCount++;
                    totalIter += result.actualIterations;
                    totalCorr += result.correspondenceCount;
                    stats.processedSourcePoints += result.sourcePointCount;
                }
                if (result.converged) {
                    convergedCount++;
                }
            }

            stats.successfulChunks = successCount;
            stats.failedChunks = job.totalChunks - successCount;
            stats.convergedChunks = convergedCount;
            stats.totalFineIterations = totalIter;
            stats.totalCorrespondences = totalCorr;

            if (successCount > 0) {
                stats.averageFineIterations = static_cast<double>(totalIter) / successCount;
            }

            if (stats.coarseRMSE > 0.0) {
                stats.rmseImprovement = (stats.coarseRMSE - stats.finalRMSE) / stats.coarseRMSE * 100.0;
            }

            stats.totalProcessingTimeSec = job.getElapsedTimeSec();
            stats.coarseAlignmentTimeSec = job.coarseAlignmentTimeMs / 1000.0;
            stats.fineAlignmentTimeSec = job.fineAlignmentTimeMs / 1000.0;

            return stats;
        }
    };

} // namespace icp
