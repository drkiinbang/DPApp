#pragma once

/// ============================================================================
/// IcpCore.hpp - ICP 핵심 알고리즘 구현
/// ============================================================================
///
/// 이 헤더는 ICP 알고리즘의 핵심 구성요소들을 제공한다:
/// - KD-Tree(nanoflann)를 이용한 대응점(correspondence) 탐색
/// - 이상치(outlier) 제거 (MAD 기반)
/// - SVD(Eigen)를 이용한 강체 변환(rigid transformation) 추정
/// - RMSE 계산
/// - 포인트클라우드 변환 및 다운샘플링
/// - ICP 메인 반복 루프
///
/// 위치: D:\repository\DPApp_II\include\bimtree\IcpCore.hpp
/// ============================================================================

#include "../IcpTypes.h"
#include "nanoflann.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <vector>

#include "../Logger.h"

#include "SSMMatrixd.h"
#include "rotation.hpp"

const double Rad2Sec = ((180. * 3600.) / acos(-1.0));
const double Rad2Deg = ((180.) / acos(-1.0));
const double minParamSd = 1.0e-9;
const double maxParamSd = 1.0e+9;
const unsigned int uknw = 6;

namespace icp {

    //=========================================================================
    // nanoflann KD-Tree용 포인트클라우드 어댑터
    //=========================================================================

    /// std::vector<std::array<double,3>>를 nanoflann과 함께 사용하기 위한 어댑터 클래스
    struct PointCloudAdaptor {
        const std::vector<std::array<double, 3>>& points;

        explicit PointCloudAdaptor(const std::vector<std::array<double, 3>>& pts)
            : points(pts) {
        }

        /// 점 개수
        inline size_t kdtree_get_point_count() const {
            return points.size();
        }

        /// 점 p1과 인덱스 idx_p2의 점 사이의 거리 제곱
        inline double kdtree_distance(const double* p1, const size_t idx_p2, size_t /*size*/) const {
            const double d0 = p1[0] - points[idx_p2][0];
            const double d1 = p1[1] - points[idx_p2][1];
            const double d2 = p1[2] - points[idx_p2][2];
            return d0 * d0 + d1 * d1 + d2 * d2;
        }

        /// 인덱스 idx인 점의 dim번째 성분값 반환
        inline double kdtree_get_pt(const size_t idx, int dim) const {
            return points[idx][dim];
        }

        /// 선택적 바운딩 박스 계산 (false를 반환하면 nanoflann 기본 계산 방식을 사용)
        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /*bb*/) const {
            return false;
        }
    };

    /// nanoflann을 이용한 KD-Tree 타입 정의
    using KdTree3D = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, PointCloudAdaptor>,
        PointCloudAdaptor,
        3,      // 차원 수
        size_t  // 인덱스 타입
    >;

    //=========================================================================
    // 대응점(Correspondence) 구조체
    //=========================================================================

    /// 대응점 쌍 (source 인덱스, target 인덱스, 거리 제곱)
    struct Correspondence {
        size_t sourceIdx;
        size_t targetIdx;
        double squaredDistance;
    };

    /// Point-to-Plane용 대응점 구조체. target 인덱스 대신, 이미 face 위에 투영된
    /// 점(facePt)을 직접 들고 있다 -- target 점 자체가 아니라 그 점이 속한 face
    /// 평면 위로 투영된 좌표가 필요하기 때문이다.
    struct CorrespondenceFace {
        size_t sourceIdx;
        std::array<double, 3> facePt;
        double squaredDistance;
    };

    //=========================================================================
    // 대응점 탐색
    //=========================================================================

    /// KD-Tree를 이용해 최근접 이웃 대응점을 탐색한다 (Point-to-Point 방식)
    /// @param sourcePoints source 포인트클라우드 (변환 대상)
    /// @param targetTree target 점들로 만든 KD-Tree
    /// @param maxDistanceSquared 유효한 대응점으로 인정할 최대 거리의 제곱
    /// @param correspondences 결과로 채워질 대응점 벡터
    inline void findCorrespondences(
        const std::vector<std::array<double, 3>>& sourcePoints,
        const KdTree3D& targetTree,
        double maxDistanceSquared,
        std::vector<Correspondence>& correspondences)
    {
        correspondences.clear();
        correspondences.reserve(sourcePoints.size());

        for (size_t i = 0; i < sourcePoints.size(); ++i) {
            size_t resultIndex = 0;
            double resultDistSq = 0.0;

            /// 최근접 이웃 1개만 탐색
            nanoflann::KNNResultSet<double> resultSet(1);
            resultSet.init(&resultIndex, &resultDistSq);

            targetTree.findNeighbors(resultSet, sourcePoints[i].data(),
                nanoflann::SearchParams());

            /// 거리 임계값 확인
            if (resultDistSq <= maxDistanceSquared) {
                Correspondence corr;
                corr.sourceIdx = i;
                corr.targetIdx = resultIndex;
                corr.squaredDistance = resultDistSq;
                correspondences.push_back(corr);
            }
        }
    }

    /// 점 P에서 평면(기준점 Q, 법선 N)까지의 부호 있는 거리와, 그 평면 위로의
    /// 투영점을 계산한다. N이 단위 벡터가 아니어도 되도록 정규화를 포함한다
    /// (호출 빈도가 높은 findCorrespondencesWithNormals의 다른 오버로드에서는
    /// 단위 벡터를 미리 보장해 sqrt를 생략한 더 빠른 버전을 쓴다 -- 아래
    /// pointTriangleDistanceAndProjectionUnitN 참고).
    inline bool pointPlaneDistanceAndProjection(
        const std::array<double, 3>& P,
        const std::array<double, 3>& Q,
        const std::array<double, 3>& N,
        double& distance,
        std::array<double, 3>& proj)
    {
        constexpr double EPSILON_SQ = 1e-12;
        double nlen2 = N[0] * N[0] + N[1] * N[1] + N[2] * N[2];
        if (nlen2 < EPSILON_SQ)
            return false;

        double PQx = P[0] - Q[0];
        double PQy = P[1] - Q[1];
        double PQz = P[2] - Q[2];

        double pqn_dot = PQx * N[0] + PQy * N[1] + PQz * N[2];

        distance = pqn_dot / std::sqrt(nlen2);

        double t = pqn_dot / nlen2;
        proj[0] = P[0] - N[0] * t;
        proj[1] = P[1] - N[1] * t;
        proj[2] = P[2] - N[2] * t;
        return true;
    }

    /// normal(법선) 정보를 활용하는 Point-to-Plane 대응점 탐색. face별 대표점 1개
    /// (facePt)와 그 face의 normal만으로 점-평면 거리를 계산하는, 저비용 버전이다
    /// (face 전체 삼각형 대신 대표점 하나만 있으면 되는 이유는
    /// doc/DPApp_ICP_Technical_Documentation.docx §5.1 참고).
    inline void findCorrespondencesWithNormals(
        const std::vector<std::array<double, 3>>& sourcePoints,
        const KdTree3D& targetTree,
        const std::vector<size_t>& faceIdx,
        const std::vector<std::array<double, 3>>& allTargetNormals,
        const std::vector<std::array<double, 3>>& allFacePts,
        double maxDistanceSquared,
        double maxAngleDeg,
        std::vector<CorrespondenceFace>& correspondences)
    {
        correspondences.clear();
        correspondences.reserve(sourcePoints.size());

        /// face 인덱스로부터 점 단위 배열을 구성 (target KD-Tree의 각 target 점이
        /// 어느 face에 속하는지를 미리 풀어서(펼쳐서) 저장해 둔다)
        std::vector<std::array<double, 3>> targetNormals(faceIdx.size());
        std::vector<std::array<double, 3>> facePts(faceIdx.size());
        for (size_t i = 0; i < faceIdx.size(); ++i) {
            targetNormals[i] = allTargetNormals[faceIdx[i]];
            facePts[i] = allFacePts[faceIdx[i]];
        }

        double minDist = 1.0e+12;
        double maxDist = 1.0e-12;
        double sumDist = 0.0;
        size_t count = 0;
        for (size_t i = 0; i < sourcePoints.size(); ++i) {
            size_t resultIndex = 0;
            double resultDistSq = 0.0;

            nanoflann::KNNResultSet<double> resultSet(1);
            resultSet.init(&resultIndex, &resultDistSq);
            targetTree.findNeighbors(resultSet, sourcePoints[i].data(),
                nanoflann::SearchParams());

            if (resultDistSq > maxDistanceSquared) continue;

            double distance;
            std::array<double, 3> proj;

            /// Point-to-Plane 거리 및 투영점 계산
            if (!pointPlaneDistanceAndProjection(
                sourcePoints[i],           // P: source 점
                facePts[resultIndex],      // Q: face 위의 점
                targetNormals[resultIndex], // N: face normal
                distance,
                proj))
                continue;

            if (minDist > distance) minDist = distance;
            if (maxDist < distance) maxDist = distance;
            sumDist += distance;
            ++count;

            CorrespondenceFace corr;
            corr.sourceIdx = i;
            corr.facePt = proj;
            corr.squaredDistance = distance * distance;
            correspondences.push_back(corr);
        }

        if (count > 1) {
            double aveDist = sumDist / count;
            ILOG << "[Find correspondence] " << count << "correspondences: min distance(" << minDist << ") max distance(" << maxDist << " ave distance(" << aveDist << ")";
        }
        else {
            ILOG << "[Find correspondence] NONE";
        }
    }

    /// 가장 빠른 버전: 단위 법선 벡터(|N| = 1)가 이미 보장된 경우에 사용.
    /// 삼각형(v0, v1, v2) 위로 점 P를 투영하고, 그 투영점이 실제로 삼각형 내부에
    /// 있는지(barycentric 좌표 기준)까지 함께 확인한다.
    inline bool pointTriangleDistanceAndProjectionUnitN(
        const std::array<double, 3>& P,
        const std::array<double, 3>& v0,
        const std::array<double, 3>& v1,
        const std::array<double, 3>& v2,
        const std::array<double, 3>& N,  /// 반드시 단위 벡터여야 함
        double& distance,
        std::array<double, 3>& proj)
    {
        constexpr double EPSILON_SQ = 1e-12;
        constexpr double EDGE_TOLERANCE = 1e-6;

        /// 삼각형의 두 변(edge) 벡터
        const double e0x = v1[0] - v0[0];
        const double e0y = v1[1] - v0[1];
        const double e0z = v1[2] - v0[2];

        const double e1x = v2[0] - v0[0];
        const double e1y = v2[1] - v0[1];
        const double e1z = v2[2] - v0[2];

        /// barycentric 좌표 계산을 위한 내적들
        const double dot00 = e0x * e0x + e0y * e0y + e0z * e0z;
        const double dot01 = e0x * e1x + e0y * e1y + e0z * e1z;
        const double dot11 = e1x * e1x + e1y * e1y + e1z * e1z;

        const double denom = dot00 * dot11 - dot01 * dot01;
        if (denom < EPSILON_SQ)
            return false;

        /// P - v0
        const double pvx = P[0] - v0[0];
        const double pvy = P[1] - v0[1];
        const double pvz = P[2] - v0[2];

        const double dot02 = e0x * pvx + e0y * pvy + e0z * pvz;
        const double dot12 = e1x * pvx + e1y * pvy + e1z * pvz;

        /// barycentric u, v 계산 (조기 반려(early rejection)로 불필요한 연산을 줄임)
        const double invDenom = 1.0 / denom;
        const double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        if (u < -EDGE_TOLERANCE || u > 1.0 + EDGE_TOLERANCE)
            return false;

        const double v = (dot00 * dot12 - dot01 * dot02) * invDenom;
        if (v < -EDGE_TOLERANCE || (u + v) > 1.0 + EDGE_TOLERANCE)
            return false;

        /// 거리 = (P-v0)·N (단위 법선이므로 sqrt 불필요)
        distance = pvx * N[0] + pvy * N[1] + pvz * N[2];

        /// 투영점: P - N * distance
        proj[0] = P[0] - N[0] * distance;
        proj[1] = P[1] - N[1] * distance;
        proj[2] = P[2] - N[2] * distance;

        return true;
    }

    /// 위 findCorrespondencesWithNormals의 변형: face 대표점 1개 대신, face를
    /// 이루는 삼각형 3개 정점(allFaceVtx)을 모두 받아 실제 삼각형 내부 투영
    /// 여부까지 검사하는 더 정밀한(대신 더 비싼) 버전.
    inline void findCorrespondencesWithNormals(
        const std::vector<std::array<double, 3>>& sourcePoints,
        const KdTree3D& targetTree,
        const std::vector<size_t>& faceIdx,
        const std::vector<std::array<double, 3>>& allTargetNormals,
        const std::vector<std::array<std::array<double, 3>, 3>>& allFaceVtx,
        double maxDistanceSquared,
        double maxAngleDeg,
        std::vector<CorrespondenceFace>& correspondences)
    {
        correspondences.clear();
        correspondences.reserve(sourcePoints.size());

        /// face 인덱스로부터 점 단위 배열을 구성
        std::vector<std::array<double, 3>> targetNormals(faceIdx.size());
        std::vector<std::array<std::array<double, 3>, 3>> faceVtx(faceIdx.size());
        for (size_t i = 0; i < faceIdx.size(); ++i) {
            targetNormals[i] = allTargetNormals[faceIdx[i]];
            faceVtx[i] = allFaceVtx[faceIdx[i]];
        }

        for (size_t i = 0; i < sourcePoints.size(); ++i) {
            size_t resultIndex = 0;
            double resultDistSq = 0.0;

            nanoflann::KNNResultSet<double> resultSet(1);
            resultSet.init(&resultIndex, &resultDistSq);
            targetTree.findNeighbors(resultSet, sourcePoints[i].data(),
                nanoflann::SearchParams());

            if (resultDistSq > maxDistanceSquared) continue;

            double distance;
            std::array<double, 3> proj;

            /// Point-to-Plane 거리 및 투영점 계산
            if (!pointTriangleDistanceAndProjectionUnitN(
                sourcePoints[i],           /// P: source 점
                faceVtx[resultIndex][0],   /// V0
                faceVtx[resultIndex][1],   /// V1
                faceVtx[resultIndex][2],   /// V2
                targetNormals[resultIndex],/// N: face normal
                distance,
                proj))
                continue;

            CorrespondenceFace corr;
            corr.sourceIdx = i;
            corr.facePt = proj;
            corr.squaredDistance = distance * distance;
            correspondences.push_back(corr);
        }
    }

    //=========================================================================
    // 이상치(Outlier) 제거
    //=========================================================================

    /// MAD(Median Absolute Deviation, 중위절대편차)를 이용해 이상치를 제거한다
    /// @param correspondences 필터링할 대응점 목록 (제자리에서 수정됨)
    /// @param threshold MAD에 곱할 배수 (보통 2.0 ~ 3.0)
    inline void rejectOutliersMAD(
        std::vector<Correspondence>& correspondences,
        double threshold)
    {
        if (correspondences.size() < 10) return;

        /// 거리값 추출
        std::vector<double> distances;
        distances.reserve(correspondences.size());
        for (const auto& c : correspondences) {
            distances.push_back(std::sqrt(c.squaredDistance));
        }

        /// 중앙값(median) 계산
        std::vector<double> sortedDist = distances;
        std::sort(sortedDist.begin(), sortedDist.end());
        const double median = sortedDist[sortedDist.size() / 2];

        /// MAD(중위절대편차) 계산
        std::vector<double> absDevs;
        absDevs.reserve(distances.size());
        for (double d : distances) {
            absDevs.push_back(std::abs(d - median));
        }
        std::sort(absDevs.begin(), absDevs.end());
        const double mad = absDevs[absDevs.size() / 2];

        /// MAD를 표준편차로 근사 변환하는 스케일 계수
        const double madScale = 1.4826;
        const double sigma = mad * madScale;

        /// 중앙값으로부터 threshold * sigma를 넘는 대응점은 이상치로 제거
        const double maxDist = median + threshold * sigma;
        const double maxDistSq = maxDist * maxDist;

        correspondences.erase(
            std::remove_if(correspondences.begin(), correspondences.end(),
                [maxDistSq](const Correspondence& c) {
                    return c.squaredDistance > maxDistSq;
                }),
            correspondences.end());
    }

    /// 백분위수(percentile) 기준으로 이상치를 제거한다
    /// @param correspondences 필터링할 대응점 목록
    /// @param percentile 이 백분위수 이하의 대응점만 유지 (0-100)
    inline void rejectOutliersPercentile(
        std::vector<Correspondence>& correspondences,
        double percentile)
    {
        if (correspondences.size() < 10) return;

        std::vector<double> distances;
        distances.reserve(correspondences.size());
        for (const auto& c : correspondences) {
            distances.push_back(c.squaredDistance);
        }
        std::sort(distances.begin(), distances.end());

        size_t idx = static_cast<size_t>((percentile / 100.0) * distances.size());
        idx = (std::min)(idx, distances.size() - 1);
        const double maxDistSq = distances[idx];

        correspondences.erase(
            std::remove_if(correspondences.begin(), correspondences.end(),
                [maxDistSq](const Correspondence& c) {
                    return c.squaredDistance > maxDistSq;
                }),
            correspondences.end());
    }

    //=========================================================================
    // SVD를 이용한 변환 추정
    //=========================================================================

    /// SVD를 이용해 강체 변환(회전 + 이동)을 추정한다 (Point-to-Point, Kabsch/Umeyama 방식)
    /// target ≈ R * source + t 를 만족하는 T를 찾는다
    /// @param sourcePoints source 포인트클라우드
    /// @param targetPoints target 포인트클라우드
    /// @param correspondences 점 대응 관계
    /// @return 4x4 변환 행렬
    inline Transform4x4 estimateRigidTransformSVD(
        const std::vector<std::array<double, 3>>& sourcePoints,
        const std::vector<std::array<double, 3>>& targetPoints,
        const std::vector<Correspondence>& correspondences)
    {
        if (correspondences.size() < 3) {
            return Transform4x4::identity();
        }

        const size_t n = correspondences.size();

        /// 중심점(centroid) 계산
        Eigen::Vector3d srcCentroid = Eigen::Vector3d::Zero();
        Eigen::Vector3d tgtCentroid = Eigen::Vector3d::Zero();

        for (const auto& c : correspondences) {
            const auto& sp = sourcePoints[c.sourceIdx];
            const auto& tp = targetPoints[c.targetIdx];
            srcCentroid += Eigen::Vector3d(sp[0], sp[1], sp[2]);
            tgtCentroid += Eigen::Vector3d(tp[0], tp[1], tp[2]);
        }
        srcCentroid /= static_cast<double>(n);
        tgtCentroid /= static_cast<double>(n);

        /// 교차공분산 행렬(cross-covariance matrix) H 계산
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

        for (const auto& c : correspondences) {
            const auto& sp = sourcePoints[c.sourceIdx];
            const auto& tp = targetPoints[c.targetIdx];

            Eigen::Vector3d srcCentered(
                sp[0] - srcCentroid.x(),
                sp[1] - srcCentroid.y(),
                sp[2] - srcCentroid.z());

            Eigen::Vector3d tgtCentered(
                tp[0] - tgtCentroid.x(),
                tp[1] - tgtCentroid.y(),
                tp[2] - tgtCentroid.z());

            H += srcCentered * tgtCentered.transpose();
        }

        /// SVD 분해: H = U * S * V^T
        Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(H);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();

        /// 회전: R = V * U^T
        Eigen::Matrix3d R = V * U.transpose();

        /// 올바른 회전(det(R) = 1)인지 확인 -- 반사(reflection, det = -1)가 나오면
        /// V의 마지막 열의 부호를 뒤집어 보정한다 (Kabsch 알고리즘의 표준적인
        /// reflection 방지 처리)
        if (R.determinant() < 0) {
            V.col(2) *= -1;
            R = V * U.transpose();
        }

        /// 이동: t = tgtCentroid - R * srcCentroid
        Eigen::Vector3d t = tgtCentroid - R * srcCentroid;

        /// 4x4 변환 행렬 구성
        double rotMat[9];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotMat[i * 3 + j] = R(i, j);
            }
        }

        return Transform4x4::fromRotationTranslation(rotMat, t.x(), t.y(), t.z());
    }

    //=========================================================================
    // RMSE 계산
    //=========================================================================

    /// 대응점들로부터 평균제곱근오차(RMSE)를 계산
    /// @param correspondences 거리 제곱값을 담고 있는 대응점 목록
    /// @return RMSE 값
    inline double computeRMSE(const std::vector<Correspondence>& correspondences)
    {
        if (correspondences.empty()) {
            return std::numeric_limits<double>::max();
        }

        double sum = 0.0;
        for (const auto& c : correspondences) {
            sum += c.squaredDistance;
        }
        return std::sqrt(sum / correspondences.size());
    }

    /// 변환을 적용한 뒤의 RMSE를 계산
    /// @param sourcePoints source 점들 (원본, 아직 변환 전)
    /// @param targetPoints target 점들
    /// @param correspondences 점 대응 관계
    /// @param transform source에 적용할 변환
    /// @return 변환 적용 후의 RMSE 값
    inline double computeRMSE(
        const std::vector<std::array<double, 3>>& sourcePoints,
        const std::vector<std::array<double, 3>>& targetPoints,
        const std::vector<Correspondence>& correspondences,
        const Transform4x4& transform)
    {
        if (correspondences.empty()) {
            return std::numeric_limits<double>::max();
        }

        double sumSquaredError = 0.0;

        for (const auto& c : correspondences) {
            const auto& sp = sourcePoints[c.sourceIdx];
            const auto& tp = targetPoints[c.targetIdx];

            /// source 점 변환
            double tx, ty, tz;
            transform.transformPoint(sp[0], sp[1], sp[2], tx, ty, tz);

            /// 거리 제곱 계산
            const double dx = tx - tp[0];
            const double dy = ty - tp[1];
            const double dz = tz - tp[2];
            sumSquaredError += dx * dx + dy * dy + dz * dz;
        }

        return std::sqrt(sumSquaredError / correspondences.size());
    }

    //=========================================================================
    // 포인트클라우드 변환
    //=========================================================================

    /// 포인트클라우드에 변환을 제자리(in-place)로 적용
    inline void transformPointCloud(
        std::vector<std::array<double, 3>>& points,
        const Transform4x4& transform)
    {
        for (auto& p : points) {
            transform.transformPointInPlace(p[0], p[1], p[2]);
        }
    }

    /// 포인트클라우드에 변환을 적용 (복사본 반환 버전)
    inline std::vector<std::array<double, 3>> transformPointCloudCopy(
        const std::vector<std::array<double, 3>>& points,
        const Transform4x4& transform)
    {
        std::vector<std::array<double, 3>> result;
        result.reserve(points.size());

        for (const auto& p : points) {
            double tx, ty, tz;
            transform.transformPoint(p[0], p[1], p[2], tx, ty, tz);
            result.push_back({ tx, ty, tz });
        }
        return result;
    }

    /// float로 저장된 포인트클라우드에 변환을 적용 (복사본 반환 버전). 각 점은
    /// 변환 연산의 정밀도를 위해 double로 승격시킨 뒤 계산하고, 저장을 위해 다시
    /// float로 축소한다 -- Master가 들고 있는 대량의 (offset-재배치된) 포인트클라우드
    /// 배열에 사용되며, 이 배열은 항상 float로 유지되지만 변환 자체는 항상
    /// double로 계산한다.
    inline std::vector<std::array<float, 3>> transformPointCloudCopy(
        const std::vector<std::array<float, 3>>& points,
        const Transform4x4& transform)
    {
        std::vector<std::array<float, 3>> result;
        result.reserve(points.size());

        for (const auto& p : points) {
            double tx, ty, tz;
            transform.transformPoint(
                static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2]),
                tx, ty, tz);
            result.push_back({ static_cast<float>(tx), static_cast<float>(ty), static_cast<float>(tz) });
        }
        return result;
    }

    //=========================================================================
    // 다운샘플링
    //=========================================================================

    /// 균등 선택(매 N번째 점) 방식으로 포인트클라우드를 다운샘플링
    inline bool downsampleUniform(
        const std::vector<std::array<double, 3>>& points,
        const std::vector<size_t>& faceIdx,
        std::vector<std::array<double, 3>>& downPts,
        std::vector<size_t>& downFaceIdx,
        int ratio)
    {
        if (ratio <= 0) return false;
        if (ratio == 1) { downPts = points; downFaceIdx = faceIdx; return true; }

        auto numSamples = points.size() / ratio + 1;
        downPts.reserve(numSamples);
        downFaceIdx.reserve(numSamples);

        for (size_t i = 0; i < points.size(); i += ratio) {
            downPts.push_back(points[i]);
            downFaceIdx.push_back(faceIdx[i]);
        }

        return true;
    }

    /// 균등 선택(매 N번째 점) 방식으로 포인트클라우드를 다운샘플링
    inline std::vector<std::array<double, 3>> downsampleUniform(
        const std::vector<std::array<double, 3>>& points,
        int ratio)
    {
        if (ratio <= 0) return {};
        if (ratio == 1) return points;

        std::vector<std::array<double, 3>> downPts;
        auto numSamples = points.size() / ratio + 1;
        downPts.reserve(numSamples);

        for (size_t i = 0; i < points.size(); i += ratio) {
            downPts.push_back(points[i]);
        }

        return downPts;
    }

    /// float로 저장된 포인트클라우드를 균등 선택(매 N번째 점) 방식으로
    /// 다운샘플링. 단순 원소 복사이며 정밀도에 민감한 연산은 없다 -- Master가
    /// 들고 있는 대량의 (offset-재배치된) 포인트클라우드 배열에 사용된다.
    inline std::vector<std::array<float, 3>> downsampleUniform(
        const std::vector<std::array<float, 3>>& points,
        int ratio)
    {
        if (ratio <= 0) return {};
        if (ratio == 1) return points;

        std::vector<std::array<float, 3>> downPts;
        auto numSamples = points.size() / ratio + 1;
        downPts.reserve(numSamples);

        for (size_t i = 0; i < points.size(); i += ratio) {
            downPts.push_back(points[i]);
        }

        return downPts;
    }

    /// float 점 배열을 double로 승격시킨다 (예: 다운샘플링된 일부를 IcpChunk에
    /// 넘길 때 -- IcpChunk는 아래의 (수정되지 않은) ICP 대응점탐색/변환추정
    /// 코드가 사용할 수 있도록 double로 저장한다).
    inline std::vector<std::array<double, 3>> widenToDouble(
        const std::vector<std::array<float, 3>>& points)
    {
        std::vector<std::array<double, 3>> result;
        result.reserve(points.size());
        for (const auto& p : points) {
            result.push_back({ static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2]) });
        }
        return result;
    }

    /// double 점 배열을 float로 축소한다 (예: 전체 LAS 포인트클라우드를
    /// 로딩/offset-재배치한 직후, Master의 대량 float 저장소에 자리잡기 전에 사용).
    inline std::vector<std::array<float, 3>> narrowToFloat(
        const std::vector<std::array<double, 3>>& points)
    {
        std::vector<std::array<float, 3>> result;
        result.reserve(points.size());
        for (const auto& p : points) {
            result.push_back({ static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]) });
        }
        return result;
    }

    /// 복셀 격자(voxel grid) 방식으로 포인트클라우드를 다운샘플링
    /// @param points 입력 포인트클라우드
    /// @param voxelSize 복셀 하나의 크기
    /// @return 다운샘플링된 포인트클라우드 (각 복셀의 중심점(centroid))
    inline std::vector<std::array<double, 3>> downsampleVoxelGrid(
        const std::vector<std::array<double, 3>>& points,
        double voxelSize)
    {
        if (points.empty() || voxelSize <= 0.0) return points;

        /// 바운딩 박스 계산
        double minX = points[0][0], maxX = points[0][0];
        double minY = points[0][1], maxY = points[0][1];
        double minZ = points[0][2], maxZ = points[0][2];

        for (const auto& p : points) {
            if (p[0] < minX) minX = p[0]; if (p[0] > maxX) maxX = p[0];
            if (p[1] < minY) minY = p[1]; if (p[1] > maxY) maxY = p[1];
            if (p[2] < minZ) minZ = p[2]; if (p[2] > maxZ) maxZ = p[2];
        }

        /// 복셀 격자의 차원(개수)
        const int nx = static_cast<int>((maxX - minX) / voxelSize) + 1;
        const int ny = static_cast<int>((maxY - minY) / voxelSize) + 1;

        /// 희소(sparse) 복셀 저장을 위해 map 사용
        /// 키: 복셀 인덱스, 값: (점 좌표 합, 개수)
        std::unordered_map<size_t, std::pair<std::array<double, 3>, size_t>> voxelMap;

        for (const auto& p : points) {
            const int ix = static_cast<int>((p[0] - minX) / voxelSize);
            const int iy = static_cast<int>((p[1] - minY) / voxelSize);
            const int iz = static_cast<int>((p[2] - minZ) / voxelSize);

            const size_t key = static_cast<size_t>(ix) +
                static_cast<size_t>(iy) * nx +
                static_cast<size_t>(iz) * nx * ny;

            auto& voxel = voxelMap[key];
            voxel.first[0] += p[0];
            voxel.first[1] += p[1];
            voxel.first[2] += p[2];
            voxel.second++;
        }

        /// 각 복셀의 중심점(centroid) 추출
        std::vector<std::array<double, 3>> result;
        result.reserve(voxelMap.size());

        for (const auto& kv : voxelMap) {
            const auto& sum = kv.second.first;
            const size_t count = kv.second.second;
            result.push_back({
                sum[0] / count,
                sum[1] / count,
                sum[2] / count
                });
        }

        return result;
    }

    //=========================================================================
    // CorrespondenceFace를 위한 이상치 제거
    //=========================================================================

    /// CorrespondenceFace(Point-to-Plane)에 대해 MAD 기반 이상치 제거
    inline void rejectOutliersMAD(
        std::vector<CorrespondenceFace>& correspondences,
        double threshold)
    {
        if (correspondences.size() < 10) return;

        std::vector<double> distances;
        distances.reserve(correspondences.size());
        for (const auto& c : correspondences) {
            distances.push_back(std::sqrt(c.squaredDistance));
        }

        std::vector<double> sortedDist = distances;
        std::sort(sortedDist.begin(), sortedDist.end());
        const double median = sortedDist[sortedDist.size() / 2];

        std::vector<double> absDevs;
        absDevs.reserve(distances.size());
        for (double d : distances) {
            absDevs.push_back(std::abs(d - median));
        }
        std::sort(absDevs.begin(), absDevs.end());
        const double mad = absDevs[absDevs.size() / 2];

        const double madScale = 1.4826;
        const double sigma = mad * madScale;
        const double maxDist = median + threshold * sigma;
        const double maxDistSq = maxDist * maxDist;

        correspondences.erase(
            std::remove_if(correspondences.begin(), correspondences.end(),
                [maxDistSq](const CorrespondenceFace& c) {
                    return c.squaredDistance > maxDistSq;
                }),
            correspondences.end());
    }

    //=========================================================================
    // CorrespondenceFace의 RMSE 계산
    //=========================================================================

    /// CorrespondenceFace(Point-to-Plane 거리)로부터 RMSE 계산
    inline double computeRMSE(const std::vector<CorrespondenceFace>& correspondences)
    {
        if (correspondences.empty()) {
            return std::numeric_limits<double>::max();
        }

        double sum = 0.0;
        for (const auto& c : correspondences) {
            sum += c.squaredDistance;
        }
        return std::sqrt(sum / correspondences.size());
    }

    //=========================================================================
    // Point-to-Plane 변환 추정
    //=========================================================================

    /// Point-to-Plane 선형화 기법으로 강체 변환을 추정한다.
    /// 소각 근사(small angle approximation)를 이용해 회전을 (rx, ry, rz)
    /// 세 각도로 선형화하고, 6개 미지수(tx,ty,tz,rx,ry,rz)에 대한 선형 최소제곱
    /// 문제 Ax=b를 SVD로 풀어 한 번에 계산한다 (반복적으로 각도를 갱신하는
    /// 비선형 최적화가 아니라, ICP 반복 루프 자체가 이 선형화 오차를 상쇄한다 --
    /// 자세한 수식 유도는 기술문서 §5.1 참고).
    inline Transform4x4 estimateRigidTransformPointToPlane(
        const std::vector<std::array<double, 3>>& sourcePoints,
        const std::vector<CorrespondenceFace>& correspondences,
        const std::vector<size_t>& faceIdx,
        const std::vector<std::array<double, 3>>& allFaceNormals)
    {
        if (correspondences.size() < uknw) {
            return Transform4x4::identity();
        }

        const size_t n = correspondences.size();

        Eigen::MatrixXd A(n, uknw);
        Eigen::VectorXd b(n);

        for (size_t i = 0; i < n; ++i) {
            const auto& corr = correspondences[i];
            const auto& s = sourcePoints[corr.sourceIdx];
            const auto& p = corr.facePt;

            /// source 점 s와 투영점 p 사이의 방향을 그 대응점의 근사 normal로 사용
            double dx = s[0] - p[0];
            double dy = s[1] - p[1];
            double dz = s[2] - p[2];
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            double nx, ny, nz;
            if (dist > 1e-10) {
                nx = dx / dist;
                ny = dy / dist;
                nz = dz / dist;
            }
            else {
                nx = 0; ny = 0; nz = 1;
            }

            double sx = s[0], sy = s[1], sz = s[2];

            /// A 행렬의 각 행: [nx, ny, nz, sy*nz-sz*ny, sz*nx-sx*nz, sx*ny-sy*nx]
            A(i, 0) = nx;
            A(i, 1) = ny;
            A(i, 2) = nz;
            A(i, 3) = sy * nz - sz * ny;
            A(i, 4) = sz * nx - sx * nz;
            A(i, 5) = sx * ny - sy * nx;

            /// b 벡터: 부호 있는 point-to-plane 거리
            b(i) = nx * (p[0] - sx) + ny * (p[1] - sy) + nz * (p[2] - sz);
        }

        /// SVD 최소제곱으로 풀이 (최신 Eigen에서는 BDCSVD 권장)
        Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(A);
        Eigen::VectorXd x = svd.solve(b);

        double tx = x(0), ty = x(1), tz = x(2);
        double rx = x(3), ry = x(4), rz = x(5);

        /// 선형화로 얻은 소각(rx,ry,rz)으로부터 실제 회전 행렬(R) 구성 (ZYX 오일러 순서)
        double cx = std::cos(rx), sx_r = std::sin(rx);
        double cy = std::cos(ry), sy = std::sin(ry);
        double cz = std::cos(rz), sz = std::sin(rz);

        Eigen::Matrix3d R;
        R(0, 0) = cy * cz;
        R(0, 1) = cz * sx_r * sy - cx * sz;
        R(0, 2) = sx_r * sz + cx * cz * sy;
        R(1, 0) = cy * sz;
        R(1, 1) = cx * cz + sx_r * sy * sz;
        R(1, 2) = cx * sy * sz - cz * sx_r;
        R(2, 0) = -sy;
        R(2, 1) = cy * sx_r;
        R(2, 2) = cx * cy;

        double rotMat[9];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotMat[i * 3 + j] = R(i, j);
            }
        }

        return Transform4x4::fromRotationTranslation(rotMat, tx, ty, tz);
    }

    /// [미완성/미사용] 아래 calDiscrepancy부터 runLSM까지는 SSMMatrixd 기반의
    /// 비선형 최소제곱(NLLS) 방식으로 Point-to-Plane 변환을 추정하려던 실험적
    /// 구현이다. 현재 runIcp()는 이 대신 위쪽의 estimateRigidTransformPointToPlane
    /// (Eigen SVD 선형화 버전)을 사용하며, 이 구간은 코드베이스 어디에서도 호출되지
    /// 않는다("[ToDo] under construction" 원문 주석 그대로 유지). 삭제하지 않고
    /// 남겨둔 이유는 알 수 없으므로, 수정 시 이 구간이 실제로 쓰이고 있는지 반드시
    /// grep으로 재확인할 것.
    inline void calDiscrepancy(const std::array<double, 3>& src, const std::array<double, 3>& tar, const std::array<double, uknw>& params, math::Matrixd& ai, math::Matrixd& yi)
    {
        math::Matrixd GA(3, 1), GB(3, 1);

        GA(0, 0) = src[0];
        GA(1, 0) = src[1];
        GA(2, 0) = src[2];

        GB(0, 0) = tar[0];
        GB(1, 0) = tar[1];
        GB(2, 0) = tar[2];

        auto R = math::getRMat(params[3], params[4], params[5]);
        auto RGA = R % GA;

        auto dRdO = math::getPartial_dRdO(params[3], params[4], params[5]);
        auto Partial_O = dRdO % GA;

        auto dRdP = math::getPartial_dRdP(params[3], params[4], params[5]);
        auto Partial_P = dRdP % GA;

        auto dRdK = math::getPartial_dRdK(params[3], params[4], params[5]);
        auto Partial_K = dRdK % GA;

        math::Matrixd T(3, 1);

        T(0, 0) = params[0];
        T(1, 0) = params[1];
        T(2, 0) = params[2];

        //dTx, dTy, dTz, dO, dP, dK, dS에 대한 편미분 계수 행렬 구성
        ai(0, 0) = 1;
        ai(0, 1) = 0;
        ai(0, 2) = 0;
        ai(0, 3) = Partial_O(0, 0);
        ai(0, 4) = Partial_P(0, 0);
        ai(0, 5) = Partial_K(0, 0);

        ai(1, 0) = 0;
        ai(1, 1) = 1;
        ai(1, 2) = 0;
        ai(1, 3) = Partial_O(1, 0);
        ai(1, 4) = Partial_P(1, 0);
        ai(1, 5) = Partial_K(1, 0);

        ai(2, 0) = 0;
        ai(2, 1) = 0;
        ai(2, 2) = 1;
        ai(2, 3) = Partial_O(2, 0);
        ai(2, 4) = Partial_P(2, 0);
        ai(2, 5) = Partial_K(2, 0);

        yi = GB - RGA - T;
    }

    /// [ToDo] 미완성
    /// const double benchSd 추가 필요
    /// std::array<double, 3> xyzSd 추가 필요
    struct RETVAL_NLLS {
        std::array<double, uknw> params;
        math::Matrixd Correlation;
        double var;
        math::Matrixd varX;
    };

    inline RETVAL_NLLS estimateRigidTransformPointToPlane_NLLS(
        const std::vector<std::array<double, 3>>& sourcePoints,
        const std::vector<CorrespondenceFace>& correspondences,
        const std::vector<size_t>& faceIdx,
        const std::vector<std::array<double, 3>>& allFaceNormals,
        const std::array<double, uknw>& params0,
        const std::array<double, uknw>& paramsSd)
    {
        RETVAL_NLLS retval;
        retval.params = params0;

        if (correspondences.size() < uknw) {
            retval.Correlation.resize(uknw, uknw);
            retval.Correlation.makeIdentityMat();
            retval.var = 0.;
            retval.varX.resize(uknw, 1, 0.0);
            return retval;
        }

        const size_t n = correspondences.size();

        Eigen::MatrixXd A(n, uknw);
        Eigen::VectorXd b(n);

        math::Matrixd N(uknw, uknw, 0.);
        math::Matrixd C(uknw, 1, 0.);
        math::Matrixd ai(3, uknw);
        math::Matrixd yi(3, 1);
        const size_t numEqs = n;
        double etpe = 0.;

        for (size_t i = 0; i < n; ++i) {
            const auto& corr = correspondences[i];
            const auto& s = sourcePoints[corr.sourceIdx];
            const auto& p = corr.facePt;

            calDiscrepancy(s, p, retval.params, ai, yi);

            auto ait = ai.transpose();
            auto ci = ait % yi;
            auto ni = ait % ai;

            etpe += (yi.transpose() % yi)(0, 0);

            N += ni;
            C += ci;
        }

        for (size_t i = 0; i < uknw; ++i) {
            if (paramsSd[i] <= minParamSd) {
                for (size_t j = 0; j < uknw; ++j) {
                    if (i == j) {
                        N(i, j) = 1.0;
                    }
                    else {
                        N(i, j) = 0.0; N(j, i) = 0.0;
                    }
                }

                C(i, 0) = 0.0;
            }
        }

        auto Ninv = N.inverse();

        retval.Correlation.resize(uknw, uknw, 0.0);
        for (unsigned int i = 0; i < uknw; i++)
        {
            for (unsigned int j = 0; j < uknw; j++)
            {
                retval.Correlation(i, j) = Ninv(i, j) / sqrt(Ninv(i, i)) / sqrt(Ninv(j, j));
            }
        }

        auto X = Ninv % C;

#ifdef _DEBUG
        std::cout << "[N mat]\n";
        std::cout << math::matrixout(N) << "\n";
        std::cout << "[Ninv mat]\n";
        std::cout << math::matrixout(Ninv) << "\n";
        std::cout << "[C mat]\n";
        std::cout << math::matrixout(C) << "\n";
        std::cout << "[X mat]\n";
        std::cout << math::matrixout(X) << "\n";
#endif

        for (unsigned int i = 0; i < uknw; i++) {
            if (paramsSd[i] > minParamSd) {
                if (i > 2) retval.params[i] += X(i, 0);
                else retval.params[i] += X(i, 0);
            }
        }

        retval.var = etpe / (numEqs - uknw);
        retval.varX = Ninv * retval.var;

        return retval;
    }

    /// [ToDo]
    /// CorrespondenceFace가 필요한가, 전부 다 필요한가???
    inline Transform4x4 runLSM(
        const std::vector<std::array<double, 3>>& sourcePoints,
        const std::vector<CorrespondenceFace>& correspondences,
        const std::vector<size_t>& faceIdx,
        const std::vector<std::array<double, 3>>& allFaceNormals,
        const std::array<double, uknw>& params0,
        const double sdThr,
        const unsigned int maxIter)
    {
        if (correspondences.size() < uknw) {
            return Transform4x4::identity();
        }

        std::array<double, 3> sumSrc = { 0., 0., 0. };
        std::array<double, 3> sumTar = { 0., 0., 0. };

        for (size_t i = 0; i < correspondences.size(); ++i) {
            auto srcIdx = correspondences[i].sourceIdx;
            for (size_t j = 0; j < 3; ++j) {
                sumTar[j] += correspondences[i].facePt[j];
                sumSrc[j] += sourcePoints[srcIdx][j];
            }
        }

        std::array<double, uknw> paramsSd;
        paramsSd[0] = maxParamSd;
        paramsSd[1] = maxParamSd;
        paramsSd[2] = maxParamSd;
        paramsSd[3] = maxParamSd;
        paramsSd[4] = maxParamSd;
        paramsSd[5] = maxParamSd;

        RETVAL_NLLS retval;
        retval.params = params0;
        for (size_t i = 0; i < 3; ++i) {
            retval.params[i] = (sumSrc[i] - sumTar[i]) / correspondences.size();
        }

        double preSd = 0.0;
        bool continueIter = true;
        unsigned int iter = 0;
        do {
            retval = estimateRigidTransformPointToPlane_NLLS(sourcePoints,
                correspondences,
                faceIdx,
                allFaceNormals,
                retval.params,
                paramsSd);

            ILOG << "[ICP] Iter " << (iter + 1) << ": T(" << retval.params[0] << ", " << retval.params[1] << ", " << retval.params[2] << ")" \
                << " R(" << retval.params[3] * Rad2Deg << ", " << retval.params[4] * Rad2Deg << ", " << retval.params[5] * Rad2Deg << ")deg";

            std::cout << "[Correlation]\n";
            std::cout << math::matrixout(retval.Correlation);

            std::cout << "[X_VarCov]\n";
            std::cout << math::matrixout(retval.varX);
            std::cout << "[X sd]\n";
            for (unsigned int i = 0; i < uknw; ++i)
                std::cout << sqrt(retval.varX(i, i)) << "\n";

            if (fabs(preSd - sqrt(retval.var)) < sdThr) {
                continueIter = false;
            }

            ++iter;

        } while (iter < maxIter && continueIter);

        double tx = retval.params[0], ty = retval.params[1], tz = retval.params[2];
        double rx = retval.params[3], ry = retval.params[4], rz = retval.params[5];

        double cx = std::cos(rx), sx_r = std::sin(rx);
        double cy = std::cos(ry), sy = std::sin(ry);
        double cz = std::cos(rz), sz = std::sin(rz);

        Eigen::Matrix3d R;
        R(0, 0) = cy * cz;
        R(0, 1) = cz * sx_r * sy - cx * sz;
        R(0, 2) = sx_r * sz + cx * cz * sy;
        R(1, 0) = cy * sz;
        R(1, 1) = cx * cz + sx_r * sy * sz;
        R(1, 2) = cx * sy * sz - cz * sx_r;
        R(2, 0) = -sy;
        R(2, 1) = cy * sx_r;
        R(2, 2) = cx * cy;

        double rotMat[9];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotMat[i * 3 + j] = R(i, j);
            }
        }

        return Transform4x4::fromRotationTranslation(rotMat, tx, ty, tz);
    }

    //=========================================================================
    // ICP 메인 알고리즘
    //=========================================================================

    /// ICP 알고리즘을 실행한다 (Point-to-Plane 또는 Point-to-Point)
    /// @param chunk source/target 점들과 설정값을 담은 ICP 청크
    /// @param cancelRequested 취소 확인용 atomic 플래그
    /// @return 변환 행렬과 통계치를 담은 ICP 결과
    inline IcpResult runIcp(const IcpChunk& chunk, std::atomic<bool>& cancelRequested)
    {
        IcpResult result;
        result.chunk_id = chunk.chunk_id;
        result.sourcePointCount = static_cast<int>(chunk.sourcePoints.size());
        result.targetPointCount = static_cast<int>(chunk.targetPoints.size());

        auto startTime = std::chrono::high_resolution_clock::now();

        /// 입력 유효성 검사
        if (chunk.sourcePoints.empty()) {
            result.success = false;
            result.errorMessage = "Source points are empty";
            return result;
        }
        if (chunk.targetPoints.empty()) {
            result.success = false;
            result.errorMessage = "Target points are empty";
            return result;
        }

        /// 청크의 점 배열은 (offsetX, offsetY, offsetZ)만큼 이동된 float로 저장되어
        /// 있다 -- 실제 연산을 시작하기 전에, 여기서 딱 한 번 double로 승격시키고
        /// offset을 다시 더한다. 이 지점 아래의 모든 코드는 이 float+offset
        /// 최적화가 도입되기 이전과 완전히 동일하게, 이 로컬 double 벡터들을
        /// 대상으로 동작한다. faceNormals는 방향 벡터이므로 승격만 하고 offset은
        /// 더하지 않는다.
        const auto unshiftWiden = [&](const std::vector<std::array<float, 3>>& src) {
            std::vector<std::array<double, 3>> out;
            out.reserve(src.size());
            for (const auto& p : src) {
                out.push_back({
                    static_cast<double>(p[0]) + chunk.offsetX,
                    static_cast<double>(p[1]) + chunk.offsetY,
                    static_cast<double>(p[2]) + chunk.offsetZ });
            }
            return out;
            };
        std::vector<std::array<double, 3>> sourcePointsD = unshiftWiden(chunk.sourcePoints);
        std::vector<std::array<double, 3>> targetPointsD = unshiftWiden(chunk.targetPoints);
        std::vector<std::array<double, 3>> facePtsD = unshiftWiden(chunk.facePts);
        std::vector<std::array<double, 3>> faceNormalsD = widenToDouble(chunk.faceNormals);

        const auto& config = chunk.config;

        /// Point-to-Plane 모드를 쓸 수 있는지 확인 (face 정보가 모두 있어야 함)
        const bool usePointToPlane = !chunk.faceIndices.empty() &&
            !chunk.faceNormals.empty() &&
            !chunk.facePts.empty();

        /// target 점들로 KD-Tree 구성
        PointCloudAdaptor targetAdaptor(targetPointsD);
        KdTree3D targetTree(3, targetAdaptor,
            nanoflann::KDTreeSingleIndexAdaptorParams(50));
        targetTree.buildIndex();

        /// 변환 초기화
        Transform4x4 currentTransform = chunk.initialTransform;
        Transform4x4 accumulatedLocalTransform = Transform4x4::identity();

        /// 변환을 적용할 source 점들의 복사본
        std::vector<std::array<double, 3>> transformedSource = sourcePointsD;
        transformPointCloud(transformedSource, currentTransform);

        const double maxDistSq = config.maxCorrespondenceDistance *
            config.maxCorrespondenceDistance;
        double prevRMSE = std::numeric_limits<double>::max();

        /// =====================================================
        /// Point-to-Plane ICP
        /// =====================================================
        if (usePointToPlane) {
            std::vector<CorrespondenceFace> correspondences;

            /// 초기 RMSE 계산
            findCorrespondencesWithNormals(
                transformedSource, targetTree, chunk.faceIndices,
                faceNormalsD, facePtsD,
                maxDistSq, config.normalAngleThreshold, correspondences);
            result.initialRMSE = computeRMSE(correspondences);

            /// ICP 반복
            for (int iter = 0; iter < config.maxIterations; ++iter) {
                if (cancelRequested.load()) {
                    result.success = false;
                    result.errorMessage = "Cancelled";
                    return result;
                }

                /// 1. 대응점 탐색
                findCorrespondencesWithNormals(
                    transformedSource, targetTree, chunk.faceIndices,
                    faceNormalsD, facePtsD,
                    maxDistSq, config.normalAngleThreshold, correspondences);

                if (static_cast<int>(correspondences.size()) < config.minCorrespondences) {
                    result.success = false;
                    result.errorMessage = "Not enough correspondences: " +
                        std::to_string(correspondences.size());
                    result.actualIterations = iter;
                    return result;
                }

                /// 2. 이상치 제거
                rejectOutliersMAD(correspondences, config.outlierRejectionThreshold);

                if (static_cast<int>(correspondences.size()) < config.minCorrespondences) {
                    result.success = false;
                    result.errorMessage = "Not enough correspondences after outlier rejection";
                    result.actualIterations = iter;
                    return result;
                }

                /// 3. 변환 추정 (Point-to-Plane)
                Transform4x4 deltaT = estimateRigidTransformPointToPlane(
                    transformedSource, correspondences,
                    chunk.faceIndices, faceNormalsD);
#ifdef _DEBUG
                /// [ToDo]: config로 옮길 것
                std::array<double, uknw> params0 = { 0., 0., 0., 0., 0., 0. };
                double sdThr = 0.000001;
                unsigned int maxIter = 50;
                Transform4x4 deltaT2 = runLSM(
                    transformedSource, correspondences,
                    chunk.faceIndices, faceNormalsD,
                    params0, sdThr, maxIter);
#endif

#ifdef _DEBUG
                {
                    /// 이동 성분 추출
                    double tx = deltaT.m[12];
                    double ty = deltaT.m[13];
                    double tz = deltaT.m[14];

                    /// 회전 행렬로부터 오일러 각도 추출
                    /// R = Rz * Ry * Rx (ZYX 관례)
                    double r00 = deltaT.m[0], r01 = deltaT.m[4], r02 = deltaT.m[8];
                    double r10 = deltaT.m[1], r11 = deltaT.m[5], r12 = deltaT.m[9];
                    double r20 = deltaT.m[2], r21 = deltaT.m[6], r22 = deltaT.m[10];

                    double pitch = std::asin(-r20);  /// ry (Y축 회전)
                    double yaw, roll;

                    if (std::abs(r20) < 0.99999) {
                        yaw = std::atan2(r10, r00);   /// rz (Z축 회전)
                        roll = std::atan2(r21, r22);  /// rx (X축 회전)
                    }
                    else {
                        /// 짐벌락(gimbal lock) 상황
                        yaw = std::atan2(-r01, r11);
                        roll = 0.0;
                    }

                    /// 도(degree) 단위로 변환
                    const double rad2deg = 180.0 / 3.14159265;
                    roll *= rad2deg;
                    pitch *= rad2deg;
                    yaw *= rad2deg;

                    ILOG << "[ICP] Iter " << (iter + 1) << ": T(" << tx << ", " << ty << ", " << tz << ")" << " R(" << roll << ", " << pitch << ", " << yaw << ")deg";
                }
#endif

                /// 4. 변환 적용
                transformPointCloud(transformedSource, deltaT);
                accumulatedLocalTransform = deltaT * accumulatedLocalTransform;

                /// 5. RMSE 계산
                findCorrespondencesWithNormals(
                    transformedSource, targetTree, chunk.faceIndices,
                    faceNormalsD, facePtsD,
                    maxDistSq, config.normalAngleThreshold, correspondences);
                double currentRMSE = computeRMSE(correspondences);

                /// 6. 수렴 확인
                double transformChange = currentTransform.distanceTo(deltaT * currentTransform);
                currentTransform = deltaT * currentTransform;

                result.actualIterations = iter + 1;
                result.finalRMSE = currentRMSE;
                result.correspondenceCount = static_cast<int>(correspondences.size());

                if (std::abs(prevRMSE - currentRMSE) < config.convergenceThreshold ||
                    transformChange < config.convergenceThreshold) {
                    result.converged = true;
                    break;
                }

                ILOG << "[ICP] Iter " << (iter + 1) << ": Current RMSE (" << currentRMSE << ")deg";
                prevRMSE = currentRMSE;
            }
        }
        /// =====================================================
        /// 대체(Fallback) 방식: Point-to-Point ICP
        /// =====================================================
        else {
            std::vector<Correspondence> correspondences;

            /// 초기 RMSE 계산
            findCorrespondences(transformedSource, targetTree, maxDistSq, correspondences);
            result.initialRMSE = computeRMSE(correspondences);

            /// ICP 반복
            for (int iter = 0; iter < config.maxIterations; ++iter) {
                if (cancelRequested.load()) {
                    result.success = false;
                    result.errorMessage = "Cancelled";
                    return result;
                }

                /// 1. 대응점 탐색
                findCorrespondences(transformedSource, targetTree, maxDistSq, correspondences);

                if (static_cast<int>(correspondences.size()) < config.minCorrespondences) {
                    result.success = false;
                    result.errorMessage = "Not enough correspondences: " +
                        std::to_string(correspondences.size());
                    result.actualIterations = iter;
                    return result;
                }

                /// 2. 이상치 제거
                rejectOutliersMAD(correspondences, config.outlierRejectionThreshold);

                if (static_cast<int>(correspondences.size()) < config.minCorrespondences) {
                    result.success = false;
                    result.errorMessage = "Not enough correspondences after outlier rejection";
                    result.actualIterations = iter;
                    return result;
                }

                /// 3. 변환 추정 (Point-to-Point)
                Transform4x4 deltaT = estimateRigidTransformSVD(
                    transformedSource, targetPointsD, correspondences);

                /// 4. 변환 적용
                transformPointCloud(transformedSource, deltaT);
                accumulatedLocalTransform = deltaT * accumulatedLocalTransform;

                /// 5. RMSE 계산
                findCorrespondences(transformedSource, targetTree, maxDistSq, correspondences);
                double currentRMSE = computeRMSE(correspondences);

                /// 6. 수렴 확인
                double transformChange = currentTransform.distanceTo(deltaT * currentTransform);
                currentTransform = deltaT * currentTransform;

                result.actualIterations = iter + 1;
                result.finalRMSE = currentRMSE;
                result.correspondenceCount = static_cast<int>(correspondences.size());

                if (std::abs(prevRMSE - currentRMSE) < config.convergenceThreshold ||
                    transformChange < config.convergenceThreshold) {
                    result.converged = true;
                    break;
                }

                prevRMSE = currentRMSE;
            }
        }

        /// 최종 변환 결과 저장
        result.transform = currentTransform;
        result.localTransform = accumulatedLocalTransform;
        result.success = true;

        auto endTime = std::chrono::high_resolution_clock::now();
        result.processingTimeMs = std::chrono::duration<double, std::milli>(
            endTime - startTime).count();

        return result;
    }

    //=========================================================================
    // 결과 통합(Aggregation)
    //=========================================================================

    /// 여러 청크의 ICP 결과를 가중 평균(RMSE의 역수를 가중치로 사용)으로 통합한다.
    /// 주의: 이 함수는 "RMSE 역수 가중치" 방식이며, 실제 부재별 분산 미세정합
    /// 결과를 통합하는 MasterApplication_ICP.cpp의 aggregateWeightedTransforms는
    /// 이와 달리 "source 점 개수" 가중치를 사용하는 별도의 구현이다 -- 서로
    /// 혼동하지 말 것 (자세한 통합 방식 비교는 doc/handover/08_부재별_정합_통합_방법론.md 참고).
    /// @param results 여러 청크에서 얻은 ICP 결과 벡터
    /// @return 통합된 변환 행렬
    inline Transform4x4 aggregateResultsWeighted(const std::vector<IcpResult>& results)
    {
        if (results.empty()) {
            return Transform4x4::identity();
        }

        /// 성공한 결과만 필터링
        std::vector<const IcpResult*> validResults;
        for (const auto& r : results) {
            if (r.success && r.finalRMSE > 0.0) {
                validResults.push_back(&r);
            }
        }

        if (validResults.empty()) {
            return Transform4x4::identity();
        }

        if (validResults.size() == 1) {
            return validResults[0]->transform;
        }

        /// 가중치 계산 (RMSE의 역수)
        std::vector<double> weights;
        double totalWeight = 0.0;
        for (const auto* r : validResults) {
            double w = 1.0 / (r->finalRMSE + 1e-6);
            weights.push_back(w);
            totalWeight += w;
        }

        /// 변환 행렬들의 가중 평균
        Transform4x4 result;
        std::memset(result.m, 0, sizeof(result.m));

        for (size_t i = 0; i < validResults.size(); ++i) {
            double w = weights[i] / totalWeight;
            for (int j = 0; j < 16; ++j) {
                result.m[j] += w * validResults[i]->transform.m[j];
            }
        }

        /// 회전 성분을 SVD로 재직교화(re-orthogonalize) -- 행렬 원소별 가중 평균을
        /// 그대로 쓰면 회전 행렬의 직교성(orthogonality)이 깨지므로, 가장 가까운
        /// 회전 행렬로 재투영해 준다 (Orthogonal Procrustes 방식과 동일한 원리)
        Eigen::Matrix3d R;
        R << result.m[0], result.m[4], result.m[8],
            result.m[1], result.m[5], result.m[9],
            result.m[2], result.m[6], result.m[10];

        Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(R);
        R = svd.matrixU() * svd.matrixV().transpose();

        result.m[0] = R(0, 0); result.m[4] = R(0, 1); result.m[8] = R(0, 2);
        result.m[1] = R(1, 0); result.m[5] = R(1, 1); result.m[9] = R(1, 2);
        result.m[2] = R(2, 0); result.m[6] = R(2, 1); result.m[10] = R(2, 2);

        result.m[3] = result.m[7] = result.m[11] = 0.0;
        result.m[15] = 1.0;

        return result;
    }

    /// 가장 낮은 RMSE를 기준으로 최선의 결과를 선택
    /// @param results ICP 결과 벡터
    /// @return 최선의 결과에 대한 포인터, 유효한 결과가 없으면 nullptr
    inline const IcpResult* selectBestResult(const std::vector<IcpResult>& results)
    {
        const IcpResult* best = nullptr;
        double bestRMSE = std::numeric_limits<double>::max();

        for (const auto& r : results) {
            if (r.success && r.finalRMSE < bestRMSE) {
                best = &r;
                bestRMSE = r.finalRMSE;
            }
        }

        return best;
    }

} // namespace icp
