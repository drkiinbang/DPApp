#pragma once

/// ============================================================================
/// PseudoPointGenerator.hpp - 메시 표면으로부터 가상점(pseudo point) 생성
/// ============================================================================
///
/// 삼각형 메시 표면 위에 균일하게 분포된 점들을 생성한다.
/// ICP 정합의 target 점으로 사용된다.
///
/// 위치: D:\repository\DPApp_II\include\bimtree\PseudoPointGenerator.hpp
/// ============================================================================

#include "../bim/MeshChunk.h"
#include "../IcpTypes.h"

#include <vector>
#include <array>
#include <cmath>
#include <random>

namespace icp {

    /// 외적(cross product)을 이용해 삼각형 면적 계산
    inline double triangleArea(
        const pctree::XYZPoint& v0,
        const pctree::XYZPoint& v1,
        const pctree::XYZPoint& v2)
    {
        /// 변(edge) 벡터
        double e1x = v1[0] - v0[0];
        double e1y = v1[1] - v0[1];
        double e1z = v1[2] - v0[2];

        double e2x = v2[0] - v0[0];
        double e2y = v2[1] - v0[1];
        double e2z = v2[2] - v0[2];

        /// 외적
        double cx = e1y * e2z - e1z * e2y;
        double cy = e1z * e2x - e1x * e2z;
        double cz = e1x * e2y - e1y * e2x;

        /// 면적 = 0.5 * |외적|
        return 0.5 * std::sqrt(cx * cx + cy * cy + cz * cz);
    }

    /// 점 p에서 삼각형(a, b, c) 위 최근접점까지의 정확한 거리 제곱을 계산한다.
    /// 표준적인 point-to-triangle 최근접점 알고리즘(Ericson, "Real-Time Collision
    /// Detection")을 사용하며, 이를 통해 BIM-포인트클라우드 품질검사(QC) 거리가
    /// (가까운 가상점까지의 거리가 아니라) 실제 표면까지의 참(true) 거리를
    /// 반영하도록 한다 (후보 삼각형을 어떻게 고르는지는 아래
    /// pointToNearestPseudoPointCandidates() 참고).
    inline double pointToTriangleDistanceSquared(
        const std::array<double, 3>& p,
        const std::array<double, 3>& a,
        const std::array<double, 3>& b,
        const std::array<double, 3>& c)
    {
        auto sub = [](const std::array<double, 3>& x, const std::array<double, 3>& y) {
            return std::array<double, 3>{ x[0] - y[0], x[1] - y[1], x[2] - y[2] };
            };
        auto dot = [](const std::array<double, 3>& x, const std::array<double, 3>& y) {
            return x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
            };
        auto distSq = [&](const std::array<double, 3>& x) { return dot(x, x); };

        std::array<double, 3> ab = sub(b, a);
        std::array<double, 3> ac = sub(c, a);
        std::array<double, 3> ap = sub(p, a);

        double d1 = dot(ab, ap);
        double d2 = dot(ac, ap);
        if (d1 <= 0.0 && d2 <= 0.0) {
            return distSq(sub(p, a)); /// barycentric (1,0,0): 정점 a에 가장 가까움
        }

        std::array<double, 3> bp = sub(p, b);
        double d3 = dot(ab, bp);
        double d4 = dot(ac, bp);
        if (d3 >= 0.0 && d4 <= d3) {
            return distSq(sub(p, b)); /// 정점 b에 가장 가까움
        }

        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
            double v = d1 / (d1 - d3);
            std::array<double, 3> proj = { a[0] + v * ab[0], a[1] + v * ab[1], a[2] + v * ab[2] };
            return distSq(sub(p, proj)); /// 변 ab에 가장 가까움
        }

        std::array<double, 3> cp = sub(p, c);
        double d5 = dot(ab, cp);
        double d6 = dot(ac, cp);
        if (d6 >= 0.0 && d5 <= d6) {
            return distSq(sub(p, c)); /// 정점 c에 가장 가까움
        }

        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            double w = d2 / (d2 - d6);
            std::array<double, 3> proj = { a[0] + w * ac[0], a[1] + w * ac[1], a[2] + w * ac[2] };
            return distSq(sub(p, proj)); /// 변 ac에 가장 가까움
        }

        double va = d3 * d6 - d5 * d4;
        if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
            double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            std::array<double, 3> proj = { b[0] + w * (c[0] - b[0]), b[1] + w * (c[1] - b[1]), b[2] + w * (c[2] - b[2]) };
            return distSq(sub(p, proj)); /// 변 bc에 가장 가까움
        }

        /// 최근접점이 face 내부에 있는 경우
        double denom = 1.0 / (va + vb + vc);
        double v = vb * denom;
        double w = vc * denom;
        std::array<double, 3> proj = {
            a[0] + ab[0] * v + ac[0] * w,
            a[1] + ab[1] * v + ac[1] * w,
            a[2] + ab[2] * v + ac[2] * w
        };
        return distSq(sub(p, proj));
    }

    /// 하나의 MeshChunk로부터 가상점을 생성한다. `mesh` 자체는 (offset 이동 없이)
    /// 항상 자신의 절대좌표계로 읽히지만, 생성되는 outPoints/facePts는 기록될 때
    /// 선택적으로 (offsetX, offsetY, offsetZ)만큼 이동시킬 수 있다 -- 이는 IcpChunk를
    /// 채울 때 사용되며, 가상점들이 매칭 대상인 (offset-재배치된) 포인트클라우드와
    /// 동일한 이동된 좌표계에 놓이도록 하기 위함이다. 다른 호출자(예: 합성 테스트
    /// 데이터 생성기)에서는 기본값 0(이동 없음)을 사용한다. outNormals는 방향
    /// 벡터이므로 절대 이동시키지 않는다.
    inline void generatePseudoPointsFromMesh(
        const chunkbim::MeshChunk& mesh,
        double gridSize,
        std::vector<std::array<double, 3>>& outPoints,
        std::vector<size_t>& faceIdx,
        std::vector<std::array<double, 3>>& facePts,
        std::vector<std::array<double, 3>>& outNormals,
        double offsetX = 0.0, double offsetY = 0.0, double offsetZ = 0.0)
    {
        if (mesh.faces.empty())
            return;

        double gridSizeSquared = gridSize * gridSize;

        for (size_t f = 0; f < mesh.faces.size(); ++f) {
            const auto& face = mesh.faces[f];

            std::array<double, 3> fPt;
            fPt[0] = face.vertices[0][0] - offsetX;
            fPt[1] = face.vertices[0][1] - offsetY;
            fPt[2] = face.vertices[0][2] - offsetZ;

            facePts.push_back(fPt);

            /// face의 normal
            std::array<double, 3> fNormal;
            fNormal[0] = face.normal[0];
            fNormal[1] = face.normal[1];
            fNormal[2] = face.normal[2];

            outNormals.push_back(fNormal);

            const auto& v0 = face.vertices[0];
            const auto& v1 = face.vertices[1];
            const auto& v2 = face.vertices[2];

            /// 삼각형 면적 계산
            double area = triangleArea(v0, v1, v2);
            if (area < 1e-10) continue;  /// 퇴화(degenerate) 삼각형은 건너뜀

            /// 면적과 격자 간격을 기준으로 생성할 점 개수 결정
            int numPoints = static_cast<int>(std::ceil(area / gridSizeSquared));
            numPoints = (std::max)(1, (std::min)(numPoints, 1000));  /// 범위 제한

            /// barycentric 좌표를 이용해 점 생성
            /// 재현 가능성을 위해 결정론적(deterministic) 패턴 사용 (난수 아님)
            int gridDim = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(numPoints))));

            for (int i = 0; i < gridDim; ++i) {
                for (int j = 0; j < gridDim - i; ++j) {
                    /// barycentric 좌표
                    double u = (i + 0.5) / gridDim;
                    double v = (j + 0.5) / gridDim;
                    double w = 1.0 - u - v;

                    if (w < 0.0) continue;  /// 삼각형 바깥이므로 제외

                    /// 위치 보간
                    std::array<double, 3> point;
                    point[0] = u * v0[0] + v * v1[0] + w * v2[0] - offsetX;
                    point[1] = u * v0[1] + v * v1[1] + w * v2[1] - offsetY;
                    point[2] = u * v0[2] + v * v1[2] + w * v2[2] - offsetZ;

                    outPoints.push_back(point);
                    faceIdx.push_back(f);
                }
            }
        }
    }

    /// 여러 MeshChunk로부터 가상점 생성
    inline void generatePseudoPoints(
        const std::vector<chunkbim::MeshChunk>& chunks,
        double gridSize,
        std::vector<std::array<double, 3>>& outPoints,
        std::vector<size_t>& faceIdx,
        std::vector<std::array<double, 3>>& facePts,
        std::vector<std::array<double, 3>>& outNormals,
        double offsetX = 0.0, double offsetY = 0.0, double offsetZ = 0.0)
    {
        outPoints.clear();
        faceIdx.clear();
        facePts.clear();
        outNormals.clear();

        /// 예약할 전체 점 개수 추정
        size_t totalFaces = 0;
        for (const auto& chunk : chunks) {
            totalFaces += chunk.faces.size();
        }

        outPoints.reserve(totalFaces * 4);  /// 대략적인 추정치
        faceIdx.reserve(totalFaces * 4);  /// 대략적인 추정치
        facePts.reserve(totalFaces);
        outNormals.reserve(totalFaces);

        /// 각 메시로부터 생성
        for (const auto& chunk : chunks) {
            generatePseudoPointsFromMesh(chunk, gridSize, outPoints, faceIdx, facePts, outNormals,
                offsetX, offsetY, offsetZ);
        }

        outPoints.shrink_to_fit();
        faceIdx.shrink_to_fit();
    }

    /// PointCloudChunk를 ICP 포맷으로 변환
    inline std::vector<std::array<double, 3>> convertPointCloudToIcp(
        const std::vector<chunkpc::PointCloudChunk>& chunks)
    {
        std::vector<std::array<double, 3>> result;

        /// 전체 점 개수 계산
        size_t totalPoints = 0;
        for (const auto& chunk : chunks) {
            totalPoints += chunk.points.size();
        }
        result.reserve(totalPoints);

        /// 변환
        for (const auto& chunk : chunks) {
            for (const auto& pt : chunk.points) {
                result.push_back({ pt[0], pt[1], pt[2] });
            }
        }

        return result;
    }

    /// 메시들의 전체 삼각형 개수 계산
    inline size_t countTotalTriangles(const std::vector<chunkbim::MeshChunk>& meshes) {
        size_t total = 0;
        for (const auto& mesh : meshes) {
            total += mesh.faces.size();
        }
        return total;
    }

    /// BIM 부재(element) 집합에 대한 재배치(rebase) offset을 계산한다: 전체를
    /// 포괄하는 바운딩 박스의 중심점. 포인트클라우드와 가상점 좌표를 float로
    /// 저장/전송하기 전에 이 값만큼 이동시켜, 플랜트의 절대 현장 좌표가 아무리
    /// 크더라도 두 좌표 모두 원점 부근에 머물도록(따라서 float로도 밀리미터
    /// 이하 정밀도가 유지되도록) 하는 double 정밀도 "원점" 역할을 한다.
    /// 각 부재의 bounds는 미리 최신 상태여야 한다(calculateBounds()가 호출된 상태).
    inline std::array<double, 3> computeBimRebaseOffset(const std::vector<chunkbim::MeshChunk>& elements) {
        if (elements.empty()) return { 0.0, 0.0, 0.0 };

        double min_x = elements[0].min_x, min_y = elements[0].min_y, min_z = elements[0].min_z;
        double max_x = elements[0].max_x, max_y = elements[0].max_y, max_z = elements[0].max_z;

        for (const auto& e : elements) {
            min_x = (std::min)(min_x, e.min_x); max_x = (std::max)(max_x, e.max_x);
            min_y = (std::min)(min_y, e.min_y); max_y = (std::max)(max_y, e.max_y);
            min_z = (std::min)(min_z, e.min_z); max_z = (std::max)(max_z, e.max_z);
        }

        return { (min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5 };
    }

} // namespace icp
