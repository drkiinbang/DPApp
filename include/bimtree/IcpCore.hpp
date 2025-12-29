#pragma once

/// ============================================================================
/// IcpCore.hpp - ICP Core Algorithm Implementation
/// ============================================================================
/// 
/// This header provides the core ICP algorithm components:
/// - Correspondence finding using KD-Tree (nanoflann)
/// - Outlier rejection (MAD-based)
/// - Rigid transformation estimation using SVD (Eigen)
/// - RMSE calculation
/// - Point cloud transformation and downsampling
/// - Main ICP iteration loop
/// 
/// Location: F:\repository\DPApp\include\bimtree\IcpCore.hpp
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

const double Rad2Sec = ((180.*3600.)/acos(-1.0));

namespace icp {

    //=========================================================================
    // Point Cloud Adapter for nanoflann KD-Tree
    //=========================================================================

    /// Adapter class for using std::vector<std::array<float,3>> with nanoflann
    struct PointCloudAdaptor {
        const std::vector<std::array<float, 3>>& points;

        explicit PointCloudAdaptor(const std::vector<std::array<float, 3>>& pts)
            : points(pts) {
        }

        /// Number of points
        inline size_t kdtree_get_point_count() const {
            return points.size();
        }

        /// Squared distance between point p1 and point at index idx_p2
        inline float kdtree_distance(const float* p1, const size_t idx_p2, size_t /*size*/) const {
            const float d0 = p1[0] - points[idx_p2][0];
            const float d1 = p1[1] - points[idx_p2][1];
            const float d2 = p1[2] - points[idx_p2][2];
            return d0 * d0 + d1 * d1 + d2 * d2;
        }

        /// Get dim-th component of point at index idx
        inline float kdtree_get_pt(const size_t idx, int dim) const {
            return points[idx][dim];
        }

        /// Optional bounding box computation (return false to use default)
        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /*bb*/) const {
            return false;
        }
    };

    /// KD-Tree type definition using nanoflann
    using KdTree3D = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>,
        PointCloudAdaptor,
        3,      // dimensions
        size_t  // index type
    >;

    //=========================================================================
    // Correspondence Structure
    //=========================================================================

    /// Correspondence pair (source index, target index, squared distance)
    struct Correspondence {
        size_t sourceIdx;
        size_t targetIdx;
        float squaredDistance;
    };

    struct CorrespondenceFace {
        size_t sourceIdx;
        std::array<float, 3> facePt;
        float squaredDistance;
    };

    //=========================================================================
    // Correspondence Finding
    //=========================================================================

    /// Find nearest neighbor correspondences using KD-Tree
    /// @param sourcePoints Source point cloud (to be transformed)
    /// @param targetTree KD-Tree built from target points
    /// @param maxDistanceSquared Maximum squared distance for valid correspondence
    /// @param correspondences Output vector of correspondences
    inline void findCorrespondences(
        const std::vector<std::array<float, 3>>& sourcePoints,
        const KdTree3D& targetTree,
        float maxDistanceSquared,
        std::vector<Correspondence>& correspondences)
    {
        correspondences.clear();
        correspondences.reserve(sourcePoints.size());

        for (size_t i = 0; i < sourcePoints.size(); ++i) {
            size_t resultIndex = 0;
            float resultDistSq = 0.0f;

            /// Find single nearest neighbor
            nanoflann::KNNResultSet<float> resultSet(1);
            resultSet.init(&resultIndex, &resultDistSq);

            targetTree.findNeighbors(resultSet, sourcePoints[i].data(),
                nanoflann::SearchParams());

            /// Check distance threshold
            if (resultDistSq <= maxDistanceSquared) {
                Correspondence corr;
                corr.sourceIdx = i;
                corr.targetIdx = resultIndex;
                corr.squaredDistance = resultDistSq;
                correspondences.push_back(corr);
            }
        }
    }

    /// Find correspondences with normal filtering
    /// @param sourcePoints Source point cloud
    /// @param targetTree KD-Tree built from target points
    /// @param targetNormals Normals at target points
    /// @param sourceNormals Normals at source points (can be empty)
    /// @param maxDistanceSquared Maximum squared distance
    /// @param maxAngleDeg Maximum angle between normals (degrees)
    /// @param correspondences Output correspondences
    inline void findCorrespondencesWithNormals(
        const std::vector<std::array<float, 3>>& sourcePoints,
        const KdTree3D& targetTree,
        const std::vector<size_t>& faceIdx,
        const std::vector<std::array<float, 3>>& allTargetNormals,
        const std::vector<std::array<float, 3>>& sourceNormals,
        float maxDistanceSquared,
        float maxAngleDeg,
        std::vector<Correspondence>& correspondences)
    {
        std::vector<std::array<float, 3>> targetNormals(faceIdx.size());
        for (size_t i = 0; i < faceIdx.size(); ++i) {
            targetNormals[i] = allTargetNormals[faceIdx[i]];
        }

        correspondences.clear();
        correspondences.reserve(sourcePoints.size());

        const float cosMaxAngle = std::cos(maxAngleDeg * 3.14159265f / 180.0f);
        const bool hasSourceNormals = !sourceNormals.empty() &&
            sourceNormals.size() == sourcePoints.size();

        bool  NormalFilter = false;
        if (!targetNormals.empty() && hasSourceNormals)
            NormalFilter = true;

        for (size_t i = 0; i < sourcePoints.size(); ++i) {
            size_t resultIndex = 0;
            float resultDistSq = 0.0f;

            nanoflann::KNNResultSet<float> resultSet(1);
            resultSet.init(&resultIndex, &resultDistSq);

            targetTree.findNeighbors(resultSet, sourcePoints[i].data(),
                nanoflann::SearchParams());

            if (resultDistSq > maxDistanceSquared) continue;

            /// Normal filtering (if normals available)
            if (NormalFilter && resultIndex < targetNormals.size()) {
                const auto& tn = targetNormals[resultIndex];
                const auto& sn = sourceNormals[i];
                /// Dot product of normals (absolute value for bidirectional)
                float dot = std::abs(sn[0] * tn[0] + sn[1] * tn[1] + sn[2] * tn[2]);
                if (dot < cosMaxAngle) continue;
            }

            Correspondence corr;
            corr.sourceIdx = i;
            corr.targetIdx = resultIndex;
            corr.squaredDistance = resultDistSq;
            correspondences.push_back(corr);
        }
    }

    inline bool pointPlaneDistanceAndProjection(
        const std::array<float, 3>& P,
        const std::array<float, 3>& Q,
        const std::array<float, 3>& N,
        double& distance,
        std::array<float, 3>& proj)
    {
        constexpr double EPSILON_SQ = 1e-12;
        double nlen2 = static_cast<double>(N[0]) * N[0] +
            static_cast<double>(N[1]) * N[1] +
            static_cast<double>(N[2]) * N[2];
        if (nlen2 < EPSILON_SQ)
            return false;

        double PQx = static_cast<double>(P[0]) - Q[0];
        double PQy = static_cast<double>(P[1]) - Q[1];
        double PQz = static_cast<double>(P[2]) - Q[2];

        double pqn_dot = PQx * N[0] + PQy * N[1] + PQz * N[2];

        distance = pqn_dot / std::sqrt(nlen2);

        double t = pqn_dot / nlen2;
        proj[0] = static_cast<float>(P[0] - N[0] * t);
        proj[1] = static_cast<float>(P[1] - N[1] * t);
        proj[2] = static_cast<float>(P[2] - N[2] * t);
        return true;
    }

    inline void findCorrespondencesWithNormals(
        const std::vector<std::array<float, 3>>& sourcePoints,
        const KdTree3D& targetTree,
        const std::vector<size_t>& faceIdx,
        const std::vector<std::array<float, 3>>& allTargetNormals,
        const std::vector<std::array<float, 3>>& allFacePts,
        float maxDistanceSquared,
        float maxAngleDeg,
        std::vector<CorrespondenceFace>& correspondences)
    {
        correspondences.clear();
        correspondences.reserve(sourcePoints.size());

        /// Build point-wise arrays from face indices
        std::vector<std::array<float, 3>> targetNormals(faceIdx.size());
        std::vector<std::array<float, 3>> facePts(faceIdx.size());
        for (size_t i = 0; i < faceIdx.size(); ++i) {
            targetNormals[i] = allTargetNormals[faceIdx[i]];
            facePts[i] = allFacePts[faceIdx[i]];
        }

        for (size_t i = 0; i < sourcePoints.size(); ++i) {
            size_t resultIndex = 0;
            float resultDistSq = 0.0f;

            nanoflann::KNNResultSet<float> resultSet(1);
            resultSet.init(&resultIndex, &resultDistSq);
            targetTree.findNeighbors(resultSet, sourcePoints[i].data(),
                nanoflann::SearchParams());

            if (resultDistSq > maxDistanceSquared) continue;

            double distance;
            std::array<float, 3> proj;

            /// Point-to-Plane distance and projection
            if (!pointPlaneDistanceAndProjection(
                sourcePoints[i],           // P: source point
                facePts[resultIndex],      // Q: point on face
                targetNormals[resultIndex], // N: face normal
                distance,
                proj))
                continue;

            CorrespondenceFace corr;
            corr.sourceIdx = i;
            corr.facePt = proj;
            corr.squaredDistance = static_cast<float>(distance * distance);
            correspondences.push_back(corr);
        }
    }

    //=========================================================================
    // Outlier Rejection
    //=========================================================================

    /// Reject outliers using Median Absolute Deviation (MAD)
    /// @param correspondences Correspondences to filter (modified in-place)
    /// @param threshold Multiplier for MAD (typically 2.0 - 3.0)
    inline void rejectOutliersMAD(
        std::vector<Correspondence>& correspondences,
        float threshold)
    {
        if (correspondences.size() < 10) return;

        /// Extract distances
        std::vector<float> distances;
        distances.reserve(correspondences.size());
        for (const auto& c : correspondences) {
            distances.push_back(std::sqrt(c.squaredDistance));
        }

        /// Compute median
        std::vector<float> sortedDist = distances;
        std::sort(sortedDist.begin(), sortedDist.end());
        const float median = sortedDist[sortedDist.size() / 2];

        /// Compute MAD (Median Absolute Deviation)
        std::vector<float> absDevs;
        absDevs.reserve(distances.size());
        for (float d : distances) {
            absDevs.push_back(std::abs(d - median));
        }
        std::sort(absDevs.begin(), absDevs.end());
        const float mad = absDevs[absDevs.size() / 2];

        /// MAD to standard deviation approximation
        const float madScale = 1.4826f;
        const float sigma = mad * madScale;

        /// Reject outliers beyond threshold * sigma from median
        const float maxDist = median + threshold * sigma;
        const float maxDistSq = maxDist * maxDist;

        correspondences.erase(
            std::remove_if(correspondences.begin(), correspondences.end(),
                [maxDistSq](const Correspondence& c) {
                    return c.squaredDistance > maxDistSq;
                }),
            correspondences.end());
    }

    /// Reject outliers using percentile threshold
    /// @param correspondences Correspondences to filter
    /// @param percentile Keep only correspondences below this percentile (0-100)
    inline void rejectOutliersPercentile(
        std::vector<Correspondence>& correspondences,
        float percentile)
    {
        if (correspondences.size() < 10) return;

        std::vector<float> distances;
        distances.reserve(correspondences.size());
        for (const auto& c : correspondences) {
            distances.push_back(c.squaredDistance);
        }
        std::sort(distances.begin(), distances.end());

        size_t idx = static_cast<size_t>((percentile / 100.0f) * distances.size());
        idx = (std::min)(idx, distances.size() - 1);
        const float maxDistSq = distances[idx];

        correspondences.erase(
            std::remove_if(correspondences.begin(), correspondences.end(),
                [maxDistSq](const Correspondence& c) {
                    return c.squaredDistance > maxDistSq;
                }),
            correspondences.end());
    }

    //=========================================================================
    // Transformation Estimation using SVD
    //=========================================================================

    /// Estimate rigid transformation (rotation + translation) using SVD
    /// Finds T such that target ≈ R * source + t
    /// @param sourcePoints Source point cloud
    /// @param targetPoints Target point cloud
    /// @param correspondences Point correspondences
    /// @return 4x4 transformation matrix
    inline Transform4x4 estimateRigidTransformSVD(
        const std::vector<std::array<float, 3>>& sourcePoints,
        const std::vector<std::array<float, 3>>& targetPoints,
        const std::vector<Correspondence>& correspondences)
    {
        if (correspondences.size() < 3) {
            return Transform4x4::identity();
        }

        const size_t n = correspondences.size();

        /// Compute centroids
        Eigen::Vector3f srcCentroid = Eigen::Vector3f::Zero();
        Eigen::Vector3f tgtCentroid = Eigen::Vector3f::Zero();

        for (const auto& c : correspondences) {
            const auto& sp = sourcePoints[c.sourceIdx];
            const auto& tp = targetPoints[c.targetIdx];
            srcCentroid += Eigen::Vector3f(sp[0], sp[1], sp[2]);
            tgtCentroid += Eigen::Vector3f(tp[0], tp[1], tp[2]);
        }
        srcCentroid /= static_cast<float>(n);
        tgtCentroid /= static_cast<float>(n);

        /// Compute cross-covariance matrix H
        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();

        for (const auto& c : correspondences) {
            const auto& sp = sourcePoints[c.sourceIdx];
            const auto& tp = targetPoints[c.targetIdx];

            Eigen::Vector3f srcCentered(
                sp[0] - srcCentroid.x(),
                sp[1] - srcCentroid.y(),
                sp[2] - srcCentroid.z());

            Eigen::Vector3f tgtCentered(
                tp[0] - tgtCentroid.x(),
                tp[1] - tgtCentroid.y(),
                tp[2] - tgtCentroid.z());

            H += srcCentered * tgtCentered.transpose();
        }

        /// SVD decomposition: H = U * S * V^T
        Eigen::JacobiSVD<Eigen::Matrix3f, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(H);
        Eigen::Matrix3f U = svd.matrixU();
        Eigen::Matrix3f V = svd.matrixV();

        /// Rotation: R = V * U^T
        Eigen::Matrix3f R = V * U.transpose();

        /// Ensure proper rotation (det(R) = 1, not -1 for reflection)
        if (R.determinant() < 0) {
            V.col(2) *= -1;
            R = V * U.transpose();
        }

        /// Translation: t = tgtCentroid - R * srcCentroid
        Eigen::Vector3f t = tgtCentroid - R * srcCentroid;

        /// Build 4x4 transformation matrix
        float rotMat[9];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotMat[i * 3 + j] = R(i, j);
            }
        }

        return Transform4x4::fromRotationTranslation(rotMat, t.x(), t.y(), t.z());
    }

    //=========================================================================
    // RMSE Calculation
    //=========================================================================

    /// Compute Root Mean Square Error from correspondences
    /// @param correspondences Correspondences with squared distances
    /// @return RMSE value
    inline float computeRMSE(const std::vector<Correspondence>& correspondences)
    {
        if (correspondences.empty()) {
            return std::numeric_limits<float>::max();
        }

        double sum = 0.0;
        for (const auto& c : correspondences) {
            sum += c.squaredDistance;
        }
        return static_cast<float>(std::sqrt(sum / correspondences.size()));
    }

    /// Compute RMSE with transformation applied
    /// @param sourcePoints Source points (original, not transformed)
    /// @param targetPoints Target points
    /// @param correspondences Point correspondences
    /// @param transform Transformation to apply to source
    /// @return RMSE value after transformation
    inline float computeRMSE(
        const std::vector<std::array<float, 3>>& sourcePoints,
        const std::vector<std::array<float, 3>>& targetPoints,
        const std::vector<Correspondence>& correspondences,
        const Transform4x4& transform)
    {
        if (correspondences.empty()) {
            return std::numeric_limits<float>::max();
        }

        double sumSquaredError = 0.0;

        for (const auto& c : correspondences) {
            const auto& sp = sourcePoints[c.sourceIdx];
            const auto& tp = targetPoints[c.targetIdx];

            /// Transform source point
            float tx, ty, tz;
            transform.transformPoint(sp[0], sp[1], sp[2], tx, ty, tz);

            /// Compute squared distance
            const float dx = tx - tp[0];
            const float dy = ty - tp[1];
            const float dz = tz - tp[2];
            sumSquaredError += dx * dx + dy * dy + dz * dz;
        }

        return static_cast<float>(std::sqrt(sumSquaredError / correspondences.size()));
    }

    //=========================================================================
    // Point Cloud Transformation
    //=========================================================================

    /// Apply transformation to point cloud in-place
    inline void transformPointCloud(
        std::vector<std::array<float, 3>>& points,
        const Transform4x4& transform)
    {
        for (auto& p : points) {
            transform.transformPointInPlace(p[0], p[1], p[2]);
        }
    }

    /// Apply transformation to point cloud (copy version)
    inline std::vector<std::array<float, 3>> transformPointCloudCopy(
        const std::vector<std::array<float, 3>>& points,
        const Transform4x4& transform)
    {
        std::vector<std::array<float, 3>> result;
        result.reserve(points.size());

        for (const auto& p : points) {
            float tx, ty, tz;
            transform.transformPoint(p[0], p[1], p[2], tx, ty, tz);
            result.push_back({ tx, ty, tz });
        }
        return result;
    }

    //=========================================================================
    // Downsampling
    //=========================================================================

    /// Downsample point cloud by uniform selection (every N-th point)
    inline bool downsampleUniform(
        const std::vector<std::array<float, 3>>& points,
        const std::vector<size_t>& faceIdx,
        std::vector<std::array<float, 3>>& downPts,
        std::vector<size_t>& downFaceIdx,
        int ratio)
    {
        if (ratio <= 1) return false;

        auto numSamples = points.size() / ratio + 1;
        downPts.reserve(numSamples);
        downFaceIdx.reserve(numSamples);

        for (size_t i = 0; i < points.size(); i += ratio) {
            downPts.push_back(points[i]);
            downFaceIdx.push_back(faceIdx[i]);
        }

        return true;
    }

    /// Downsample point cloud by uniform selection (every N-th point)
    inline std::vector<std::array<float, 3>> downsampleUniform(
        const std::vector<std::array<float, 3>>& points,
        int ratio)
    {
        std::vector<std::array<float, 3>> downPts;

        if (ratio <= 1) return downPts;

        auto numSamples = points.size() / ratio + 1;
        downPts.reserve(numSamples);

        for (size_t i = 0; i < points.size(); i += ratio) {
            downPts.push_back(points[i]);
        }

        return downPts;
    }

    /// Downsample point cloud using voxel grid
    /// @param points Input point cloud
    /// @param voxelSize Size of each voxel
    /// @return Downsampled point cloud (voxel centroids)
    inline std::vector<std::array<float, 3>> downsampleVoxelGrid(
        const std::vector<std::array<float, 3>>& points,
        float voxelSize)
    {
        if (points.empty() || voxelSize <= 0.0f) return points;

        /// Find bounding box
        float minX = points[0][0], maxX = points[0][0];
        float minY = points[0][1], maxY = points[0][1];
        float minZ = points[0][2], maxZ = points[0][2];

        for (const auto& p : points) {
            if (p[0] < minX) minX = p[0]; if (p[0] > maxX) maxX = p[0];
            if (p[1] < minY) minY = p[1]; if (p[1] > maxY) maxY = p[1];
            if (p[2] < minZ) minZ = p[2]; if (p[2] > maxZ) maxZ = p[2];
        }

        /// Voxel grid dimensions
        const int nx = static_cast<int>((maxX - minX) / voxelSize) + 1;
        const int ny = static_cast<int>((maxY - minY) / voxelSize) + 1;

        /// Use map for sparse voxel storage
        /// Key: voxel index, Value: (sum of points, count)
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

        /// Extract centroids
        std::vector<std::array<float, 3>> result;
        result.reserve(voxelMap.size());

        for (const auto& kv : voxelMap) {
            const auto& sum = kv.second.first;
            const size_t count = kv.second.second;
            result.push_back({
                static_cast<float>(sum[0] / count),
                static_cast<float>(sum[1] / count),
                static_cast<float>(sum[2] / count)
                });
        }

        return result;
    }

    //=========================================================================
    // Outlier Rejection for CorrespondenceFace
    //=========================================================================

    /// Reject outliers using MAD for CorrespondenceFace (Point-to-Plane)
    inline void rejectOutliersMAD(
        std::vector<CorrespondenceFace>& correspondences,
        float threshold)
    {
        if (correspondences.size() < 10) return;

        std::vector<float> distances;
        distances.reserve(correspondences.size());
        for (const auto& c : correspondences) {
            distances.push_back(std::sqrt(c.squaredDistance));
        }

        std::vector<float> sortedDist = distances;
        std::sort(sortedDist.begin(), sortedDist.end());
        const float median = sortedDist[sortedDist.size() / 2];

        std::vector<float> absDevs;
        absDevs.reserve(distances.size());
        for (float d : distances) {
            absDevs.push_back(std::abs(d - median));
        }
        std::sort(absDevs.begin(), absDevs.end());
        const float mad = absDevs[absDevs.size() / 2];

        const float madScale = 1.4826f;
        const float sigma = mad * madScale;
        const float maxDist = median + threshold * sigma;
        const float maxDistSq = maxDist * maxDist;

        correspondences.erase(
            std::remove_if(correspondences.begin(), correspondences.end(),
                [maxDistSq](const CorrespondenceFace& c) {
                    return c.squaredDistance > maxDistSq;
                }),
            correspondences.end());
    }

    //=========================================================================
    // RMSE Calculation for CorrespondenceFace
    //=========================================================================

    /// Compute RMSE from CorrespondenceFace (Point-to-Plane distance)
    inline float computeRMSE(const std::vector<CorrespondenceFace>& correspondences)
    {
        if (correspondences.empty()) {
            return std::numeric_limits<float>::max();
        }

        double sum = 0.0;
        for (const auto& c : correspondences) {
            sum += c.squaredDistance;
        }
        return static_cast<float>(std::sqrt(sum / correspondences.size()));
    }

    //=========================================================================
    // Point-to-Plane Transformation Estimation
    //=========================================================================

    /// Estimate rigid transformation using Point-to-Plane linearized method
    inline Transform4x4 estimateRigidTransformPointToPlane(
        const std::vector<std::array<float, 3>>& sourcePoints,
        const std::vector<CorrespondenceFace>& correspondences,
        const std::vector<size_t>& faceIdx,
        const std::vector<std::array<float, 3>>& allFaceNormals)
    {
        if (correspondences.size() < 6) {
            return Transform4x4::identity();
        }

        const size_t n = correspondences.size();

        Eigen::MatrixXf A(n, 6);
        Eigen::VectorXf b(n);

        for (size_t i = 0; i < n; ++i) {
            const auto& corr = correspondences[i];
            const auto& s = sourcePoints[corr.sourceIdx];
            const auto& p = corr.facePt;

            float dx = s[0] - p[0];
            float dy = s[1] - p[1];
            float dz = s[2] - p[2];
            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            float nx, ny, nz;
            if (dist > 1e-10f) {
                nx = dx / dist;
                ny = dy / dist;
                nz = dz / dist;
            }
            else {
                nx = 0; ny = 0; nz = 1;
            }

            float sx = s[0], sy = s[1], sz = s[2];

            A(i, 0) = nx;
            A(i, 1) = ny;
            A(i, 2) = nz;
            A(i, 3) = sy * nz - sz * ny;
            A(i, 4) = sz * nx - sx * nz;
            A(i, 5) = sx * ny - sy * nx;

            b(i) = nx * (p[0] - sx) + ny * (p[1] - sy) + nz * (p[2] - sz);
        }

        /// Solve using SVD least squares (BDCSVD for newer Eigen)
        Eigen::BDCSVD<Eigen::MatrixXf, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(A);
        Eigen::VectorXf x = svd.solve(b);

        float tx = x(0), ty = x(1), tz = x(2);
        float rx = x(3), ry = x(4), rz = x(5);

        float cx = std::cos(rx), sx_r = std::sin(rx);
        float cy = std::cos(ry), sy = std::sin(ry);
        float cz = std::cos(rz), sz = std::sin(rz);

        Eigen::Matrix3f R;
        R(0, 0) = cy * cz;
        R(0, 1) = cz * sx_r * sy - cx * sz;
        R(0, 2) = sx_r * sz + cx * cz * sy;
        R(1, 0) = cy * sz;
        R(1, 1) = cx * cz + sx_r * sy * sz;
        R(1, 2) = cx * sy * sz - cz * sx_r;
        R(2, 0) = -sy;
        R(2, 1) = cy * sx_r;
        R(2, 2) = cx * cy;

        float rotMat[9];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotMat[i * 3 + j] = R(i, j);
            }
        }

        return Transform4x4::fromRotationTranslation(rotMat, tx, ty, tz);
    }

    inline void calDiscrepancy(const std::array<float, 3>& src, const std::array<float, 3>& tar, const std::array<double, 6>& params, math::Matrixd& ai, math::Matrixd& yi)
    {
        math::Matrixd GA(3, 1), GB(3, 1);

        GA(0, 0) = src[0];
        GA(1, 0) = src[1];
        GA(2, 0) = src[2];

        GB(0, 0) = src[0];
        GB(1, 0) = src[1];
        GB(2, 0) = src[2];

        auto R = math::getRMat(params[3], params[3], params[5]);
        auto RGA = R % GA;

        auto dRdO = math::getPartial_dRdO(params[3], params[3], params[5]);
        auto Partial_O = dRdO% GA;

        auto dRdP = math::getPartial_dRdP(params[3], params[3], params[5]);
        auto Partial_P = dRdP % GA;

        auto dRdK = math::getPartial_dRdK(params[3], params[3], params[5]);
        auto Partial_K = dRdK % GA;

        math::Matrixd T(3, 1);

        T(0, 0) = params[0];
        T(1, 0) = params[1];
        T(2, 0) = params[2];

        //dTx, dTy, dTz, dO, dP, dK, and dS
        ai(0, 0) = 1;
        ai(0, 1) = 0;
        ai(0, 2) = 0;
        ai(0, 3) = Partial_O(0, 0) / Rad2Sec;
        ai(0, 4) = Partial_P(0, 0) / Rad2Sec;
        ai(0, 5) = Partial_K(0, 0) / Rad2Sec;

        ai(1, 0) = 0;
        ai(1, 1) = 1;
        ai(1, 2) = 0;
        ai(1, 3) = Partial_O(1, 0) / Rad2Sec;
        ai(1, 4) = Partial_P(1, 0) / Rad2Sec;
        ai(1, 5) = Partial_K(1, 0) / Rad2Sec;

        ai(2, 0) = 0;
        ai(2, 1) = 0;
        ai(2, 2) = 1;
        ai(2, 3) = Partial_O(2, 0) / Rad2Sec;
        ai(2, 4) = Partial_P(2, 0) / Rad2Sec;
        ai(2, 5) = Partial_K(2, 0) / Rad2Sec;

        yi = GB - RGA - T;
    }

    /// [ToDo]
    inline Transform4x4 estimateRigidTransformPointToPlane_NLLS(
        const std::vector<std::array<float, 3>>& sourcePoints,
        const std::vector<CorrespondenceFace>& correspondences,
        const std::vector<size_t>& faceIdx,
        const std::vector<std::array<float, 3>>& allFaceNormals,
        const std::array<double, 6>& params0)
    {
        if (correspondences.size() < 6) {
            return Transform4x4::identity();
        }

        const size_t n = correspondences.size();

        Eigen::MatrixXf A(n, 6);
        Eigen::VectorXf b(n);

        auto params = params0;

        math::Matrixd N(6, 6, 0.);
        math::Matrixd C(6, 1, 0.);
        math::Matrixd ai(1, 6);
        math::Matrixd yi(1, 1);
        const size_t numEqs = n;
        double ytpy = 0.;

        for (size_t i = 0; i < n; ++i) {
            const auto& corr = correspondences[i];
            const auto& s = sourcePoints[corr.sourceIdx];
            const auto& p = corr.facePt;

            calDiscrepancy(s, p, params, ai, yi);

            auto ait = ai.transpose();
            auto ci = ait % yi;
            auto ni = ait % ai;

            ytpy += (yi.transpose() % yi)(0, 0);

            N += ni;
            C += ci;
        }

        auto Ninv = N.inverse();

        math:: Matrixd Correlation(6, 6, 0.0);
        for (unsigned int i = 0; i < 6; i++)
        {
            for (unsigned int j = 0; j < 6; j++)
            {
                Correlation(i, j) = Ninv(i, j) / sqrt(Ninv(i, i)) / sqrt(Ninv(j, j));
            }
        }

        auto X = Ninv % C;

        for (unsigned int i = 0; i < 6; i++) {
            if (i > 2)
                params[i] += X(i, 0) / Rad2Sec;
            else
                params[i] += X(i, 0);
        }

        auto new_S = sqrt(ytpy / (numEqs - 6));

        float tx = params[0], ty = params[1], tz = params[2];
        float rx = params[3], ry = params[3], rz = params[5];

        float cx = std::cos(rx), sx_r = std::sin(rx);
        float cy = std::cos(ry), sy = std::sin(ry);
        float cz = std::cos(rz), sz = std::sin(rz);

        Eigen::Matrix3f R;
        R(0, 0) = cy * cz;
        R(0, 1) = cz * sx_r * sy - cx * sz;
        R(0, 2) = sx_r * sz + cx * cz * sy;
        R(1, 0) = cy * sz;
        R(1, 1) = cx * cz + sx_r * sy * sz;
        R(1, 2) = cx * sy * sz - cz * sx_r;
        R(2, 0) = -sy;
        R(2, 1) = cy * sx_r;
        R(2, 2) = cx * cy;

        float rotMat[9];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotMat[i * 3 + j] = R(i, j);
            }
        }

        return Transform4x4::fromRotationTranslation(rotMat, tx, ty, tz);
    }

    //=========================================================================
    // ICP Main Algorithm
    //=========================================================================

    /// Run ICP algorithm (Point-to-Plane or Point-to-Point)
    /// @param chunk ICP chunk containing source/target points and config
    /// @param cancelRequested Atomic flag for cancellation check
    /// @return ICP result with transformation and statistics
    inline IcpResult runIcp(
        const IcpChunk& chunk,
        std::atomic<bool>& cancelRequested)
    {
        IcpResult result;
        result.chunk_id = chunk.chunk_id;
        result.sourcePointCount = static_cast<int>(chunk.sourcePoints.size());
        result.targetPointCount = static_cast<int>(chunk.targetPoints.size());

        auto startTime = std::chrono::high_resolution_clock::now();

        /// Validate input
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

        const auto& config = chunk.config;

        /// Check if Point-to-Plane mode is available
        const bool usePointToPlane = !chunk.faceIndices.empty() &&
            !chunk.faceNormals.empty() &&
            !chunk.facePts.empty();

        /// Build KD-Tree for target points
        PointCloudAdaptor targetAdaptor(chunk.targetPoints);
        KdTree3D targetTree(3, targetAdaptor,
            nanoflann::KDTreeSingleIndexAdaptorParams(50));
        targetTree.buildIndex();

        /// Initialize transformation
        Transform4x4 currentTransform = chunk.initialTransform;
        Transform4x4 accumulatedLocalTransform = Transform4x4::identity();

        /// Copy source points for transformation
        std::vector<std::array<float, 3>> transformedSource = chunk.sourcePoints;
        transformPointCloud(transformedSource, currentTransform);

        const float maxDistSq = config.maxCorrespondenceDistance *
            config.maxCorrespondenceDistance;
        float prevRMSE = std::numeric_limits<float>::max();

        /// =====================================================
        /// Point-to-Plane ICP
        /// =====================================================
        if (usePointToPlane) {
            std::vector<CorrespondenceFace> correspondences;

            /// Initial RMSE
            findCorrespondencesWithNormals(
                transformedSource, targetTree, chunk.faceIndices,
                chunk.faceNormals, chunk.facePts,
                maxDistSq, config.normalAngleThreshold, correspondences);
            result.initialRMSE = computeRMSE(correspondences);

            /// ICP iterations
            for (int iter = 0; iter < config.maxIterations; ++iter) {
                if (cancelRequested.load()) {
                    result.success = false;
                    result.errorMessage = "Cancelled";
                    return result;
                }

                /// 1. Find correspondences
                findCorrespondencesWithNormals(
                    transformedSource, targetTree, chunk.faceIndices,
                    chunk.faceNormals, chunk.facePts,
                    maxDistSq, config.normalAngleThreshold, correspondences);

                if (static_cast<int>(correspondences.size()) < config.minCorrespondences) {
                    result.success = false;
                    result.errorMessage = "Not enough correspondences: " +
                        std::to_string(correspondences.size());
                    result.actualIterations = iter;
                    return result;
                }

                /// 2. Reject outliers
                rejectOutliersMAD(correspondences, config.outlierRejectionThreshold);

                if (static_cast<int>(correspondences.size()) < config.minCorrespondences) {
                    result.success = false;
                    result.errorMessage = "Not enough correspondences after outlier rejection";
                    result.actualIterations = iter;
                    return result;
                }

                /// 3. Estimate transformation (Point-to-Plane)
                Transform4x4 deltaT = estimateRigidTransformPointToPlane(
                    transformedSource, correspondences,
                    chunk.faceIndices, chunk.faceNormals);

#ifdef _DEBUG
                {
                    /// Extract translation
                    float tx = deltaT.m[12];
                    float ty = deltaT.m[13];
                    float tz = deltaT.m[14];

                    /// Extract rotation angles from rotation matrix (Euler angles)
                    /// R = Rz * Ry * Rx (ZYX convention)
                    float r00 = deltaT.m[0], r01 = deltaT.m[4], r02 = deltaT.m[8];
                    float r10 = deltaT.m[1], r11 = deltaT.m[5], r12 = deltaT.m[9];
                    float r20 = deltaT.m[2], r21 = deltaT.m[6], r22 = deltaT.m[10];

                    float pitch = std::asin(-r20);  /// ry (Y rotation)
                    float yaw, roll;

                    if (std::abs(r20) < 0.99999f) {
                        yaw = std::atan2(r10, r00);   /// rz (Z rotation)
                        roll = std::atan2(r21, r22);  /// rx (X rotation)
                    }
                    else {
                        /// Gimbal lock
                        yaw = std::atan2(-r01, r11);
                        roll = 0.0f;
                    }

                    /// Convert to degrees
                    const float rad2deg = 180.0f / 3.14159265f;
                    roll *= rad2deg;
                    pitch *= rad2deg;
                    yaw *= rad2deg;

                    ILOG << "[ICP] Iter " << (iter + 1) << ": T(" << tx << ", " << ty << ", " << tz << ")" << " R(" << roll << ", " << pitch << ", " << yaw << ")deg";
                }
#endif

                /// 4. Apply transformation
                transformPointCloud(transformedSource, deltaT);
                accumulatedLocalTransform = deltaT * accumulatedLocalTransform;

                /// 5. Compute RMSE
                findCorrespondencesWithNormals(
                    transformedSource, targetTree, chunk.faceIndices,
                    chunk.faceNormals, chunk.facePts,
                    maxDistSq, config.normalAngleThreshold, correspondences);
                float currentRMSE = computeRMSE(correspondences);

                /// 6. Check convergence
                float transformChange = currentTransform.distanceTo(deltaT * currentTransform);
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
        /// Fallback: Point-to-Point ICP
        /// =====================================================
        else {
            std::vector<Correspondence> correspondences;

            /// Initial RMSE
            findCorrespondences(transformedSource, targetTree, maxDistSq, correspondences);
            result.initialRMSE = computeRMSE(correspondences);

            /// ICP iterations
            for (int iter = 0; iter < config.maxIterations; ++iter) {
                if (cancelRequested.load()) {
                    result.success = false;
                    result.errorMessage = "Cancelled";
                    return result;
                }

                /// 1. Find correspondences
                findCorrespondences(transformedSource, targetTree, maxDistSq, correspondences);

                if (static_cast<int>(correspondences.size()) < config.minCorrespondences) {
                    result.success = false;
                    result.errorMessage = "Not enough correspondences: " +
                        std::to_string(correspondences.size());
                    result.actualIterations = iter;
                    return result;
                }

                /// 2. Reject outliers
                rejectOutliersMAD(correspondences, config.outlierRejectionThreshold);

                if (static_cast<int>(correspondences.size()) < config.minCorrespondences) {
                    result.success = false;
                    result.errorMessage = "Not enough correspondences after outlier rejection";
                    result.actualIterations = iter;
                    return result;
                }

                /// 3. Estimate transformation (Point-to-Point)
                Transform4x4 deltaT = estimateRigidTransformSVD(
                    transformedSource, chunk.targetPoints, correspondences);

                /// 4. Apply transformation
                transformPointCloud(transformedSource, deltaT);
                accumulatedLocalTransform = deltaT * accumulatedLocalTransform;

                /// 5. Compute RMSE
                findCorrespondences(transformedSource, targetTree, maxDistSq, correspondences);
                float currentRMSE = computeRMSE(correspondences);

                /// 6. Check convergence
                float transformChange = currentTransform.distanceTo(deltaT * currentTransform);
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

        /// Store final transformation
        result.transform = currentTransform;
        result.localTransform = accumulatedLocalTransform;
        result.success = true;

        auto endTime = std::chrono::high_resolution_clock::now();
        result.processingTimeMs = std::chrono::duration<double, std::milli>(
            endTime - startTime).count();

        return result;
    }

    //=========================================================================
    // Result Aggregation
    //=========================================================================

    /// Aggregate multiple ICP results using weighted average (inverse RMSE)
    /// @param results Vector of ICP results from different chunks
    /// @return Aggregated transformation matrix
    inline Transform4x4 aggregateResultsWeighted(const std::vector<IcpResult>& results)
    {
        if (results.empty()) {
            return Transform4x4::identity();
        }

        /// Filter successful results
        std::vector<const IcpResult*> validResults;
        for (const auto& r : results) {
            if (r.success && r.finalRMSE > 0.0f) {
                validResults.push_back(&r);
            }
        }

        if (validResults.empty()) {
            return Transform4x4::identity();
        }

        if (validResults.size() == 1) {
            return validResults[0]->transform;
        }

        /// Compute weights (inverse RMSE)
        std::vector<float> weights;
        float totalWeight = 0.0f;
        for (const auto* r : validResults) {
            float w = 1.0f / (r->finalRMSE + 1e-6f);
            weights.push_back(w);
            totalWeight += w;
        }

        /// Weighted average of transformation matrices
        Transform4x4 result;
        std::memset(result.m, 0, sizeof(result.m));

        for (size_t i = 0; i < validResults.size(); ++i) {
            float w = weights[i] / totalWeight;
            for (int j = 0; j < 16; ++j) {
                result.m[j] += w * validResults[i]->transform.m[j];
            }
        }

        /// Re-orthogonalize rotation matrix using SVD
        Eigen::Matrix3f R;
        R << result.m[0], result.m[4], result.m[8],
            result.m[1], result.m[5], result.m[9],
            result.m[2], result.m[6], result.m[10];

        Eigen::JacobiSVD<Eigen::Matrix3f, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(R);
        R = svd.matrixU() * svd.matrixV().transpose();

        result.m[0] = R(0, 0); result.m[4] = R(0, 1); result.m[8] = R(0, 2);
        result.m[1] = R(1, 0); result.m[5] = R(1, 1); result.m[9] = R(1, 2);
        result.m[2] = R(2, 0); result.m[6] = R(2, 1); result.m[10] = R(2, 2);

        result.m[3] = result.m[7] = result.m[11] = 0.0f;
        result.m[15] = 1.0f;

        return result;
    }

    /// Select best result based on lowest RMSE
    /// @param results Vector of ICP results
    /// @return Pointer to best result, or nullptr if no valid results
    inline const IcpResult* selectBestResult(const std::vector<IcpResult>& results)
    {
        const IcpResult* best = nullptr;
        float bestRMSE = std::numeric_limits<float>::max();

        for (const auto& r : results) {
            if (r.success && r.finalRMSE < bestRMSE) {
                best = &r;
                bestRMSE = r.finalRMSE;
            }
        }

        return best;
    }

} // namespace icp