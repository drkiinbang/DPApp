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

const double Rad2Sec = ((180. * 3600.) / acos(-1.0));
const double Rad2Deg = ((180.) / acos(-1.0));
const double minParamSd = 1.0e-9;
const double maxParamSd = 1.0e+9;
const unsigned int uknw = 6;

namespace icp {

    //=========================================================================
    // Point Cloud Adapter for nanoflann KD-Tree
    //=========================================================================

    /// Adapter class for using std::vector<std::array<double,3>> with nanoflann
    struct PointCloudAdaptor {
        const std::vector<std::array<double, 3>>& points;

        explicit PointCloudAdaptor(const std::vector<std::array<double, 3>>& pts)
            : points(pts) {
        }

        /// Number of points
        inline size_t kdtree_get_point_count() const {
            return points.size();
        }

        /// Squared distance between point p1 and point at index idx_p2
        inline double kdtree_distance(const double* p1, const size_t idx_p2, size_t /*size*/) const {
            const double d0 = p1[0] - points[idx_p2][0];
            const double d1 = p1[1] - points[idx_p2][1];
            const double d2 = p1[2] - points[idx_p2][2];
            return d0 * d0 + d1 * d1 + d2 * d2;
        }

        /// Get dim-th component of point at index idx
        inline double kdtree_get_pt(const size_t idx, int dim) const {
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
        nanoflann::L2_Simple_Adaptor<double, PointCloudAdaptor>,
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
        double squaredDistance;
    };

    struct CorrespondenceFace {
        size_t sourceIdx;
        std::array<double, 3> facePt;
        double squaredDistance;
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

            /// Find single nearest neighbor
            nanoflann::KNNResultSet<double> resultSet(1);
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

        /// Build point-wise arrays from face indices
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

            /// Point-to-Plane distance and projection
            if (!pointPlaneDistanceAndProjection(
                sourcePoints[i],           // P: source point
                facePts[resultIndex],      // Q: point on face
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

    /// Fastest version: Unit normal vector provided (|N| = 1)
    inline bool pointTriangleDistanceAndProjectionUnitN(
        const std::array<double, 3>& P,
        const std::array<double, 3>& v0,
        const std::array<double, 3>& v1,
        const std::array<double, 3>& v2,
        const std::array<double, 3>& N,  /// Must be unit vector
        double& distance,
        std::array<double, 3>& proj)
    {
        constexpr double EPSILON_SQ = 1e-12;
        constexpr double EDGE_TOLERANCE = 1e-6;

        /// Triangle edges
        const double e0x = v1[0] - v0[0];
        const double e0y = v1[1] - v0[1];
        const double e0z = v1[2] - v0[2];

        const double e1x = v2[0] - v0[0];
        const double e1y = v2[1] - v0[1];
        const double e1z = v2[2] - v0[2];

        /// Dot products for barycentric
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

        /// Barycentric (early rejection)
        const double invDenom = 1.0 / denom;
        const double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        if (u < -EDGE_TOLERANCE || u > 1.0 + EDGE_TOLERANCE)
            return false;

        const double v = (dot00 * dot12 - dot01 * dot02) * invDenom;
        if (v < -EDGE_TOLERANCE || (u + v) > 1.0 + EDGE_TOLERANCE)
            return false;

        /// Distance = (P-v0)·N (no sqrt needed for unit normal)
        distance = pvx * N[0] + pvy * N[1] + pvz * N[2];

        /// Projection: P - N * distance
        proj[0] = P[0] - N[0] * distance;
        proj[1] = P[1] - N[1] * distance;
        proj[2] = P[2] - N[2] * distance;

        return true;
    }

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

        /// Build point-wise arrays from face indices
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

            /// Point-to-Plane distance and projection
            if (!pointTriangleDistanceAndProjectionUnitN(
                sourcePoints[i],           /// P: source point
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
    // Outlier Rejection
    //=========================================================================

    /// Reject outliers using Median Absolute Deviation (MAD)
    /// @param correspondences Correspondences to filter (modified in-place)
    /// @param threshold Multiplier for MAD (typically 2.0 - 3.0)
    inline void rejectOutliersMAD(
        std::vector<Correspondence>& correspondences,
        double threshold)
    {
        if (correspondences.size() < 10) return;

        /// Extract distances
        std::vector<double> distances;
        distances.reserve(correspondences.size());
        for (const auto& c : correspondences) {
            distances.push_back(std::sqrt(c.squaredDistance));
        }

        /// Compute median
        std::vector<double> sortedDist = distances;
        std::sort(sortedDist.begin(), sortedDist.end());
        const double median = sortedDist[sortedDist.size() / 2];

        /// Compute MAD (Median Absolute Deviation)
        std::vector<double> absDevs;
        absDevs.reserve(distances.size());
        for (double d : distances) {
            absDevs.push_back(std::abs(d - median));
        }
        std::sort(absDevs.begin(), absDevs.end());
        const double mad = absDevs[absDevs.size() / 2];

        /// MAD to standard deviation approximation
        const double madScale = 1.4826;
        const double sigma = mad * madScale;

        /// Reject outliers beyond threshold * sigma from median
        const double maxDist = median + threshold * sigma;
        const double maxDistSq = maxDist * maxDist;

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
    // Transformation Estimation using SVD
    //=========================================================================

    /// Estimate rigid transformation (rotation + translation) using SVD
    /// Finds T such that target ≈ R * source + t
    /// @param sourcePoints Source point cloud
    /// @param targetPoints Target point cloud
    /// @param correspondences Point correspondences
    /// @return 4x4 transformation matrix
    inline Transform4x4 estimateRigidTransformSVD(
        const std::vector<std::array<double, 3>>& sourcePoints,
        const std::vector<std::array<double, 3>>& targetPoints,
        const std::vector<Correspondence>& correspondences)
    {
        if (correspondences.size() < 3) {
            return Transform4x4::identity();
        }

        const size_t n = correspondences.size();

        /// Compute centroids
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

        /// Compute cross-covariance matrix H
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

        /// SVD decomposition: H = U * S * V^T
        Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(H);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();

        /// Rotation: R = V * U^T
        Eigen::Matrix3d R = V * U.transpose();

        /// Ensure proper rotation (det(R) = 1, not -1 for reflection)
        if (R.determinant() < 0) {
            V.col(2) *= -1;
            R = V * U.transpose();
        }

        /// Translation: t = tgtCentroid - R * srcCentroid
        Eigen::Vector3d t = tgtCentroid - R * srcCentroid;

        /// Build 4x4 transformation matrix
        double rotMat[9];
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

    /// Compute RMSE with transformation applied
    /// @param sourcePoints Source points (original, not transformed)
    /// @param targetPoints Target points
    /// @param correspondences Point correspondences
    /// @param transform Transformation to apply to source
    /// @return RMSE value after transformation
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

            /// Transform source point
            double tx, ty, tz;
            transform.transformPoint(sp[0], sp[1], sp[2], tx, ty, tz);

            /// Compute squared distance
            const double dx = tx - tp[0];
            const double dy = ty - tp[1];
            const double dz = tz - tp[2];
            sumSquaredError += dx * dx + dy * dy + dz * dz;
        }

        return std::sqrt(sumSquaredError / correspondences.size());
    }

    //=========================================================================
    // Point Cloud Transformation
    //=========================================================================

    /// Apply transformation to point cloud in-place
    inline void transformPointCloud(
        std::vector<std::array<double, 3>>& points,
        const Transform4x4& transform)
    {
        for (auto& p : points) {
            transform.transformPointInPlace(p[0], p[1], p[2]);
        }
    }

    /// Apply transformation to point cloud (copy version)
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

    /// Apply transformation to a float-stored point cloud (copy version). Each point is
    /// widened to double for the transform math (precision), then narrowed back to float for
    /// storage -- used for the Master's bulk (offset-rebased) point cloud arrays, which stay
    /// float in memory even though the transform itself is always double.
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
    // Downsampling
    //=========================================================================

    /// Downsample point cloud by uniform selection (every N-th point)
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

    /// Downsample point cloud by uniform selection (every N-th point)
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

    /// Downsample a float-stored point cloud by uniform selection (every N-th point). Plain
    /// element copy, no precision-sensitive math -- used for the Master's bulk (offset-rebased)
    /// point cloud arrays.
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

    /// Widen a float point array to double (e.g. when handing a downsampled subset off to
    /// IcpChunk, which stores double for use by the (unmodified) ICP correspondence/transform
    /// estimation code below).
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

    /// Narrow a double point array to float (e.g. right after loading/offset-rebasing the full
    /// LAS point cloud, before it settles into the Master's bulk float storage).
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

    /// Downsample point cloud using voxel grid
    /// @param points Input point cloud
    /// @param voxelSize Size of each voxel
    /// @return Downsampled point cloud (voxel centroids)
    inline std::vector<std::array<double, 3>> downsampleVoxelGrid(
        const std::vector<std::array<double, 3>>& points,
        double voxelSize)
    {
        if (points.empty() || voxelSize <= 0.0) return points;

        /// Find bounding box
        double minX = points[0][0], maxX = points[0][0];
        double minY = points[0][1], maxY = points[0][1];
        double minZ = points[0][2], maxZ = points[0][2];

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
    // Outlier Rejection for CorrespondenceFace
    //=========================================================================

    /// Reject outliers using MAD for CorrespondenceFace (Point-to-Plane)
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
    // RMSE Calculation for CorrespondenceFace
    //=========================================================================

    /// Compute RMSE from CorrespondenceFace (Point-to-Plane distance)
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
    // Point-to-Plane Transformation Estimation
    //=========================================================================

    /// Estimate rigid transformation using Point-to-Plane linearized method
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

            A(i, 0) = nx;
            A(i, 1) = ny;
            A(i, 2) = nz;
            A(i, 3) = sy * nz - sz * ny;
            A(i, 4) = sz * nx - sx * nz;
            A(i, 5) = sx * ny - sy * nx;

            b(i) = nx * (p[0] - sx) + ny * (p[1] - sy) + nz * (p[2] - sz);
        }

        /// Solve using SVD least squares (BDCSVD for newer Eigen)
        Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(A);
        Eigen::VectorXd x = svd.solve(b);

        double tx = x(0), ty = x(1), tz = x(2);
        double rx = x(3), ry = x(4), rz = x(5);

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

        //dTx, dTy, dTz, dO, dP, dK, and dS
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

    /// [ToDo] under construction
    /// add const double benchSd
    /// add std::array<double, 3> xyzSd
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
    /// CorrespondenceFace, need it, all of them???
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
    // ICP Main Algorithm
    //=========================================================================

    /// Run ICP algorithm (Point-to-Plane or Point-to-Point)
    /// @param chunk ICP chunk containing source/target points and config
    /// @param cancelRequested Atomic flag for cancellation check
    /// @return ICP result with transformation and statistics
    inline IcpResult runIcp(const IcpChunk& chunk, std::atomic<bool>& cancelRequested)
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

        /// chunk's point arrays are stored float, shifted by (offsetX, offsetY, offsetZ) --
        /// widen to double and add the offset back exactly once, here, before any actual
        /// computation. Everything below this point operates on these local double vectors
        /// exactly as it did before this offset-rebase optimization existed. faceNormals are
        /// direction vectors and are widened but never shifted.
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

        /// Check if Point-to-Plane mode is available
        const bool usePointToPlane = !chunk.faceIndices.empty() &&
            !chunk.faceNormals.empty() &&
            !chunk.facePts.empty();

        /// Build KD-Tree for target points
        PointCloudAdaptor targetAdaptor(targetPointsD);
        KdTree3D targetTree(3, targetAdaptor,
            nanoflann::KDTreeSingleIndexAdaptorParams(50));
        targetTree.buildIndex();

        /// Initialize transformation
        Transform4x4 currentTransform = chunk.initialTransform;
        Transform4x4 accumulatedLocalTransform = Transform4x4::identity();

        /// Copy source points for transformation
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

            /// Initial RMSE
            findCorrespondencesWithNormals(
                transformedSource, targetTree, chunk.faceIndices,
                faceNormalsD, facePtsD,
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
                    faceNormalsD, facePtsD,
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
                    chunk.faceIndices, faceNormalsD);
#ifdef _DEBUG
                /// [ToDo]: config
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
                    /// Extract translation
                    double tx = deltaT.m[12];
                    double ty = deltaT.m[13];
                    double tz = deltaT.m[14];

                    /// Extract rotation angles from rotation matrix (Euler angles)
                    /// R = Rz * Ry * Rx (ZYX convention)
                    double r00 = deltaT.m[0], r01 = deltaT.m[4], r02 = deltaT.m[8];
                    double r10 = deltaT.m[1], r11 = deltaT.m[5], r12 = deltaT.m[9];
                    double r20 = deltaT.m[2], r21 = deltaT.m[6], r22 = deltaT.m[10];

                    double pitch = std::asin(-r20);  /// ry (Y rotation)
                    double yaw, roll;

                    if (std::abs(r20) < 0.99999) {
                        yaw = std::atan2(r10, r00);   /// rz (Z rotation)
                        roll = std::atan2(r21, r22);  /// rx (X rotation)
                    }
                    else {
                        /// Gimbal lock
                        yaw = std::atan2(-r01, r11);
                        roll = 0.0;
                    }

                    /// Convert to degrees
                    const double rad2deg = 180.0 / 3.14159265;
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
                    faceNormalsD, facePtsD,
                    maxDistSq, config.normalAngleThreshold, correspondences);
                double currentRMSE = computeRMSE(correspondences);

                /// 6. Check convergence
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
                    transformedSource, targetPointsD, correspondences);

                /// 4. Apply transformation
                transformPointCloud(transformedSource, deltaT);
                accumulatedLocalTransform = deltaT * accumulatedLocalTransform;

                /// 5. Compute RMSE
                findCorrespondences(transformedSource, targetTree, maxDistSq, correspondences);
                double currentRMSE = computeRMSE(correspondences);

                /// 6. Check convergence
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

        /// Compute weights (inverse RMSE)
        std::vector<double> weights;
        double totalWeight = 0.0;
        for (const auto* r : validResults) {
            double w = 1.0 / (r->finalRMSE + 1e-6);
            weights.push_back(w);
            totalWeight += w;
        }

        /// Weighted average of transformation matrices
        Transform4x4 result;
        std::memset(result.m, 0, sizeof(result.m));

        for (size_t i = 0; i < validResults.size(); ++i) {
            double w = weights[i] / totalWeight;
            for (int j = 0; j < 16; ++j) {
                result.m[j] += w * validResults[i]->transform.m[j];
            }
        }

        /// Re-orthogonalize rotation matrix using SVD
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

    /// Select best result based on lowest RMSE
    /// @param results Vector of ICP results
    /// @return Pointer to best result, or nullptr if no valid results
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