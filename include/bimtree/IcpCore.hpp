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

namespace icp {

    //=========================================================================
    // Point Cloud Adapter for nanoflann KD-Tree
    //=========================================================================

    /// Adapter class for using std::vector<std::array<float,3>> with nanoflann
    struct PointCloudAdaptor {
        const std::vector<std::array<float, 3>>& points;

        explicit PointCloudAdaptor(const std::vector<std::array<float, 3>>& pts) 
            : points(pts) {}

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
        const std::vector<std::array<float, 3>>& targetNormals,
        const std::vector<std::array<float, 3>>& sourceNormals,
        float maxDistanceSquared,
        float maxAngleDeg,
        std::vector<Correspondence>& correspondences)
    {
        correspondences.clear();
        correspondences.reserve(sourcePoints.size());

        const float cosMaxAngle = std::cos(maxAngleDeg * 3.14159265f / 180.0f);
        const bool hasSourceNormals = !sourceNormals.empty() && 
                                      sourceNormals.size() == sourcePoints.size();

        for (size_t i = 0; i < sourcePoints.size(); ++i) {
            size_t resultIndex = 0;
            float resultDistSq = 0.0f;

            nanoflann::KNNResultSet<float> resultSet(1);
            resultSet.init(&resultIndex, &resultDistSq);

            targetTree.findNeighbors(resultSet, sourcePoints[i].data(),
                                     nanoflann::SearchParams());

            if (resultDistSq > maxDistanceSquared) continue;

            /// Normal filtering (if normals available)
            if (!targetNormals.empty() && resultIndex < targetNormals.size()) {
                const auto& tn = targetNormals[resultIndex];

                if (hasSourceNormals) {
                    const auto& sn = sourceNormals[i];
                    /// Dot product of normals (absolute value for bidirectional)
                    float dot = std::abs(sn[0] * tn[0] + sn[1] * tn[1] + sn[2] * tn[2]);
                    if (dot < cosMaxAngle) continue;
                }
            }

            Correspondence corr;
            corr.sourceIdx = i;
            corr.targetIdx = resultIndex;
            corr.squaredDistance = resultDistSq;
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
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
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
    /// @param points Input point cloud
    /// @param ratio Keep every N-th point (ratio = N)
    /// @return Downsampled point cloud
    inline std::vector<std::array<float, 3>> downsampleUniform(
        const std::vector<std::array<float, 3>>& points,
        int ratio)
    {
        if (ratio <= 1) return points;

        std::vector<std::array<float, 3>> result;
        result.reserve(points.size() / ratio + 1);

        for (size_t i = 0; i < points.size(); i += ratio) {
            result.push_back(points[i]);
        }

        return result;
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
    // ICP Main Algorithm
    //=========================================================================

    /// Run ICP algorithm on a single chunk
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

        std::vector<Correspondence> correspondences;

        /// Initial RMSE calculation
        if (config.useNormalFiltering && !chunk.targetNormals.empty()) {
            findCorrespondencesWithNormals(
                transformedSource, targetTree, chunk.targetNormals,
                {}, maxDistSq, config.normalAngleThreshold, correspondences);
        }
        else {
            findCorrespondences(transformedSource, targetTree, maxDistSq, correspondences);
        }
        result.initialRMSE = computeRMSE(correspondences);

        /// ICP iterations
        for (int iter = 0; iter < config.maxIterations; ++iter) {
            /// Check cancellation
            if (cancelRequested.load()) {
                result.success = false;
                result.errorMessage = "Cancelled";
                return result;
            }

            /// 1. Find correspondences
            if (config.useNormalFiltering && !chunk.targetNormals.empty()) {
                findCorrespondencesWithNormals(
                    transformedSource, targetTree, chunk.targetNormals,
                    {}, maxDistSq, config.normalAngleThreshold, correspondences);
            }
            else {
                findCorrespondences(transformedSource, targetTree, maxDistSq, correspondences);
            }

            /// Check minimum correspondences
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

            /// 3. Estimate transformation
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

            /// Convergence check
            if (std::abs(prevRMSE - currentRMSE) < config.convergenceThreshold ||
                transformChange < config.convergenceThreshold) {
                result.converged = true;
                break;
            }

            prevRMSE = currentRMSE;
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

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R = svd.matrixU() * svd.matrixV().transpose();

        result.m[0] = R(0, 0); result.m[4] = R(0, 1); result.m[8]  = R(0, 2);
        result.m[1] = R(1, 0); result.m[5] = R(1, 1); result.m[9]  = R(1, 2);
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
