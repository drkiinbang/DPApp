#pragma once

/// ============================================================================
/// IcpTypes.h - ICP (Iterative Closest Point) Registration Types
/// ============================================================================
/// 
/// This header defines data structures for distributed ICP alignment
/// between point clouds (LAS) and meshes (GLTF).
/// 
/// Location: F:\repository\DPApp\include\IcpTypes.h
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
    // Transform4x4 - 4x4 Transformation Matrix (Column-Major)
    //=========================================================================

    /// 4x4 transformation matrix for rigid body transformations
    /// Layout (column-major, OpenGL style):
    /// | m[0]  m[4]  m[8]   m[12] |   | R00 R01 R02 Tx |
    /// | m[1]  m[5]  m[9]   m[13] | = | R10 R11 R12 Ty |
    /// | m[2]  m[6]  m[10]  m[14] |   | R20 R21 R22 Tz |
    /// | m[3]  m[7]  m[11]  m[15] |   | 0   0   0   1  |
    struct Transform4x4 {
        double m[16];

        /// Create identity matrix
        static Transform4x4 identity() {
            Transform4x4 result;
            std::memset(result.m, 0, sizeof(result.m));
            result.m[0] = result.m[5] = result.m[10] = result.m[15] = 1.0;
            return result;
        }

        /// Create translation matrix
        static Transform4x4 translation(double tx, double ty, double tz) {
            Transform4x4 result = identity();
            result.m[12] = tx;
            result.m[13] = ty;
            result.m[14] = tz;
            return result;
        }

        /// Create from rotation matrix (3x3, row-major) and translation vector
        static Transform4x4 fromRotationTranslation(const double R[9], double tx, double ty, double tz) {
            Transform4x4 result;
            std::memset(result.m, 0, sizeof(result.m));

            /// Rotation (row-major R to column-major m)
            result.m[0] = R[0];  result.m[4] = R[1];  result.m[8] = R[2];
            result.m[1] = R[3];  result.m[5] = R[4];  result.m[9] = R[5];
            result.m[2] = R[6];  result.m[6] = R[7];  result.m[10] = R[8];

            /// Translation
            result.m[12] = tx;
            result.m[13] = ty;
            result.m[14] = tz;

            /// Homogeneous coordinate
            result.m[15] = 1.0;

            return result;
        }

        /// Matrix multiplication: this * other
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

        /// Transform a 3D point: result = R * p + t
        void transformPoint(double x, double y, double z, double& ox, double& oy, double& oz) const {
            ox = m[0] * x + m[4] * y + m[8] * z + m[12];
            oy = m[1] * x + m[5] * y + m[9] * z + m[13];
            oz = m[2] * x + m[6] * y + m[10] * z + m[14];
        }

        /// Transform a 3D point (in-place version)
        void transformPointInPlace(double& x, double& y, double& z) const {
            double ox, oy, oz;
            transformPoint(x, y, z, ox, oy, oz);
            x = ox; y = oy; z = oz;
        }

        /// Get rotation matrix (3x3, row-major)
        void getRotation(double R[9]) const {
            R[0] = m[0];  R[1] = m[4];  R[2] = m[8];
            R[3] = m[1];  R[4] = m[5];  R[5] = m[9];
            R[6] = m[2];  R[7] = m[6];  R[8] = m[10];
        }

        /// Get translation vector
        void getTranslation(double& tx, double& ty, double& tz) const {
            tx = m[12];
            ty = m[13];
            tz = m[14];
        }

        /// Compute inverse (assuming rigid transformation: R^T, -R^T * t)
        Transform4x4 inverse() const {
            Transform4x4 result;

            /// Transpose rotation part
            result.m[0] = m[0];   result.m[4] = m[1];   result.m[8] = m[2];
            result.m[1] = m[4];   result.m[5] = m[5];   result.m[9] = m[6];
            result.m[2] = m[8];   result.m[6] = m[9];   result.m[10] = m[10];

            /// Translation: -R^T * t
            result.m[12] = -(result.m[0] * m[12] + result.m[4] * m[13] + result.m[8] * m[14]);
            result.m[13] = -(result.m[1] * m[12] + result.m[5] * m[13] + result.m[9] * m[14]);
            result.m[14] = -(result.m[2] * m[12] + result.m[6] * m[13] + result.m[10] * m[14]);

            /// Homogeneous part
            result.m[3] = result.m[7] = result.m[11] = 0.0;
            result.m[15] = 1.0;

            return result;
        }

        /// Calculate Frobenius norm of difference (for convergence check)
        double distanceTo(const Transform4x4& other) const {
            double sum = 0.0;
            for (int i = 0; i < 16; ++i) {
                double diff = m[i] - other.m[i];
                sum += diff * diff;
            }
            return std::sqrt(sum);
        }
    };

    /// Given a rigid transform valid for absolute-frame points (R*p_abs + t ~= q_abs), derive
    /// the equivalent transform for points already shifted by a rebase offset
    /// (p_shifted = p_abs - offset), such that applying it directly to p_shifted yields a
    /// result in the same shifted frame (q_shifted = q_abs - offset):
    ///   t_shifted = t_absolute - (I - R) * offset = t_absolute - offset + R * offset
    /// Rotation is unchanged by a common translation offset. Used only to apply an
    /// already-computed (absolute-frame) ICP transform directly to the Master's bulk,
    /// offset-rebased float point array without widening the whole array to double first --
    /// the ICP computation itself always operates on absolute-frame (unshifted) chunk data.
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
    // IcpConfig - ICP Algorithm Configuration
    //=========================================================================

    /// Configuration parameters for ICP algorithm
    struct IcpConfig {
        /// Maximum number of iterations
        int maxIterations = 50;

        /// Convergence threshold (transformation change)
        double convergenceThreshold = 1e-6;

        /// Maximum distance for correspondence matching (meters)
        double maxCorrespondenceDistance = 1.0;

        /// Outlier rejection threshold (multiplier of MAD)
        double outlierRejectionThreshold = 2.5;

        /// Downsample ratio for coarse alignment (1 = no downsampling)
        int downsampleRatio = 10;

        /// Use normal-based filtering for correspondences
        bool useNormalFiltering = true;

        /// Normal angle threshold (degrees) for filtering
        double normalAngleThreshold = 45.0;

        /// Minimum number of correspondences required
        int minCorrespondences = 100;

        /// Grid size for mesh pseudo-point generation (meters)
        double pseudoPointGridSize = 0.1;

        /// Enable verbose logging
        bool verbose = false;

        /// Validate configuration
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
    // IcpChunk - Data chunk for distributed ICP processing
    //=========================================================================

    /// Chunk data sent to Slave for local ICP processing. Point arrays are stored as `float`,
    /// shifted by (offsetX, offsetY, offsetZ) relative to absolute coordinates -- this keeps
    /// wire/storage size small while preserving sub-mm precision (see icp::computeBimRebaseOffset
    /// in PseudoPointGenerator.hpp for how the offset is chosen). Consumers (runIcp) widen back
    /// to double and add the offset back exactly once, at the top of the function, before doing
    /// any actual math.
    struct IcpChunk {
        /// Unique chunk identifier
        uint32_t chunk_id = 0;

        /// Source points (from point cloud) - [x, y, z], shifted, float
        std::vector<std::array<float, 3>> sourcePoints;

        /// Target points (pseudo-points from mesh) - [x, y, z], shifted, float
        std::vector<std::array<float, 3>> targetPoints;

        /// Index indicating which face each point belongs to
        std::vector<size_t> faceIndices;

        /// Face normal vectors - [nx, ny, nz]. Direction vectors, never shifted by offset.
        std::vector<std::array<float, 3>> faceNormals;

        /// Face representative points (one vertex per face) - [x, y, z], shifted, float
        std::vector<std::array<float, 3>> facePts;

        /// Initial transformation (from coarse alignment)
        Transform4x4 initialTransform;

        /// ICP configuration for this chunk
        IcpConfig config;

        /// Rebase offset applied to sourcePoints/targetPoints/facePts (double, exact) -- add
        /// this back after widening to double to recover absolute coordinates.
        double offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;

        /// Bounding box of source points
        double source_min_x = 0.0, source_min_y = 0.0, source_min_z = 0.0;
        double source_max_x = 0.0, source_max_y = 0.0, source_max_z = 0.0;

        /// Bounding box of target points
        double target_min_x = 0.0, target_min_y = 0.0, target_min_z = 0.0;
        double target_max_x = 0.0, target_max_y = 0.0, target_max_z = 0.0;

        /// Default constructor
        IcpChunk() : initialTransform(Transform4x4::identity()) {}

        /// Calculate source bounding box from points
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

        /// Calculate target bounding box from points
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

        /// Get approximate memory size in bytes
        size_t getApproximateSize() const {
            return sizeof(IcpChunk) +
                sourcePoints.size() * sizeof(std::array<float, 3>) +
                targetPoints.size() * sizeof(std::array<float, 3>) +
                faceIndices.size() * sizeof(size_t) +
                faceNormals.size() * sizeof(std::array<float, 3>);
        }

        /// Check if chunk has valid data for processing
        bool isValid() const {
            return !sourcePoints.empty() &&
                !targetPoints.empty() &&
                config.isValid();
        }
    };

    //=========================================================================
    // IcpResult - Result of ICP processing
    //=========================================================================

    /// Result from ICP processing on a single chunk
    struct IcpResult {
        /// Chunk identifier (matches IcpChunk::chunk_id)
        uint32_t chunk_id = 0;

        /// Final transformation matrix (combined: initial * local refinement)
        Transform4x4 transform;

        /// Local refinement transformation only (without initial)
        Transform4x4 localTransform;

        /// Final Root Mean Square Error
        double finalRMSE = 0.0;

        /// Initial RMSE (before ICP iterations)
        double initialRMSE = 0.0;

        /// Number of iterations actually performed
        int actualIterations = 0;

        /// Number of valid correspondences in final iteration
        int correspondenceCount = 0;

        /// Number of source points processed
        int sourcePointCount = 0;

        /// Number of target points used
        int targetPointCount = 0;

        /// Whether ICP converged within threshold
        bool converged = false;

        /// Whether processing was successful
        bool success = false;

        /// Error message (if failed)
        std::string errorMessage;

        /// Processing time in milliseconds
        double processingTimeMs = 0.0;

        /// Default constructor
        IcpResult() : transform(Transform4x4::identity()),
            localTransform(Transform4x4::identity()) {
        }

        /// Get RMSE improvement ratio (0.0 ~ 1.0)
        double getImprovementRatio() const {
            if (initialRMSE <= 0.0) return 0.0;
            return (initialRMSE - finalRMSE) / initialRMSE;
        }

        /// Check if result is good quality
        bool isGood(double maxRMSE = 0.1) const {
            return success && converged && finalRMSE < maxRMSE;
        }
    };

    //=========================================================================
    // IcpJobStatus - Status of entire ICP job
    //=========================================================================

    /// Status enumeration for ICP job lifecycle
    enum class IcpJobStatus {
        PENDING,            ///< Job created, waiting to start
        LOADING_DATA,       ///< Loading LAS and GLTF data
        GENERATING_PSEUDO,  ///< Generating pseudo-points from mesh
        COARSE_ALIGNMENT,   ///< Running coarse alignment on master
        DISTRIBUTING,       ///< Distributing chunks to slaves
        FINE_ALIGNMENT,     ///< Running fine alignment on slaves
        AGGREGATING,        ///< Aggregating results from slaves
        APPLYING_TRANSFORM, ///< Applying final transformation
        SAVING_RESULT,      ///< Saving aligned point cloud
        COMPLETED,          ///< Job completed successfully
        FAILED,             ///< Job failed with error
        CANCELLED           ///< Job was cancelled by user
    };

    /// Convert IcpJobStatus to string
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
    // IcpElementAlignment - Per-BIM-element (부재) alignment result
    //=========================================================================

    /// One BIM element's individually-estimated alignment, kept alongside the combined
    /// job-wide result so callers can inspect/save per-element quality (e.g. one element's
    /// fine alignment converging poorly doesn't have to be guessed at from the combined RMSE
    /// alone). `transform` here is the chunk's *local* refinement only (IcpResult::transform,
    /// composed with the job's coarse transform the same way aggregateWeightedTransforms
    /// combines all chunks) -- see CHANGELOG/doc for the combination math.
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
    // IcpJob - Master-side job management structure
    //=========================================================================

    /// Complete ICP job information (managed by Master)
    struct IcpJob {
        /// Unique job identifier
        std::string jobId;

        /// Input file paths
        std::string lasFilePath;
        std::string bimFolderPath;

        /// LAS offset (user-supplied manual pre-shift, applied after loading)
        double offsetX = 0.0;
        double offsetY = 0.0;
        double offsetZ = 0.0;

        /// Rebase offset (computed automatically from the BIM centroid, distinct from the
        /// manual offsetX/Y/Z above). Point-cloud coordinates are stored/transmitted shifted
        /// by this amount (as float) to preserve sub-mm precision near the origin; the final
        /// transform is corrected back to the original frame exactly once, when the job
        /// completes (see aggregateWeightedTransforms usage in processIcpJob).
        double rebaseOffsetX = 0.0;
        double rebaseOffsetY = 0.0;
        double rebaseOffsetZ = 0.0;

        /// Output file path for aligned point cloud
        std::string outputPath;

        /// Job configuration
        IcpConfig config;

        /// Current status
        IcpJobStatus status = IcpJobStatus::PENDING;

        /// Error message (if status == FAILED)
        std::string errorMessage;

        /// Coarse alignment result (Phase 1)
        Transform4x4 coarseTransform;
        double coarseRMSE = 0.0;
        bool coarseConverged = false;

        /// Chunk results (Phase 2)
        std::vector<IcpResult> chunkResults;
        int totalChunks = 0;
        int completedChunks = 0;
        int failedChunks = 0;

        /// Final aggregated result (Phase 3)
        Transform4x4 finalTransform;
        double finalRMSE = 0.0;

        /// Per-BIM-element (부재) alignment results, one per successfully-processed chunk
        /// (empty when fine alignment ran as a single Master-only pass instead of per-element
        /// chunks). Saved to disk alongside the combined result -- see
        /// alignmentOutputPath/saveAlignmentResults() in MasterApplication_ICP.cpp.
        std::vector<IcpElementAlignment> elementResults;

        /// Path to the saved per-element + combined alignment JSON file (set once written)
        std::string alignmentOutputPath;

        /// Statistics
        size_t totalPointCount = 0;
        size_t totalMeshTriangles = 0;
        size_t totalPseudoPoints = 0;

        /// Timing information (milliseconds since epoch)
        double startTimeMs = 0.0;
        double endTimeMs = 0.0;
        double coarseAlignmentTimeMs = 0.0;
        double fineAlignmentTimeMs = 0.0;

        /// Cancellation flag (atomic for thread safety)
        std::atomic<bool> cancelRequested{ false };

        /// Default constructor
        IcpJob() : coarseTransform(Transform4x4::identity()),
            finalTransform(Transform4x4::identity()) {
        }

        /// Copy constructor (handle atomic)
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

        /// Get progress percentage (0-100)
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
                return 25.0 + chunkProgress * 60.0; // 25% ~ 85%
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

        /// Get elapsed time in seconds (uses current time if job is still running)
        double getElapsedTimeSec() const {
            if (startTimeMs <= 0.0) return 0.0;
            double end = (endTimeMs > 0.0) ? endTimeMs
                : static_cast<double>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count());
            return (end - startTimeMs) / 1000.0;
        }

        /// Check if job is finished (success, failed, or cancelled)
        bool isFinished() const {
            return status == IcpJobStatus::COMPLETED ||
                status == IcpJobStatus::FAILED ||
                status == IcpJobStatus::CANCELLED;
        }

        /// Check if job is currently running
        bool isRunning() const {
            return !isFinished() && status != IcpJobStatus::PENDING;
        }
    };

    //=========================================================================
    // IcpStatistics - Aggregated statistics for reporting
    //=========================================================================

    /// Statistics for ICP job results
    struct IcpStatistics {
        /// Point cloud statistics
        size_t totalSourcePoints = 0;
        size_t processedSourcePoints = 0;

        /// Mesh statistics
        size_t totalMeshTriangles = 0;
        size_t totalPseudoPoints = 0;

        /// Alignment statistics
        double initialRMSE = 0.0;
        double coarseRMSE = 0.0;
        double finalRMSE = 0.0;
        double rmseImprovement = 0.0; ///< percentage

        /// Iteration statistics
        int coarseIterations = 0;
        int totalFineIterations = 0;
        double averageFineIterations = 0.0;

        /// Correspondence statistics
        int totalCorrespondences = 0;
        double averageCorrespondenceDistance = 0.0;

        /// Chunk statistics
        int totalChunks = 0;
        int successfulChunks = 0;
        int failedChunks = 0;
        int convergedChunks = 0;

        /// Timing (seconds)
        double totalProcessingTimeSec = 0.0;
        double coarseAlignmentTimeSec = 0.0;
        double fineAlignmentTimeSec = 0.0;
        double dataLoadingTimeSec = 0.0;

        /// Calculate statistics from IcpJob
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