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
        float m[16];

        /// Create identity matrix
        static Transform4x4 identity() {
            Transform4x4 result;
            std::memset(result.m, 0, sizeof(result.m));
            result.m[0] = result.m[5] = result.m[10] = result.m[15] = 1.0f;
            return result;
        }

        /// Create translation matrix
        static Transform4x4 translation(float tx, float ty, float tz) {
            Transform4x4 result = identity();
            result.m[12] = tx;
            result.m[13] = ty;
            result.m[14] = tz;
            return result;
        }

        /// Create from rotation matrix (3x3, row-major) and translation vector
        static Transform4x4 fromRotationTranslation(const float R[9], float tx, float ty, float tz) {
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
            result.m[15] = 1.0f;

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
        void transformPoint(float x, float y, float z, float& ox, float& oy, float& oz) const {
            ox = m[0] * x + m[4] * y + m[8] * z + m[12];
            oy = m[1] * x + m[5] * y + m[9] * z + m[13];
            oz = m[2] * x + m[6] * y + m[10] * z + m[14];
        }

        /// Transform a 3D point (in-place version)
        void transformPointInPlace(float& x, float& y, float& z) const {
            float ox, oy, oz;
            transformPoint(x, y, z, ox, oy, oz);
            x = ox; y = oy; z = oz;
        }

        /// Get rotation matrix (3x3, row-major)
        void getRotation(float R[9]) const {
            R[0] = m[0];  R[1] = m[4];  R[2] = m[8];
            R[3] = m[1];  R[4] = m[5];  R[5] = m[9];
            R[6] = m[2];  R[7] = m[6];  R[8] = m[10];
        }

        /// Get translation vector
        void getTranslation(float& tx, float& ty, float& tz) const {
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
            result.m[3] = result.m[7] = result.m[11] = 0.0f;
            result.m[15] = 1.0f;

            return result;
        }

        /// Calculate Frobenius norm of difference (for convergence check)
        float distanceTo(const Transform4x4& other) const {
            float sum = 0.0f;
            for (int i = 0; i < 16; ++i) {
                float diff = m[i] - other.m[i];
                sum += diff * diff;
            }
            return std::sqrt(sum);
        }
    };

    //=========================================================================
    // IcpConfig - ICP Algorithm Configuration
    //=========================================================================

    /// Configuration parameters for ICP algorithm
    struct IcpConfig {
        /// Maximum number of iterations
        int maxIterations = 50;

        /// Convergence threshold (transformation change)
        float convergenceThreshold = 1e-6f;

        /// Maximum distance for correspondence matching (meters)
        float maxCorrespondenceDistance = 1.0f;

        /// Outlier rejection threshold (multiplier of MAD)
        float outlierRejectionThreshold = 2.5f;

        /// Downsample ratio for coarse alignment (1 = no downsampling)
        int downsampleRatio = 10;

        /// Use normal-based filtering for correspondences
        bool useNormalFiltering = true;

        /// Normal angle threshold (degrees) for filtering
        float normalAngleThreshold = 45.0f;

        /// Minimum number of correspondences required
        int minCorrespondences = 100;

        /// Grid size for mesh pseudo-point generation (meters)
        float pseudoPointGridSize = 0.1f;

        /// Enable verbose logging
        bool verbose = false;

        /// Validate configuration
        bool isValid() const {
            return maxIterations > 0 &&
                convergenceThreshold > 0.0f &&
                maxCorrespondenceDistance > 0.0f &&
                outlierRejectionThreshold > 0.0f &&
                downsampleRatio >= 1 &&
                normalAngleThreshold > 0.0f && normalAngleThreshold < 180.0f &&
                minCorrespondences > 0 &&
                pseudoPointGridSize > 0.0f;
        }
    };

    //=========================================================================
    // IcpChunk - Data chunk for distributed ICP processing
    //=========================================================================

    /// Chunk data sent to Slave for local ICP processing
    struct IcpChunk {
        /// Unique chunk identifier
        uint32_t chunk_id = 0;

        /// Source points (from point cloud) - [x, y, z]
        std::vector<std::array<float, 3>> sourcePoints;

        /// Target points (pseudo-points from mesh) - [x, y, z]
        std::vector<std::array<float, 3>> targetPoints;

        /// Index indicating which face each point belongs to
        std::vector<size_t> faceIndices;

        /// Face normal vectors - [nx, ny, nz]
        std::vector<std::array<float, 3>> faceNormals;

        /// Face representative points (one vertex per face) - [x, y, z]
        std::vector<std::array<float, 3>> facePts;
        
        /// Initial transformation (from coarse alignment)
        Transform4x4 initialTransform;

        /// ICP configuration for this chunk
        IcpConfig config;

        /// Bounding box of source points
        float source_min_x = 0.0f, source_min_y = 0.0f, source_min_z = 0.0f;
        float source_max_x = 0.0f, source_max_y = 0.0f, source_max_z = 0.0f;

        /// Bounding box of target points
        float target_min_x = 0.0f, target_min_y = 0.0f, target_min_z = 0.0f;
        float target_max_x = 0.0f, target_max_y = 0.0f, target_max_z = 0.0f;

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
        float finalRMSE = 0.0f;

        /// Initial RMSE (before ICP iterations)
        float initialRMSE = 0.0f;

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
        float getImprovementRatio() const {
            if (initialRMSE <= 0.0f) return 0.0f;
            return (initialRMSE - finalRMSE) / initialRMSE;
        }

        /// Check if result is good quality
        bool isGood(float maxRMSE = 0.1f) const {
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
    // IcpJob - Master-side job management structure
    //=========================================================================

    /// Complete ICP job information (managed by Master)
    struct IcpJob {
        /// Unique job identifier
        std::string jobId;

        /// Input file paths
        std::string lasFilePath;
        std::string bimFolderPath;

        /// LAS offset (applied after loading)
        float offsetX = 0.0f;
        float offsetY = 0.0f;
        float offsetZ = 0.0f;

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
        float coarseRMSE = 0.0f;
        bool coarseConverged = false;

        /// Chunk results (Phase 2)
        std::vector<IcpResult> chunkResults;
        int totalChunks = 0;
        int completedChunks = 0;
        int failedChunks = 0;

        /// Final aggregated result (Phase 3)
        Transform4x4 finalTransform;
        float finalRMSE = 0.0f;

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
        float getProgress() const {
            switch (status) {
            case IcpJobStatus::PENDING:           return 0.0f;
            case IcpJobStatus::LOADING_DATA:      return 5.0f;
            case IcpJobStatus::GENERATING_PSEUDO: return 10.0f;
            case IcpJobStatus::COARSE_ALIGNMENT:  return 20.0f;
            case IcpJobStatus::DISTRIBUTING:      return 25.0f;
            case IcpJobStatus::FINE_ALIGNMENT: {
                if (totalChunks == 0) return 25.0f;
                float chunkProgress = static_cast<float>(completedChunks) / totalChunks;
                return 25.0f + chunkProgress * 60.0f; // 25% ~ 85%
            }
            case IcpJobStatus::AGGREGATING:        return 90.0f;
            case IcpJobStatus::APPLYING_TRANSFORM: return 95.0f;
            case IcpJobStatus::SAVING_RESULT:      return 98.0f;
            case IcpJobStatus::COMPLETED:          return 100.0f;
            case IcpJobStatus::FAILED:             return 0.0f;
            case IcpJobStatus::CANCELLED:          return 0.0f;
            default:                               return 0.0f;
            }
        }

        /// Get elapsed time in seconds
        double getElapsedTimeSec() const {
            if (endTimeMs > 0.0) {
                return (endTimeMs - startTimeMs) / 1000.0;
            }
            return 0.0;
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
        float initialRMSE = 0.0f;
        float coarseRMSE = 0.0f;
        float finalRMSE = 0.0f;
        float rmseImprovement = 0.0f; ///< percentage

        /// Iteration statistics
        int coarseIterations = 0;
        int totalFineIterations = 0;
        int averageFineIterations = 0;

        /// Correspondence statistics
        int totalCorrespondences = 0;
        float averageCorrespondenceDistance = 0.0f;

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
                stats.averageFineIterations = totalIter / successCount;
            }

            if (stats.coarseRMSE > 0.0f) {
                stats.rmseImprovement = (stats.coarseRMSE - stats.finalRMSE) / stats.coarseRMSE * 100.0f;
            }

            stats.totalProcessingTimeSec = job.getElapsedTimeSec();
            stats.coarseAlignmentTimeSec = job.coarseAlignmentTimeMs / 1000.0;
            stats.fineAlignmentTimeSec = job.fineAlignmentTimeMs / 1000.0;

            return stats;
        }
    };

} // namespace icp