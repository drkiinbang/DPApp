#pragma once

/// ============================================================================
/// IcpSerialization.hpp - ICP Data Serialization/Deserialization
/// ============================================================================
/// 
/// This header provides functions for serializing/deserializing ICP data 
/// structures for network transmission between Master and Slave.
/// 
/// Location: F:\repository\DPApp\include\bimtree\IcpSerialization.hpp
/// ============================================================================

#include "../IcpTypes.h"

#include <vector>
#include <cstdint>
#include <cstring>
#include <string>

namespace icp {

    //=========================================================================
    // Transform4x4 Serialization (64 bytes)
    //=========================================================================

    /// Serialize Transform4x4 to binary buffer
    /// @param transform Transform to serialize
    /// @param buffer Output buffer (appended)
    /// [Fix] Transform4x4::m is double[16] (128 bytes), not float[16] (64 bytes). This used to
    /// memcpy only 64 bytes, silently truncating the matrix -- never caught because this code
    /// path had never been exercised (ICP has always run Master-only, with no network transfer
    /// of IcpChunk/IcpResult, until distributed fine alignment was implemented).
    inline void serializeTransform(const Transform4x4& transform, std::vector<uint8_t>& buffer) {
        size_t startSize = buffer.size();
        buffer.resize(startSize + sizeof(double) * 16);
        std::memcpy(buffer.data() + startSize, transform.m, sizeof(double) * 16);
    }

    /// Deserialize Transform4x4 from binary data
    /// @param data Input data pointer
    /// @param offset Current offset (updated after read)
    /// @return Deserialized Transform4x4
    inline Transform4x4 deserializeTransform(const uint8_t* data, size_t& offset) {
        Transform4x4 result;
        std::memcpy(result.m, data + offset, sizeof(double) * 16);
        offset += sizeof(double) * 16;
        return result;
    }

    //=========================================================================
    // IcpConfig Serialization (~30 bytes)
    //=========================================================================

    /// Serialize IcpConfig to binary buffer
    /// [Fix] The 5 threshold fields (convergenceThreshold, maxCorrespondenceDistance,
    /// outlierRejectionThreshold, normalAngleThreshold, pseudoPointGridSize) are all `double`
    /// in IcpConfig, not `float`; this used to memcpy only 4 of each field's 8 bytes.
    inline void serializeConfig(const IcpConfig& config, std::vector<uint8_t>& buffer) {
        size_t startSize = buffer.size();

        /// Calculate required size
        size_t configSize = sizeof(int) * 3 +      // maxIterations, downsampleRatio, minCorrespondences
                           sizeof(double) * 5 +    // 5 double thresholds
                           sizeof(uint8_t) * 2;    // 2 bool flags

        buffer.resize(startSize + configSize);
        uint8_t* ptr = buffer.data() + startSize;

        /// Write integers
        std::memcpy(ptr, &config.maxIterations, sizeof(int));
        ptr += sizeof(int);
        std::memcpy(ptr, &config.downsampleRatio, sizeof(int));
        ptr += sizeof(int);
        std::memcpy(ptr, &config.minCorrespondences, sizeof(int));
        ptr += sizeof(int);

        /// Write doubles
        std::memcpy(ptr, &config.convergenceThreshold, sizeof(double));
        ptr += sizeof(double);
        std::memcpy(ptr, &config.maxCorrespondenceDistance, sizeof(double));
        ptr += sizeof(double);
        std::memcpy(ptr, &config.outlierRejectionThreshold, sizeof(double));
        ptr += sizeof(double);
        std::memcpy(ptr, &config.normalAngleThreshold, sizeof(double));
        ptr += sizeof(double);
        std::memcpy(ptr, &config.pseudoPointGridSize, sizeof(double));
        ptr += sizeof(double);

        /// Write bools as uint8_t
        *ptr++ = config.useNormalFiltering ? 1 : 0;
        *ptr++ = config.verbose ? 1 : 0;
    }

    /// Deserialize IcpConfig from binary data
    inline IcpConfig deserializeConfig(const uint8_t* data, size_t& offset) {
        IcpConfig config;

        /// Read integers
        std::memcpy(&config.maxIterations, data + offset, sizeof(int));
        offset += sizeof(int);
        std::memcpy(&config.downsampleRatio, data + offset, sizeof(int));
        offset += sizeof(int);
        std::memcpy(&config.minCorrespondences, data + offset, sizeof(int));
        offset += sizeof(int);

        /// Read doubles
        std::memcpy(&config.convergenceThreshold, data + offset, sizeof(double));
        offset += sizeof(double);
        std::memcpy(&config.maxCorrespondenceDistance, data + offset, sizeof(double));
        offset += sizeof(double);
        std::memcpy(&config.outlierRejectionThreshold, data + offset, sizeof(double));
        offset += sizeof(double);
        std::memcpy(&config.normalAngleThreshold, data + offset, sizeof(double));
        offset += sizeof(double);
        std::memcpy(&config.pseudoPointGridSize, data + offset, sizeof(double));
        offset += sizeof(double);

        /// Read bools
        config.useNormalFiltering = (data[offset++] != 0);
        config.verbose = (data[offset++] != 0);

        return config;
    }

    //=========================================================================
    // IcpChunk Serialization
    //=========================================================================
    
    /// Binary format:
    /// [chunk_id: 4 bytes]
    /// [sourcePoints count: 4 bytes][sourcePoints data: count * 24 bytes]
    /// [targetPoints count: 4 bytes][targetPoints data: count * 24 bytes]
    /// [targetNormals count: 4 bytes][targetNormals data: count * 24 bytes]
    /// [initialTransform: 128 bytes]
    /// [config: ~68 bytes]
    /// [source bbox: 24 bytes (6 floats)]
    /// [target bbox: 24 bytes (6 floats)]
    ///
    /// [Fix] IcpChunk's point arrays (sourcePoints/targetPoints/faceNormals/facePts) are all
    /// std::vector<std::array<double,3>> (24 bytes/point), not float (12 bytes/point). This
    /// used to memcpy assuming 12-byte points against 24-byte data, corrupting every point.

    /// Serialize IcpChunk to binary buffer
    inline std::vector<uint8_t> serializeIcpChunk(const IcpChunk& chunk) {
        std::vector<uint8_t> buffer;

        /// Estimate total size for reservation
        size_t estimatedSize = sizeof(uint32_t) * 5 +  // chunk_id + 4 counts
            chunk.sourcePoints.size() * sizeof(double) * 3 +
            chunk.targetPoints.size() * sizeof(double) * 3 +
            chunk.faceIndices.size() * sizeof(uint64_t) +
            chunk.faceNormals.size() * sizeof(double) * 3 +
            sizeof(double) * 16 +    // transform
            80 +                     // config (overestimate)
            sizeof(float) * 12;      // bboxes
        buffer.reserve(estimatedSize);

        /// 1. Chunk ID
        buffer.resize(sizeof(uint32_t));
        std::memcpy(buffer.data(), &chunk.chunk_id, sizeof(uint32_t));

        /// 2. Source points
        uint32_t sourceCount = static_cast<uint32_t>(chunk.sourcePoints.size());
        size_t pos = buffer.size();
        buffer.resize(pos + sizeof(uint32_t) + sourceCount * sizeof(double) * 3);
        std::memcpy(buffer.data() + pos, &sourceCount, sizeof(uint32_t));
        pos += sizeof(uint32_t);
        if (sourceCount > 0) {
            std::memcpy(buffer.data() + pos, chunk.sourcePoints.data(),
                sourceCount * sizeof(double) * 3);
        }

        /// 3. Target points
        uint32_t targetCount = static_cast<uint32_t>(chunk.targetPoints.size());
        pos = buffer.size();
        buffer.resize(pos + sizeof(uint32_t) + targetCount * sizeof(double) * 3);
        std::memcpy(buffer.data() + pos, &targetCount, sizeof(uint32_t));
        pos += sizeof(uint32_t);
        if (targetCount > 0) {
            std::memcpy(buffer.data() + pos, chunk.targetPoints.data(),
                targetCount * sizeof(double) * 3);
        }

        /// 4. Face indices (size_t → uint64_t for cross-platform)
        uint32_t faceIdxCount = static_cast<uint32_t>(chunk.faceIndices.size());
        pos = buffer.size();
        buffer.resize(pos + sizeof(uint32_t) + faceIdxCount * sizeof(uint64_t));
        std::memcpy(buffer.data() + pos, &faceIdxCount, sizeof(uint32_t));
        pos += sizeof(uint32_t);
        if (faceIdxCount > 0) {
            for (uint32_t i = 0; i < faceIdxCount; ++i) {
                uint64_t idx = static_cast<uint64_t>(chunk.faceIndices[i]);
                std::memcpy(buffer.data() + pos + i * sizeof(uint64_t), &idx, sizeof(uint64_t));
            }
        }

        /// 5. Face normals (all normals from mesh faces)
        uint32_t faceNormalCount = static_cast<uint32_t>(chunk.faceNormals.size());
        pos = buffer.size();
        buffer.resize(pos + sizeof(uint32_t) + faceNormalCount * sizeof(double) * 3);
        std::memcpy(buffer.data() + pos, &faceNormalCount, sizeof(uint32_t));
        pos += sizeof(uint32_t);
        if (faceNormalCount > 0) {
            std::memcpy(buffer.data() + pos, chunk.faceNormals.data(),
                faceNormalCount * sizeof(double) * 3);
        }

        /// 6. Face points (one per face)
        uint32_t facePtsCount = static_cast<uint32_t>(chunk.facePts.size());
        pos = buffer.size();
        buffer.resize(pos + sizeof(uint32_t) + facePtsCount * sizeof(double) * 3);
        std::memcpy(buffer.data() + pos, &facePtsCount, sizeof(uint32_t));
        pos += sizeof(uint32_t);
        if (facePtsCount > 0) {
            std::memcpy(buffer.data() + pos, chunk.facePts.data(),
                facePtsCount * sizeof(double) * 3);
        }

        /// 7. Initial transform
        serializeTransform(chunk.initialTransform, buffer);

        /// 8. Config
        serializeConfig(chunk.config, buffer);

        /// 9. Bounding boxes
        pos = buffer.size();
        buffer.resize(pos + sizeof(float) * 12);
        float* bboxPtr = reinterpret_cast<float*>(buffer.data() + pos);
        bboxPtr[0] = chunk.source_min_x;
        bboxPtr[1] = chunk.source_min_y;
        bboxPtr[2] = chunk.source_min_z;
        bboxPtr[3] = chunk.source_max_x;
        bboxPtr[4] = chunk.source_max_y;
        bboxPtr[5] = chunk.source_max_z;
        bboxPtr[6] = chunk.target_min_x;
        bboxPtr[7] = chunk.target_min_y;
        bboxPtr[8] = chunk.target_min_z;
        bboxPtr[9] = chunk.target_max_x;
        bboxPtr[10] = chunk.target_max_y;
        bboxPtr[11] = chunk.target_max_z;

        return buffer;
    }

    /// Deserialize IcpChunk from binary data
    inline IcpChunk deserializeIcpChunk(const std::vector<uint8_t>& data) {
        IcpChunk chunk;
        size_t offset = 0;
        const uint8_t* ptr = data.data();

        /// 1. Chunk ID
        std::memcpy(&chunk.chunk_id, ptr + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);

        /// 2. Source points
        /// [Fix] points are std::array<double,3> (24 bytes/point), not float (12 bytes/point).
        uint32_t sourceCount;
        std::memcpy(&sourceCount, ptr + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        chunk.sourcePoints.resize(sourceCount);
        if (sourceCount > 0) {
            std::memcpy(chunk.sourcePoints.data(), ptr + offset,
                sourceCount * sizeof(double) * 3);
            offset += sourceCount * sizeof(double) * 3;
        }

        /// 3. Target points
        uint32_t targetCount;
        std::memcpy(&targetCount, ptr + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        chunk.targetPoints.resize(targetCount);
        if (targetCount > 0) {
            std::memcpy(chunk.targetPoints.data(), ptr + offset,
                targetCount * sizeof(double) * 3);
            offset += targetCount * sizeof(double) * 3;
        }

        /// 4. Face indices
        uint32_t faceIdxCount;
        std::memcpy(&faceIdxCount, ptr + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        chunk.faceIndices.resize(faceIdxCount);
        if (faceIdxCount > 0) {
            for (uint32_t i = 0; i < faceIdxCount; ++i) {
                uint64_t idx;
                std::memcpy(&idx, ptr + offset + i * sizeof(uint64_t), sizeof(uint64_t));
                chunk.faceIndices[i] = static_cast<size_t>(idx);
            }
            offset += faceIdxCount * sizeof(uint64_t);
        }

        /// 5. Face normals
        uint32_t faceNormalCount;
        std::memcpy(&faceNormalCount, ptr + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        chunk.faceNormals.resize(faceNormalCount);
        if (faceNormalCount > 0) {
            std::memcpy(chunk.faceNormals.data(), ptr + offset,
                faceNormalCount * sizeof(double) * 3);
            offset += faceNormalCount * sizeof(double) * 3;
        }

        /// 6. Face points
        /// [Fix] This used to be read *before* the initial transform below, but
        /// serializeIcpChunk() writes face points *before* the transform -- the field order
        /// here was swapped relative to the writer, corrupting both fields.
        uint32_t facePtsCount;
        std::memcpy(&facePtsCount, ptr + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        chunk.facePts.resize(facePtsCount);
        if (facePtsCount > 0) {
            std::memcpy(chunk.facePts.data(), ptr + offset,
                facePtsCount * sizeof(double) * 3);
            offset += facePtsCount * sizeof(double) * 3;
        }

        /// 7. Initial transform
        chunk.initialTransform = deserializeTransform(ptr, offset);

        /// 8. Config
        chunk.config = deserializeConfig(ptr, offset);

        /// 9. Bounding boxes
        const float* bboxPtr = reinterpret_cast<const float*>(ptr + offset);
        chunk.source_min_x = bboxPtr[0];
        chunk.source_min_y = bboxPtr[1];
        chunk.source_min_z = bboxPtr[2];
        chunk.source_max_x = bboxPtr[3];
        chunk.source_max_y = bboxPtr[4];
        chunk.source_max_z = bboxPtr[5];
        chunk.target_min_x = bboxPtr[6];
        chunk.target_min_y = bboxPtr[7];
        chunk.target_min_z = bboxPtr[8];
        chunk.target_max_x = bboxPtr[9];
        chunk.target_max_y = bboxPtr[10];
        chunk.target_max_z = bboxPtr[11];

        return chunk;
    }

    //=========================================================================
    // IcpResult Serialization
    //=========================================================================
    
    /// Binary format:
    /// [chunk_id: 4 bytes]
    /// [transform: 64 bytes]
    /// [localTransform: 64 bytes]
    /// [finalRMSE, initialRMSE: 8 bytes]
    /// [actualIterations, correspondenceCount, sourcePointCount, targetPointCount: 16 bytes]
    /// [converged, success: 2 bytes]
    /// [processingTimeMs: 8 bytes (double)]
    /// [errorMessage length: 4 bytes][errorMessage: variable]

    /// Serialize IcpResult to binary buffer
    inline std::vector<uint8_t> serializeIcpResult(const IcpResult& result) {
        std::vector<uint8_t> buffer;
        
        size_t estimatedSize = sizeof(uint32_t) +      // chunk_id
                              sizeof(double) * 32 +    // 2 transforms
                              sizeof(double) * 2 +     // RMSE values
                              sizeof(int) * 4 +        // int counters
                              sizeof(uint8_t) * 2 +    // bool flags
                              sizeof(double) +         // processing time
                              sizeof(uint32_t) +       // error message length
                              result.errorMessage.size();
        buffer.reserve(estimatedSize);

        /// 1. Chunk ID
        buffer.resize(sizeof(uint32_t));
        std::memcpy(buffer.data(), &result.chunk_id, sizeof(uint32_t));

        /// 2. Transform matrices
        serializeTransform(result.transform, buffer);
        serializeTransform(result.localTransform, buffer);

        /// 3. RMSE values
        /// [Fix] finalRMSE/initialRMSE are double, not float.
        size_t pos = buffer.size();
        buffer.resize(pos + sizeof(double) * 2);
        std::memcpy(buffer.data() + pos, &result.finalRMSE, sizeof(double));
        std::memcpy(buffer.data() + pos + sizeof(double), &result.initialRMSE, sizeof(double));
        
        /// 4. Integer counters
        pos = buffer.size();
        buffer.resize(pos + sizeof(int) * 4);
        int* intPtr = reinterpret_cast<int*>(buffer.data() + pos);
        intPtr[0] = result.actualIterations;
        intPtr[1] = result.correspondenceCount;
        intPtr[2] = result.sourcePointCount;
        intPtr[3] = result.targetPointCount;
        
        /// 5. Bool flags
        buffer.push_back(result.converged ? 1 : 0);
        buffer.push_back(result.success ? 1 : 0);
        
        /// 6. Processing time
        pos = buffer.size();
        buffer.resize(pos + sizeof(double));
        std::memcpy(buffer.data() + pos, &result.processingTimeMs, sizeof(double));
        
        /// 7. Error message (length + data)
        uint32_t msgLen = static_cast<uint32_t>(result.errorMessage.size());
        pos = buffer.size();
        buffer.resize(pos + sizeof(uint32_t) + msgLen);
        std::memcpy(buffer.data() + pos, &msgLen, sizeof(uint32_t));
        if (msgLen > 0) {
            std::memcpy(buffer.data() + pos + sizeof(uint32_t), 
                       result.errorMessage.data(), msgLen);
        }
        
        return buffer;
    }

    /// Deserialize IcpResult from binary data
    inline IcpResult deserializeIcpResult(const std::vector<uint8_t>& data) {
        IcpResult result;
        size_t offset = 0;
        const uint8_t* ptr = data.data();
        
        /// 1. Chunk ID
        std::memcpy(&result.chunk_id, ptr + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        
        /// 2. Transform matrices
        result.transform = deserializeTransform(ptr, offset);
        result.localTransform = deserializeTransform(ptr, offset);
        
        /// 3. RMSE values
        std::memcpy(&result.finalRMSE, ptr + offset, sizeof(double));
        offset += sizeof(double);
        std::memcpy(&result.initialRMSE, ptr + offset, sizeof(double));
        offset += sizeof(double);
        
        /// 4. Integer counters
        const int* intPtr = reinterpret_cast<const int*>(ptr + offset);
        result.actualIterations = intPtr[0];
        result.correspondenceCount = intPtr[1];
        result.sourcePointCount = intPtr[2];
        result.targetPointCount = intPtr[3];
        offset += sizeof(int) * 4;
        
        /// 5. Bool flags
        result.converged = (ptr[offset++] != 0);
        result.success = (ptr[offset++] != 0);
        
        /// 6. Processing time
        std::memcpy(&result.processingTimeMs, ptr + offset, sizeof(double));
        offset += sizeof(double);
        
        /// 7. Error message
        uint32_t msgLen;
        std::memcpy(&msgLen, ptr + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        if (msgLen > 0 && offset + msgLen <= data.size()) {
            result.errorMessage.assign(
                reinterpret_cast<const char*>(ptr + offset), msgLen);
        }
        
        return result;
    }

    //=========================================================================
    // Utility Functions
    //=========================================================================

    /// Get serialized size of IcpChunk (approximate)
    inline size_t getSerializedSize(const IcpChunk& chunk) {
        return sizeof(uint32_t) * 5 +  // chunk_id + 4 counts
            chunk.sourcePoints.size() * sizeof(double) * 3 +
            chunk.targetPoints.size() * sizeof(double) * 3 +
            chunk.faceIndices.size() * sizeof(uint64_t) +
            chunk.faceNormals.size() * sizeof(double) * 3 +
            sizeof(double) * 16 +   // transform
            80 +                    // config
            sizeof(float) * 12;     // bboxes
    }

    /// Get serialized size of IcpResult (approximate)
    inline size_t getSerializedSize(const IcpResult& result) {
        return sizeof(uint32_t) +      // chunk_id
               sizeof(double) * 32 +   // 2 transforms
               sizeof(double) * 2 +    // RMSE
               sizeof(int) * 4 +       // counters
               sizeof(uint8_t) * 2 +   // flags
               sizeof(double) +        // time
               sizeof(uint32_t) +      // msg length
               result.errorMessage.size();
    }

} // namespace icp


namespace DPApp {
namespace NetworkUtils {

    //=========================================================================
    // NetworkUtils Extensions for ICP (DPApp namespace)
    //=========================================================================

    /// Serialize ICP chunk for network transmission
    inline std::vector<uint8_t> serializeIcpChunk(const icp::IcpChunk& chunk) {
        return icp::serializeIcpChunk(chunk);
    }

    /// Deserialize ICP chunk from network data
    inline icp::IcpChunk deserializeIcpChunk(const std::vector<uint8_t>& data) {
        return icp::deserializeIcpChunk(data);
    }

    /// Serialize ICP result for network transmission
    inline std::vector<uint8_t> serializeIcpResult(const icp::IcpResult& result) {
        return icp::serializeIcpResult(result);
    }

    /// Deserialize ICP result from network data
    inline icp::IcpResult deserializeIcpResult(const std::vector<uint8_t>& data) {
        return icp::deserializeIcpResult(data);
    }

} // namespace NetworkUtils
} // namespace DPApp
