#pragma once

/// ============================================================================
/// IcpSerialization.hpp - ICP 데이터 직렬화/역직렬화
/// ============================================================================
///
/// 이 헤더는 Master와 Slave 사이의 네트워크 전송을 위해 ICP 데이터 구조체를
/// 직렬화/역직렬화하는 함수들을 제공한다.
///
/// 위치: D:\repository\DPApp_II\include\bimtree\IcpSerialization.hpp
/// ============================================================================

#include "../IcpTypes.h"

#include <vector>
#include <cstdint>
#include <cstring>
#include <string>

namespace icp {

    //=========================================================================
    // Transform4x4 직렬화 (128바이트)
    //=========================================================================

    /// Transform4x4을 바이너리 버퍼로 직렬화
    /// @param transform 직렬화할 변환 행렬
    /// @param buffer 출력 버퍼 (뒤에 이어붙임)
    /// [수정 이력] Transform4x4::m은 double[16](128바이트)인데 float[16](64바이트)로
    /// 착각하여 64바이트만 memcpy하던 버그가 있었다 -- 행렬 값의 절반이 조용히
    /// 잘려나갔던 것이다. 분산 미세정합이 구현되기 전까지는 ICP가 항상
    /// Master 단독으로만 실행되어 IcpChunk/IcpResult를 네트워크로 전송할 일이
    /// 없었기 때문에 이 경로가 실제로 실행된 적이 없어 발견되지 못했었다.
    inline void serializeTransform(const Transform4x4& transform, std::vector<uint8_t>& buffer) {
        size_t startSize = buffer.size();
        buffer.resize(startSize + sizeof(double) * 16);
        std::memcpy(buffer.data() + startSize, transform.m, sizeof(double) * 16);
    }

    /// 바이너리 데이터로부터 Transform4x4을 역직렬화
    /// @param data 입력 데이터 포인터
    /// @param offset 현재 오프셋 (읽은 뒤 갱신됨)
    /// @return 역직렬화된 Transform4x4
    inline Transform4x4 deserializeTransform(const uint8_t* data, size_t& offset) {
        Transform4x4 result;
        std::memcpy(result.m, data + offset, sizeof(double) * 16);
        offset += sizeof(double) * 16;
        return result;
    }

    //=========================================================================
    // IcpConfig 직렬화 (약 68바이트)
    //=========================================================================

    /// IcpConfig를 바이너리 버퍼로 직렬화
    /// [수정 이력] 5개의 임계값 필드(convergenceThreshold, maxCorrespondenceDistance,
    /// outlierRejectionThreshold, normalAngleThreshold, pseudoPointGridSize)는
    /// IcpConfig에서 모두 `double`인데, 예전에는 각 필드의 8바이트 중 4바이트만
    /// memcpy하던 버그가 있었다.
    inline void serializeConfig(const IcpConfig& config, std::vector<uint8_t>& buffer) {
        size_t startSize = buffer.size();

        /// 필요한 크기 계산
        size_t configSize = sizeof(int) * 3 +      // maxIterations, downsampleRatio, minCorrespondences
                           sizeof(double) * 5 +    // double 임계값 5개
                           sizeof(uint8_t) * 2;    // bool 플래그 2개

        buffer.resize(startSize + configSize);
        uint8_t* ptr = buffer.data() + startSize;

        /// 정수형 필드 기록
        std::memcpy(ptr, &config.maxIterations, sizeof(int));
        ptr += sizeof(int);
        std::memcpy(ptr, &config.downsampleRatio, sizeof(int));
        ptr += sizeof(int);
        std::memcpy(ptr, &config.minCorrespondences, sizeof(int));
        ptr += sizeof(int);

        /// double 필드 기록
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

        /// bool 필드를 uint8_t로 기록
        *ptr++ = config.useNormalFiltering ? 1 : 0;
        *ptr++ = config.verbose ? 1 : 0;
    }

    /// 바이너리 데이터로부터 IcpConfig를 역직렬화
    inline IcpConfig deserializeConfig(const uint8_t* data, size_t& offset) {
        IcpConfig config;

        /// 정수형 필드 읽기
        std::memcpy(&config.maxIterations, data + offset, sizeof(int));
        offset += sizeof(int);
        std::memcpy(&config.downsampleRatio, data + offset, sizeof(int));
        offset += sizeof(int);
        std::memcpy(&config.minCorrespondences, data + offset, sizeof(int));
        offset += sizeof(int);

        /// double 필드 읽기
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

        /// bool 필드 읽기
        config.useNormalFiltering = (data[offset++] != 0);
        config.verbose = (data[offset++] != 0);

        return config;
    }

    //=========================================================================
    // IcpChunk 직렬화
    //=========================================================================

    /// 바이너리 포맷:
    /// [chunk_id: 4바이트]
    /// [sourcePoints 개수: 4바이트][sourcePoints 데이터: 개수 * 12바이트]
    /// [targetPoints 개수: 4바이트][targetPoints 데이터: 개수 * 12바이트]
    /// [targetNormals 개수: 4바이트][targetNormals 데이터: 개수 * 12바이트]
    /// [initialTransform: 128바이트]
    /// [config: 약 68바이트]
    /// [source bbox: 24바이트 (float 6개)]
    /// [target bbox: 24바이트 (float 6개)]
    /// [offset: 24바이트 (double 3개)]
    ///
    /// IcpChunk의 점 배열(sourcePoints/targetPoints/faceNormals/facePts)은
    /// std::vector<std::array<float,3>>(점당 12바이트)이며, (offsetX, offsetY,
    /// offsetZ)만큼 이동되어(shifted) 있다 -- 이를 사용하는 쪽(runIcp)에서
    /// double로 승격시키고 offset을 다시 더한다.

    /// IcpChunk를 바이너리 버퍼로 직렬화
    inline std::vector<uint8_t> serializeIcpChunk(const IcpChunk& chunk) {
        std::vector<uint8_t> buffer;

        /// 미리 예약할 전체 크기 추정
        size_t estimatedSize = sizeof(uint32_t) * 5 +  // chunk_id + 개수 필드 4개
            chunk.sourcePoints.size() * sizeof(float) * 3 +
            chunk.targetPoints.size() * sizeof(float) * 3 +
            chunk.faceIndices.size() * sizeof(uint64_t) +
            chunk.faceNormals.size() * sizeof(float) * 3 +
            sizeof(double) * 16 +    // transform
            80 +                     // config (여유있게 추정)
            sizeof(float) * 12 +     // bbox
            sizeof(double) * 3;      // offset
        buffer.reserve(estimatedSize);

        /// 1. 청크 ID
        buffer.resize(sizeof(uint32_t));
        std::memcpy(buffer.data(), &chunk.chunk_id, sizeof(uint32_t));

        /// 2. source 점들
        uint32_t sourceCount = static_cast<uint32_t>(chunk.sourcePoints.size());
        size_t pos = buffer.size();
        buffer.resize(pos + sizeof(uint32_t) + sourceCount * sizeof(float) * 3);
        std::memcpy(buffer.data() + pos, &sourceCount, sizeof(uint32_t));
        pos += sizeof(uint32_t);
        if (sourceCount > 0) {
            std::memcpy(buffer.data() + pos, chunk.sourcePoints.data(),
                sourceCount * sizeof(float) * 3);
        }

        /// 3. target 점들
        uint32_t targetCount = static_cast<uint32_t>(chunk.targetPoints.size());
        pos = buffer.size();
        buffer.resize(pos + sizeof(uint32_t) + targetCount * sizeof(float) * 3);
        std::memcpy(buffer.data() + pos, &targetCount, sizeof(uint32_t));
        pos += sizeof(uint32_t);
        if (targetCount > 0) {
            std::memcpy(buffer.data() + pos, chunk.targetPoints.data(),
                targetCount * sizeof(float) * 3);
        }

        /// 4. face 인덱스 (플랫폼 간 호환을 위해 size_t → uint64_t로 변환)
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

        /// 5. face normal (메시의 모든 face로부터의 normal)
        uint32_t faceNormalCount = static_cast<uint32_t>(chunk.faceNormals.size());
        pos = buffer.size();
        buffer.resize(pos + sizeof(uint32_t) + faceNormalCount * sizeof(float) * 3);
        std::memcpy(buffer.data() + pos, &faceNormalCount, sizeof(uint32_t));
        pos += sizeof(uint32_t);
        if (faceNormalCount > 0) {
            std::memcpy(buffer.data() + pos, chunk.faceNormals.data(),
                faceNormalCount * sizeof(float) * 3);
        }

        /// 6. face 대표점 (face당 1개)
        uint32_t facePtsCount = static_cast<uint32_t>(chunk.facePts.size());
        pos = buffer.size();
        buffer.resize(pos + sizeof(uint32_t) + facePtsCount * sizeof(float) * 3);
        std::memcpy(buffer.data() + pos, &facePtsCount, sizeof(uint32_t));
        pos += sizeof(uint32_t);
        if (facePtsCount > 0) {
            std::memcpy(buffer.data() + pos, chunk.facePts.data(),
                facePtsCount * sizeof(float) * 3);
        }

        /// 7. 초기 변환
        serializeTransform(chunk.initialTransform, buffer);

        /// 8. 설정값
        serializeConfig(chunk.config, buffer);

        /// 9. 재배치(rebase) offset
        pos = buffer.size();
        buffer.resize(pos + sizeof(double) * 3);
        std::memcpy(buffer.data() + pos, &chunk.offsetX, sizeof(double));
        std::memcpy(buffer.data() + pos + sizeof(double), &chunk.offsetY, sizeof(double));
        std::memcpy(buffer.data() + pos + sizeof(double) * 2, &chunk.offsetZ, sizeof(double));

        /// 10. 바운딩 박스 (double 원본을 float로 축소 저장 -- 청크 분배/선별
        /// 용도로만 쓰이므로 근사 정밀도로 충분하다)
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

    /// 바이너리 데이터로부터 IcpChunk를 역직렬화
    inline IcpChunk deserializeIcpChunk(const std::vector<uint8_t>& data) {
        IcpChunk chunk;
        size_t offset = 0;
        const uint8_t* ptr = data.data();

        /// 1. 청크 ID
        std::memcpy(&chunk.chunk_id, ptr + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);

        /// 2. source 점들 (float, 점당 12바이트)
        uint32_t sourceCount;
        std::memcpy(&sourceCount, ptr + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        chunk.sourcePoints.resize(sourceCount);
        if (sourceCount > 0) {
            std::memcpy(chunk.sourcePoints.data(), ptr + offset,
                sourceCount * sizeof(float) * 3);
            offset += sourceCount * sizeof(float) * 3;
        }

        /// 3. target 점들
        uint32_t targetCount;
        std::memcpy(&targetCount, ptr + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        chunk.targetPoints.resize(targetCount);
        if (targetCount > 0) {
            std::memcpy(chunk.targetPoints.data(), ptr + offset,
                targetCount * sizeof(float) * 3);
            offset += targetCount * sizeof(float) * 3;
        }

        /// 4. face 인덱스
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

        /// 5. face normal
        uint32_t faceNormalCount;
        std::memcpy(&faceNormalCount, ptr + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        chunk.faceNormals.resize(faceNormalCount);
        if (faceNormalCount > 0) {
            std::memcpy(chunk.faceNormals.data(), ptr + offset,
                faceNormalCount * sizeof(float) * 3);
            offset += faceNormalCount * sizeof(float) * 3;
        }

        /// 6. face 대표점
        /// [주의] 여기 필드 순서는 반드시 serializeIcpChunk()와 일치해야 한다
        /// (face 대표점이 transform *앞에* 와야 함) -- 예전 버전에서 이 둘의
        /// 순서가 뒤바뀌어 있어서 두 필드 모두 깨진 값을 읽었던 적이 있다.
        uint32_t facePtsCount;
        std::memcpy(&facePtsCount, ptr + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        chunk.facePts.resize(facePtsCount);
        if (facePtsCount > 0) {
            std::memcpy(chunk.facePts.data(), ptr + offset,
                facePtsCount * sizeof(float) * 3);
            offset += facePtsCount * sizeof(float) * 3;
        }

        /// 7. 초기 변환
        chunk.initialTransform = deserializeTransform(ptr, offset);

        /// 8. 설정값
        chunk.config = deserializeConfig(ptr, offset);

        /// 9. 재배치(rebase) offset
        std::memcpy(&chunk.offsetX, ptr + offset, sizeof(double));
        std::memcpy(&chunk.offsetY, ptr + offset + sizeof(double), sizeof(double));
        std::memcpy(&chunk.offsetZ, ptr + offset + sizeof(double) * 2, sizeof(double));
        offset += sizeof(double) * 3;

        /// 10. 바운딩 박스
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
    // IcpResult 직렬화
    //=========================================================================

    /// 바이너리 포맷:
    /// [chunk_id: 4바이트]
    /// [transform: 128바이트]
    /// [localTransform: 128바이트]
    /// [finalRMSE, initialRMSE: 16바이트]
    /// [actualIterations, correspondenceCount, sourcePointCount, targetPointCount: 16바이트]
    /// [converged, success: 2바이트]
    /// [processingTimeMs: 8바이트 (double)]
    /// [errorMessage 길이: 4바이트][errorMessage: 가변 길이]

    /// IcpResult를 바이너리 버퍼로 직렬화
    inline std::vector<uint8_t> serializeIcpResult(const IcpResult& result) {
        std::vector<uint8_t> buffer;

        size_t estimatedSize = sizeof(uint32_t) +      // chunk_id
                              sizeof(double) * 32 +    // transform 2개
                              sizeof(double) * 2 +     // RMSE 값들
                              sizeof(int) * 4 +        // int 카운터들
                              sizeof(uint8_t) * 2 +    // bool 플래그들
                              sizeof(double) +         // 처리 시간
                              sizeof(uint32_t) +       // 오류 메시지 길이
                              result.errorMessage.size();
        buffer.reserve(estimatedSize);

        /// 1. 청크 ID
        buffer.resize(sizeof(uint32_t));
        std::memcpy(buffer.data(), &result.chunk_id, sizeof(uint32_t));

        /// 2. 변환 행렬들
        serializeTransform(result.transform, buffer);
        serializeTransform(result.localTransform, buffer);

        /// 3. RMSE 값들
        /// [수정 이력] finalRMSE/initialRMSE는 float가 아니라 double이다.
        size_t pos = buffer.size();
        buffer.resize(pos + sizeof(double) * 2);
        std::memcpy(buffer.data() + pos, &result.finalRMSE, sizeof(double));
        std::memcpy(buffer.data() + pos + sizeof(double), &result.initialRMSE, sizeof(double));

        /// 4. 정수 카운터들
        pos = buffer.size();
        buffer.resize(pos + sizeof(int) * 4);
        int* intPtr = reinterpret_cast<int*>(buffer.data() + pos);
        intPtr[0] = result.actualIterations;
        intPtr[1] = result.correspondenceCount;
        intPtr[2] = result.sourcePointCount;
        intPtr[3] = result.targetPointCount;

        /// 5. bool 플래그들
        buffer.push_back(result.converged ? 1 : 0);
        buffer.push_back(result.success ? 1 : 0);

        /// 6. 처리 시간
        pos = buffer.size();
        buffer.resize(pos + sizeof(double));
        std::memcpy(buffer.data() + pos, &result.processingTimeMs, sizeof(double));

        /// 7. 오류 메시지 (길이 + 데이터)
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

    /// 바이너리 데이터로부터 IcpResult를 역직렬화
    inline IcpResult deserializeIcpResult(const std::vector<uint8_t>& data) {
        IcpResult result;
        size_t offset = 0;
        const uint8_t* ptr = data.data();

        /// 1. 청크 ID
        std::memcpy(&result.chunk_id, ptr + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);

        /// 2. 변환 행렬들
        result.transform = deserializeTransform(ptr, offset);
        result.localTransform = deserializeTransform(ptr, offset);

        /// 3. RMSE 값들
        std::memcpy(&result.finalRMSE, ptr + offset, sizeof(double));
        offset += sizeof(double);
        std::memcpy(&result.initialRMSE, ptr + offset, sizeof(double));
        offset += sizeof(double);

        /// 4. 정수 카운터들
        const int* intPtr = reinterpret_cast<const int*>(ptr + offset);
        result.actualIterations = intPtr[0];
        result.correspondenceCount = intPtr[1];
        result.sourcePointCount = intPtr[2];
        result.targetPointCount = intPtr[3];
        offset += sizeof(int) * 4;

        /// 5. bool 플래그들
        result.converged = (ptr[offset++] != 0);
        result.success = (ptr[offset++] != 0);

        /// 6. 처리 시간
        std::memcpy(&result.processingTimeMs, ptr + offset, sizeof(double));
        offset += sizeof(double);

        /// 7. 오류 메시지
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
    // 유틸리티 함수
    //=========================================================================

    /// IcpChunk의 직렬화 크기 계산 (근사치)
    inline size_t getSerializedSize(const IcpChunk& chunk) {
        return sizeof(uint32_t) * 5 +  // chunk_id + 개수 필드 4개
            chunk.sourcePoints.size() * sizeof(float) * 3 +
            chunk.targetPoints.size() * sizeof(float) * 3 +
            chunk.faceIndices.size() * sizeof(uint64_t) +
            chunk.faceNormals.size() * sizeof(float) * 3 +
            sizeof(double) * 16 +   // transform
            80 +                    // config
            sizeof(float) * 12 +    // bbox
            sizeof(double) * 3;     // offset
    }

    /// IcpResult의 직렬화 크기 계산 (근사치)
    inline size_t getSerializedSize(const IcpResult& result) {
        return sizeof(uint32_t) +      // chunk_id
               sizeof(double) * 32 +   // transform 2개
               sizeof(double) * 2 +    // RMSE
               sizeof(int) * 4 +       // 카운터
               sizeof(uint8_t) * 2 +   // 플래그
               sizeof(double) +        // 시간
               sizeof(uint32_t) +      // 메시지 길이
               result.errorMessage.size();
    }

} // namespace icp


namespace DPApp {
namespace NetworkUtils {

    //=========================================================================
    // ICP를 위한 NetworkUtils 확장 (DPApp 네임스페이스)
    //=========================================================================

    /// 네트워크 전송을 위해 ICP 청크를 직렬화
    inline std::vector<uint8_t> serializeIcpChunk(const icp::IcpChunk& chunk) {
        return icp::serializeIcpChunk(chunk);
    }

    /// 네트워크 데이터로부터 ICP 청크를 역직렬화
    inline icp::IcpChunk deserializeIcpChunk(const std::vector<uint8_t>& data) {
        return icp::deserializeIcpChunk(data);
    }

    /// 네트워크 전송을 위해 ICP 결과를 직렬화
    inline std::vector<uint8_t> serializeIcpResult(const icp::IcpResult& result) {
        return icp::serializeIcpResult(result);
    }

    /// 네트워크 데이터로부터 ICP 결과를 역직렬화
    inline icp::IcpResult deserializeIcpResult(const std::vector<uint8_t>& data) {
        return icp::deserializeIcpResult(data);
    }

} // namespace NetworkUtils
} // namespace DPApp
