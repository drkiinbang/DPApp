#pragma once

/// ============================================================================
/// IcpProcessor.hpp - Slave용 ICP 태스크 처리기
/// ============================================================================
///
/// 이 헤더는 Slave 노드에서 실행되는 ICP 태스크 처리기를 제공한다.
/// DPApp에 이미 있는 TaskProcessor 패턴과 통합된다.
///
/// 위치: D:\repository\DPApp_II\include\bimtree\IcpProcessor.hpp
/// ============================================================================

#include "../IcpTypes.h"
#include "IcpCore.hpp"
#include "IcpSerialization.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <string>

namespace icp {

    /// Slave 노드용 ICP 태스크 처리기
    class IcpProcessor {
    public:
        IcpProcessor() = default;
        ~IcpProcessor() = default;

        /// ICP 미세 정합(fine alignment) 태스크 처리
        /// @param chunk Master로부터 받은 ICP 청크 데이터
        /// @param cancelRequested 태스크 취소용 atomic 플래그
        /// @return ICP 결과
        IcpResult processFineAlignment(
            const IcpChunk& chunk,
            std::atomic<bool>& cancelRequested)
        {
            IcpResult result;
            result.chunk_id = chunk.chunk_id;

            auto startTime = std::chrono::high_resolution_clock::now();

            try {
                /// 입력 유효성 검사
                if (!chunk.isValid()) {
                    result.success = false;
                    result.errorMessage = "Invalid chunk data";
                    return result;
                }

                /// 시작 로그 (verbose 모드일 때만)
                if (chunk.config.verbose) {
                    std::cout << "[ICP] Processing chunk " << chunk.chunk_id
                              << " (source: " << chunk.sourcePoints.size()
                              << ", target: " << chunk.targetPoints.size() << " pts)\n";
                }

                /// ICP 알고리즘 실행
                result = runIcp(chunk, cancelRequested);

                /// 완료 로그 (verbose 모드일 때만)
                if (chunk.config.verbose) {
                    std::cout << "[ICP] Chunk " << chunk.chunk_id << " completed: "
                              << (result.success ? "SUCCESS" : "FAILED")
                              << ", RMSE: " << result.finalRMSE
                              << ", iterations: " << result.actualIterations
                              << ", time: " << result.processingTimeMs << "ms\n";
                }
            }
            catch (const std::exception& e) {
                result.success = false;
                result.errorMessage = std::string("Exception: ") + e.what();
            }
            catch (...) {
                result.success = false;
                result.errorMessage = "Unknown exception";
            }

            auto endTime = std::chrono::high_resolution_clock::now();
            result.processingTimeMs = std::chrono::duration<double, std::milli>(
                endTime - startTime).count();

            return result;
        }

        /// 직렬화된 데이터로부터 ICP 태스크를 처리한다 (네트워크 연동용)
        /// @param chunkData 직렬화된 IcpChunk 데이터
        /// @param cancelRequested 취소 플래그
        /// @return 직렬화된 IcpResult 데이터
        std::vector<uint8_t> processTaskFromBytes(
            const std::vector<uint8_t>& chunkData,
            std::atomic<bool>& cancelRequested)
        {
            try {
                /// 청크 역직렬화
                IcpChunk chunk = deserializeIcpChunk(chunkData);

                /// 처리
                IcpResult result = processFineAlignment(chunk, cancelRequested);

                /// 결과를 직렬화하여 반환
                return serializeIcpResult(result);
            }
            catch (const std::exception& e) {
                IcpResult errorResult;
                errorResult.success = false;
                errorResult.errorMessage = std::string("Deserialization error: ") + e.what();
                return serializeIcpResult(errorResult);
            }
        }
    };

} // namespace icp


namespace DPApp {

    /// ICP 처리기 네임스페이스 (기존 TaskProcessor 패턴과 통합)
    namespace IcpProcessors {

        /// ICP 미세 정합(fine alignment) 태스크 처리
        /// SlaveApplication 태스크 처리의 진입점
        inline icp::IcpResult processFineAlignment(
            const icp::IcpChunk& chunk,
            std::atomic<bool>& cancelRequested)
        {
            icp::IcpProcessor processor;
            return processor.processFineAlignment(chunk, cancelRequested);
        }

        /// 태스크 타입이 ICP 관련인지 확인
        inline bool isIcpTask(TaskType type) {
            return type == TaskType::ICP_COARSE_ALIGNMENT ||
                   type == TaskType::ICP_FINE_ALIGNMENT ||
                   type == TaskType::ICP_APPLY_TRANSFORM;
        }

        /// 직렬화된 청크 데이터로부터 ICP 태스크를 처리
        /// @param chunkData 직렬화된 IcpChunk
        /// @param cancelRequested 취소 플래그
        /// @return IcpResult
        inline icp::IcpResult processIcpTask(
            const std::vector<uint8_t>& chunkData,
            std::atomic<bool>& cancelRequested)
        {
            icp::IcpChunk chunk = icp::deserializeIcpChunk(chunkData);
            return processFineAlignment(chunk, cancelRequested);
        }

        /// ICP 태스크를 처리하고 직렬화된 결과를 반환
        /// @param chunkData 직렬화된 IcpChunk
        /// @param cancelRequested 취소 플래그
        /// @return 직렬화된 IcpResult
        inline std::vector<uint8_t> processIcpTaskToBytes(
            const std::vector<uint8_t>& chunkData,
            std::atomic<bool>& cancelRequested)
        {
            icp::IcpResult result = processIcpTask(chunkData, cancelRequested);
            return icp::serializeIcpResult(result);
        }

    } // namespace IcpProcessors

} // namespace DPApp
