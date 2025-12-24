#pragma once

/// ============================================================================
/// IcpProcessor.hpp - ICP Task Processor for Slave
/// ============================================================================
/// 
/// This header provides the ICP task processor that runs on Slave nodes.
/// Integrates with the existing TaskProcessor pattern in DPApp.
/// 
/// Location: F:\repository\DPApp\include\bimtree\IcpProcessor.hpp
/// ============================================================================

#include "../IcpTypes.h"
#include "IcpCore.hpp"
#include "IcpSerialization.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <string>

namespace icp {

    /// ICP Task Processor for Slave nodes
    class IcpProcessor {
    public:
        IcpProcessor() = default;
        ~IcpProcessor() = default;

        /// Process ICP fine alignment task
        /// @param chunk ICP chunk data received from Master
        /// @param cancelRequested Atomic flag for task cancellation
        /// @return ICP result
        IcpResult processFineAlignment(
            const IcpChunk& chunk,
            std::atomic<bool>& cancelRequested)
        {
            IcpResult result;
            result.chunk_id = chunk.chunk_id;

            auto startTime = std::chrono::high_resolution_clock::now();

            try {
                /// Validate input
                if (!chunk.isValid()) {
                    result.success = false;
                    result.errorMessage = "Invalid chunk data";
                    return result;
                }

                /// Log start (if verbose)
                if (chunk.config.verbose) {
                    std::cout << "[ICP] Processing chunk " << chunk.chunk_id
                              << " (source: " << chunk.sourcePoints.size()
                              << ", target: " << chunk.targetPoints.size() << " pts)\n";
                }

                /// Run ICP algorithm
                result = runIcp(chunk, cancelRequested);

                /// Log completion (if verbose)
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

        /// Process ICP task from serialized data (for network integration)
        /// @param chunkData Serialized IcpChunk data
        /// @param cancelRequested Cancellation flag
        /// @return Serialized IcpResult data
        std::vector<uint8_t> processTaskFromBytes(
            const std::vector<uint8_t>& chunkData,
            std::atomic<bool>& cancelRequested)
        {
            try {
                /// Deserialize chunk
                IcpChunk chunk = deserializeIcpChunk(chunkData);

                /// Process
                IcpResult result = processFineAlignment(chunk, cancelRequested);

                /// Serialize and return result
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

    /// ICP Processors namespace (integrates with existing TaskProcessor pattern)
    namespace IcpProcessors {

        /// Process ICP fine alignment task
        /// Entry point for SlaveApplication task processing
        inline icp::IcpResult processFineAlignment(
            const icp::IcpChunk& chunk,
            std::atomic<bool>& cancelRequested)
        {
            icp::IcpProcessor processor;
            return processor.processFineAlignment(chunk, cancelRequested);
        }

        /// Check if task type is ICP-related
        inline bool isIcpTask(TaskType type) {
            return type == TaskType::ICP_COARSE_ALIGNMENT ||
                   type == TaskType::ICP_FINE_ALIGNMENT ||
                   type == TaskType::ICP_APPLY_TRANSFORM;
        }

        /// Process ICP task from serialized chunk data
        /// @param chunkData Serialized IcpChunk
        /// @param cancelRequested Cancellation flag
        /// @return IcpResult
        inline icp::IcpResult processIcpTask(
            const std::vector<uint8_t>& chunkData,
            std::atomic<bool>& cancelRequested)
        {
            icp::IcpChunk chunk = icp::deserializeIcpChunk(chunkData);
            return processFineAlignment(chunk, cancelRequested);
        }

        /// Process ICP task and return serialized result
        /// @param chunkData Serialized IcpChunk
        /// @param cancelRequested Cancellation flag
        /// @return Serialized IcpResult
        inline std::vector<uint8_t> processIcpTaskToBytes(
            const std::vector<uint8_t>& chunkData,
            std::atomic<bool>& cancelRequested)
        {
            icp::IcpResult result = processIcpTask(chunkData, cancelRequested);
            return icp::serializeIcpResult(result);
        }

    } // namespace IcpProcessors

} // namespace DPApp
