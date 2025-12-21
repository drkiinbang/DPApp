#pragma once
/// @file TestTask.h
/// @brief Task types and processing logic for testing
/// 
/// Implementation of test Tasks for verifying the Master/Slave system.
/// Each Task takes approximately 10 seconds or more to process.

#include <vector>
#include <string>
#include <cstdint>
#include <chrono>
#include <thread>
#include <cmath>
#include <numeric>
#include <random>
#include <iostream>
#include <sstream>

namespace DPApp {

    /// =========================================
    /// TaskType Extension for Testing
    /// =========================================
    /// 
    /// The following should be added to the TaskType enum in PointCloudTypes.h:
    /// 
    /// enum class TaskType : uint8_t {
    ///     UNKNOWN = 0,
    ///     CONVERT_PTS = 1,
    ///     BIM_DISTANCE_CALCULATION = 2,
    ///     BIM_PC2_DIST = 3,
    ///     
    ///     // Add types for testing
    ///     TEST_ECHO = 10,       // Simple data round-trip
    ///     TEST_COMPUTE = 11,    // Computation verification (square operation)
    ///     TEST_DELAY = 12,      // Intentional delay (timeout test)
    ///     TEST_FAIL = 13,       // Intentional failure (retry test)
    /// };

    /// =========================================
    /// Test Data Structures
    /// =========================================

    /// @brief Chunk data for testing
    struct TestChunk {
        uint32_t chunk_id = 0;

        /// Input data (integer array)
        std::vector<int64_t> input_numbers;

        /// Expected result (for validation - set by Master)
        std::vector<int64_t> expected_output;

        /// For TEST_DELAY: Additional delay time (milliseconds)
        uint32_t extra_delay_ms = 0;

        /// For TEST_FAIL: Failure count (succeeds after failing this many times)
        uint32_t fail_count = 0;

        /// Current attempt count (tracked by Slave)
        mutable uint32_t current_attempt = 0;

        /// Base processing time (milliseconds) - minimum 10 seconds
        uint32_t base_processing_time_ms = 10000;

        TestChunk() = default;

        /// @brief Generate test data
        /// @param id Chunk ID
        /// @param count Number of elements
        /// @param base_time_ms Base processing time (Default: 10000ms = 10s)
        static TestChunk generate(uint32_t id, size_t count = 1000, uint32_t base_time_ms = 10000) {
            TestChunk chunk;
            chunk.chunk_id = id;
            chunk.base_processing_time_ms = base_time_ms;

            /// Generate random numbers
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int64_t> dist(1, 1000);

            chunk.input_numbers.reserve(count);
            chunk.expected_output.reserve(count);

            for (size_t i = 0; i < count; ++i) {
                int64_t num = dist(gen);
                chunk.input_numbers.push_back(num);
                chunk.expected_output.push_back(num * num);  /// Square is the expected result
            }

            return chunk;
        }

        /// @brief Serialization (for network transmission)
        std::vector<uint8_t> serialize() const {
            std::vector<uint8_t> data;

            /// chunk_id (4 bytes)
            data.push_back((chunk_id >> 0) & 0xFF);
            data.push_back((chunk_id >> 8) & 0xFF);
            data.push_back((chunk_id >> 16) & 0xFF);
            data.push_back((chunk_id >> 24) & 0xFF);

            /// base_processing_time_ms (4 bytes)
            data.push_back((base_processing_time_ms >> 0) & 0xFF);
            data.push_back((base_processing_time_ms >> 8) & 0xFF);
            data.push_back((base_processing_time_ms >> 16) & 0xFF);
            data.push_back((base_processing_time_ms >> 24) & 0xFF);

            /// extra_delay_ms (4 bytes)
            data.push_back((extra_delay_ms >> 0) & 0xFF);
            data.push_back((extra_delay_ms >> 8) & 0xFF);
            data.push_back((extra_delay_ms >> 16) & 0xFF);
            data.push_back((extra_delay_ms >> 24) & 0xFF);

            /// fail_count (4 bytes)
            data.push_back((fail_count >> 0) & 0xFF);
            data.push_back((fail_count >> 8) & 0xFF);
            data.push_back((fail_count >> 16) & 0xFF);
            data.push_back((fail_count >> 24) & 0xFF);

            /// input_numbers count (4 bytes)
            uint32_t count = static_cast<uint32_t>(input_numbers.size());
            data.push_back((count >> 0) & 0xFF);
            data.push_back((count >> 8) & 0xFF);
            data.push_back((count >> 16) & 0xFF);
            data.push_back((count >> 24) & 0xFF);

            /// input_numbers (8 bytes each)
            for (int64_t num : input_numbers) {
                for (int i = 0; i < 8; ++i) {
                    data.push_back((num >> (i * 8)) & 0xFF);
                }
            }

            return data;
        }

        /// @brief Deserialization
        static TestChunk deserialize(const std::vector<uint8_t>& data) {
            TestChunk chunk;
            size_t offset = 0;

            if (data.size() < 20) return chunk;  /// Minimum header size

            /// chunk_id
            chunk.chunk_id = data[offset] | (data[offset + 1] << 8) |
                (data[offset + 2] << 16) | (data[offset + 3] << 24);
            offset += 4;

            /// base_processing_time_ms
            chunk.base_processing_time_ms = data[offset] | (data[offset + 1] << 8) |
                (data[offset + 2] << 16) | (data[offset + 3] << 24);
            offset += 4;

            /// extra_delay_ms
            chunk.extra_delay_ms = data[offset] | (data[offset + 1] << 8) |
                (data[offset + 2] << 16) | (data[offset + 3] << 24);
            offset += 4;

            /// fail_count
            chunk.fail_count = data[offset] | (data[offset + 1] << 8) |
                (data[offset + 2] << 16) | (data[offset + 3] << 24);
            offset += 4;

            /// input_numbers count
            uint32_t count = data[offset] | (data[offset + 1] << 8) |
                (data[offset + 2] << 16) | (data[offset + 3] << 24);
            offset += 4;

            /// input_numbers
            chunk.input_numbers.reserve(count);
            for (uint32_t i = 0; i < count && offset + 8 <= data.size(); ++i) {
                int64_t num = 0;
                for (int j = 0; j < 8; ++j) {
                    num |= (static_cast<int64_t>(data[offset + j]) << (j * 8));
                }
                chunk.input_numbers.push_back(num);
                offset += 8;
            }

            return chunk;
        }
    };

    /// @brief Test result structure
    struct TestResult {
        uint32_t task_id = 0;
        uint32_t chunk_id = 0;
        bool success = false;
        std::string error_message;

        /// Output data (processing result)
        std::vector<int64_t> output_numbers;

        /// Processing time (milliseconds)
        double processing_time_ms = 0;

        /// Verification result
        bool verified = false;
        uint32_t mismatch_count = 0;

        /// @brief Serialization
        std::vector<uint8_t> serialize() const {
            std::vector<uint8_t> data;

            /// task_id (4 bytes)
            data.push_back((task_id >> 0) & 0xFF);
            data.push_back((task_id >> 8) & 0xFF);
            data.push_back((task_id >> 16) & 0xFF);
            data.push_back((task_id >> 24) & 0xFF);

            /// chunk_id (4 bytes)
            data.push_back((chunk_id >> 0) & 0xFF);
            data.push_back((chunk_id >> 8) & 0xFF);
            data.push_back((chunk_id >> 16) & 0xFF);
            data.push_back((chunk_id >> 24) & 0xFF);

            /// success (1 byte)
            data.push_back(success ? 1 : 0);

            /// processing_time_ms (8 bytes as double)
            const uint8_t* time_ptr = reinterpret_cast<const uint8_t*>(&processing_time_ms);
            for (int i = 0; i < 8; ++i) {
                data.push_back(time_ptr[i]);
            }

            /// error_message length + data
            uint32_t err_len = static_cast<uint32_t>(error_message.size());
            data.push_back((err_len >> 0) & 0xFF);
            data.push_back((err_len >> 8) & 0xFF);
            data.push_back((err_len >> 16) & 0xFF);
            data.push_back((err_len >> 24) & 0xFF);
            for (char c : error_message) {
                data.push_back(static_cast<uint8_t>(c));
            }

            /// output_numbers count (4 bytes)
            uint32_t count = static_cast<uint32_t>(output_numbers.size());
            data.push_back((count >> 0) & 0xFF);
            data.push_back((count >> 8) & 0xFF);
            data.push_back((count >> 16) & 0xFF);
            data.push_back((count >> 24) & 0xFF);

            /// output_numbers (8 bytes each)
            for (int64_t num : output_numbers) {
                for (int i = 0; i < 8; ++i) {
                    data.push_back((num >> (i * 8)) & 0xFF);
                }
            }

            return data;
        }

        /// @brief Deserialization
        static TestResult deserialize(const std::vector<uint8_t>& data) {
            TestResult result;
            size_t offset = 0;

            if (data.size() < 21) return result;

            /// task_id
            result.task_id = data[offset] | (data[offset + 1] << 8) |
                (data[offset + 2] << 16) | (data[offset + 3] << 24);
            offset += 4;

            /// chunk_id
            result.chunk_id = data[offset] | (data[offset + 1] << 8) |
                (data[offset + 2] << 16) | (data[offset + 3] << 24);
            offset += 4;

            /// success
            result.success = (data[offset] != 0);
            offset += 1;

            /// processing_time_ms
            uint8_t time_bytes[8];
            for (int i = 0; i < 8; ++i) {
                time_bytes[i] = data[offset + i];
            }
            result.processing_time_ms = *reinterpret_cast<double*>(time_bytes);
            offset += 8;

            /// error_message
            uint32_t err_len = data[offset] | (data[offset + 1] << 8) |
                (data[offset + 2] << 16) | (data[offset + 3] << 24);
            offset += 4;
            result.error_message.assign(data.begin() + offset, data.begin() + offset + err_len);
            offset += err_len;

            /// output_numbers
            if (offset + 4 <= data.size()) {
                uint32_t count = data[offset] | (data[offset + 1] << 8) |
                    (data[offset + 2] << 16) | (data[offset + 3] << 24);
                offset += 4;

                result.output_numbers.reserve(count);
                for (uint32_t i = 0; i < count && offset + 8 <= data.size(); ++i) {
                    int64_t num = 0;
                    for (int j = 0; j < 8; ++j) {
                        num |= (static_cast<int64_t>(data[offset + j]) << (j * 8));
                    }
                    result.output_numbers.push_back(num);
                    offset += 8;
                }
            }

            return result;
        }
    };

    /// =========================================
    /// Test Task Processing Functions
    /// =========================================

    namespace TestProcessors {

        /// @brief Dummy operation to induce CPU load (consumes time)
        /// @param duration_ms Duration (milliseconds)
        inline void simulateWork(uint32_t duration_ms) {
            auto start = std::chrono::steady_clock::now();
            auto target_duration = std::chrono::milliseconds(duration_ms);

            /// Perform actual CPU work instead of simple sleep (for more realistic simulation)
            volatile double result = 0;
            uint64_t iterations = 0;

            while (true) {
                auto now = std::chrono::steady_clock::now();
                if (now - start >= target_duration) break;

                /// CPU work: Repeat square root calculation
                for (int i = 0; i < 10000; ++i) {
                    result += std::sqrt(static_cast<double>(iterations + i));
                }
                iterations += 10000;

                /// Check time every 100ms equivalent iterations
                if (iterations % 1000000 == 0) {
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
                    /// Output progress (Optional)
                    /// std::cout << "  Progress: " << elapsed.count() << "/" << duration_ms << " ms\r" << std::flush;
                }
            }
        }

        /// @brief Process TEST_ECHO: Returns data as is
        inline TestResult processEcho(uint32_t task_id, const TestChunk& chunk) {
            TestResult result;
            result.task_id = task_id;
            result.chunk_id = chunk.chunk_id;

            auto start = std::chrono::steady_clock::now();

            std::cout << "[TEST_ECHO] Task " << task_id << " Started (Expected time: "
                << chunk.base_processing_time_ms / 1000.0 << "s)" << std::endl;

            /// Consume time (Default 10s)
            simulateWork(chunk.base_processing_time_ms);

            /// Copy data as is
            result.output_numbers = chunk.input_numbers;
            result.success = true;

            auto end = std::chrono::steady_clock::now();
            result.processing_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            std::cout << "[TEST_ECHO] Task " << task_id << " Completed (Actual time: "
                << result.processing_time_ms / 1000.0 << "s)" << std::endl;

            return result;
        }

        /// @brief Process TEST_COMPUTE: Returns the square of each number
        inline TestResult processCompute(uint32_t task_id, const TestChunk& chunk) {
            TestResult result;
            result.task_id = task_id;
            result.chunk_id = chunk.chunk_id;

            auto start = std::chrono::steady_clock::now();

            std::cout << "[TEST_COMPUTE] Task " << task_id << " Started (Data: "
                << chunk.input_numbers.size() << " items, Expected time: "
                << chunk.base_processing_time_ms / 1000.0 << "s)" << std::endl;

            /// Consume time (Default 10s)
            simulateWork(chunk.base_processing_time_ms);

            /// Actual calculation: Square each number
            result.output_numbers.reserve(chunk.input_numbers.size());
            for (int64_t num : chunk.input_numbers) {
                result.output_numbers.push_back(num * num);
            }

            result.success = true;

            auto end = std::chrono::steady_clock::now();
            result.processing_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            std::cout << "[TEST_COMPUTE] Task " << task_id << " Completed (Actual time: "
                << result.processing_time_ms / 1000.0 << "s)" << std::endl;

            return result;
        }

        /// @brief Process TEST_DELAY: Calculation after additional delay
        inline TestResult processDelay(uint32_t task_id, const TestChunk& chunk) {
            TestResult result;
            result.task_id = task_id;
            result.chunk_id = chunk.chunk_id;

            auto start = std::chrono::steady_clock::now();

            uint32_t total_delay = chunk.base_processing_time_ms + chunk.extra_delay_ms;
            std::cout << "[TEST_DELAY] Task " << task_id << " Started (Base: "
                << chunk.base_processing_time_ms / 1000.0 << "s + Extra: "
                << chunk.extra_delay_ms / 1000.0 << "s = Total "
                << total_delay / 1000.0 << "s)" << std::endl;

            /// Base time + Extra delay
            simulateWork(total_delay);

            /// Calculation
            result.output_numbers.reserve(chunk.input_numbers.size());
            for (int64_t num : chunk.input_numbers) {
                result.output_numbers.push_back(num * num);
            }

            result.success = true;

            auto end = std::chrono::steady_clock::now();
            result.processing_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            std::cout << "[TEST_DELAY] Task " << task_id << " Completed (Actual time: "
                << result.processing_time_ms / 1000.0 << "s)" << std::endl;

            return result;
        }

        /// @brief Process TEST_FAIL: Succeed after failing a specified number of times
        /// @note Simulates failure 'chunk.fail_count' times
        inline TestResult processFail(uint32_t task_id, TestChunk& chunk) {
            TestResult result;
            result.task_id = task_id;
            result.chunk_id = chunk.chunk_id;

            auto start = std::chrono::steady_clock::now();

            chunk.current_attempt++;

            std::cout << "[TEST_FAIL] Task " << task_id << " Attempt #" << chunk.current_attempt
                << " (Scheduled failures: " << chunk.fail_count << ")" << std::endl;

            /// Consume time (Default 10s)
            simulateWork(chunk.base_processing_time_ms);

            /// Fail for the specified number of times
            if (chunk.current_attempt <= chunk.fail_count) {
                result.success = false;
                result.error_message = "Intentional Failure (Attempt " + std::to_string(chunk.current_attempt) +
                    "/" + std::to_string(chunk.fail_count + 1) + ")";

                std::cout << "[TEST_FAIL] Task " << task_id << " Failed: " << result.error_message << std::endl;
            }
            else {
                /// Success
                result.output_numbers.reserve(chunk.input_numbers.size());
                for (int64_t num : chunk.input_numbers) {
                    result.output_numbers.push_back(num * num);
                }
                result.success = true;

                std::cout << "[TEST_FAIL] Task " << task_id << " Succeeded (Attempt #"
                    << chunk.current_attempt << ")" << std::endl;
            }

            auto end = std::chrono::steady_clock::now();
            result.processing_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            return result;
        }

        /// @brief Verify test results
        /// @param chunk Original chunk (contains expected_output)
        /// @param result Processing result
        /// @return Verification success status
        inline bool verifyResult(const TestChunk& chunk, TestResult& result) {
            if (!result.success) {
                result.verified = false;
                return false;
            }

            if (result.output_numbers.size() != chunk.expected_output.size()) {
                result.verified = false;
                result.mismatch_count = static_cast<uint32_t>(
                    std::max(result.output_numbers.size(), chunk.expected_output.size()));
                return false;
            }

            result.mismatch_count = 0;
            for (size_t i = 0; i < chunk.expected_output.size(); ++i) {
                if (result.output_numbers[i] != chunk.expected_output[i]) {
                    result.mismatch_count++;
                }
            }

            result.verified = (result.mismatch_count == 0);
            return result.verified;
        }

    } // namespace TestProcessors

    /// =========================================
    /// Test Utilities
    /// =========================================

    namespace TestUtils {

        /// @brief Print test summary
        inline void printTestSummary(const std::string& test_name,
            uint32_t total_tasks,
            uint32_t completed,
            uint32_t failed,
            uint32_t verified,
            double total_time_sec) {
            std::cout << "\n========================================" << std::endl;
            std::cout << "Test: " << test_name << std::endl;
            std::cout << "========================================" << std::endl;
            std::cout << "Total Tasks:     " << total_tasks << std::endl;
            std::cout << "Completed:       " << completed << " ("
                << (100.0 * completed / total_tasks) << "%)" << std::endl;
            std::cout << "Failed:          " << failed << std::endl;
            std::cout << "Verified:        " << verified << " ("
                << (100.0 * verified / total_tasks) << "%)" << std::endl;
            std::cout << "Total Time:      " << total_time_sec << "s" << std::endl;
            std::cout << "========================================\n" << std::endl;
        }

        /// @brief Generate test chunk array
        inline std::vector<TestChunk> generateTestChunks(
            uint32_t count,
            size_t numbers_per_chunk = 100,
            uint32_t base_time_ms = 10000) {

            std::vector<TestChunk> chunks;
            chunks.reserve(count);

            for (uint32_t i = 0; i < count; ++i) {
                chunks.push_back(TestChunk::generate(i, numbers_per_chunk, base_time_ms));
            }

            return chunks;
        }

    } /// namespace TestUtils

} /// namespace DPApp