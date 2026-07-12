#pragma once
/// @file TestTask.h
/// @brief 테스트용 태스크 타입 및 처리 로직
///
/// Master/Slave 시스템을 검증하기 위한 테스트 태스크 구현.
/// 각 태스크는 처리에 약 10초 이상이 걸린다.

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
    /// 테스트를 위한 TaskType 확장
    /// =========================================
    ///
    /// 아래 내용은 PointCloudTypes.h의 TaskType enum에 추가되어 있어야 한다:
    ///
    /// enum class TaskType : uint8_t {
    ///     UNKNOWN = 0,
    ///     CONVERT_PTS = 1,
    ///     BIM_DISTANCE_CALCULATION = 2,
    ///     BIM_PC2_DIST = 3,
    ///
    ///     // 테스트용 타입 추가
    ///     TEST_ECHO = 10,       // 단순 데이터 왕복
    ///     TEST_COMPUTE = 11,    // 계산 검증 (제곱 연산)
    ///     TEST_DELAY = 12,      // 의도적 지연 (타임아웃 테스트)
    ///     TEST_FAIL = 13,       // 의도적 실패 (재시도 테스트)
    /// };

    /// =========================================
    /// 테스트 데이터 구조체
    /// =========================================

    /// @brief 테스트용 청크 데이터
    struct TestChunk {
        uint32_t chunk_id = 0;

        /// 입력 데이터 (정수 배열)
        std::vector<int64_t> input_numbers;

        /// 기대 결과 (검증용 - Master가 설정)
        std::vector<int64_t> expected_output;

        /// TEST_DELAY용: 추가 지연 시간 (밀리초)
        uint32_t extra_delay_ms = 0;

        /// TEST_FAIL용: 실패 횟수 (이 횟수만큼 실패한 뒤 성공)
        uint32_t fail_count = 0;

        /// 현재 시도 횟수 (Slave가 추적)
        mutable uint32_t current_attempt = 0;

        /// 기본 처리 시간 (밀리초) - 최소 10초
        uint32_t base_processing_time_ms = 10000;

        TestChunk() = default;

        /// @brief 테스트 데이터 생성
        /// @param id 청크 ID
        /// @param count 원소 개수
        /// @param base_time_ms 기본 처리 시간 (기본값: 10000ms = 10초)
        static TestChunk generate(uint32_t id, size_t count = 1000, uint32_t base_time_ms = 10000) {
            TestChunk chunk;
            chunk.chunk_id = id;
            chunk.base_processing_time_ms = base_time_ms;

            /// 난수 생성
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int64_t> dist(1, 1000);

            chunk.input_numbers.reserve(count);
            chunk.expected_output.reserve(count);

            for (size_t i = 0; i < count; ++i) {
                int64_t num = dist(gen);
                chunk.input_numbers.push_back(num);
                chunk.expected_output.push_back(num * num);  /// 제곱이 기대 결과
            }

            return chunk;
        }

        /// @brief 직렬화 (네트워크 전송용)
        std::vector<uint8_t> serialize() const {
            std::vector<uint8_t> data;

            /// chunk_id (4바이트)
            data.push_back((chunk_id >> 0) & 0xFF);
            data.push_back((chunk_id >> 8) & 0xFF);
            data.push_back((chunk_id >> 16) & 0xFF);
            data.push_back((chunk_id >> 24) & 0xFF);

            /// base_processing_time_ms (4바이트)
            data.push_back((base_processing_time_ms >> 0) & 0xFF);
            data.push_back((base_processing_time_ms >> 8) & 0xFF);
            data.push_back((base_processing_time_ms >> 16) & 0xFF);
            data.push_back((base_processing_time_ms >> 24) & 0xFF);

            /// extra_delay_ms (4바이트)
            data.push_back((extra_delay_ms >> 0) & 0xFF);
            data.push_back((extra_delay_ms >> 8) & 0xFF);
            data.push_back((extra_delay_ms >> 16) & 0xFF);
            data.push_back((extra_delay_ms >> 24) & 0xFF);

            /// fail_count (4바이트)
            data.push_back((fail_count >> 0) & 0xFF);
            data.push_back((fail_count >> 8) & 0xFF);
            data.push_back((fail_count >> 16) & 0xFF);
            data.push_back((fail_count >> 24) & 0xFF);

            /// input_numbers 개수 (4바이트)
            uint32_t count = static_cast<uint32_t>(input_numbers.size());
            data.push_back((count >> 0) & 0xFF);
            data.push_back((count >> 8) & 0xFF);
            data.push_back((count >> 16) & 0xFF);
            data.push_back((count >> 24) & 0xFF);

            /// input_numbers (각 8바이트)
            for (int64_t num : input_numbers) {
                for (int i = 0; i < 8; ++i) {
                    data.push_back((num >> (i * 8)) & 0xFF);
                }
            }

            return data;
        }

        /// @brief 역직렬화
        static TestChunk deserialize(const std::vector<uint8_t>& data) {
            TestChunk chunk;
            size_t offset = 0;

            if (data.size() < 20) return chunk;  /// 최소 헤더 크기

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

            /// input_numbers 개수
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

    /// @brief 테스트 결과 구조체
    struct TestResult {
        uint32_t task_id = 0;
        uint32_t chunk_id = 0;
        bool success = false;
        std::string error_message;

        /// 출력 데이터 (처리 결과)
        std::vector<int64_t> output_numbers;

        /// 처리 시간 (밀리초)
        double processing_time_ms = 0;

        /// 검증 결과
        bool verified = false;
        uint32_t mismatch_count = 0;

        /// @brief 직렬화
        std::vector<uint8_t> serialize() const {
            std::vector<uint8_t> data;

            /// task_id (4바이트)
            data.push_back((task_id >> 0) & 0xFF);
            data.push_back((task_id >> 8) & 0xFF);
            data.push_back((task_id >> 16) & 0xFF);
            data.push_back((task_id >> 24) & 0xFF);

            /// chunk_id (4바이트)
            data.push_back((chunk_id >> 0) & 0xFF);
            data.push_back((chunk_id >> 8) & 0xFF);
            data.push_back((chunk_id >> 16) & 0xFF);
            data.push_back((chunk_id >> 24) & 0xFF);

            /// success (1바이트)
            data.push_back(success ? 1 : 0);

            /// processing_time_ms (double을 8바이트로)
            const uint8_t* time_ptr = reinterpret_cast<const uint8_t*>(&processing_time_ms);
            for (int i = 0; i < 8; ++i) {
                data.push_back(time_ptr[i]);
            }

            /// error_message 길이 + 데이터
            uint32_t err_len = static_cast<uint32_t>(error_message.size());
            data.push_back((err_len >> 0) & 0xFF);
            data.push_back((err_len >> 8) & 0xFF);
            data.push_back((err_len >> 16) & 0xFF);
            data.push_back((err_len >> 24) & 0xFF);
            for (char c : error_message) {
                data.push_back(static_cast<uint8_t>(c));
            }

            /// output_numbers 개수 (4바이트)
            uint32_t count = static_cast<uint32_t>(output_numbers.size());
            data.push_back((count >> 0) & 0xFF);
            data.push_back((count >> 8) & 0xFF);
            data.push_back((count >> 16) & 0xFF);
            data.push_back((count >> 24) & 0xFF);

            /// output_numbers (각 8바이트)
            for (int64_t num : output_numbers) {
                for (int i = 0; i < 8; ++i) {
                    data.push_back((num >> (i * 8)) & 0xFF);
                }
            }

            return data;
        }

        /// @brief 역직렬화
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
    /// 테스트 태스크 처리 함수
    /// =========================================

    namespace TestProcessors {

        /// @brief CPU 부하를 유발하는 더미 연산 (시간을 소비함)
        /// @param duration_ms 지속 시간 (밀리초)
        inline void simulateWork(uint32_t duration_ms) {
            auto start = std::chrono::steady_clock::now();
            auto target_duration = std::chrono::milliseconds(duration_ms);

            /// 단순 sleep 대신 실제 CPU 연산을 수행 (더 현실적인 시뮬레이션을 위해)
            volatile double result = 0;
            uint64_t iterations = 0;

            while (true) {
                auto now = std::chrono::steady_clock::now();
                if (now - start >= target_duration) break;

                /// CPU 연산: 제곱근 계산을 반복
                for (int i = 0; i < 10000; ++i) {
                    result += std::sqrt(static_cast<double>(iterations + i));
                }
                iterations += 10000;

                /// 약 100ms에 해당하는 반복마다 시간 확인
                if (iterations % 1000000 == 0) {
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
                    /// 진행 상황 출력 (선택 사항)
                    /// std::cout << "  Progress: " << elapsed.count() << "/" << duration_ms << " ms\r" << std::flush;
                }
            }
        }

        /// @brief TEST_ECHO 처리: 데이터를 그대로 반환
        inline TestResult processEcho(uint32_t task_id, const TestChunk& chunk) {
            TestResult result;
            result.task_id = task_id;
            result.chunk_id = chunk.chunk_id;

            auto start = std::chrono::steady_clock::now();

            std::cout << "[TEST_ECHO] Task " << task_id << " Started (Expected time: "
                << chunk.base_processing_time_ms / 1000.0 << "s)" << std::endl;

            /// 시간 소비 (기본 10초)
            simulateWork(chunk.base_processing_time_ms);

            /// 데이터를 그대로 복사
            result.output_numbers = chunk.input_numbers;
            result.success = true;

            auto end = std::chrono::steady_clock::now();
            result.processing_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            std::cout << "[TEST_ECHO] Task " << task_id << " Completed (Actual time: "
                << result.processing_time_ms / 1000.0 << "s)" << std::endl;

            return result;
        }

        /// @brief TEST_COMPUTE 처리: 각 숫자의 제곱을 반환
        inline TestResult processCompute(uint32_t task_id, const TestChunk& chunk) {
            TestResult result;
            result.task_id = task_id;
            result.chunk_id = chunk.chunk_id;

            auto start = std::chrono::steady_clock::now();

            std::cout << "[TEST_COMPUTE] Task " << task_id << " Started (Data: "
                << chunk.input_numbers.size() << " items, Expected time: "
                << chunk.base_processing_time_ms / 1000.0 << "s)" << std::endl;

            /// 시간 소비 (기본 10초)
            simulateWork(chunk.base_processing_time_ms);

            /// 실제 계산: 각 숫자를 제곱
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

        /// @brief TEST_DELAY 처리: 추가 지연 이후 계산
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

            /// 기본 시간 + 추가 지연
            simulateWork(total_delay);

            /// 계산
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

        /// @brief TEST_FAIL 처리: 지정된 횟수만큼 실패한 뒤 성공
        /// @note 'chunk.fail_count'번만큼 실패를 시뮬레이션함
        inline TestResult processFail(uint32_t task_id, TestChunk& chunk) {
            TestResult result;
            result.task_id = task_id;
            result.chunk_id = chunk.chunk_id;

            auto start = std::chrono::steady_clock::now();

            chunk.current_attempt++;

            std::cout << "[TEST_FAIL] Task " << task_id << " Attempt #" << chunk.current_attempt
                << " (Scheduled failures: " << chunk.fail_count << ")" << std::endl;

            /// 시간 소비 (기본 10초)
            simulateWork(chunk.base_processing_time_ms);

            /// 지정된 횟수만큼 실패
            if (chunk.current_attempt <= chunk.fail_count) {
                result.success = false;
                result.error_message = "Intentional Failure (Attempt " + std::to_string(chunk.current_attempt) +
                    "/" + std::to_string(chunk.fail_count + 1) + ")";

                std::cout << "[TEST_FAIL] Task " << task_id << " Failed: " << result.error_message << std::endl;
            }
            else {
                /// 성공
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

        /// @brief 테스트 결과 검증
        /// @param chunk 원본 청크 (expected_output을 담고 있음)
        /// @param result 처리 결과
        /// @return 검증 성공 여부
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
    /// 테스트 유틸리티
    /// =========================================

    namespace TestUtils {

        /// @brief 테스트 요약 출력
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

        /// @brief 테스트 청크 배열 생성
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
