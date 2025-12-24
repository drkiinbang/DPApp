/// ============================================================================
/// SlaveApplication.cpp 수정 가이드 - ICP Task 지원 추가 (업데이트)
/// ============================================================================
/// 
/// 파일 위치: F:\repository\DPApp\SlaveApp\SlaveApplication.cpp
/// 
/// 총 6곳을 수정합니다.
/// ============================================================================


/// ============================================================================
/// 수정 1: Include 추가 (Line 21 근처)
/// ============================================================================
/// 
/// #include "../include/TestTask.h" 아래에 추가:

#include "../include/bimtree/IcpProcessor.hpp"


/// ============================================================================
/// 수정 2: TaskQueueItem 구조체 수정 (Line 295-313)
/// ============================================================================
/// 
/// [기존 코드]
    struct TaskQueueItem {
        ProcessingTask task;
        PointCloudChunk chunk;
        BimPcChunk bimpc_chunk;
        TestChunk test_chunk;  // 테스트용 청크
        bool is_bimpc;
        bool is_test;  // 테스트 Task 여부

        TaskQueueItem() : is_bimpc(false), is_test(false) {}
        TaskQueueItem(const ProcessingTask& t, const PointCloudChunk& c)
            : task(t), chunk(c), is_bimpc(false), is_test(false) {
        }
        TaskQueueItem(const ProcessingTask& t, const BimPcChunk& bc)
            : task(t), bimpc_chunk(bc), is_bimpc(true), is_test(false) {
        }
        TaskQueueItem(const ProcessingTask& t, const TestChunk& tc)
            : task(t), test_chunk(tc), is_bimpc(false), is_test(true) {
        }
    };

/// [수정 후]
    struct TaskQueueItem {
        ProcessingTask task;
        PointCloudChunk chunk;
        BimPcChunk bimpc_chunk;
        TestChunk test_chunk;
        icp::IcpChunk icp_chunk;       /// <-- 추가
        bool is_bimpc;
        bool is_test;
        bool is_icp;                   /// <-- 추가

        TaskQueueItem() : is_bimpc(false), is_test(false), is_icp(false) {}
        TaskQueueItem(const ProcessingTask& t, const PointCloudChunk& c)
            : task(t), chunk(c), is_bimpc(false), is_test(false), is_icp(false) {
        }
        TaskQueueItem(const ProcessingTask& t, const BimPcChunk& bc)
            : task(t), bimpc_chunk(bc), is_bimpc(true), is_test(false), is_icp(false) {
        }
        TaskQueueItem(const ProcessingTask& t, const TestChunk& tc)
            : task(t), test_chunk(tc), is_bimpc(false), is_test(true), is_icp(false) {
        }
        /// ICP 청크용 생성자 추가
        TaskQueueItem(const ProcessingTask& t, const icp::IcpChunk& ic)
            : task(t), icp_chunk(ic), is_bimpc(false), is_test(false), is_icp(true) {
        }
    };


/// ============================================================================
/// 수정 3: TASK_ASSIGNMENT 처리 (Line 495-516)
/// ============================================================================
/// 
/// [위치] "bool is_test_task = " 를 검색하세요 (Line 496)
/// 
/// [기존 코드]
            /// 5. 테스트 Task 타입인지 확인
            bool is_test_task = (task.task_type == TaskType::TEST_ECHO ||
                task.task_type == TaskType::TEST_COMPUTE ||
                task.task_type == TaskType::TEST_DELAY ||
                task.task_type == TaskType::TEST_FAIL);

            if (is_test_task && chunk_data_size > 0) {
                /// Handle TestChunk
                ...

/// [수정 후] - is_test_task 정의 앞에 ICP 체크 추가, if를 else if로 변경
            /// ICP Task 타입인지 확인
            bool is_icp_task = (task.task_type == TaskType::ICP_FINE_ALIGNMENT);

            /// 5. 테스트 Task 타입인지 확인
            bool is_test_task = (task.task_type == TaskType::TEST_ECHO ||
                task.task_type == TaskType::TEST_COMPUTE ||
                task.task_type == TaskType::TEST_DELAY ||
                task.task_type == TaskType::TEST_FAIL);

            /// ICP Task 처리
            if (is_icp_task && chunk_data_size > 0) {
                /// Handle IcpChunk
                std::vector<uint8_t> chunk_buffer(data + offset, data + offset + chunk_data_size);
                icp::IcpChunk icp_chunk = icp::deserializeIcpChunk(chunk_buffer);

                ILOG << "Task " << task.task_id << " is ICP task ("
                    << "source: " << icp_chunk.sourcePoints.size() << " pts, "
                    << "target: " << icp_chunk.targetPoints.size() << " pts)";

                {
                    std::lock_guard<std::mutex> lock(task_queue_mutex_);
                    task_queue_.push(TaskQueueItem(task, icp_chunk));
                }
                task_queue_cv_.notify_one();
            }
            else if (is_test_task && chunk_data_size > 0) {   /// <-- if를 else if로 변경!
                /// Handle TestChunk
                ... (기존 코드 그대로)


/// ============================================================================
/// 수정 4: processingLoop 함수에서 ICP 처리 분기 추가 (Line 609)
/// ============================================================================
/// 
/// [위치] processingLoop 함수 내에서 "if (item.is_test)" 검색 (Line 609)
/// 
/// [기존 코드]
            /// 테스트 Task 처리
            if (item.is_test) {
                TestResult test_result;
                try {
                    test_result = processTestTask(item.task, item.test_chunk);
                }
                ...

/// [수정 후] - ICP 분기를 앞에 추가, 기존 if를 else if로 변경
            /// ICP Task 처리
            if (item.is_icp) {
                icp::IcpResult icp_result;
                try {
                    icp_result = processIcpTask(item.task, item.icp_chunk);
                }
                catch (const std::exception& e) {
                    icp_result.chunk_id = item.icp_chunk.chunk_id;
                    icp_result.success = false;
                    icp_result.errorMessage = "ICP processing exception: " + std::string(e.what());
                }

                auto end_time = std::chrono::steady_clock::now();
                auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    end_time - start_time).count();

                ILOG << "Thread " << thread_id << " completed ICP task " << item.task.task_id
                    << " in " << processing_time << "ms (success: "
                    << (icp_result.success ? "Yes" : "No")
                    << ", RMSE: " << icp_result.finalRMSE << ")";

                sendIcpResult(icp_result, item.task.task_id);
                updateStats(icp_result.success, processing_time / 1000.0);
            }
            /// 테스트 Task 처리
            else if (item.is_test) {   /// <-- if를 else if로 변경!
                TestResult test_result;
                try {
                    test_result = processTestTask(item.task, item.test_chunk);
                }
                ... (기존 코드 그대로)


/// ============================================================================
/// 수정 5: processIcpTask 함수 추가 (Line 783 근처, processTestTask 함수 뒤)
/// ============================================================================
/// 
/// processTestTask 함수가 끝나는 지점을 찾아서 그 아래에 추가:

    /// Process ICP task
    icp::IcpResult processIcpTask(const ProcessingTask& task, const icp::IcpChunk& chunk) {
        ILOG << "[ICP] Processing task " << task.task_id
             << " (chunk: " << chunk.chunk_id
             << ", source: " << chunk.sourcePoints.size()
             << ", target: " << chunk.targetPoints.size() << " pts)";

        icp::IcpResult result;
        result.chunk_id = chunk.chunk_id;

        try {
            /// Run ICP processing
            result = DPApp::IcpProcessors::processFineAlignment(chunk, g_cancel_requested);
            
            if (!result.success) {
                WLOG << "[ICP] Task " << task.task_id << " failed: " << result.errorMessage;
            }
        }
        catch (const std::exception& e) {
            result.success = false;
            result.errorMessage = std::string("Exception: ") + e.what();
            ELOG << "[ICP] Task " << task.task_id << " exception: " << e.what();
        }

        return result;
    }


/// ============================================================================
/// 수정 6: sendIcpResult 함수 추가 (Line 809 근처, sendTestResult 함수 뒤)
/// ============================================================================
/// 
/// sendTestResult 함수가 끝나는 지점을 찾아서 그 아래에 추가:

    /// Send ICP result to master
    void sendIcpResult(const icp::IcpResult& result, uint32_t task_id) {
        try {
            /// Serialize ICP result
            std::vector<uint8_t> result_data = icp::serializeIcpResult(result);

            /// Message format: [result_type: 1B][task_id: 4B][result_data]
            /// result_type: 0=normal, 1=bimpc, 2=test, 3=icp
            std::vector<uint8_t> message_data;
            message_data.reserve(1 + sizeof(uint32_t) + result_data.size());
            
            message_data.push_back(3);  /// result_type = 3 (ICP)
            
            /// Add task_id
            size_t pos = message_data.size();
            message_data.resize(pos + sizeof(uint32_t));
            std::memcpy(message_data.data() + pos, &task_id, sizeof(uint32_t));
            
            /// Add result data
            message_data.insert(message_data.end(), result_data.begin(), result_data.end());

            NetworkMessage message(MessageType::TASK_RESULT, message_data);

            if (client_->sendMessage(message)) {
                ILOG << "IcpResult sent to master (task: " << task_id
                     << ", chunk: " << result.chunk_id
                     << ", success: " << (result.success ? "Yes" : "No")
                     << ", RMSE: " << result.finalRMSE
                     << ", time: " << result.processingTimeMs << "ms)";
            }
            else {
                ELOG << "Failed to send IcpResult (task: " << task_id << ")";
            }
        }
        catch (const std::exception& e) {
            ELOG << "Error sending IcpResult: " << e.what();
        }
    }


/// ============================================================================
/// 수정 요약
/// ============================================================================
/// 
/// 1. Line 21: #include "../include/bimtree/IcpProcessor.hpp" 추가
/// 
/// 2. Line 295-313: TaskQueueItem 구조체에 icp_chunk, is_icp 추가
/// 
/// 3. Line 496: is_icp_task 체크 추가, if(is_test_task)를 else if로 변경
/// 
/// 4. Line 609: if(item.is_icp) 분기 추가, if(item.is_test)를 else if로 변경
/// 
/// 5. Line 783 근처: processIcpTask() 함수 추가
/// 
/// 6. Line 809 근처: sendIcpResult() 함수 추가
/// 
/// ============================================================================
/// 빌드 후 확인
/// ============================================================================
/// 
/// 1. SlaveApp 프로젝트 빌드 (Ctrl+Shift+B)
/// 2. 컴파일 오류 없는지 확인
/// 
/// 컴파일 오류 시 확인사항:
/// - g_cancel_requested: DPApp::g_cancel_requested 로 접근
/// - memcpy: std::memcpy 또는 <cstring> include 확인
/// ============================================================================
