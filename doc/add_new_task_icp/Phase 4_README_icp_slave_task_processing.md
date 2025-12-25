# Phase 4: Slave 태스크 처리기

## 개요
SlaveApp에서 ICP 태스크를 처리할 수 있도록 합니다.

## 파일 목록

| 파일 | 작업 | 대상 위치 |
|------|------|-----------|
| `IcpProcessor.hpp` | 신규 생성 | `F:\repository\DPApp\include\bimtree\IcpProcessor.hpp` |
| `SlaveApplication_MODIFY_GUIDE.txt` | 수정 가이드 | (참조용) |

## 적용 순서

### Step 1: IcpProcessor.hpp 복사
```
IcpProcessor.hpp → F:\repository\DPApp\include\bimtree\IcpProcessor.hpp
```

### Step 2: SlaveApplication.cpp 수정

`F:\repository\DPApp\SlaveApp\SlaveApplication.cpp` 파일을 수정합니다.

#### 2.1 Include 추가 (파일 상단)
```cpp
#include "../include/bimtree/IcpProcessor.hpp"
```

#### 2.2 TaskQueueItem 구조체에 ICP 멤버 추가
```cpp
icp::IcpChunk icp_chunk;    // 멤버 추가
bool has_icp = false;       // 플래그 추가

// 생성자 추가
TaskQueueItem(const ProcessingTask& t, const icp::IcpChunk& ic)
    : task(t), icp_chunk(ic), has_icp(true) {}
```

#### 2.3 TASK_ASSIGNMENT 처리에 ICP 분기 추가
`is_test_task` 체크 앞에 ICP 체크 추가:
```cpp
bool is_icp_task = (task.task_type == TaskType::ICP_FINE_ALIGNMENT);

if (is_icp_task && chunk_data_size > 0) {
    std::vector<uint8_t> chunk_buffer(data + offset, data + offset + chunk_data_size);
    icp::IcpChunk icp_chunk = icp::deserializeIcpChunk(chunk_buffer);
    // ... 큐에 추가
}
else if (is_test_task && chunk_data_size > 0) {  // else if로 변경!
    // 기존 코드
}
```

#### 2.4 태스크 처리 루프에 ICP 분기 추가
```cpp
if (item.has_icp) {
    icp::IcpResult result = processIcpTask(item.task, item.icp_chunk);
    sendIcpResult(result, item.task.task_id);
}
else if (item.has_test) {  // else if로 변경!
    // 기존 코드
}
```

#### 2.5 processIcpTask 함수 추가
```cpp
icp::IcpResult processIcpTask(const ProcessingTask& task, const icp::IcpChunk& chunk) {
    return DPApp::IcpProcessors::processFineAlignment(chunk, g_cancel_requested);
}
```

#### 2.6 sendIcpResult 함수 추가
```cpp
void sendIcpResult(const icp::IcpResult& result, uint32_t task_id) {
    std::vector<uint8_t> result_data = icp::serializeIcpResult(result);
    
    std::vector<uint8_t> message_data;
    message_data.push_back(3);  // ICP result type
    // task_id 추가
    // result_data 추가
    
    NetworkMessage message(MessageType::TASK_RESULT, message_data);
    client_->sendMessage(message);
}
```

### Step 3: SlaveApp 빌드
SlaveApp 프로젝트만 빌드하여 컴파일 오류 확인

## 구현 내용

### IcpProcessor 클래스

```cpp
namespace icp {
    class IcpProcessor {
    public:
        IcpResult processFineAlignment(const IcpChunk& chunk, 
                                       std::atomic<bool>& cancelRequested);
        
        std::vector<uint8_t> processTaskFromBytes(const std::vector<uint8_t>& chunkData,
                                                   std::atomic<bool>& cancelRequested);
    };
}
```

### DPApp::IcpProcessors 네임스페이스

```cpp
namespace DPApp::IcpProcessors {
    bool isIcpTask(TaskType type);
    
    icp::IcpResult processFineAlignment(const icp::IcpChunk& chunk,
                                        std::atomic<bool>& cancelRequested);
    
    icp::IcpResult processIcpTask(const std::vector<uint8_t>& chunkData,
                                  std::atomic<bool>& cancelRequested);
}
```

## 메시지 프로토콜

### TASK_RESULT 메시지 포맷

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | result_type | 0=normal, 1=bimpc, 2=test, 3=icp |
| 1 | 4 | task_id | Task identifier |
| 5 | N | result_data | Serialized result |

## 예상 오류 및 해결

### g_cancel_requested 접근 오류
```
error: 'g_cancel_requested' was not declared
```
**해결**: `DPApp::g_cancel_requested` 또는 전역 선언 확인

### TaskQueueItem 생성자 오류
```
error: no matching constructor for initialization
```
**해결**: 생성자 시그니처와 호출부 일치 확인

## 다음 단계
Phase 4 완료 후 Phase 5 (Master REST API)로 진행합니다.
