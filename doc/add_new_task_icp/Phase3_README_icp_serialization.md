# Phase 3: ICP 데이터 직렬화

## 개요
Master ↔ Slave 간 ICP 데이터 교환을 위한 직렬화/역직렬화 기능을 구현합니다.

## 파일 목록

| 파일 | 작업 | 대상 위치 |
|------|------|-----------|
| `IcpSerialization.hpp` | 신규 생성 | `F:\repository\DPApp\include\bimtree\IcpSerialization.hpp` |
| `Phase3_CompileTest.cpp` | 테스트 코드 | (참조용) |

## 적용 순서

### Step 1: IcpSerialization.hpp 복사
```
IcpSerialization.hpp → F:\repository\DPApp\include\bimtree\IcpSerialization.hpp
```

### Step 2: 컴파일 확인
솔루션 빌드 (Ctrl+Shift+B)

### Step 3: (권장) 직렬화 테스트
`Phase3_CompileTest.cpp`의 테스트 함수로 직렬화가 정확히 동작하는지 확인합니다.

## 구현 내용

### 직렬화 함수

| 함수 | 용도 |
|------|------|
| `serializeTransform()` | Transform4x4 → 64 bytes |
| `deserializeTransform()` | 64 bytes → Transform4x4 |
| `serializeConfig()` | IcpConfig → ~30 bytes |
| `deserializeConfig()` | ~30 bytes → IcpConfig |
| `serializeIcpChunk()` | IcpChunk → variable bytes |
| `deserializeIcpChunk()` | variable bytes → IcpChunk |
| `serializeIcpResult()` | IcpResult → variable bytes |
| `deserializeIcpResult()` | variable bytes → IcpResult |

### 바이너리 포맷

**IcpChunk:**
```
[chunk_id: 4B]
[sourcePoints count: 4B][sourcePoints: count*12B]
[targetPoints count: 4B][targetPoints: count*12B]
[targetNormals count: 4B][targetNormals: count*12B]
[initialTransform: 64B]
[config: ~30B]
[source bbox: 24B]
[target bbox: 24B]
```

**IcpResult:**
```
[chunk_id: 4B]
[transform: 64B]
[localTransform: 64B]
[finalRMSE: 4B][initialRMSE: 4B]
[iterations: 4B][correspondences: 4B][sourcePts: 4B][targetPts: 4B]
[converged: 1B][success: 1B]
[processingTimeMs: 8B]
[errorMessage length: 4B][errorMessage: variable]
```

### 사용 예시

```cpp
#include "bimtree/IcpSerialization.hpp"

// IcpChunk 직렬화
icp::IcpChunk chunk;
chunk.chunk_id = 1;
chunk.sourcePoints = {{1.0f, 2.0f, 3.0f}, ...};
chunk.targetPoints = {{4.0f, 5.0f, 6.0f}, ...};

std::vector<uint8_t> buffer = icp::serializeIcpChunk(chunk);

// 네트워크 전송 후 역직렬화
icp::IcpChunk restored = icp::deserializeIcpChunk(buffer);

// IcpResult 직렬화
icp::IcpResult result;
result.success = true;
result.finalRMSE = 0.01f;

std::vector<uint8_t> resultBuffer = icp::serializeIcpResult(result);
icp::IcpResult restoredResult = icp::deserializeIcpResult(resultBuffer);
```

### NetworkUtils 래퍼

기존 DPApp::NetworkUtils 네임스페이스와 호환:

```cpp
// DPApp 네임스페이스에서 사용
auto buffer = DPApp::NetworkUtils::serializeIcpChunk(chunk);
auto chunk = DPApp::NetworkUtils::deserializeIcpChunk(buffer);

auto resultBuf = DPApp::NetworkUtils::serializeIcpResult(result);
auto result = DPApp::NetworkUtils::deserializeIcpResult(resultBuf);
```

## 예상 테스트 결과

```
=== Phase 3 Serialization Test ===
Transform4x4 serialization: PASS
IcpConfig serialization: PASS
IcpChunk serialization: PASS (buffer size: XXX bytes)
IcpResult serialization (success): PASS (buffer size: XXX bytes)
IcpResult serialization (error): PASS (message: "Test error message...")
NetworkUtils wrapper: PASS

=== Phase 3 Test Result: ALL PASSED ===
```

## 다음 단계
Phase 3 완료 후 Phase 4 (Slave 태스크 처리기)로 진행합니다.
