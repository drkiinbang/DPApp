# Phase 1: ICP 타입 정의

## 개요
ICP(Iterative Closest Point) 정합 작업에 필요한 기본 데이터 타입을 정의합니다.

## 파일 목록

| 파일 | 작업 | 대상 위치 |
|------|------|-----------|
| `IcpTypes.h` | 신규 생성 | `F:\repository\DPApp\include\IcpTypes.h` |
| `PointCloudTypes_MODIFY_GUIDE.txt` | 수정 가이드 | (참조용) |
| `Phase1_CompileTest.cpp` | 테스트 코드 | (참조용) |

## 적용 순서

### Step 1: IcpTypes.h 복사
```
IcpTypes.h → F:\repository\DPApp\include\IcpTypes.h
```

### Step 2: PointCloudTypes.h 수정
`F:\repository\DPApp\include\PointCloudTypes.h` 파일을 열고:

1. **TaskType enum 수정** (Line 19-30 근처)
   - `TEST_FAIL = 13,` 다음에 추가:
   ```cpp
        // ICP 정합 타입
        ICP_COARSE_ALIGNMENT = 20,  // Coarse alignment (Master에서 수행)
        ICP_FINE_ALIGNMENT = 21,    // Fine alignment (Slave 분산 처리)
        ICP_APPLY_TRANSFORM = 22,   // Transform 적용 (결과 저장)
   ```

2. **taskStr 함수 수정** (Line 33-44 근처)
   - `default: return "unknown";` 앞에 추가:
   ```cpp
        case TaskType::ICP_COARSE_ALIGNMENT: return "icp_coarse";
        case TaskType::ICP_FINE_ALIGNMENT: return "icp_fine";
        case TaskType::ICP_APPLY_TRANSFORM: return "icp_apply";
   ```

3. **strTask 함수 수정** (Line 47-57 근처)
   - `return TaskType::UNKNOWN;` 앞에 추가:
   ```cpp
        if (str == "icp_coarse") return TaskType::ICP_COARSE_ALIGNMENT;
        if (str == "icp_fine") return TaskType::ICP_FINE_ALIGNMENT;
        if (str == "icp_apply") return TaskType::ICP_APPLY_TRANSFORM;
        if (str == "icp") return TaskType::ICP_FINE_ALIGNMENT;
   ```

### Step 3: 컴파일 테스트
1. Visual Studio에서 솔루션 열기
2. 빌드 → 솔루션 다시 빌드 (Ctrl+Shift+B)
3. 오류 없이 빌드되면 완료!

## 선택적: 상세 테스트
`Phase1_CompileTest.cpp`의 테스트 코드를 임시로 추가하여 
모든 타입이 정상 동작하는지 확인할 수 있습니다.

## Phase 1 정의 내용

### 새로운 타입
- `icp::Transform4x4` - 4x4 변환 행렬
- `icp::IcpConfig` - ICP 알고리즘 설정
- `icp::IcpChunk` - Slave로 전송할 청크 데이터
- `icp::IcpResult` - ICP 처리 결과
- `icp::IcpJobStatus` - 작업 상태 열거형
- `icp::IcpJob` - Master의 작업 관리 구조체
- `icp::IcpStatistics` - 통계 정보

### TaskType 추가
- `ICP_COARSE_ALIGNMENT (20)` - 전역 정합
- `ICP_FINE_ALIGNMENT (21)` - 정밀 정합 (분산 처리)
- `ICP_APPLY_TRANSFORM (22)` - 변환 적용

## 다음 단계
Phase 1 빌드 성공 후 Phase 2 (ICP 알고리즘 핵심)로 진행합니다.
