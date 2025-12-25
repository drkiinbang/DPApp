# Phase 5: Master REST API

## 개요
MasterApp에 ICP REST API를 추가하여 웹 인터페이스로 ICP 작업을 관리합니다.

## 파일 목록

| 파일 | 작업 | 대상 위치 |
|------|------|-----------|
| `MasterApplication_h_MODIFY_GUIDE.txt` | 수정 가이드 | (참조용) |
| `MasterApplication_ICP.cpp` | 신규 생성 | `F:\repository\DPApp\MasterApp\MasterApplication_ICP.cpp` |
| `MasterApplication_REST_MODIFY_GUIDE.txt` | 수정 가이드 | (참조용) |

## 적용 순서

### Step 1: MasterApplication.h 수정

#### 1.1 Include 추가 (Line 21, <memory> 아래)
```cpp
#include <map>
```

#### 1.2 Include 추가 (Line 29, Logger.h 아래)
```cpp
#include "../include/IcpTypes.h"
```

#### 1.3 ICP 멤버 변수 추가 (Line 101 근처)
```cpp
    /// =========================================
    /// ICP Task Management
    /// =========================================

    std::map<std::string, std::shared_ptr<icp::IcpJob>> icp_jobs_;
    std::mutex icp_jobs_mutex_;
    uint32_t next_icp_task_id_ = 10000;
```

#### 1.4 ICP 함수 선언 추가 (Line 191 근처)
```cpp
    /// =========================================
    /// ICP REST API Functions (MasterApplication_ICP.cpp)
    /// =========================================
    void setupIcpApiRoutes();
    HttpResponse handleIcpStart(const HttpRequest& req);
    HttpResponse handleIcpJobs(const HttpRequest& req);
    HttpResponse handleIcpJobStatus(const HttpRequest& req);
    HttpResponse handleIcpJobResult(const HttpRequest& req);
    HttpResponse handleIcpJobCancel(const HttpRequest& req);
    HttpResponse handleIcpJobStats(const HttpRequest& req);
    void processIcpJob(std::shared_ptr<icp::IcpJob> job);
    std::string generateIcpJobId();
    void handleIcpResult(const icp::IcpResult& result, uint32_t task_id);
```

### Step 2: MasterApplication_ICP.cpp 추가
```
MasterApplication_ICP.cpp → F:\repository\DPApp\MasterApp\MasterApplication_ICP.cpp
```

### Step 3: Visual Studio 프로젝트에 파일 추가

1. 솔루션 탐색기에서 MasterApp 프로젝트 우클릭
2. **추가** → **기존 항목**
3. `MasterApplication_ICP.cpp` 선택

### Step 4: MasterApplication_REST.cpp 수정

setupRestApiRoutes() 함수 끝에 추가:
```cpp
    /// =========================================
    /// ICP API Routes
    /// =========================================
    setupIcpApiRoutes();
}
```

### Step 5: MasterApp 빌드

## REST API 엔드포인트

| Method | Endpoint | 설명 |
|--------|----------|------|
| POST | `/api/icp/start` | ICP 작업 시작 |
| GET | `/api/icp/jobs` | 모든 작업 목록 |
| GET | `/api/icp/jobs/{id}` | 작업 상태 조회 |
| GET | `/api/icp/jobs/{id}/result` | 작업 결과 (변환 행렬) |
| POST | `/api/icp/jobs/{id}/cancel` | 작업 취소 |
| GET | `/api/icp/jobs/{id}/stats` | 상세 통계 |

## API 사용 예시

### ICP 작업 시작
```bash
curl -X POST http://localhost:8081/api/icp/start \
  -H "Content-Type: application/json" \
  -d '{
    "las_file": "C:/data/pointcloud.las",
    "bim_folder": "C:/data/bim",
    "config": {
      "max_iterations": 50,
      "convergence_threshold": 1e-6,
      "downsample_ratio": 10
    }
  }'
```

### 응답 예시
```json
{
  "success": true,
  "job_id": "icp_20241224_153045_123",
  "status": "PENDING",
  "message": "ICP job started"
}
```

### 작업 상태 조회
```bash
curl http://localhost:8081/api/icp/jobs/icp_20241224_153045_123
```

### 작업 결과 조회
```bash
curl http://localhost:8081/api/icp/jobs/icp_20241224_153045_123/result
```

## 현재 구현 상태

### 완료된 기능
- ✅ REST API 엔드포인트 구조
- ✅ Job 생성 및 관리
- ✅ 상태 조회 및 취소
- ✅ 통계 정보 제공

### TODO (실제 연동 필요)
- ⏳ LAS 파일 로딩 (LaslibReader 연동)
- ⏳ GLTF/BIM 로딩 (BimInfo 연동)
- ⏳ Pseudo point 생성 (PseudoPtsInMesh 연동)
- ⏳ Slave로 태스크 분배 (TaskManager 연동)
- ⏳ 결과 수신 및 집계
- ⏳ 정렬된 LAS 저장 (LaslibWriter 연동)

현재는 **스켈레톤 구현**으로, 실제 데이터 처리는 시뮬레이션됩니다.
실제 연동은 Phase 6에서 진행합니다.

## 다음 단계
Phase 5 완료 후 Phase 6 (실제 데이터 연동)로 진행합니다.
