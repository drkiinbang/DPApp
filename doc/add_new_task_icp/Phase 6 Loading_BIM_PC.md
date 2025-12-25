# Phase 6: 실제 데이터 연동

## 개요
시뮬레이션 코드를 실제 LAS/GLTF 데이터 로딩으로 교체합니다.

## 파일 목록

| 파일 | 작업 | 대상 위치 |
|------|------|-----------|
| `PseudoPointGenerator.hpp` | 신규 생성 | `F:\repository\DPApp\include\bimtree\PseudoPointGenerator.hpp` |
| `MasterApplication_ICP.cpp` | 덮어쓰기 | `F:\repository\DPApp\MasterApp\MasterApplication_ICP.cpp` |

## 적용 순서

### Step 1: PseudoPointGenerator.hpp 복사
```
PseudoPointGenerator.hpp → F:\repository\DPApp\include\bimtree\
```

### Step 2: MasterApplication_ICP.cpp 덮어쓰기
```
MasterApplication_ICP.cpp → F:\repository\DPApp\MasterApp\
```
기존 파일을 이 파일로 교체합니다.

### Step 3: Include 경로 확인
`MasterApplication_ICP.cpp`에서 DLL 헤더 include 경로를 확인하세요:
```cpp
#include "../LasImport/LasImport.h"
#include "../BimImport/BimImport.h"
```

프로젝트 구조에 맞게 경로를 수정하세요. 예:
- `#include "LasImport.h"` (같은 폴더)
- `#include "../include/LasImport.h"` (include 폴더)

### Step 4: MasterApp 빌드

## 변경 내용

### processIcpJob() 함수 변경

| 단계 | 이전 (시뮬레이션) | 이후 (실제 연동) |
|------|------------------|-----------------|
| LAS 로딩 | 하드코딩 100000 pts | `loadLasFile()` |
| GLTF 로딩 | 하드코딩 50000 tri | `loadGltf()` |
| Pseudo Points | 하드코딩 200000 pts | `generatePseudoPoints()` |
| Coarse ICP | Identity 반환 | 실제 ICP 실행 |
| Fine ICP | 시뮬레이션 | 실제 ICP 실행 |

### 워크플로우

```
1. loadLasFile() → sourcePoints
2. loadGltf() → meshes
3. generatePseudoPoints() → targetPoints + targetNormals
4. Coarse ICP (downsampled x2)
5. Apply coarse transform
6. Fine ICP (downsampled)
7. Combine transforms → finalTransform
```

## PseudoPointGenerator.hpp

메시 표면에서 균일하게 분포된 pseudo points를 생성합니다.

### 주요 함수

```cpp
/// 메시에서 pseudo points 생성
void generatePseudoPoints(
    const std::vector<chunkbim::MeshChunk>& meshes,
    float gridSize,                              // 0.1m 기본값
    std::vector<std::array<float, 3>>& outPoints,
    std::vector<std::array<float, 3>>& outNormals);

/// PointCloudChunk를 ICP 포맷으로 변환
std::vector<std::array<float, 3>> convertPointCloudToIcp(
    const std::vector<chunkpc::PointCloudChunk>& chunks);
```

## 출력 결과

ICP 완료 시 반환되는 정보:
- `final_transform`: 4x4 변환 행렬
- `final_rmse`: 최종 RMSE
- `coarse_rmse`: Coarse 정렬 RMSE
- `elapsed_time_sec`: 처리 시간
- `total_points`: 총 포인트 수

## 테스트

빌드 후 동일한 curl 명령으로 테스트:
```bash
curl -X POST http://localhost:8081/api/icp/start \
  -H "Content-Type: application/json" \
  -d "{\"las_file\": \"F:/path/to/file.las\", \"bim_folder\": \"F:/path/to/glb\"}"
```

이번에는 실제 데이터가 로딩되고 ICP가 실행됩니다.
