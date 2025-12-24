# Phase 2: ICP 알고리즘 핵심

## 개요
ICP(Iterative Closest Point) 알고리즘의 핵심 기능을 구현합니다.

## 파일 목록

| 파일 | 작업 | 대상 위치 |
|------|------|-----------|
| `IcpCore.hpp` | 신규 생성 | `F:\repository\DPApp\include\bimtree\IcpCore.hpp` |
| `Phase2_CompileTest.cpp` | 테스트 코드 | (참조용) |

## 의존성

| 라이브러리 | 용도 | 상태 |
|------------|------|------|
| nanoflann.hpp | KD-Tree | 이미 있음 (`include/bimtree/`) |
| Eigen | SVD 계산 | 이미 설치됨 |

## 적용 순서

### Step 1: IcpCore.hpp 복사
```
IcpCore.hpp → F:\repository\DPApp\include\bimtree\IcpCore.hpp
```

### Step 2: 컴파일 확인
1. Visual Studio에서 솔루션 열기
2. 빌드 → 솔루션 다시 빌드 (Ctrl+Shift+B)

### Step 3: (선택) 기능 테스트
`Phase2_CompileTest.cpp`의 테스트 코드를 사용하여 ICP 알고리즘이 
정상 동작하는지 확인할 수 있습니다.

## 구현 내용

### 1. KD-Tree 기반 Correspondence 찾기
```cpp
icp::PointCloudAdaptor adaptor(points);
icp::KdTree3D tree(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(50));
tree.buildIndex();

std::vector<icp::Correspondence> correspondences;
icp::findCorrespondences(source, tree, maxDistSq, correspondences);
```

### 2. Outlier 제거 (MAD 기반)
```cpp
icp::rejectOutliersMAD(correspondences, 2.5f);  // 2.5 * MAD
icp::rejectOutliersPercentile(correspondences, 90.0f);  // 90th percentile
```

### 3. SVD 기반 변환 추정
```cpp
icp::Transform4x4 T = icp::estimateRigidTransformSVD(source, target, correspondences);
```

### 4. RMSE 계산
```cpp
float rmse = icp::computeRMSE(correspondences);
```

### 5. 다운샘플링
```cpp
auto downsampled = icp::downsampleUniform(points, 10);      // 10% 추출
auto voxelized = icp::downsampleVoxelGrid(points, 0.1f);   // 0.1m 복셀
```

### 6. ICP 메인 알고리즘
```cpp
icp::IcpChunk chunk;
chunk.sourcePoints = sourcePoints;
chunk.targetPoints = targetPoints;
chunk.config.maxIterations = 50;

std::atomic<bool> cancel{false};
icp::IcpResult result = icp::runIcp(chunk, cancel);
```

### 7. 결과 집계
```cpp
icp::Transform4x4 avg = icp::aggregateResultsWeighted(results);
const icp::IcpResult* best = icp::selectBestResult(results);
```

## 주요 함수 목록

| 함수 | 설명 |
|------|------|
| `findCorrespondences()` | 최근접점 대응 찾기 |
| `findCorrespondencesWithNormals()` | 법선 필터링 포함 대응 찾기 |
| `rejectOutliersMAD()` | MAD 기반 이상치 제거 |
| `rejectOutliersPercentile()` | 백분위 기반 이상치 제거 |
| `estimateRigidTransformSVD()` | SVD로 변환 행렬 추정 |
| `computeRMSE()` | RMSE 계산 |
| `transformPointCloud()` | 점군 변환 (in-place) |
| `transformPointCloudCopy()` | 점군 변환 (복사본) |
| `downsampleUniform()` | 균일 다운샘플링 |
| `downsampleVoxelGrid()` | 복셀 그리드 다운샘플링 |
| `runIcp()` | ICP 메인 알고리즘 |
| `aggregateResultsWeighted()` | 가중 평균 결과 집계 |
| `selectBestResult()` | 최적 결과 선택 |

## 예상 오류 및 해결

### Eigen include 오류
```
fatal error: Eigen/Dense: No such file or directory
```
**해결**: Eigen include 경로 확인. 프로젝트 설정의 추가 포함 디렉터리에 Eigen 경로 추가.

### min/max 매크로 충돌
```
error C2589: '(' : illegal token on right side of '::'
```
**해결**: 이미 `IcpCore.hpp`에서 `(std::min)`, `(std::max)` 형태로 처리함.

## 다음 단계
Phase 2 빌드 성공 후 Phase 3 (직렬화)로 진행합니다.
