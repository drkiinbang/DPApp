# 09. 다음 릴리즈 Future Works (로드맵)

이 문서는 **다음 릴리즈에 포함할 개선 과제**를 우선순위·작업량·위험도와 함께 정리한 단일 로드맵입니다.
신규 기능(1~2번)부터 사용성·성능·유지보수 후속 과제(3~11번)까지, 다음 작업자가 "무엇을 어디부터 만들지"를 한눈에 볼 수 있게 정리했습니다. (이전에 별도로 있던 "향후 개선 방향" 문서의 살아있는 항목들도 이 문서로 통합했습니다 — 이미 구현 완료된 항목은 폐기.)

작성 기준일: 2026-07-13

---

## 요약 (Summary)

| # | 과제 | 분류 | 우선순위 | 예상 작업량 | 위험도 |
|---|---|---|---|---|---|
| 1 | **Linux(Ubuntu) Slave 지원** — 이식 + 테스트 | 플랫폼 | 높음 | 큼 (2~4주) | 중 |
| 2 | **단위 원자 형상(primitive) 기반 유사도 측정** | 알고리즘/정확도 | 높음 | 큼 (3~6주) | 중~높음 |
| 3 | 동시 다중 ICP 작업 지원 (icp_dispatch_mutex_ 제거) | 아키텍처 | 중 | 중 | 중 |
| 4 | 부재 청킹의 공간 인덱스(spatial index) 도입 | 성능(대규모) | 중 | 중 | 낮음 |
| 5 | 타임아웃/재시도 조율 정교화 | 견고성 | 중 | 작음 | 낮음 |
| 6 | 브라우저 뷰어 LAZ 입력 지원 | 사용성 | 낮음 | 중 | 낮음 |
| 7 | Slave 측 가상점 생성(대역폭 최적화) | 성능(네트워크) | 낮음 | 중 | 중 |
| 8 | 알려진 결함 정리 + 죽은 코드 제거 | 유지보수 | 중 | 작음 | 낮음 |
| 9 | 단계적(progressive) 다운샘플링 | 성능(ICP) | 낮음 | 작음 | 낮음 |
| 10 | 사용성(편의성) 개선 (CLI 프리셋 / config 명령 / 한국어 오류 메시지) | 사용성 | 중 | 작음~중 | 낮음 |
| 11 | SVD 계산 최적화 | 성능(ICP) | 낮음 | 작음 | 낮음 |

> 1번과 2번이 이번 로드맵의 핵심이며, 3~11번은 이번 세션의 코드 리뷰·검증 및 기존 인수인계 문서에서 확인된 후속 과제들입니다. 이 중 10~11번은 핵심 알고리즘(IcpCore.hpp)을 건드리지 않고 바깥쪽(CLI·REST·설정 읽기)만 손대면 되는 비교적 안전한 작업이라, 코드 구조에 익숙해지려는 신규 작업자에게 추천합니다.

---

## 1. Linux(Ubuntu) Slave 지원 ⭐

### 목표
현재 전 구성요소가 Windows 전용(MSVC + winsock + Win32 API)입니다. **연산 자원이 풍부한 Ubuntu 서버/워크스테이션을 Slave로 투입**할 수 있게 하면, 이기종(Windows Master + Linux Slave) 클러스터로 대규모 정합 처리량을 크게 늘릴 수 있습니다.

### 왜 Slave부터인가
Master(작업 분배·REST API·CLI)는 Windows에 남겨두고 **Slave만 먼저 Linux로 이식**하는 것이 가장 실용적입니다. `SlaveApp`은 리스닝 포트가 없는 **순수 아웃바운드 TCP 클라이언트**라 (Master에 접속만 하면 됨) 이식 표면이 좁고, 실제 무거운 ICP/거리계산 연산이 여기서 일어나므로 투자 대비 효과가 가장 큽니다.

### 이식 범위 (파일별 구체 진단)

| 구성요소 | 현재 상태 | 이식 작업 |
|---|---|---|
| **네트워크 계층** (`src/NetworkManager.cpp`, `include/NetworkManager.h`) | 이미 `#ifdef _WIN32 / #else`로 recv/send/소켓 정리 등 상당 부분이 POSIX 분기를 갖춤 (부분 이식성 확보) | winsock 전용 심볼(`WSAStartup`/`WSACleanup`/`closesocket`/`WSAGetLastError`/`SD_BOTH`)을 남은 곳까지 `#ifdef`로 감싸거나 얇은 소켓 추상화 계층으로 통일. **가장 작업량 적음** |
| **파일 IO** (`include/Pc2Reader.hpp`) | 완전 Windows 전용 (`CreateFileA`/`ReadFile`/`OVERLAPPED`/`GetFileSizeEx`/`CloseHandle`) | POSIX `open`/`pread`(오프셋 지정 읽기 = OVERLAPPED 대체)/`fstat`/`close`로 재작성, 또는 `std::ifstream`+`seekg` 기반으로 단순화. `read_at()` 하나만 플랫폼 분기하면 됨 |
| **핵심 연산** (`include/bimtree/*`, `include/TaskManager.h`, `include/IcpTypes.h`) | Eigen + nanoflann + 표준 C++17. **대부분 이미 이식 가능** | `localtime_s`→`localtime_r`, `strcpy_s`→`strncpy` 등 안전 함수 소수만 치환. `#include <windows.h>` 잔재 제거 |
| **LAStools 링크** | `vendors/LAStools/`에 Windows용 `.lib`/`.dll`만 존재 | LASlib/LASzip는 Linux 빌드 제공 → `.a`/`.so`를 별도 확보하거나 소스에서 빌드 |
| **빌드 시스템** | `.vcxproj`(MSBuild) 중심, 루트에 `CMakeLists.txt`는 존재하나 크로스플랫폼 검증 안 됨 | CMake를 표준 빌드 경로로 정비 (Windows/Linux 모두 지원). 컴파일러 정의(`_WIN32` 등) 정리 |
| **프로세스 생성** (`SlaveAgent/agent.cpp`, `MasterLauncher/LauncherService.cpp`) | `CreateProcessA`/`TerminateProcess`로 Slave 프로세스 자동 기동 | **1단계에서는 이식 불필요** — Linux Slave는 `./SlaveApp <master_ip> <port>`로 수동/스크립트 기동하면 됨. Agent 자동기동은 2단계로 미룸 |
| **콘솔/시그널** | `SetConsoleCtrlHandler` 등 | `signal(SIGINT/SIGTERM)` 기반으로 치환 (SlaveApp에 이미 `signal()` 사용부 있음) |

### 단계 계획
1. **1단계 (MVP)**: `SlaveApp`만 Linux 콘솔 바이너리로 빌드. Windows Master + Linux Slave 1대로 REST `/api/icp/start` end-to-end 성공시키기. Agent 자동기동은 제외(수동 실행).
2. **2단계**: `SlaveAgent`도 이식(`fork`/`exec`)해 Master의 `start-slaves`로 Linux Slave도 자동 기동.
3. **3단계**: 이기종 혼합 클러스터(Windows N대 + Linux M대) 대규모 부하 테스트, 성능·안정성 비교.

### 완료 조건 (Acceptance)
- Ubuntu 22.04에서 `SlaveApp` 빌드 성공 + Windows Master에 접속·등록됨(`slaves` 명령에 표시).
- 합성 데이터(gensynth) 분산 ICP를 Linux Slave가 처리해 결과가 Windows Slave와 동일 범위(RMSE)로 나옴.
- **이기종 직렬화 호환성 회귀 테스트**: Windows↔Linux 간 `IcpChunk`/`BimPcChunk`/`IcpResult` 바이너리 직렬화가 엔디안·구조체 정렬·`size_t` 폭(Win64 `unsigned long long` vs Linux `unsigned long`) 차이 없이 왕복되는지 반드시 검증 (직렬화 코드가 고정폭 타입 `uint32_t`/`uint64_t`를 쓰는지 점검 필요 — 여기가 가장 흔한 이기종 버그 지점).

---

## 2. 단위 원자 형상(primitive) 기반 유사도 측정 ⭐

### 현재 방식과 한계
지금은 BIM 메시를 삼각형에서 샘플링한 **가상점(pseudo-point) 구름**으로 바꾼 뒤, 스캔 포인트와 **point-to-plane / point-to-triangle 법선거리(normal distance)**로 대응점을 찾아 RMSE로 정확도를 평가합니다 (`include/bimtree/IcpCore.hpp`의 `findCorrespondencesWithNormals`, `include/bimtree/PseudoPointGenerator.hpp`의 `pointToTriangleDistanceSquared`).

한계:
- **메시 테셀레이션 밀도에 결과가 흔들림** — 같은 실린더라도 삼각형을 촘촘히/성기게 쪼갠 정도에 따라 가상점 분포가 달라짐.
- **삼각형 이산화 오차** — 곡면(배관·탱크)을 평면 조각으로 근사하므로, 참(true) 표면까지의 거리가 아니라 "가장 가까운 삼각형까지의 거리"만 봄.
- **형상 의미를 못 봄** — "이 배관의 지름이 설계대로인가", "찌그러졌는가" 같은 **형상 기반 시공 QC**를 할 수 없음. 단지 "점이 면에 얼마나 가까운가"만 앎.

### 제안: 파라메트릭 원자 형상 모델
BIM 부재를 **면(plane) / 실린더(cylinder) / 구(sphere) / 반구(hemisphere) / 도넛(torus)** 등 파라메트릭 원시 형상으로 표현하고, 스캔 포인트와 그 **형상 표면 사이의 해석적(analytic) 거리**로 유사도를 측정합니다.

예: 실린더는 `(축 위의 한 점 c, 축 방향 단위벡터 a, 반지름 r)`로 정의 → 점 p의 표면거리 = `| distance(p, 축직선) − r |`. 삼각형 근사 없이 **참 표면까지의 정확한 거리**를 O(1)로 계산.

### 기대 효과
- **테셀레이션 무관**: 형상 파라미터만 쓰므로 메시를 어떻게 쪼갰든 동일 결과.
- **정확한 거리**: 곡면에 대한 참 거리 → 정합 RMSE와 QC 판정이 더 신뢰성 있음.
- **형상 기반 QC**: 반지름·길이·중심축 같은 파라미터를 설계값과 직접 비교 → "지름 오차", "설치 각도 오차", "국부 변형(dent)" 정량화 가능.
- **이번 릴리즈의 twist/shift 분해와 자연스럽게 연결**: 이번에 부재별 정합 결과를 "중심축 기준 twist/swing/axial/lateral"로 분해하는 기능을 넣었는데(`IcpElementAlignment`), **실린더로 피팅하면 그 축이 곧 중심축**이 됩니다. 지금은 centerline이 없으면 PCA로 축을 추정(대칭 형상에서 불안정)하는데, primitive 피팅이 이 축 추정을 훨씬 견고하게 대체할 수 있음.

### 구현 접근
1. **형상 타입 결정**:
   - (권장) **BIM/Revit 메타데이터 활용** — Revit은 배관=실린더, 탱크=실린더+반구 돔 등 형상 정보를 원래 알고 있음. GLTF export 시 `extras`에 형상 타입/파라미터를 실어보내면 피팅 없이 바로 사용 (이번 릴리즈의 `centerline` extras 훅과 동일한 확장 지점).
   - (폴백) **형상 자동 피팅** — 메타데이터가 없으면 부재의 정점/포인트에 대해 RANSAC 또는 최소제곱(LSQ)으로 형상을 피팅. 실린더/구는 표준 피팅 알고리즘 존재. 여러 형상 후보 중 잔차(residual)가 가장 작은 것을 채택.
2. **거리/유사도 함수**: 형상별 `pointToPrimitiveDistance()` 구현 (plane/cylinder/sphere/hemisphere/torus). 기존 `pointToTriangleDistanceSquared`와 같은 자리(`PseudoPointGenerator.hpp` 또는 신규 `PrimitiveShapes.hpp`)에 배치.
3. **ICP 통합**: `findCorrespondencesWithNormals`의 대응점 거리 계산을 "가장 가까운 삼각형" 대신 "부재의 primitive 표면"으로 교체(또는 옵션으로 선택). 알고리즘 골격(반복·SVD 변환추정)은 그대로 재사용.
4. **QC 리포트 확장**: `_alignment.json`에 부재별 `primitive_type`, 피팅된 파라미터(반지름/축/중심), 설계값 대비 오차, 표면 잔차 통계를 추가.

### 단계 계획
1. **1단계**: 가장 흔하고 효과 큰 **plane + cylinder** 두 형상만 먼저 (배관·평판·벽이 대부분). 자동 피팅 + point-to-primitive 거리 → 기존 방식과 정확도/안정성 비교.
2. **2단계**: sphere/hemisphere/torus 추가 (탱크 돔, 엘보 등).
3. **3단계**: BIM extras 기반 형상 파라미터 직접 수신 경로 추가(피팅 생략), 형상 기반 QC 리포트 완성.

### 위험/유의
- 자동 피팅은 노이즈·부분 가림(occlusion)·복합 형상(한 부재가 여러 primitive)에서 실패할 수 있음 → 피팅 신뢰도(잔차) 임계값으로 게이팅하고, 실패 시 기존 메시 기반 방식으로 폴백하는 **하이브리드**가 안전.
- "한 부재 = 한 primitive"가 아닌 경우(밸브+배관 조합 등) 부재를 여러 primitive로 분해하는 세그멘테이션이 필요 → 초기에는 단일 primitive로 근사하고 잔차로 품질 관리.

---

## 3. 동시 다중 ICP 작업 지원

**현재 한계**: `icp_dispatch_mutex_` 때문에 여러 사용자가 동시에 서로 다른 ICP 정합을 REST로 요청하면, 첫 요청만 분산 처리되고 나머지는 Master 단독(로컬) 처리로 폴백함 (`MasterApplication.h`의 해당 뮤텍스 주석 참고).

**개선 방향**: `task_manager_`가 단일 공유 자원인 구조를 **Job 단위 세션**으로 분리. 각 ICP 작업이 자기 TaskManager(또는 격리된 태스크 네임스페이스)를 갖게 하면 동시 분산 처리가 가능.

**유의**: 이번 릴리즈에서 이미 **task_id를 프로세스 수명 동안 전역 유일하게** 만들어 두었으므로(세대 간 충돌 방지), 이 위에서 세션 분리를 얹기가 한결 수월해짐.

---

## 4. 부재 청킹의 공간 인덱스 도입

**현재 한계**: `MasterApplication.cpp`의 `loadIcpElementChunks()`가 "모든 포인트 × 모든 부재" 이중 루프로 bbox 매칭(O(points × elements)). 수억 포인트 × 수천 부재(전체 플랜트) 규모에서 병목이 될 수 있음.

**개선 방향**: BIM 부재 bbox들에 대해 공간 인덱스(uniform grid / R-tree / BVH)를 만들고, 각 포인트가 **후보 부재만** 검사하도록 변경.

**유의**: 실측으로 이 루프가 병목이라고 **측정된 적은 아직 없음**. 대규모 실사용 전에 프로파일링으로 실제 비용을 먼저 확인한 뒤 착수하는 것이 합리적.

---

## 5. 타임아웃/재시도 조율 정교화

이번 릴리즈에서 (a) 결과 수신 데이터 경쟁 제거, (b) 타임아웃 시 부분 결과 명시(`partial_result`), (c) TaskManager 재시도 상태(`isTaskTerminal`) 기반 대기로 개선했습니다.

**남은 개선**:
- Slave 연결 끊김을 **이벤트 기반으로 즉시 감지**해 재할당(현재는 타임아웃 경과 후에야 감지 — `unregisterSlave`가 `is_active=false`만 표시).
- `max_retries`를 런타임 설정으로 노출(현재 `TaskManagerTypes.h`에 3으로 하드코딩).

---

## 6. 브라우저 뷰어 LAZ 입력 지원

**현재**: `tools/pointcloud_viewer.html`은 순수 JS(외부 라이브러리 없음) 원칙 때문에 `.laz`(LASzip 압축)를 명시적으로 거부하고 ".las로 변환 후 열라"고 안내. C++ 엔진은 `.laz`를 정상 지원(LASlib+LASzip).

**개선 방향**: 순수 JS LASzip 디코더(WASM 포함) 도입 검토. "외부 라이브러리 없음" 원칙을 유지하려면 자체 구현이 필요하나 비용이 큼 → WASM 형태의 검증된 디코더를 옵션으로 로드하는 절충안이 현실적.

---

## 7. Slave 측 가상점 생성 (대역폭 최적화)

**현재**: 가상점(pseudo-point) 생성을 Master에서 수행해 청크에 담아 전송. 촘촘한 가상점 구름이 네트워크 전송량의 큰 부분을 차지.

**개선 방향**: 메시(작음)만 Slave에 보내고 가상점 생성을 Slave에서 수행 → 전송량 감소. 순수 대역폭 최적화라 이전 검토에서 보류했으나, 대규모/원거리 클러스터에서는 효과가 커질 수 있음. **2번(primitive 기반)이 도입되면 가상점 자체가 불필요해질 수 있어**, 이 항목은 2번 진행 상황을 보고 판단하는 것이 좋음.

---

## 8. 알려진 결함 정리 + 죽은 코드 제거

이번 세션 코드 정리 과정에서 발견해 **주석으로 표시만 해둔** 항목들 (실사용 경로가 아니거나 범위 밖이라 수정 보류):

- **`include/bim/face.h`의 `Face::generate_pseudo_points()`**: `edge2`가 `v2 - v2`(항상 영벡터)로 계산됨 — 원래 의도는 `v3 - v2`로 추정. 이 함수는 `TaskManager.h`에서 실제 호출되므로 동작 확인 후 수정 필요.
- **`include/DefinePointType.hpp`의 `BOUNDING_BOX::make_cube()`**: z 범위 계산에 `z_center` 대신 `x_center`를 씀(복붙 실수). 현재는 미사용(호출부 없음)이나 사용 전 수정 필요.
- **`include/bimtree/IcpCore.hpp`의 NLLS 블록**: `calDiscrepancy`/`RETVAL_NLLS`/`estimateRigidTransformPointToPlane_NLLS`/`runLSM` — 확인된 미사용(dead) 실험 코드. 유지할지 제거할지 결정 필요.
- **`include/bim/mbr.h`의 `MBR::is_intersect()`**: 비대칭 교차 판정(한쪽 박스 모서리만 검사) — 완전한 AABB 교차가 필요하면 양방향 검사로 보완.
- **`include/bim/gltf_mesh.h`의 `ExtractMeshFromNode`**: 퇴화 삼각형 폴백 블록이 `if (false)`로 비활성화됨 — 의도인지 실수인지 확인 필요.

---

## 9. 단계적(progressive) 다운샘플링

**현재 상태**: 균일 추출(`downsampleUniform`)과 격자 추출(`downsampleVoxelGrid`) 두 방식이 있으나, `downsampleRatio`가 반복 회차와 무관하게 고정임.

**개선 방향**: 초반 반복(전역 정합)은 강하게 다운샘플해 빠르게 큰 틀을 맞추고, 후반 반복(정밀 정합)은 점을 더 많이 써서 정밀도를 높이는 "단계적 다운샘플링"으로 같은 정확도에서 속도를 높임. `include/bimtree/IcpCore.hpp`의 `runIcp()` 반복 루프에서 `downsampleRatio`를 회차에 따라 조정.

---

## 10. 사용성(편의성) 개선

핵심 알고리즘을 건드리지 않고 바깥쪽(CLI·REST·설정)만 손대는 비교적 안전한 개선들입니다.

| 항목 | 현재 상태 | 개선 방향 | 손볼 위치 |
|---|---|---|---|
| **명령어를 매번 타이핑** | `bimpc 폴더 파일`처럼 콘솔에 직접 입력 | 자주 쓰는 설정 조합을 "프리셋"으로 저장/불러오기 | `MasterApp/MasterApplication_CLI.cpp` |
| **설정을 .ini 수동 편집** | 메모장으로 `master.ini`/`slave.ini` 직접 수정 (대부분 설정은 아직 코드에 미반영). `processing_threads`만 `SlaveApp.exe -t <개수>` 또는 `start-slaves <개수>`로 조정 가능 | 콘솔에서 바로 조회/변경(`config get`/`config set`) 하거나 `.ini` 파싱 자체를 구현 | `MasterApplication_CLI.cpp`, `include/RuntimeConfig.h` |
| **오류 메시지가 개발자 위주(영어/코드 용어)** | 예: `unresolved correspondence count < minCorrespondences` | 사람이 읽기 쉬운 한국어 안내로 변환 (예: "대응점이 너무 적습니다. maxCorrespondenceDistance 값을 늘려보세요") | 오류 지점을 감싸는 얇은 래퍼 (`include/Logger.h` 활용) |

---

## 11. SVD 계산 최적화

**현재 상태**: `include/bimtree/IcpCore.hpp`의 `estimateRigidTransformSVD()`가 Eigen SVD로 회전/이동을 계산.

**개선 방향**: 점 개수가 매우 많으면 SVD 비용이 커짐. 다운샘플된 소수의 대응점 쌍만으로 SVD를 계산하도록 보장하고(이미 그럴 가능성 높음 — 먼저 확인), 필요하면 Eigen 멀티스레드 옵션(`EIGEN_USE_THREADS`) 활성화 검토. **11번은 9번(단계적 다운샘플링)과 함께 보면 좋습니다** — 후반 반복에서 점이 많아질 때 SVD 비용이 함께 커지기 때문.

---

## 부록 A: 성능 측정 방법 (개선 전후는 반드시 숫자로 비교)

성능 개선은 "느낌"이 아니라 측정으로 판단합니다. 항상 **"정확도(RMSE)는 유지하면서 속도만 빠르게"**가 목표이므로, 속도와 RMSE를 함께 기록하세요.

1. 동일 데이터(예: `data/samsung_test/`)·동일 설정으로 개선 전/후 각각 3회 실행.
2. Master 로그(`logs/master.log`)의 시작~완료 타임스탬프 차이로 처리 시간 측정, 평균 비교.
3. 최종 RMSE도 함께 기록 → "빨라졌지만 정확도가 떨어지지 않았는지" 확인.
4. 다운샘플을 과하게 키우면 정합이 부정확해질 수 있으니 RMSE 악화 여부를 반드시 확인.

## 부록 B: 작업 시작 전 체크리스트

1. [04_테스트_가이드.md](04_테스트_가이드.md)의 체크리스트를 먼저 통과시켜 "현재 상태가 정상 동작한다"는 기준점을 확보.
2. `doc/`, `data/`, `bin/` 폴더는 `.gitignore` 대상이므로, 이 폴더 안의 파일을 커밋하려면 `git add -f`로 강제 추가해야 함(소스 코드는 일반 git 워크플로 그대로). 단, LAStools 라이브러리·런타임 config·소형 테스트 fixture 등 clone에 꼭 필요한 것들은 이번 릴리즈에서 이미 강제 추가로 커밋되어 있음.
3. 한 번에 많이 고치지 말고 "항목 하나 적용 → 테스트 → 성능 측정 → 다음 항목" 순서로 진행.

---

## 부록 C: 이번 릴리즈에서 이미 반영된 것 (참고)

다음은 이 로드맵을 쓰기 직전(2026-07 세션)에 **이미 완료**되어, 위 과제들의 토대가 된 항목입니다:

- 부재별 fine alignment 분산 처리 + 가중 통합(point-count weighted rotation/translation averaging).
- 좌표 float+offset rebase(대형 절대좌표에서의 정밀도 확보).
- **부재별 정합 결과를 중심축 기준 twist/swing/axial/lateral로 분해** + 축 신뢰도(`axis_reliable`) 표시 → 시공 QC의 기반. (2번 primitive 피팅과 직결)
- 분산 결과 수신 데이터 경쟁 제거, 타임아웃 시 부분 결과 명시, task_id 세대 간 충돌 방지, 재시도 상태 기반 대기.
- 합성 테스트 데이터 생성기(gensynth) + BIM/포인트클라우드 브라우저 뷰어(`tools/pointcloud_viewer.html`, gensynth와 동일 방법으로 합성 스캔 생성/저장).
- 소형 테스트 fixture로 clone 즉시 단위 테스트 가능하도록 정비.
