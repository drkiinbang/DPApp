# 2026-07-11 변경 기록 (Slave 분산 ICP 정합 구현)

## 배경

[CHANGELOG_2026-07-11.md](CHANGELOG_2026-07-11.md) §5에서 "Master가 ICP 정합을 단독으로 처리하고, Slave는 분산 작업에 전혀 참여하지 않는다"는 아키텍처 한계를 발견하고, 당시에는 "분산 ICP 구현은 이번 세션 범위 밖"으로 결정했습니다. 이후 논의에서 **ICP 대상 데이터 규모가 수억 포인트 이상**이라는 것이 확인되어, Master 단독 처리로는 여러 대의 Slave 컴퓨터를 전혀 활용하지 못하는 것이 실제로 큰 문제라고 판단, 이번에 실제로 구현했습니다.

## 청킹 전략

건물/플랜트 전체 BIM 모델(GLB)은 개별 부재(Revit Element ID, GLTF 노드 하나에 대응)들로 구성되어 있습니다. **부재 하나 = 청크 하나**로 나누고, 각 부재의 바운딩박스로 입력 포인트클라우드를 클리핑해서, "그 부재의 메시 + 클리핑된 포인트클라우드"로 구성된 청크를 만들어 Slave에 분산 처리하는 방식으로 구현했습니다. 이 구조는 이미 거리계산(`bimpc`) 기능이 쓰고 있던 청킹 방식과 사실상 동일합니다.

Slave 쪽 실행 로직(`IcpProcessor::processFineAlignment`, 기존 `runIcp()` 재사용)은 이미 만들어져 있어서, 청크 하나당 **완전히 수렴할 때까지 Slave가 로컬로 ICP를 돌린 뒤 최종 결과 하나만 반환**하는 방식으로 구현했습니다. 이 방식은 ICP 반복마다 네트워크 왕복이 필요 없어서, 대량의 소규모 태스크를 자주 주고받을 때 발생하는 통신 오버헤드 문제를 피할 수 있습니다.

---

## 1. [최우선] `IcpSerialization.hpp` 전면 float→double 수정 — ✅ 완료

분산 ICP를 실제로 구현하기 전에, 이 파일이 **한 번도 실전 검증된 적 없는 상태로 깨져 있었다는 것**을 발견했습니다. 예전 "float to double" 마이그레이션 커밋(전체 ICP 코드를 double로 전환)에서 이 파일만 빠졌던 것으로 보입니다 — 지금까지 ICP는 항상 Master 혼자 처리해서(네트워크 전송 자체가 없어서) 이 버그가 한 번도 발동한 적이 없었습니다.

- **파일**: [include/bimtree/IcpSerialization.hpp](include/bimtree/IcpSerialization.hpp)
- **`serializeTransform`/`deserializeTransform`**: `Transform4x4::m`은 `double[16]`(128바이트)인데 `sizeof(float)*16`(64바이트)만 복사 — 변환행렬의 절반이 유실.
- **`serializeConfig`/`deserializeConfig`**: `IcpConfig`의 5개 임계값 필드가 전부 `double`인데 `sizeof(float)`로 읽고 씀.
- **`serializeIcpChunk`/`deserializeIcpChunk`**: 포인트 배열(`sourcePoints`, `targetPoints`, `faceNormals`, `facePts`)이 `std::array<double,3>`(포인트당 24바이트)인데 `sizeof(float)*3`(12바이트)으로 memcpy — **포인트 좌표 데이터가 통째로 깨짐.**
- **추가 발견 — 필드 순서 불일치**: `serializeIcpChunk`는 `facePts`를 쓴 뒤 `initialTransform`을 쓰는데, `deserializeIcpChunk`는 반대 순서(`initialTransform`을 먼저, `facePts`를 나중)로 읽고 있었습니다. 두 필드가 서로 뒤바뀐 채로 역직렬화되는 완전히 별개의 버그였습니다.
- **`serializeIcpResult`/`deserializeIcpResult`**: `finalRMSE`/`initialRMSE`가 `double`인데 `sizeof(float)`로 읽고 씀.
- **검증**: 기존 컴파일 확인용 테스트(`#ifdef _DEBUG` 블록)는 `chunk_id`만 비교하고 실제 좌표값은 검증하지 않아 이 버그들을 전혀 잡아내지 못했습니다. 이번 수정은 §5의 실측 네트워크 왕복 테스트(포인트 개수·RMSE 값이 송신측과 정확히 일치)로 검증했습니다.

## 2. [높음] `TaskManagerTypes.h`에 IcpChunk 지원 추가 — ✅ 완료

- `TaskInfo` 구조체에 `icp_chunk_data`(`std::shared_ptr<icp::IcpChunk>`) 필드 추가
- `TaskManager::addTask(std::shared_ptr<icp::IcpChunk>)` 오버로드 추가 (기존 `BimPcChunk` 패턴과 동일)

## 3. [높음] Master의 태스크 디스패치/결과 처리에 ICP 청크 연결 — ✅ 완료

- **파일**: [MasterApp/MasterApplication.cpp](MasterApp/MasterApplication.cpp)
- `sendAssignedTasks()`에 `is_icp_task` 분기 추가 (`icp::serializeIcpChunk()`로 직렬화, bimpc와 동일한 패턴)
- `handleTaskResult()`의 `result_type==3`(ICP) 처리에서 `task_manager_->completeTask()`/`failTask()`도 함께 호출하도록 수정. 기존에는 `handleIcpResult()`만 호출해서 Slave가 TaskManager 관점에서 영원히 "사용 중" 상태로 남는 누락이 있었습니다(이전 세션에서 이 경로가 실제로 도달한 적이 없어 발견되지 않았던 문제).

## 4. [높음] Revit Element 단위 ICP 청킹 (`loadIcpElementChunks`) — ✅ 완료

- **파일**: [MasterApp/MasterApplication.cpp](MasterApp/MasterApplication.cpp), [MasterApp/MasterApplication.h](MasterApp/MasterApplication.h)
- `loadBimPcChunks()`를 참고해 새로 작성. GLTF의 각 노드(부재)를 순회하며, 부재의 바운딩박스(+`maxCorrespondenceDistance` 마진)로 좌표정합용 포인트클라우드(coarse 정합까지 적용된 상태)를 클리핑하고, 그 부재의 메시로부터 가상점(target point)을 생성해 `IcpChunk`를 구성합니다.
- 거리계산(bimpc) 청킹과의 차이: bimpc는 모든 포인트를 반드시 어딘가에 배정(가장 가까운 청크로 폴백)하지만, ICP 청킹은 **어느 부재의 박스에도 안 들어가는 점은 그냥 포함하지 않습니다**(그 부재와 무관한 점을 억지로 정합에 끼워넣지 않기 위함). 마진이 겹치는 경계 부근 점은 여러 부재의 청크에 중복 포함될 수 있으며, 이는 의도된 동작입니다(경계 점은 인접한 어느 부재의 정합에도 유효한 대응점 후보이므로).
- 대응점이 너무 적은 부재(`minCorrespondences` 미만)나 메시가 퇴화된 부재는 건너뜁니다.

## 5. [최우선] `processIcpJob()` 분산 처리 + 가중평균 결합 — ✅ 완료, 실측 검증됨

- **파일**: [MasterApp/MasterApplication_ICP.cpp](MasterApp/MasterApplication_ICP.cpp), [MasterApp/MasterApplication.h](MasterApp/MasterApplication.h)
- Fine alignment 단계에서 연결된 Slave가 있으면 `loadIcpElementChunks()`로 부재별 청크를 만들어 `task_manager_->addTask()`로 분산 디스패치하고, 각 청크가 완전히 수렴된 결과를 반환할 때까지 폴링 대기(`icp_task_to_job_` 매핑으로 결과를 올바른 job에 연결). Slave가 없거나 다른 ICP 작업이 이미 분산 처리 중이면(아래 참고) 기존 Master 단독 처리로 자동 폴백합니다.
- **가중평균 결합(`aggregateWeightedTransforms`)**: 여러 부재의 결과 변환행렬을 부재별 포인트 수로 가중평균한 뒤, 회전행렬 부분을 SVD로 재직교화해서 올바른 회전행렬로 복원합니다(회전은 단순 평균이 수학적으로 부정확하므로, 표준적인 "chordal L2 mean" 방식 사용). 청크가 1개일 때는 그 청크의 변환행렬을 그대로 반환하므로, 기존 단일 처리 방식과 동일하게 동작합니다.
- **동시성 안전장치(`icp_dispatch_mutex_`)**: `task_manager_`는 Master 전체가 공유하는 단일 자원이라, 여러 ICP 작업이 동시에 분산 디스패치를 시도하면 서로의 상태를 덮어쓸 위험이 있습니다. 분산 처리 구간 전체를 뮤텍스로 잠그고, 잠금을 즉시 획득하지 못하면(다른 ICP 작업이 이미 분산 중) 블로킹하지 않고 바로 Master 단독 처리로 폴백하도록 만들어 안전하게 처리했습니다.

### 실측 검증 (소규모, 371,643점, 부재 1개)

```
[ICP] Distributing fine alignment across 1 slave(s), one chunk per BIM element...
loadIcpElementChunks: 1 chunks created from 1 BIM elements (0 skipped: too few nearby points, 0 skipped: degenerate mesh)
Task added (Icp): 1 (icp_fine) - source points: 371643, target points: 26786
Task 1 assigned to slave_c91768de
```
Slave 로그:
```
Received task 1 (type: icp_fine, is_bimpc: No, chunk_data_size: 11036838 bytes)
Task 1 is ICP task (source: 371643 pts, target: 26786 pts)   <- 포인트 개수가 송신측과 정확히 일치(직렬화 수정 검증)
[ICP] Iter 1: Current RMSE (0.013514)deg
[ICP] Iter 2: Current RMSE (0.0135167)deg
Thread 0 completed ICP task 1 in 735ms (success: Yes, RMSE: 0.0135168)
```
Master 최종 결과: `final_rmse: 0.0135168` — 같은 데이터로 기존 Master 단독 처리했을 때의 `0.0134973`과 거의 동일(5번째 소수자리 수준 차이, 분산/비분산 경로 간 정상적인 수치 편차 범위). **정확성 확인.**

### 대규모 검증 (samsung.las 3.4GB, 부재 약 1,294개) — Slave 1대·스레드 1개로 45분간 실행

파이프라인 자체(디스패치→역직렬화→ICP 실행→결과 반환→집계)는 대규모에서도 정상 동작함을 확인했지만, **완주하지 못했고, 실제 데이터 특성상 중요한 튜닝 이슈를 발견**했습니다.

- **정상 동작 확인**: Slave가 실제로 수백만 점 규모의 청크(예: 730만 대응점)를 정확히 수신해 ICP 반복을 실행했고, 실패한 청크는 Master가 자동 재시도했으며(기존 재시도 메커니즘 정상 작동), 크래시나 데이터 손상 없이 45분간 안정적으로 동작했습니다.
- **완주 실패**: Slave 1대·스레드 1개로 45분간 처리한 결과 "Completed: 59, Failed: 102" (총 1,294개 청크 중 161개만 처리). 청크당 평균 처리시간이 약 21초로(총 처리시간 1080.5초 ÷ 161청크), 전체 1,294개를 순차 처리하면 **약 7~8시간**이 걸리는 것으로 추정됩니다. 이는 이번 테스트가 Slave를 1대만 사용했기 때문이며(실제 운영에서는 여러 Slave가 청크를 나눠 병렬 처리), 이번 세션에서는 여러 Slave 프로세스를 동시에 띄워 실제 처리시간 단축을 검증할 자원/시간이 없었습니다.
- **⚠️ 새로 발견한 튜닝 이슈 — 대응점 부족으로 인한 높은 실패율**: 실패한 청크는 전부 `"Not enough correspondences: 0"` — 즉 청크에 포인트는 있었지만(사전에 `minCorrespondences` 검사를 통과했음), 실제 ICP 대응점 탐색 단계에서 **기본 임계값(`maxCorrespondenceDistance = 1.0m`) 이내의 대응점을 하나도 찾지 못했습니다.** 성공한 대형 청크(730만 대응점)의 로그를 보면 대응 거리가 0.16~0.93m 사이에 많이 분포하며 RMSE가 0.48~0.49(라디안 단위로 보이나 실제로는 큰 잔차)로 수렴이 매우 느렸습니다. 이는 **플랜트 전체를 하나의 강체 변환으로 계산하는 1차(coarse) 정합이, 플랜트의 중심에서 먼 개별 부재에는 지역적으로 충분히 정확하지 않을 수 있음**을 시사합니다(대형 구조물의 경우 회전 오차가 중심에서 멀어질수록 병진 오차로 누적되는 전형적인 현상).
- **권장 후속 조치**:
  1. REST 요청의 `config.max_correspondence_distance` 값을 이런 대형 데이터셋에서는 기본값(1.0m)보다 크게 설정해서 재시도 (예: 2~5m)
  2. 그래도 실패율이 높다면, 1차 정합을 플랜트 전체가 아니라 구역별/부재군별로 나눠서 계산하는 "지역 coarse 정합" 전략을 검토
  3. 실제 처리 시간 단축 효과를 확인하려면 Slave를 여러 대(또는 `-t` 옵션으로 여러 스레드) 동시에 연결한 상태로 재실행 필요

---

## 알려진 제한 사항

- **동시 ICP 작업 제한**: 위에서 설명한 `icp_dispatch_mutex_` 때문에, 분산 처리를 사용하는 ICP 작업은 한 번에 하나만 진행됩니다. 두 번째 REST 요청이 들어오면 자동으로 Master 단독 처리로 폴백합니다(실패하지는 않지만, 분산의 이점을 못 받음). 여러 ICP 작업을 동시에 분산 처리해야 한다면 `task_manager_`를 세션별로 분리하는 추가 작업이 필요합니다.
- **부재 경계에서의 포인트 중복**: 마진이 겹치는 경계 부근 포인트가 여러 부재의 청크에 중복 포함될 수 있습니다(§4 참고, 의도된 동작이지만 전체 처리량에는 약간의 오버헤드가 됩니다).
- **대규모 부재의 처리 시간**: 이전 bimpc 대규모 테스트에서 확인했듯, 부재 하나당 포인트가 수천만 개에 달할 수 있습니다. 그런 대형 청크는 ICP 반복(대응점 탐색 + SVD)이 여러 번 필요해서 단순 거리계산보다 처리 시간이 오래 걸릴 수 있습니다.

## 부록. 변경된 파일 목록

| 파일 | 변경 내용 |
|---|---|
| `include/bimtree/IcpSerialization.hpp` | float→double 전면 수정, facePts/initialTransform 필드 순서 수정 (§1) |
| `include/TaskManagerTypes.h` | `icp_chunk_data` 필드, `addTask(IcpChunk)` 오버로드 추가 (§2) |
| `MasterApp/MasterApplication.cpp` | `sendAssignedTasks()` ICP 분기, `handleTaskResult()` 북키핑 수정, `loadIcpElementChunks()` 신규 (§3, §4) |
| `MasterApp/MasterApplication.h` | `loadIcpElementChunks` 선언, `icp_dispatch_mutex_` 추가 (§4, §5) |
| `MasterApp/MasterApplication_ICP.cpp` | `processIcpJob()` 분산 처리 분기, `aggregateWeightedTransforms()` 신규 (§5) |
