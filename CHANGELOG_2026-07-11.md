# 2026-07-11 변경 기록 (외부 분석 리포트 검토 → 9개 업무 처리)

2026-07-10에 공유받은 외부 분석 리포트를 코드/로그로 검증한 뒤, 리포트에서 도출된 9개 업무를 시급한 순서대로 전부 처리한 기록입니다. 각 항목은 코드 수정 후 **Master/Slave를 실제로 재기동해 재현 테스트로 검증**했습니다.

관련 문서: [CHANGELOG_2026-07-10.md](CHANGELOG_2026-07-10.md), [doc/handover/05_사용자_매뉴얼.md](doc/handover/05_사용자_매뉴얼.md)

---

## 1. [최우선] `bimpc` 직렬화/역직렬화 Buffer overflow 수정 — ✅ 완료

- **파일**: [src/NetworkManager.cpp](src/NetworkManager.cpp) `deserializeBimPcChunk()`, `deserializeMeshChunk()`
- **근본 원인**: `BimPcChunk`(`include/PointCloudTypes.h`)와 `chunkbim::MeshChunk`(`include/bim/MeshChunk.h`)의 바운딩박스 필드(`min_x`~`max_z`)는 전부 **`double`(8바이트)**인데, `serialize` 쪽은 `double`로 6개 필드(48바이트)를 쓰고 **`deserialize` 쪽은 `float`(4바이트)로 6개 필드(24바이트)만 읽고** 있었습니다. 24바이트가 덜 소비되면서 바로 다음 필드(`bim_data_size`, 4바이트)를 엉뚱한 위치의 바이트로 읽어 거대한 쓰레기 값이 되고, 그 값으로 버퍼를 읽으려다 `"Buffer overflow: insufficient data to read."` 예외가 발생했던 것입니다.
- **수정**: 두 함수 모두 `reader.read<float>()` → `reader.read<double>()`로 변경.
- **검증**: 소규모 샘플(371,643점)로 Slave에 태스크 전달 시 더 이상 오류 없이 `"Processing completed successfully"` 확인. 이후 **3.4GB 실데이터(1,294개 청크, 1억 2천5백만 점) 전체를 처리하는 동안 단 한 건의 오류도 발생하지 않음**(§9 참고)으로 최종 확인.

## 2. [최우선] `BimPcProcessors::processBimPc()` 실제 거리계산 로직 구현 — ✅ 완료

- **파일**: [include/TaskManager.h](include/TaskManager.h)
- **문제**: 하드코딩된 가짜 값(`min=0, max=100, avg=50`)만 반환하던 더미 함수였습니다.
- **수정**: 기존 ICP 코드(`include/bimtree/PseudoPointGenerator.hpp`, `include/bimtree/IcpCore.hpp`)의 KD-Tree/가상점 생성 로직을 재사용해 실제 point-to-mesh 최근접 거리를 계산하도록 구현. 포인트별 거리·최근접 face id를 전량 반환(기존 더미는 처음 100개만 샘플링했음), min/max/avg/표준편차, 허용오차(5cm) 내/외 포인트 수 계산.
- **검증**: 소규모 샘플 기준 `avg_distance=0.024m, min=0.0001m, max=0.129m` (해당 데이터가 의도적으로 오차를 준 테스트셋이라는 점과 일치). 371,643점을 64ms에 처리. 대용량 실데이터에서는 최대 24,235,712점짜리 청크를 3.7초에 처리.

## 3. [높음] 전체 솔루션 클린 재빌드 및 Test/* 산출물 갱신 — ✅ 완료

- **문제**: `bin/TestTaskManager.exe`가 4월 25일 빌드 그대로였고, 2026-07-10에 소스에서 고친 상대경로(`../data`)가 산출물에 전혀 반영되지 않고 있었습니다.
- **원인 및 해결**: Google Test NuGet 패키지(`Microsoft.googletest.v140...`)가 `PlatformToolset`이 v140/v141/v142/v143일 때만 활성화되도록 조건이 걸려 있어, 이 환경에 설치된 v145 툴셋에서는 자동으로 비활성화되어 `gtest/gtest.h`를 찾지 못했습니다. 패키지가 제공하는 `Force-Enable-Microsoft-googletest-v140-windesktop-msvcstl-static-rt-dyn` 강제 활성화 플래그를 MSBuild 속성으로 넘겨 우회했습니다: `/p:Force-Enable-Microsoft-googletest-v140-windesktop-msvcstl-static-rt-dyn=true`.
- **검증**: Release/Debug 전체 솔루션 재빌드 성공(오류 0건). `TestTaskManager.exe`, `TestLasImport.exe`, `TestGltfImport.exe`, `TestLaslibWrapper.exe` **4개 전부 재실행 → 20/20 PASSED**. `TestTaskManager.exe` 실행 로그에서 더 이상 `F:\repository\DPApp\...` 경로가 아닌 `../data\...` 상대경로가 사용됨을 직접 확인.

## 4. [높음] `TestTaskManager`의 `ImportPointclouds2` 방어 코드 보강 — ✅ 완료

- **파일**: [Test/TestTaskManager/test.cpp](Test/TestTaskManager/test.cpp)
- **문제**: `pointclouds2::loadPointclouds2()`의 반환값(`retval`, `retval2`)을 확인하지 않고 곧바로 `pointbuffer`/`pointbuffer2`를 인덱싱하는 구조였습니다. 로드가 실패하면 범위를 벗어난 접근(크래시 위험)이 발생할 수 있었고, 심지어 루프의 실제 비교 결과(`equalCheck`)는 검사하지 않고 엉뚱하게 `retval`만 `EXPECT_TRUE`하고 있었습니다.
- **수정**: `ASSERT_TRUE(retval)`, `ASSERT_TRUE(retval2)`, `ASSERT_GE(...)` 로 로드 성공 및 버퍼 크기를 인덱싱 전에 검증하도록 추가. 마지막 검증도 `EXPECT_TRUE(retval)` → `EXPECT_TRUE(equalCheck)`로 수정(원래 의도대로 좌표 비교 결과를 실제로 검증).
- **검증**: §3의 재빌드로 컴파일 확인, 테스트 통과.

## 5. [중간] 분산 ICP 미연결 코드(`handleIcpResult`) 처리 — ✅ 완료 (아키텍처 결정 포함)

- **파일**: [MasterApp/MasterApplication.cpp](MasterApp/MasterApplication.cpp), [MasterApp/MasterApplication.h](MasterApp/MasterApplication.h), [MasterApp/MasterApplication_ICP.cpp](MasterApp/MasterApplication_ICP.cpp)
- **발견**: `handleTaskResult()`의 결과 분배 분기에 `result_type == 3`(ICP, `SlaveApplication::sendIcpResult()`가 보내는 타입) 케이스가 아예 없어서, ICP 결과가 오면 `else` 분기로 떨어져 일반 `ProcessingResult`로 잘못 역직렬화를 시도하는 버그였습니다. `handleIcpResult()`는 실제로 어디서도 호출되지 않는 완전한 미사용 함수였습니다. `MasterApplication.cpp` 전체에서 `ICP_FINE_ALIGNMENT`를 실제로 Slave에 분배하는 코드도 찾지 못했습니다(스모크테스트용 컴파일 확인 코드 한 줄이 유일).
- **아키텍처 결정**: 분산 ICP(Master→Slave로 미세정합 청크를 실제로 분배)는 이번 세션 범위를 벗어나는 별도 기능으로 판단하고 **구현하지 않기로 결정**했습니다. 대신 (a) 디스패처의 오분류 버그를 수정하고, (b) `handleIcpResult()`의 TODO를 실제로 구현(`task_id` → `job_id` 매핑 테이블 `icp_task_to_job_` 추가, 매칭되는 Job의 `chunkResults`/`completedChunks`/`failedChunks` 갱신)했습니다. 다만 현재 아무 코드도 이 경로로 태스크를 보내지 않으므로 **이 함수는 여전히 실행 중 도달하지 않는 상태**이며, 코드에 명확한 주석으로 이를 남겼습니다. **REST API(`POST /api/icp/start`)의 Master 단독 동기 처리가 현재 유일하게 지원되는 ICP 정합 경로**라는 점을 공식화했습니다.
- **검증**: 수정 후 REST ICP(RMSE 0.0134973)와 `bimpc` 거리계산 둘 다 기존과 동일하게 정상 동작 — 회귀 없음.

## 6. [중간] `config/*.ini` 처리 방침 확정 — ✅ 완료

- **결정**: `config/master.ini`/`config/slave.ini` 전체를 실제로 파싱하는 기능을 새로 만들지는 않기로 결정(범위 대비 효과가 낮음). 대신 리포트가 구체적으로 지적한 **`processing_threads`가 상수로 고정되어 있던 문제만 실제로 고쳤습니다.**
- **파일**: [SlaveApp/SlaveApplication.cpp](SlaveApp/SlaveApplication.cpp), [SlaveAgent/agent.cpp](SlaveAgent/agent.cpp)
- **수정 내용**:
  - `processing_threads_`를 `const size_t`(항상 1) → `size_t`(가변)로 변경, `SlaveApp.exe -t <개수>` CLI 옵션 추가.
  - **추가 발견**: Master 콘솔의 `start-slaves <threads>` 명령 → REST(`/api/slaves/start`의 `threads` 필드) → `SlaveAgent`까지는 스레드 수가 이미 전달되고 있었지만, `SlaveAgent/agent.cpp`의 `handleStartSlave()`가 JSON의 `"threads"` 필드를 아예 파싱하지 않고, `startSlave()`도 그 값을 SlaveApp.exe 커맨드라인에 전혀 반영하지 않아 **중간에서 값이 버려지고 있었습니다.** `-t` 옵션 신설과 함께 이 배선도 끝까지 연결했습니다.
- **검증**: `SlaveApp.exe --help`에 `-t` 옵션 표시 확인. `SlaveApp.exe -t 4` 실행 시 로그에 `"Processing threads: 4"` 정상 출력 확인.

## 7. [낮음] `README.md`/`CMakeLists.txt`/`Makefile` 현행화 — ✅ 완료

- **문제**: 셋 다 현재 존재하지 않는 `src/master/`, `src/slave/`, `src/common/` 구조를 가정하고 있어 `cmake`/`make all`을 실제로 실행하면 빈 바이너리가 나오거나 그냥 실패하는 상태였습니다.
- **결정**: CMake/Makefile을 억지로 현재 구조에 맞춰 재작성하는 대신(실제 동작을 담보하기 어려움), **명확한 안내 메시지를 내고 즉시 실패**하도록 정리했습니다(`message(FATAL_ERROR ...)`, `@exit 1`). `README.md`는 실제 폴더 구조·빌드 방법(`DPApp.sln`)·아키텍처 요약으로 전면 재작성하고 `doc/handover/`를 가리키도록 했습니다.

## 8. [낮음] 레거시 `.docx` 가이드의 bimpc/ICP 서술 점검 — ✅ 완료

3개 문서를 검토했습니다(`pandoc`/`python-docx`가 이 환경에 없어 PowerShell로 docx 내부 XML에서 텍스트를 직접 추출):

| 문서 | 결과 |
|---|---|
| `분산처리 시스템 사용 가이드.docx` | bimpc가 REST API 미지원이라고 정확히 서술(사실과 일치). 문제 없음. |
| `Add a new task and how to use.docx` | bimpc 명령의 실제 동작(청크 분할→분산 전송→거리 결과 수집)을 코드 그대로 정확히 설명. 문제 없음. |
| `DPApp_ICP_Technical_Documentation.docx` | ⚠️ "Slave Nodes: Execute ICP computations on assigned chunks"라고 서술 — §5에서 확인했듯 **현재 구현과 불일치**(Master가 ICP 전체를 단독 처리). 이 문서는 기존 추적 댓글(`comments.xml`)이 있는 정식 기술문서라 자동으로 내용을 수정하지 않았습니다. **문서 소유자가 직접 정정하는 것을 권장합니다.** |

## 9. [낮음, 순서상 마지막] 3.4GB 실데이터(`samsung.las`) 대규모 분산 테스트 — ✅ 완료, 강력한 검증 결과

- **실행**: `samsung.las`(3.4GB) + `samsung.glb`(171MB)를 Master 1대 + Slave 1대(스레드 1개)로 처리.
- **예상 밖의 발견**: `samsung.glb`가 내부적으로 **1,294개의 개별 메시 노드**(배관, 장비 등 개별 부재)를 포함하고 있어서, `bimpc`가 노드 하나당 청크 하나씩 **총 1,294개의 태스크**를 생성했습니다. 즉 이 데이터에서는 이미 사실상 "분산 가능한" 구조였습니다 (Slave를 여러 대 붙이면 실제로 나뉘어 처리됨).
- **결과**:
  - **1,294/1,294 태스크 100% 성공**, 실패 0건, 예외 0건 (§1, §2에서 고친 버그가 재발하지 않음을 실측으로 재확인)
  - 총 **125,429,139개 포인트** 처리 완료 (전체 데이터셋 커버)
  - 최대 단일 청크: **24,235,712포인트**를 3.7초에 처리 (KD-Tree 성능 확인)
  - 총 소요시간: 약 25분 26초 (Slave 1대, 스레드 1개 기준) — 대부분의 청크가 수 ms~수 초 내 처리되는 것으로 보아, 총 시간의 상당 부분이 태스크당 네트워크/디스패치 오버헤드로 추정됨(1,294개 태스크 × 평균 약 1.2초/태스크). §6에서 고친 `-t`/`start-slaves <threads>` 옵션으로 다중 스레드·다중 Slave를 활용하면 단축 가능성이 있으나 이번 세션에서는 추가 측정을 하지 않았습니다.
- **테스트 도구 관련 참고**: 이 테스트를 위해 작성한 PowerShell 로그 수집 스크립트에서, 대용량 처리 중 초당 여러 번 발생하는 표준출력 이벤트와 진행상황 폴링용 `Get-Content` 호출이 같은 로그 파일에 동시 접근하면서 "process cannot access the file" 오류가 대량 발생해 Master 쪽 로그 일부가 유실되었습니다. **이는 테스트 스크립트 자체의 결함이며 프로그램의 결함이 아닙니다.** Slave 쪽 로그는 별도 파일이라 온전히 보존되어 위 결과를 확인할 수 있었습니다.

---

## 종합 결과

| 항목 | 상태 |
|---|---|
| bimpc 파이프라인(직렬화+거리계산) | ✅ 실제 대규모 데이터(1억 2천5백만 점)에서 100% 성공 확인 |
| 단위테스트 | ✅ 20/20 PASSED, 최신 소스로 재빌드된 산출물 기준 |
| 분산 ICP | 📋 아키텍처 결정: 미구현 상태 유지, 관련 dead code는 정리·문서화 |
| `processing_threads` 설정 | ✅ 실제로 동작하도록 배선 완료 |
| 빌드 스크립트/README | ✅ 현재 구조와 일치하도록 정리 |
| 레거시 문서 | ✅ 점검 완료, 1건 정정 필요 사항 식별(문서 소유자 조치 필요) |

## 부록. 변경된 파일 목록

| 파일 | 변경 내용 |
|---|---|
| `src/NetworkManager.cpp` | BimPcChunk/MeshChunk 역직렬화 float→double 수정 (§1) |
| `include/TaskManager.h` | `processBimPc()` 실제 거리계산 로직 구현 (§2) |
| `Test/TestTaskManager/test.cpp` | `ImportPointclouds2` 방어 코드 보강 (§4) |
| `MasterApp/MasterApplication.cpp` | `handleTaskResult()` result_type==3 라우팅 수정 (§5) |
| `MasterApp/MasterApplication.h` | `icp_task_to_job_` 매핑 테이블 추가 (§5) |
| `MasterApp/MasterApplication_ICP.cpp` | `handleIcpResult()` 구현 (§5) |
| `SlaveApp/SlaveApplication.cpp` | `-t/--threads` CLI 옵션 추가, `processing_threads_` 가변화 (§6) |
| `SlaveAgent/agent.cpp` | `threads` 파라미터를 SlaveApp.exe 커맨드라인까지 배선 (§6) |
| `README.md`, `CMakeLists.txt`, `Makefile` | 현재 구조에 맞게 재작성/명확한 미지원 안내로 정리 (§7) |
| `doc/handover/05_사용자_매뉴얼.md` | ICP/bimpc 재정정, .ini 안내 갱신 (기존 문서 보강) |

## 부록. 빌드 환경 참고

이번 세션에서 4개 `Test/*` 프로젝트를 포함한 **전체 솔루션 재빌드에 처음으로 성공**했습니다. 핵심은 Google Test NuGet 패키지의 `Force-Enable-Microsoft-googletest-v140-windesktop-msvcstl-static-rt-dyn` 속성을 `true`로 강제 지정해 v145 툴셋에서도 v140용 정적 라이브러리 include/lib 경로가 적용되도록 우회한 것입니다:

```
msbuild DPApp.sln /p:Configuration=Release /p:Platform=x64 /p:PlatformToolset=v145 ^
  "/p:Force-Enable-Microsoft-googletest-v140-windesktop-msvcstl-static-rt-dyn=true"
```

v140 빌드 산출물(정적 라이브러리)을 v145 컴파일러로 링크하는 것이므로, 원래 지정된 v143 툴셋이 설치된 환경에서 한 번 더 재확인하는 것을 권장합니다.
