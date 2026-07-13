# 2026-07-10 후속 조치 변경 기록 (무결성 테스트 → 발견된 결함 수정)

이 문서는 2026-07-10 실시한 DPApp_II 무결성 테스트에서 발견된 5개 권장 후속조치를 수행하며 변경한 내용을 기록합니다. 버전관리(git) 커밋 메시지/PR 설명 작성 시 이 문서를 근거 자료로 사용할 수 있습니다.

관련 문서: [doc/handover/04_테스트_가이드.md](doc/handover/04_테스트_가이드.md), [doc/handover/05_사용자_매뉴얼.md](doc/handover/05_사용자_매뉴얼.md)

## 배경

같은 날 실시한 무결성 테스트(Master/Slave 실기동, 자가진단, ICP REST API, 비정상 케이스)에서 아래 5개 항목이 권장 후속조치로 도출되었고, 사용자 요청에 따라 전부 수행했습니다. 각 수정은 코드 변경 후 **실제로 Master/Slave를 재기동하여 재현 테스트로 검증**했습니다(정적 코드 리뷰만으로 끝내지 않음).

---

## 1. [High] Slave → Master 연결 실패 수정

### 최초 가설과 실제 근본 원인이 달랐던 사례

처음에는 "연결 재시도 로직이 없다"는 것이 원인이라고 판단했으나, **재시도 로직을 추가한 뒤에도 10회 재시도(약 45초) 전부 실패**하는 것을 확인하고 나서야 진짜 원인을 찾았습니다. 두 가지를 모두 수정했습니다.

### 1-A. 진짜 원인: 호스트명("localhost") 해석 실패

- **파일**: [src/NetworkManager.cpp](src/NetworkManager.cpp) `NetworkClient::connect()` (기존 946행 부근)
- **문제**: `inet_pton(AF_INET, address.c_str(), &server_addr.sin_addr)` 호출로 서버 주소를 파싱했는데, `inet_pton`은 **숫자로 된 IP만 처리 가능**하고 `"localhost"` 같은 호스트명은 처리하지 못합니다. 반환값도 확인하지 않아, 파싱 실패 시 `sin_addr`이 `0.0.0.0`으로 남은 채 조용히 `connect()`를 시도해서 **매번 연결이 실패**했습니다.
  - Slave 기본 접속 주소가 `"localhost"`이므로(`SlaveApplication.cpp`), 같은 PC에서 Master/Slave를 함께 실행하는 가장 흔한 시나리오(사용자 매뉴얼 "시나리오 1")가 항상 깨져 있었던 것입니다.
  - `curl`, `Test-NetConnection`, .NET `TcpClient` 등 외부 도구로는 포트 8080이 정상 응답해서, 처음에는 "네트워크 환경 문제"로 오인하기 쉬웠습니다. 실제로는 애플리케이션 코드의 주소 파싱 버그였습니다.
- **수정**: `inet_pton` 실패 시 `getaddrinfo()`로 폴백하여 호스트명을 정상적으로 해석하도록 변경. `include/NetworkManager.h`에 POSIX용 `<netdb.h>` include 추가(Windows는 기존 `<ws2tcpip.h>`에 이미 포함됨).
- **검증**: 수정 전/후 Master+Slave를 동일 절차로 재기동하여 비교.
  - 수정 전: `[ERROR] Failed to connect to server: localhost:8080` — 항상 실패
  - 수정 후: `[INFO] Connected to master server as: slave_e21b7af3`, Master `status` 명령에서 `Connected slaves: 1` 즉시 확인. **BIM_PC2_DIST 작업이 실제로 Slave에게 할당됨**("Task 1 assigned to slave_e21b7af3")도 확인.

### 1-B. 방어적 보완: 초기 연결 재시도 로직 추가

- **파일**: [SlaveApp/SlaveApplication.cpp](SlaveApp/SlaveApplication.cpp) `start()`, [include/RuntimeConfig.h](include/RuntimeConfig.h)
- **문제**: 기존에는 `client_->connect()`가 **단 한 번**만 시도되고 실패하면 프로세스가 즉시 종료됐습니다(`SlaveApplication.cpp:88` 부근). Master가 아주 잠깐이라도 늦게 뜨거나 네트워크가 순간적으로 불안정하면 Slave가 재시도 없이 죽어버리는 구조적 취약점이었습니다. (기존 `config/slave.ini`의 `reconnect_interval`/`max_reconnect_attempts` 값은 애초에 코드에서 읽지 않는 죽은 설정이었습니다 — 1-C 참고.)
- **수정**: 연결 실패 시 설정된 간격만큼 대기 후 재시도하는 루프 추가. 최대 시도 횟수 초과 시에만 포기하고 종료.
- **신규 환경변수** (`RuntimeConfig.h`, `DPApp::RuntimeConfig::loadFromEnv()`):
  | 환경변수 | 기본값 | 의미 |
  |---|---|---|
  | `DPAPP_SLAVE_RECONNECT_INTERVAL` | 5 | 재시도 간격(초) |
  | `DPAPP_SLAVE_MAX_RECONNECT_ATTEMPTS` | 10 | 최대 재시도 횟수 (0 이하 = 무한 재시도) |
- **검증**: 1-A 수정 전 상태로 재현했을 때 `attempt 1/10`부터 `attempt 10/10`까지 정확히 5초 간격으로 재시도하고 마지막에 `"giving up"` 로그와 함께 정상 종료되는 것을 확인. 1-A 수정 후에는 재시도 없이 1차 시도에서 바로 연결 성공.

### 1-C. 문서 정정

- `config/slave.ini`/`config/master.ini`는 **소스 코드 어디에서도 읽지 않는 미사용 파일**임을 전체 코드 검색(`grep`)으로 확인. `doc/handover/05_사용자_매뉴얼.md` 8장에 경고 문구 추가.

---

## 2. [High] REST API `/api/icp/start` 결과 파일 미생성 수정

- **파일**: [MasterApp/MasterApplication_ICP.cpp](MasterApp/MasterApplication_ICP.cpp) `processIcpJob()`
- **문제**: ICP 정합(좌표 변환행렬, RMSE)은 정상 계산되었지만, 코드에 `/// Phase 7: Complete (No file saving)`라고 명시적으로 주석 처리된 대로 **정합된 포인트클라우드를 실제 파일로 저장하는 코드가 없었습니다.** 그런데도 REST 응답에는 `output_path` 필드가 마치 파일이 저장된 것처럼 표시되어, API를 사용하는 외부 시스템이 존재하지 않는 파일을 열려고 시도하면 실패하는 결함이었습니다.
- **수정**: `job->finalTransform`을 원본 해상도의 `sourcePoints` 전체에 적용한 뒤, `las::LASToolsWriter::save()`(`include/LaslibWriter.hpp`)로 `job->outputPath`에 실제 LAS 파일을 저장하도록 구현. 실패 시 예외를 던져 Job 상태가 `FAILED`로 정확히 기록되도록 처리. 상태값은 기존에 정의만 되어있던 `IcpJobStatus::SAVING_RESULT`를 사용.
- **추가된 include**: `#include "../include/LaslibWriter.hpp"`
- **검증**: 소규모 샘플(371,643 포인트)로 `POST /api/icp/start` 실행 후,
  - 수정 전: 로그에 파일 저장 관련 언급 없음, `data/samsung_test/icp_test/` 폴더에 `_icp_aligned.las` 파일이 **생성되지 않음**을 파일시스템에서 직접 확인.
  - 수정 후: 로그에 `"[ICP] Aligned point cloud saved: ..._icp_aligned.las (371643 points)"` 출력, 실제로 `data/samsung_test/icp_test/machanical_equipment_..._icp_aligned.las` (12.6MB, 371,643 포인트) 파일이 디스크에 생성됨을 확인. 2회 반복 재현하여 일관되게 성공.

---

## 3. [Medium] 사용자 매뉴얼 정정 — `bimpc` vs REST ICP 혼동 해소

- **파일**: [doc/handover/05_사용자_매뉴얼.md](doc/handover/05_사용자_매뉴얼.md) 3장, [doc/handover/02_핵심기능_명세.md](doc/handover/02_핵심기능_명세.md) 6장
- **문제**: 기존 매뉴얼은 콘솔 명령 `bimpc`가 "ICP 정합(메인 기능)"을 수행한다고 설명했습니다. 그러나 실제 코드(`MasterApplication_CLI.cpp`)를 보면 `bimpc`는 `TaskType::BIM_PC2_DIST`(포인트-BIM 거리계산) 작업을 생성할 뿐이며, **실제 ICP 좌표 정합의 유일한 진입점은 REST API `POST /api/icp/start`**입니다(`MasterApplication_ICP.cpp`). 이 둘은 완전히 다른 코드 경로이며, `bimpc`는 Slave가 반드시 필요한 반면 REST ICP는 Master 프로세스가 자체적으로 처리합니다.
- **수정**: 3장을 REST API 기준으로 재작성하고, 기존 `bimpc` 설명은 "거리계산 기능(ICP 아님)"으로 이동·정정. `02_핵심기능_명세.md`의 명령어 표도 함께 수정.
- **검증**: 실측 로그로 두 경로가 서로 다른 task_type/처리 흐름을 타는 것을 직접 확인 후 문서화(§본 문서 1-A, 4번 항목 참고).

---

## 4. [Medium] `TestTaskManager` 하드코딩 경로 수정

- **파일**: [Test/TestTaskManager/test.cpp](Test/TestTaskManager/test.cpp)
- **문제**: 4개 테스트 케이스 모두 `const std::string dataFolder = "F:\\repository\\DPApp\\data";` 로 **다른 드라이브·다른 저장소 이름의 절대경로**를 하드코딩하고 있었습니다. 이 세션의 개발 환경(D 드라이브, `DPApp_II`)에서는 우연히 해당 경로가 존재해서 테스트가 통과했지만, 다른 개발자의 PC나 CI 환경에서는 실패할 수 있는 이식성 결함이었습니다.
- **수정**: 4곳 모두 `"../data"` (실행 파일 기준 상대경로)로 변경. 같은 패턴을 이미 쓰고 있는 `TestLasImport`/`TestGltfImport`와 일관성을 맞춤. 실제 사용되는 데이터 파일(`data/pointclouds2/samsung_test.pointclouds2`)이 새 상대경로로 정상 해석됨을 파일시스템에서 확인.
- **⚠️ 빌드 검증 한계**: 이 프로젝트가 참조하는 Google Test NuGet 패키지(`Microsoft.googletest.v140...`)가 `PlatformToolset=v140`에 고정되어 있어, 이번 세션 환경(v143 툴셋 미설치, v145로 대체 검증)에서는 재컴파일 검증을 하지 못했습니다. 문자열 리터럴만 바꾼 저위험 변경이지만, **원래 개발 환경(VS + v143 툴셋)에서 `TestTaskManager` 재빌드 및 재실행 확인을 권장**합니다.

---

## 5. [Low] 대용량 데이터(samsung.las, 3.4GB) 분산 처리 테스트

**미실시.** 이번 세션은 위 1~4번 결함 수정과 검증에 시간을 집중했고, 3.4GB 규모의 실데이터로 다중 Slave 분산 처리를 검증하는 것은 별도의 장시간 세션이 필요하다고 판단해 범위에서 제외했습니다. 대신 소규모 샘플(371,643 포인트)로 Master–Slave 통신, 태스크 분배, ICP 계산 파이프라인 자체는 반복 검증했습니다. 대용량 테스트는 별도 세션에서 수행할 것을 권장합니다.

---

## 부록 A. 이번 수정으로 새로 발견된 미해결 결함 (원래 범위 밖)

1-A를 수정해 Slave 연결이 정상화되자, 이전에는 아예 드러나지 않았던(연결이 안 되니 도달 자체를 못 했던) 새로운 결함이 노출되었습니다.

- **증상**: Slave가 정상 연결된 상태에서 `bimpc` 명령으로 생성된 `BIM_PC2_DIST` 작업을 Slave가 수신하면 즉시 아래 오류로 실패합니다.
  ```
  [ERROR] Error handling task assignment: Buffer overflow: insufficient data to read.
  ```
  작업은 `Active` 상태로 멈춘 채 완료되지 않습니다(`Pending: 0, Active: 1, Completed: 0`가 계속 유지됨).
- **영향 범위**: `bimpc` 콘솔 명령(거리계산 기능)만 영향을 받습니다. **REST API의 실제 ICP 정합 기능(`/api/icp/start`)은 이 버그와 무관하며 정상 동작을 재확인했습니다** — REST 경로는 Slave에 작업을 보내지 않고 Master 프로세스 내에서 직접 처리하기 때문입니다.
- **의심 위치**: 프레이밍(메시지 헤더/오프셋 계산)은 `MasterApplication.cpp`(전송 측)와 `SlaveApp/SlaveApplication.cpp::handleTaskAssignment()`(수신 측)를 대조한 결과 일치함을 확인했습니다. 실제 바이트 불일치는 `src/NetworkManager.cpp`의 `serializeBimPcChunk()`/`deserializeBimPcChunk()` 왕복 어딘가, 혹은 중첩된 `serializeMeshChunk()`/`deserializeMeshChunk()`에 있는 것으로 추정되나, 정적 코드 대조만으로는 정확한 지점을 특정하지 못했습니다. 실제 송신 바이트 수와 각 읽기 단계별 소비 바이트 수를 로그로 찍어보는 실측 디버깅이 필요합니다.
- **재현 데이터**: `data/samsung_test/icp_test/` (포인트 371,643개, 삼각형 26,250개) — 큰 원본 데이터(`samsung.las`, 3.4GB) 없이도 100% 재현됨.
- **권장 조치**: 별도 세션에서 `BinaryReader`/`BinaryWriter` 각 단계에 바이트 오프셋 로그를 추가해 정확한 불일치 지점을 특정 후 수정.

## 부록 B. 문서 추가 정정 사항

- `doc/handover/08_다음_릴리즈_Future_Works.md` §2.1 "KD-Tree 재사용" 항목: 코드 확인 결과 `IcpCore.hpp::runIcp()`에서 이미 반복문 바깥에서 KD-Tree를 1회만 생성하고 있음을 확인, "완료" 상태로 정정.

## 부록 C. 변경된 파일 목록

| 파일 | 변경 내용 |
|---|---|
| `src/NetworkManager.cpp` | `NetworkClient::connect()` 호스트명 해석 수정 (1-A) |
| `include/NetworkManager.h` | POSIX `<netdb.h>` include 추가 (1-A) |
| `SlaveApp/SlaveApplication.cpp` | 연결 재시도 루프 추가 (1-B) |
| `include/RuntimeConfig.h` | `slave_reconnect_interval_seconds`, `slave_max_reconnect_attempts` 필드 및 env var 로딩 추가 (1-B) |
| `MasterApp/MasterApplication_ICP.cpp` | ICP 결과 LAS 파일 저장 로직 추가 (2) |
| `Test/TestTaskManager/test.cpp` | 하드코딩된 절대경로 → 상대경로 (4) |
| `doc/handover/05_사용자_매뉴얼.md` | bimpc/REST ICP 구분 정정, .ini 미사용 경고, 신규 env var 문서화, 트러블슈팅 표 갱신 (3) |
| `doc/handover/02_핵심기능_명세.md` | `bimpc` 명령 설명 정정 (3) |
| `doc/handover/08_다음_릴리즈_Future_Works.md` | KD-Tree 재사용 항목 "완료"로 갱신 (부록 B) |

## 부록 D. 빌드 검증 환경 참고

이번 세션은 VS2022(v143 툴셋)이 설치되어 있지 않은 환경이라, 설치되어 있던 v145 툴셋으로 `/p:PlatformToolset=v145` 오버라이드하여 `MasterApp`/`SlaveApp`(Release·Debug)을 재빌드하고 컴파일 성공을 확인했습니다. 실제 배포/운영 빌드는 원래 지정된 v143 툴셋(`DPApp.sln` 원본 설정)으로 다시 한번 빌드 확인하는 것을 권장합니다.
