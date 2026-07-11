# 2026-07-12 (2차) -- 청크 자체 float+offset 전환, 다중 Slave/합성 데이터 테스트 도구

## Context

같은 날 앞서 커밋한 float+offset 최적화(`CHANGELOG_2026-07-12_float-offset-rebase.md`)는 "Master가 들고 있는 벌크 배열만 float+offset, 개별 청크(IcpChunk/BimPcChunk)는 double·절대좌표"로 구현했다. 사용자가 이를 검토하며 "프로그램 전체 원칙은 청크로 이동되는 좌표 데이터 자체가 float이어야 하고, 계산은 double 정밀도를 쓰고, offset으로 원본 복원이 가능해야 한다"는 더 일관된 설계를 요청했다. 또한 ICP 상관점 탐색 메커니즘에 대한 질문, 다중 Slave 테스트 필요성, 합성 테스트 데이터 생성 필요성을 제기했다.

## 1. 청크 자체 float+offset 전환

- `IcpChunk`(`include/IcpTypes.h`)와 `BimPcChunk`(`include/PointCloudTypes.h`)에 `offsetX/Y/Z`(double) 필드를 추가하고, `sourcePoints/targetPoints/facePts/faceNormals`(IcpChunk) 및 `points`(BimPcChunk)의 타입을 `double` → `float`로 변경.
- `runIcp()`(`include/bimtree/IcpCore.hpp`)와 `processBimPc()`(`include/TaskManager.h`) 맨 앞에서 **딱 한 번** float→double 승격 + offset 덧셈으로 절대좌표 로컬 벡터를 만들고, 이후 알고리즘 본문은 전혀 수정하지 않음(상관점 탐색/SVD 변환추정/이상치 제거 로직 그대로).
- `include/bimtree/IcpSerialization.hpp`(IcpChunk 직렬화)와 `src/NetworkManager.cpp`(BimPcChunk 직렬화)를 청크 필드 타입 변경에 맞춰 갱신. `BimPcChunk::bim`(원본 메시)은 여전히 double·절대좌표를 유지하되, 실제로 네트워크로 나가는 바이트에 한해서만 offset을 적용(임시 복사본을 만들어 오프셋만큼 이동 후 직렬화, 역직렬화 후 되돌림) — `serializeMeshChunk`/`deserializeMeshChunk` 자체(다른 곳에서도 범용으로 쓰임)는 무수정.
- `MasterApp/MasterApplication_ICP.cpp`(processIcpJob)와 `MasterApp/MasterApplication.cpp`(loadIcpElementChunks, loadBimPcChunks)의 청크 생성 코드가 단순해짐 — 벌크 float+offset 배열에서 클리핑한 조각을 그대로(변환 없이) 청크 필드에 복사하고, `offsetX/Y/Z`만 채우면 됨. `icp::generatePseudoPointsFromMesh`/`generatePseudoPoints`(`include/bimtree/PseudoPointGenerator.hpp`)에 offset 매개변수를 다시 추가해 가상점도 같은 좌표계로 생성.

## 2. 다중 Slave 테스트 (Docker 불필요)

- 조사 결과 `SlaveApp.exe`는 자신의 리스닝 포트가 없는 순수 아웃바운드 TCP 클라이언트라, 같은 머신에서 여러 인스턴스를 동시에 실행해도 충돌이 없음을 확인. Docker/가상화 없이 `SlaveApp.exe`를 3개 띄워서 10개 부재 청크가 실제로 3개 Slave에 라운드로빈으로 분산되는 것을 검증(§10, 테스트 매뉴얼).

## 3. 합성 테스트 데이터 생성기 (`gensynth` CLI)

- 새 Master CLI 명령 `gensynth <bim_folder> <output.las> [num_elements] [grid_size] [noise_sigma] [shift_x/y/z] [rot_deg]` 추가(`MasterApp/MasterApplication_ICP.cpp`의 `runGenerateSyntheticPointCloud`, `MasterApp/MasterApplication_CLI.cpp`에 명령 연결).
- BIM 메시 표면에 기존 `icp::generatePseudoPointsFromMesh`로 조밀한 점을 뿌려 가상의 레이저스캔 포인트클라우드를 만들고, **선택된 부재들의 자체 중심(centroid)을 기준으로** 알려진 회전(Z축)+이동을 주입(절대좌표 원점 기준으로 회전하면 부재가 원점에서 멀 경우 지레팔 효과로 수 미터씩 어긋나는 버그를 잡음), 선택적으로 가우시안 노이즈 추가, `.las`로 저장. 주입한 변환의 절대좌표계 역변환(ICP가 복원해야 할 값)을 콘솔에 출력.
- 검증: 1개 부재로 생성한 합성 데이터에 ICP를 실행한 결과, `final_transform`이 주입한 정답과 노이즈 허용 범위 내에서 일치함을 확인(회전 성분 5자리까지 일치, 이동 성분 3~7mm 오차 — noise_sigma=3mm와 부합).
- 10개의 서로 떨어진 실제 부재를 한 번에 정합하려 하면 기본 `max_correspondence_distance`(1m)로는 일부 부재가 "상관점 부족"으로 실패하는 것을 확인 — 이는 2026-07-11 대규모 테스트에서 이미 문서화된 것과 같은 종류의 튜닝 특성이며 버그 아님(부재들이 실제 플랜트에서 서로 멀리 떨어져 있어 생기는 현상).

## 4. 테스트 매뉴얼 갱신

- `doc/handover/07_상세_테스트_절차서_비전문가용.md`에 §10(다중 Slave 분산 확인)과 §11(합성 데이터 기반 점진적 검증) 추가, 기존 §10(종료 절차)을 §12로 재배치, 결과 기록표·요약 문구 갱신.

## 검증

- 전체 솔루션 Release 재빌드: 오류 0건.
- 소규모 실측 시나리오(icp_test, 371,643점) REST ICP 재실행 — `final_rmse=0.0135172`(기존 0.0135168과 6번째 소수점 차이, 청크 레벨 float narrow 추가로 인한 예상된 미세 차이).
- `bimpc` CLI 거리계산 재검증 — `within=366458/outside=5185`(기존 366460/5183과 거의 동일, 5cm 임계값 경계의 예상된 미세 변동).
- `gensynth` + ICP 종단 검증 — 주입한 회전/이동을 노이즈 허용 범위 내에서 정확히 복원.
- 3-Slave 분산 처리 검증 — 10개 부재 청크가 3개 Slave에 실제로 분산 배정됨을 로그로 확인.
- 기존 4개 단위테스트(총 20개) 전부 통과, 회귀 없음.
