# 2026-07-12 (3차) -- offset 벤치마크, 부재별/통합 정합 결과 저장, 실제 다중 컴퓨터 Slave 가이드

## 1. 벤치마크: widen+offset vs widen-only

`runIcp()`/`processBimPc()` 맨 앞의 "float→double 승격 + offset 덧셈" 단계가 성능에 영향을 주는지 측정(`/scratchpad/bench_offset.cpp`, Release 최적화, best-of-N 측정, dead-code elimination 방지용 checksum 적용).

| 규모 | 승격만 | 승격+offset | 차이 |
|---|---|---|---|
| 150,000점 (전형적 부재 청크) | 0.412ms | 0.454ms | +0.041ms (+10%) |
| 100,000,000점 (samsung.las 규모) | 377.0ms | 381.4ms | +4.4ms (+1.2%) |

절대 시간 차이(전형적 청크 기준 0.04ms)는 실제 ICP 처리 시간(수백 ms~초 단위)의 1,000분의 1 수준으로 무시할 수 있음을 확인. 상세 표는 `doc/handover/07_부재별_정합_통합_방법론.md` §5 참고.

## 2. 부재별(BIM element) 정합 결과 저장 + 전체 정합 통합 산출

- `include/IcpTypes.h`: `IcpElementAlignment` 구조체 추가(부재별 chunk_id, 이름/Revit ID, 포인트 수, RMSE, 수렴 여부, 변환행렬), `IcpJob::elementResults`/`alignmentOutputPath` 필드 추가(copy constructor 갱신 포함).
- `MasterApp/MasterApplication.h`/`.cpp`: `loadIcpElementChunks()`에 `outElementInfo` out-parameter 추가 -- 어떤 chunk_id가 어떤 BIM 부재(이름/Revit ID)에 대응하는지 Master가 계속 추적할 수 있게 함.
- `MasterApp/MasterApplication_ICP.cpp`: Phase 6(Aggregate) 직후 부재별 결과(`job->elementResults`)를 구성하고, Phase 7(LAS 저장) 직후 `<output>_alignment.json` 파일로 부재별 결과 + 통합 결과를 저장(`saveAlignmentResults()`). REST `GET /api/icp/jobs/{id}/result` 응답에 `alignment_output_path`/`element_count` 필드 추가.
- 검증 중 발견: 테스트에 쓴 샘플 GLTF 데이터(`icp_test`, `10glb`, `samsung_test/glb`)들이 `mesh.name`/`extras.id`를 채우지 않고 있어(빈 문자열/-1 또는 순차 기본값), 부재 이름/Revit ID가 비어서 저장됨 -- 코드 자체는 GLTF의 해당 필드를 정확히 그대로 읽어오도록 연결되어 있으므로, 실제로 Element ID를 보존해 내보낸 BIM 데이터에서는 정상적으로 채워짐(코드 버그 아님, 테스트 데이터의 한계). `chunk_id`는 항상 채워지므로 부재 추적 자체는 항상 가능.

## 3. 수학적 방법론 문서화

`doc/handover/07_부재별_정합_통합_방법론.md` 신규 작성 -- 회전 평균화(가중 chordal L2 mean, SVD를 통한 SO(3) 투영, 반사 방지), 이동 평균화(가중 산술평균), 가중치 선택 근거(포인트 개수), 그리고 **"부재별 전체 변환을 나중에 평균"하는 것과 "국소 보정치를 먼저 평균한 뒤 coarse와 1회 합성"하는 것이 회전에서는 수학적으로 동일하지만 이동에서는 다르다는 것**을 수식으로 증명하며 상세히 기술.

## 4. 테스트 매뉴얼 갱신

- `doc/handover/06_상세_테스트_절차서_비전문가용.md`에 §13(실제 여러 대의 컴퓨터에 Slave 설치하기) 신규 추가 -- 준비물(Slave 컴퓨터에는 `SlaveApp.exe` 파일 하나만 필요, `dumpbin /dependents`로 확인한 실제 의존성 목록 근거로 설명), Master IP 확인, 방화벽 인바운드 규칙 추가(`netsh advfirewall`), Slave의 `-s <IP> -p <port>` 접속 방법, 문제 해결 표 포함.
- 기존 §12(종료 절차) 하위 번호 오타(10.1/10.2 → 12.1/12.2) 수정.
- 부록 A 기록표에 §13 항목 추가, 마무리 문구에 08번 문서 링크 추가.

## 검증

- 전체 솔루션 Release 재빌드: 오류 0건.
- 기존 4개 단위테스트(총 20개) 전부 통과, 회귀 없음.
- 소규모 실측 시나리오(icp_test 단일 부재) + `gensynth` 합성 데이터로 `_alignment.json` 생성 확인 -- `elements[]`/`combined_transform`/`coarse_transform` 필드 정상 기록.
- 3-Slave 분산 테스트로 여러 부재의 결과가 `chunk_id`별로 정확히 구분되어 저장됨을 확인.
