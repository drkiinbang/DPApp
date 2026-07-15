# DPApp_II - BIM/Point Cloud Distributed Processing System

LAS 포인트클라우드와 BIM(GLTF/GLB) 모델을 ICP 알고리즘으로 정합(registration)하는 Windows/C++17 프로그램입니다. Master가 작업을 관리하고, 필요 시 여러 Slave에 작업을 분산 처리하는 구조입니다.

> 이 README는 빠른 개요만 다룹니다. 아키텍처, 빌드, 테스트, 사용법 전체는 **[doc/handover/](doc/handover/00_인수인계_시작하기.md)**의 인수인계 문서 세트를 참고하세요.

## 프로젝트 구조 (2026-07 기준)

```
DPApp_II/
├── MasterApp/        Master 프로그램 소스 (CLI, REST API, ICP 작업 관리)
├── SlaveApp/          Slave 프로그램 소스 (원격 작업 처리)
├── MasterLauncher/    Master 원격 실행 보조 프로그램
├── SlaveAgent/        Slave 원격 실행/관리 에이전트
├── BimImport/          GLTF/GLB 파일 로딩 DLL 소스
├── LasImport/          LAS/LAZ 파일 로딩 DLL 소스
├── include/            공통 헤더 (TaskManager, ICP 알고리즘, 네트워크 등)
├── src/                공통 구현 (NetworkManager.cpp 등)
├── Test/                단위 테스트 (Google Test 기반, 4개 프로젝트)
├── config/              설정 파일 예시 (현재 코드에서 읽지 않는 미사용 상태 — doc/handover/05 참고)
├── data/                테스트/예제 데이터
├── tools/               보조 도구 (BIM+포인트클라우드 뷰어 — tools/README.md 참고)
├── vendors/             외부 라이브러리 (Eigen3, LAStools, gltf)
├── bin/                 빌드 결과물(.exe/.dll)이 모이는 곳
└── DPApp.sln            Visual Studio 솔루션 파일 (유일하게 지원되는 빌드 방법)
```

## 빌드

**Windows + Visual Studio 2019 이상만 공식적으로 지원합니다.**

1. `DPApp.sln`을 Visual Studio로 엽니다.
2. 빌드 구성(Debug/Release)과 플랫폼(x64)을 선택합니다.
3. 솔루션 다시 빌드(Rebuild Solution)를 실행합니다.
4. `bin/` 폴더에 `MasterApp.exe`, `SlaveApp.exe` 등이 생성됩니다.

자세한 절차와 문제 해결은 [doc/handover/03_빌드_가이드.md](doc/handover/03_빌드_가이드.md)를 참고하세요.

> ⚠️ `CMakeLists.txt`/`Makefile`은 현재 유지보수되지 않으며 사용을 권장하지 않습니다. 자세한 내용은 두 파일 안의 안내 메시지를 참고하세요.

## 실행

```cmd
cd bin
MasterApp.exe          :: Master 실행 (포트 8080, REST API 8081)
SlaveApp.exe            :: Slave 실행 (다른 콘솔 창에서, localhost로 자동 접속)
```

Master 콘솔에서 `help`를 입력하면 사용 가능한 명령어 목록을 볼 수 있습니다. ICP 정합(핵심 기능)은 콘솔 명령이 아니라 REST API(`POST /api/icp/start`)로 실행합니다 — 자세한 사용법은 [doc/handover/05_사용자_매뉴얼.md](doc/handover/05_사용자_매뉴얼.md)를 참고하세요.

## 시스템 아키텍처

- **Master**: ICP 정합 요청 처리, 작업 분할·배포·결과 수집, REST API 서버
- **Slave**: Master로부터 작업을 받아 처리 후 결과 반환 (거리계산 등 분산 가능한 작업용. ICP 정합 자체는 현재 Master가 단독으로 처리)
- **네트워크**: TCP 기반 Master-Slave 통신 (직접 구현)
- **포인트클라우드**: LAS/LAZ/XYZ/PTS 등 지원

## 더 알아보기

- [doc/handover/00_인수인계_시작하기.md](doc/handover/00_인수인계_시작하기.md) — 전체 문서 목록 및 읽는 순서
- [doc/handover/06_상세_테스트_절차서_비전문가용.md](doc/handover/06_상세_테스트_절차서_비전문가용.md) — 그대로 따라 할 수 있는 테스트 절차
- [tools/README.md](tools/README.md) — BIM+포인트클라우드 뷰어 사용법 (브라우저로 열기만 하면 됨, 합성 테스트 데이터 생성/저장 가능)
- [CHANGELOG_2026-07-10.md](CHANGELOG_2026-07-10.md), [CHANGELOG_2026-07-11.md](CHANGELOG_2026-07-11.md) — 최근 변경 이력
