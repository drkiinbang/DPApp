# DPApp - Distributed Point Cloud Processing System

분산 포인트클라우드 처리 시스템

## 프로젝트 구조

```
DPApp/
├── src/                    # 소스 코드
│   ├── master/            # Master 노드 관련 코드
│   ├── slave/             # Slave 노드 관련 코드
│   ├── common/            # 공통 모듈
│   ├── network/           # 네트워크 통신 모듈
│   └── pointcloud/        # 포인트클라우드 처리 모듈
├── include/               # 헤더 파일
├── bin/                   # 실행 파일
├── data/                  # 테스트 데이터
├── config/                # 설정 파일
└── logs/                  # 로그 파일
```

## 시스템 아키텍처

- **Master**: 작업 분할, 배포, 결과 수집 및 병합
- **Slave**: Master로부터 작업을 받아 처리 후 결과 반환
- **네트워크**: TCP/UDP 기반 Master-Slave 통신
- **포인트클라우드**: LAS/PLY/XYZ 등 다양한 포맷 지원

## 빌드 및 실행

```bash
# 빌드
make all

# Master 실행
./bin/master

# Slave 실행
./bin/slave
```