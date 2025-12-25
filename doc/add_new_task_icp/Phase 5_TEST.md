REST API 테스트
Master 앱을 실행한 후 다음 테스트를 진행합니다.

1️⃣ Master 앱 실행
bash# MasterApp 실행 (daemon 모드 권장)
MasterApp.exe -d
API 포트 기본값: 8081 (다르면 알려주세요)

2️⃣ 테스트 명령어
PowerShell 또는 CMD에서 실행:
테스트 A: ICP 작업 시작
powershellcurl -X POST http://localhost:8081/api/icp/start -H "Content-Type: application/json" -d "{\"las_file\": \"F:/repository/DPApp/data/samsung_test/samsung_las/samsung_sub_LB.las\", \"bim_folder\": \"F:/repository/DPApp/data/samsung_test/glb\"}"
테스트 B: 모든 ICP 작업 목록
powershellcurl http://localhost:8081/api/icp/jobs
테스트 C: 특정 작업 상태 조회 (job_id는 A 결과에서 복사)
powershellcurl http://localhost:8081/api/icp/jobs/icp_20241225_XXXXXX_XXX
테스트 D: 작업 결과 조회
powershellcurl http://localhost:8081/api/icp/jobs/icp_20241225_XXXXXX_XXX/result
테스트 E: 작업 통계
powershellcurl http://localhost:8081/api/icp/jobs/icp_20241225_XXXXXX_XXX/stats
테스트 F: 작업 취소
powershellcurl -X POST http://localhost:8081/api/icp/jobs/icp_20241225_XXXXXX_XXX/cancel

예상 결과 (테스트 A)
json{
  "success": true,
  "job_id": "icp_20241225_153045_123",
  "status": "PENDING",
  "message": "ICP job started"
}
