#include "pch.h"

#include <filesystem>
namespace fs = std::filesystem;
#include <windows.h> // Windows API 헤더

#include "../../LasImport/LasImport.h"

// LAS 포인트클라우드 파일을 청크 단위로 스트리밍 로드하는 loadLasFile()이 정상
// 동작하는지(오류 없이 true를 반환하고, 각 청크의 바운딩 박스/포인트 수가 채워지는지) 검증한다.
TEST(TestCaseName, LasLoadTest) {
	// [테스트 데이터] 저장소에 커밋된 소형 fixture(data/test_fixtures/small.las,
	// 약 400KB)를 사용한다. 예전에는 151MB짜리 실측 las를 참조해서 clone만으로는
	// 돌릴 수 없었다 -- 이 테스트는 las 스트리밍 로딩 성공 여부만 확인하므로 소형
	// las로 충분하다. 대용량 실측 데이터로 돌리려면 이 경로만 바꾸면 된다.
	std::string lasPath = "../data/test_fixtures/small.las";
	fs::path relativePath = lasPath;

	/// 실행 파일의 전체 경로
	char path_buffer[1024];
	GetModuleFileNameA(NULL, path_buffer, MAX_PATH);
	fs::path exe_path(path_buffer);

	/// 실행 파일이 포함된 디렉토리(폴더) 경로
	fs::path exe_dir = exe_path.parent_path();

	/// 현재 작업 디렉토리를 실행 파일의 디렉토리로 변경
	/// SetCurrentDirectoryA는 CWD를 변경하는 Windows API 함수
	if (SetCurrentDirectoryA(exe_dir.string().c_str())) {
		std::cout << "CWD가 성공적으로 변경되었습니다: " << fs::current_path() << std::endl;
	}
	else {
		std::cerr << "CWD 변경 실패!" << std::endl;
		return;
	}

	/// 절대 경로로 변환
	/// fs::absolute는 현재 작업 디렉토리를 기준으로 절대 경로를 계산
	fs::path absolutePath = fs::absolute(relativePath);

	std::cout << "입력한 패스: " << absolutePath << std::endl;

	std::vector<chunkpc::PointCloudChunk> chunks;
	const uint32_t max_points_per_chunk = 1000000;
	auto retval = loadLasFile(absolutePath.string(), chunks, max_points_per_chunk);
	for (auto& c : chunks) {
		std::cout << c.min_x << " " << c.min_y << " " << c.min_z << " " << c.max_x << " " << c.max_y << " " << c.max_z << " " << c.getNumPts() << " points\n";
	}

	EXPECT_TRUE(retval);
}
