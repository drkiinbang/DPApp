#include "pch.h"

#include <filesystem>
namespace fs = std::filesystem;
#include <windows.h> // Windows API 헤더

#include "../../LasImport/LasImport.h"

TEST(TestCaseName, LasLoadTest) {
	std::string lasPath = "../data/test/las_test/test.las";
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

	std::vector<pc::PointCloudChunk> chunks;
	const uint32_t max_points_per_chunk = 1000000;
	auto retval = loadLasFileStreaming(absolutePath.string(), chunks, max_points_per_chunk);
	for (auto& c : chunks) {
		std::cout << c.min_x << " " << c.min_y << " " << c.min_z << " " << c.max_x << " " << c.max_y << " " << c.max_z << " " << c.getNumPts() << " points\n";
	}

	EXPECT_TRUE(retval);
}