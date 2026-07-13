#include "pch.h"

#include <filesystem>
namespace fs = std::filesystem;
#include <windows.h> // Windows API 헤더

#include "../../BimImport/BimImport.h"

// GLTF/GLB 파일에 저장된 extras("id" 등) 메타데이터가 정상적으로 읽히는지 검증한다
// (BimImport::testGltfExtraId -> chunkbim::GltfMesh::verifyMeshIdsInFile 경로).
TEST(TestCaseName, VerifyExtId) {
	// [테스트 데이터] 저장소에 커밋된 소형 fixture(data/test_fixtures/)를 사용한다.
	// 예전에는 165MB짜리 실측 glb(ext_sample2)를 참조해서 clone만으로는 테스트를
	// 돌릴 수 없었다 -- 이 테스트는 glb 로딩 성공 여부만 확인하므로 355KB짜리 소형
	// glb로 충분하다. 대용량 실측 데이터로 돌리고 싶으면 이 경로만 바꾸면 된다.
	std::string glbPath = "../data/test_fixtures/glb/small.glb";
	fs::path relativePath = glbPath;

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

	auto retval = testGltfExtraId(absolutePath.string());
	EXPECT_TRUE(retval);
}

// 폴더 안의 모든 GLTF/GLB 파일을 읽어 MeshChunk 목록으로 변환하는 loadGltf()가
// 정상적으로 동작하는지(오류 없이 true를 반환하는지) 검증한다.
TEST(TestCaseName, TestName) {
	// [테스트 데이터] 소형 fixture 폴더(glb 1개) -- loadGltf가 폴더 내 모든 glb를
	// 읽으므로 소형 glb 하나만 있어도 로딩 성공을 검증할 수 있다.
	std::string bimFolder = "../data/test_fixtures/glb";
	fs::path relativePath = bimFolder;

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

	std::cout << "입력한 데이터 경로(폴더): " << absolutePath << std::endl;

	std::vector<chunkbim::MeshChunk> meshChunk;
	auto retval = loadGltf(absolutePath.string(), meshChunk);
	EXPECT_TRUE(retval);
}
