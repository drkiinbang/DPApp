#include "pch.h"

TEST(TestCaseName, TestName) {
  EXPECT_EQ(1, 1);
  EXPECT_TRUE(true);
}

#include "TaskManager.h"

#ifdef _WIN32
#include <windows.h>
#endif

/// [수정] 예전에는 데이터 경로가 문자열 그대로 "../data"였는데, 이는 테스트를 bin/을
/// 작업 디렉토리로 삼아 실행할 때만 올바르게 해석되었다(예: 탐색기에서 exe를 더블클릭하거나
/// CI에서 다른 디렉토리로부터 실행하면 테스트 데이터를 전혀 찾지 못했다). 대신 실행 중인
/// 실행 파일 자신의 위치를 기준으로 경로를 해석하도록 바꿔서, 호출자의 현재 작업
/// 디렉토리가 무엇이든 항상 동작하도록 했다.
static std::string getTestDataFolder() {
#ifdef _WIN32
    char exePath[MAX_PATH] = { 0 };
    GetModuleFileNameA(NULL, exePath, MAX_PATH);
    std::string path(exePath);
    size_t lastSlash = path.find_last_of("\\/");
    std::string exeDir = (lastSlash != std::string::npos) ? path.substr(0, lastSlash) : ".";
    return exeDir + "\\..\\data";
#else
    return "../data";
#endif
}

// GLTF/GLB BIM 파일을 읽어 .nodes2 바이너리 포맷으로 변환하는
// convert_gltf2nodes2()를 검증하기 위한 테스트.
// [참고] 아래 if(false)로 실제 검증 로직이 비활성화되어 있어, 이 TEST는 현재
// 항상 통과만 하고 실질적으로는 아무것도 검증하지 않는다(대용량 실데이터 의존 때문으로 보임).
TEST(TaskManagerTest, ImportBIM)
{
    const std::string dataFolder = getTestDataFolder();
    std::string project_name = "samsung_test";
    std::string bim_folder = dataFolder + std::string("\\10glb");
    std::string nodes2_folder = dataFolder + std::string("\\10glbnodes2_");

    bool is_offset_applied = false;
    float offset[3] = { 0.f, 0.f, 0.f };

    std::vector<BimMeshInfo> bimData;

    if (false) {
        auto retval0 = DPApp::Pt2MeshProcessors::convert_gltf2nodes2(project_name,
            bim_folder,
            nodes2_folder,
            bimData,
            is_offset_applied,
            offset);

        EXPECT_TRUE(retval0);
    }
}

// PTS 포인트클라우드 파일을 pointclouds2 바이너리 포맷으로 변환하는
// convert_pts2pc2()를 검증하기 위한 테스트 (아래 if(false)로 비활성화 — ImportBIM 참고).
TEST(TaskManagerTest, ImportPTS)
{
    const std::string dataFolder = getTestDataFolder();
    std::string project_name = "samsung_test_pts";

    bool is_offset_applied = false;
    float offset[3] = { 0.f, 0.f, 0.f };

    std::vector<BimMeshInfo> bimData;

    if (false) {
        std::string pts_path = dataFolder + std::string("\\samsung_test\\samsung.pts");
        //std::string pts_path = dataFolder + std::string("\\test.pts");
        std::string pointclouds_folder = dataFolder + std::string("\\pointclouds2_pts");
        std::string pointsclouds2path = pointclouds_folder + std::string("\\") + project_name + std::string(".pointclouds2");

        auto retval1 = DPApp::Pt2MeshProcessors::convert_pts2pc2(pts_path,
            pointsclouds2path,
            is_offset_applied,
            offset);

        EXPECT_TRUE(retval1);
    }
}

// LAS 포인트클라우드 파일을 pointclouds2 바이너리 포맷으로 변환하는
// convert_las2pc2()를 검증하기 위한 테스트 (아래 if(false)로 비활성화 — ImportBIM 참고).
TEST(TaskManagerTest, ImportLAS)
{
    const std::string dataFolder = getTestDataFolder();
    std::string project_name = "samsung_test_las";

    bool is_offset_applied = false;
    float offset[3] = { 0.f, 0.f, 0.f };

    std::vector<BimMeshInfo> bimData;

    if (false) {
        std::string las_path = dataFolder + std::string("\\samsung_test\\samsung.las");
        std::string pointclouds_folder2 = dataFolder + std::string("\\pointclouds2_las");
        std::string pointsclouds2path2 = pointclouds_folder2 + std::string("\\") + project_name + std::string(".pointclouds2");

        auto retval1 = DPApp::Pt2MeshProcessors::convert_las2pc2(las_path,
            pointsclouds2path2,
            is_offset_applied,
            offset);

        EXPECT_TRUE(retval1);
    }
}

// pointclouds2 바이너리 포맷 로더(pointclouds2::loadPointclouds2)를 검증하는 테스트.
// 전체 로드 결과와, 특정 시작 인덱스(beginPtsIdx)부터 일부 개수(numPtsToRead)만 읽은
// 부분 로드 결과가 같은 좌표를 가리키는지 비교해 부분 읽기 경로가 올바른지 확인한다.
// [참고] 위 두 테스트와 달리 if(true)로 실제로 실행되는, 이 파일에서 유일하게 활성화된 테스트다.
TEST(TaskManagerTest, ImportPointclouds2)
{
    const std::string dataFolder = getTestDataFolder();
    std::string project_name = "samsung_test_pc2";

    bool is_offset_applied = false;
    float offset[3] = { 0.f, 0.f, 0.f };

    std::vector<BimMeshInfo> bimData;

    if (true) {
        std::string pc2_path = dataFolder + std::string("\\pointclouds2\\samsung_test.pointclouds2");

        std::vector<float> pointbuffer;
        auto retval = pointclouds2::loadPointclouds2(pc2_path, pointbuffer);
        ASSERT_TRUE(retval) << "Failed to load " << pc2_path
            << " -- check that the working directory is set to bin/ so the relative "
            << "path resolves correctly.";

        std::vector<float> pointbuffer2;
        size_t beginPtsIdx = 100000;
        size_t numPtsToRead = 100;
        auto retval2 = pointclouds2::loadPointclouds2(pc2_path, pointbuffer2, numPtsToRead, beginPtsIdx);
        ASSERT_TRUE(retval2) << "Failed to load partial range from " << pc2_path;

        /// 아래 인덱싱이 범위를 벗어나지 않도록 방어: 비교 루프가 읽으려는 포인트 범위를
        /// 두 버퍼 모두 실제로 담고 있는지 확인한다.
        ASSERT_GE(pointbuffer2.size(), numPtsToRead * 3);
        ASSERT_GE(pointbuffer.size(), (beginPtsIdx + numPtsToRead) * 3);

        bool equalCheck = true;
        for (size_t i = 0; i < numPtsToRead; ++i){
            if (pointbuffer2[i * 3 + 0] != pointbuffer[(i + beginPtsIdx) * 3 + 0]) {
                std::cout << i << " th point X coordinate is not equal\n";
                equalCheck = false;
                break;
            };

            if (pointbuffer2[i * 3 + 1] != pointbuffer[(i + beginPtsIdx) * 3 + 1]) {
                std::cout << i << " th point Y coordinate is not equal\n";
                equalCheck = false;
                break;
            };

            if (pointbuffer2[i * 3 + 2] != pointbuffer[(i + beginPtsIdx) * 3 + 2]) {
                std::cout << i << " th point Z coordinate is not equal\n";
                equalCheck = false;
                break;
            };
        }

        EXPECT_TRUE(equalCheck);
    }
}