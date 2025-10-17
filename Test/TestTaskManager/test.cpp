#include "pch.h"

TEST(TestCaseName, TestName) {
  EXPECT_EQ(1, 1);
  EXPECT_TRUE(true);
}

#include "TaskManager.h"

TEST(TaskManagerTest, ImportBIM)
{
    const std::string dataFolder = "F:\\repository\\DPApp\\data";
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

TEST(TaskManagerTest, ImportPTS)
{
    const std::string dataFolder = "F:\\repository\\DPApp\\data";
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

TEST(TaskManagerTest, ImportLAS)
{
    const std::string dataFolder = "F:\\repository\\DPApp\\data";
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

TEST(TaskManagerTest, ImportPointclouds2)
{
    const std::string dataFolder = "F:\\repository\\DPApp\\data";
    std::string project_name = "samsung_test_pc2";

    bool is_offset_applied = false;
    float offset[3] = { 0.f, 0.f, 0.f };

    std::vector<BimMeshInfo> bimData;

    if (true) {
        std::string pc2_path = dataFolder + std::string("\\pointclouds2\\samsung_test.pointclouds2");

        std::vector<float> pointbuffer;
        auto retval = pointclouds2::loadPointclouds2(pc2_path, pointbuffer);

        std::vector<float> pointbuffer2;
        size_t beginPtsIdx = 100000;
        size_t numPtsToRead = 100;
        auto retval2 = pointclouds2::loadPointclouds2(pc2_path, pointbuffer2, numPtsToRead, beginPtsIdx);

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

        EXPECT_TRUE(retval);
    }
}