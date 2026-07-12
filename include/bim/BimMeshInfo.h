#pragma once

#include "BimInfo.h"

/// GLTF에서 로딩한 BIM 부재(요소) 하나를 표현하는 BimInfo 확장 클래스.
/// BimInfo가 담당하는 face/바운딩박스/직렬화 기능에 더해, GLTF 고유의 메타데이터
/// (부재 이름, Revit Element ID, 원본 GLTF mesh/primitive 개수 등)를 함께 가지고 있다.
class BimMeshInfo : public BimInfo{
private:
    /// model.mesh[].primitives.size()
    std::vector<size_t> numPriInMesh;
    /// model.mesh[].name
    std::vector<std::string> meshNames;
        
    int data_source_id = 1;
    std::string node_name_utf8;
    std::string utf8FileName;
public:
    std::array<float, 3> bbox_min;
    std::array<float, 3> bbox_max;
    std::string node_name;
    int bim_id;

public:
	BimMeshInfo() = default;
	~BimMeshInfo() = default;

    void addMesh(const std::string& meshName_, const std::vector<Point>& normal, const std::vector<Point>& v1, const std::vector<Point>& v2, const std::vector<Point>& v3) {
        numPriInMesh.push_back(normal.size());
        meshNames.push_back(meshName_);
        try {
            for (size_t i = 0; i < normal.size(); ++i) {
                BimInfo::addFace(normal[i], v1[i], v2[i], v3[i]);
            }
        }
        catch (...) {
            std::cerr << "Error in addMesh\n";
        }
    }
};
