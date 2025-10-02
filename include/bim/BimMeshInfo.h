#pragma once

#include "BimInfo.h"

class BimMeshInfo : public BimInfo{
private:
    /// model.mesh[].primitives.size()
    std::vector<size_t> numPriInMesh;
    /// model.mesh[].name
    std::vector<std::string> meshNames;
    
    std::string node_name;
    int bim_id;
    int data_source_id = 1;
    std::string node_name_utf8;
    std::string utf8FileName;
    std::array<float, 3> bbox_min;
    std::array<float, 3> bbox_max;

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
