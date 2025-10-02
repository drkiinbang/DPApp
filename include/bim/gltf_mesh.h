#pragma once

#include "gltf.h"
#include "BimMeshInfo.h"

class GltfMesh : public gltf {
public:
	GltfMesh() = default;
	~GltfMesh() = default;

    bool read(std::string fileName,
        BimMeshInfo& bimInfo,
        const bool is_offset_applied,
        const float* offset)
    {
        tinygltf::Model model;
        tinygltf::TinyGLTF loader;
        std::string err;
        std::string warn;

        bool fileread = false;

        if (fileName.substr(fileName.find_last_of(".") + 1) == "glb") {
            fileread = loader.LoadBinaryFromFile(&model, &err, &warn, fileName);
        }
        else if (fileName.substr(fileName.find_last_of(".") + 1) == "gltf") {
            fileread = loader.LoadASCIIFromFile(&model, &err, &warn, fileName);
        }
        else {
            std::cerr << "Unsupported file format" << std::endl;
            return false; // Return empty BimMeshInfo if unsupported file format
        }

        if (!warn.empty()) {
            std::cerr << "Warning: " << warn << std::endl;
        }

        if (!err.empty()) {
            std::cerr << "Error: " << err << std::endl;
        }

        if (!fileread) {
            std::cerr << "Failed to parse glTF" << std::endl;
            return false; // Return empty BimInfo if failed to parse glTF
        }

        /// Define a single transformation shared for all meshes
        Matrix4f world = Matrix4f::Identity();

        for (size_t s = 0; s < model.scenes.size(); ++s) {
            const tinygltf::Scene& scene = model.scenes[s];
            for (size_t nodeIdx : scene.nodes) {
                const tinygltf::Node& node = model.nodes[nodeIdx];
                if (node.mesh >= 0) {
                    world = ComputeWorldTransform(model, nodeIdx);
                }
            }
        }

        ///  Apply the given offset
        if (is_offset_applied) {
            world(0, 3) -= offset[0];
            world(1, 3) -= offset[1];
            world(2, 3) -= offset[2];
        }

        for (size_t meshIdx = 0; meshIdx < model.meshes.size(); ++meshIdx) {
            const tinygltf::Mesh& mesh = model.meshes[meshIdx];

            std::vector<Point> normal, v1, v2, v3;
            normal.reserve(mesh.primitives.size());
            v1.reserve(mesh.primitives.size());
            v2.reserve(mesh.primitives.size());
            v3.reserve(mesh.primitives.size());

            for (const auto& primitive : mesh.primitives) {
                if (primitive.attributes.find("POSITION") == primitive.attributes.end()) continue;
                if (primitive.attributes.find("NORMAL") == primitive.attributes.end())   continue;

                int posAccIdx = primitive.attributes.at("POSITION");
                int norAccIdx = primitive.attributes.at("NORMAL");

                const tinygltf::Accessor& posAcc = model.accessors[posAccIdx];
                const tinygltf::Accessor& norAcc = model.accessors[norAccIdx];

                const tinygltf::BufferView& posView = model.bufferViews[posAcc.bufferView];
                const tinygltf::BufferView& norView = model.bufferViews[norAcc.bufferView];

                const float* positions = reinterpret_cast<const float*>(
                    &model.buffers[posView.buffer].data[posView.byteOffset + posAcc.byteOffset]);
                const float* normals = reinterpret_cast<const float*>(
                    &model.buffers[norView.buffer].data[norView.byteOffset + norAcc.byteOffset]);

                const tinygltf::Accessor& idxAcc = model.accessors[primitive.indices];
                const tinygltf::BufferView& idxView = model.bufferViews[idxAcc.bufferView];
                const uint16_t* indices = reinterpret_cast<const uint16_t*>(
                    &model.buffers[idxView.buffer].data[idxView.byteOffset + idxAcc.byteOffset]);

                int numIndices = idxAcc.count;

                for (int i = 0; i < numIndices; i += 3) {
                    int idx1 = indices[i];
                    int idx2 = indices[i + 1];
                    int idx3 = indices[i + 2];

                    Vector4f vtx[3];
                    vtx[0] = world * Vector4f(positions[idx1 * 3], positions[idx1 * 3 + 1], positions[idx1 * 3 + 2], 1.0f);
                    vtx[1] = world * Vector4f(positions[idx2 * 3], positions[idx2 * 3 + 1], positions[idx2 * 3 + 2], 1.0f);
                    vtx[2] = world * Vector4f(positions[idx3 * 3], positions[idx3 * 3 + 1], positions[idx3 * 3 + 2], 1.0f);

                    Vector4f n1 = world * Vector4f(normals[idx1 * 3], normals[idx1 * 3 + 1], normals[idx1 * 3 + 2], 0.0f);

                    normal.push_back(Point(n1.x(), n1.y(), n1.z()));

                    v1.push_back(Point(vtx[0].x(), vtx[0].y(), vtx[0].z()));
                    v2.push_back(Point(vtx[1].x(), vtx[1].y(), vtx[1].z()));
                    v3.push_back(Point(vtx[2].x(), vtx[2].y(), vtx[2].z()));
                }
            }

            /// add a new mesh consisting of multiple primitives
            bimInfo.addMesh(mesh.name, normal, v1, v2, v3);
        }

        return true;
    }
};
