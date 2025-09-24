#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <Eigen/Dense>

#include "BimInfo.h"
#include "../bimtree/dataStructs.hpp"
#include "../bimtree/xyzpoint.hpp"

#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "gltf\tiny_gltf.h"

using namespace tinygltf;

using Matrix4f = Eigen::Matrix4f;
using Vector4f = Eigen::Vector4f;

class gltf
{
public:
  gltf() {}

  // -------------------------------
  // GLTF 읽기 (BimInfo 반환)
  // -------------------------------
  BimInfo read(std::string fileName) 
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
      return BimInfo(); // Return empty BimInfo if unsupported file format
    }

    if (!warn.empty()) {
      std::cerr << "Warning: " << warn << std::endl;
    }

    if (!err.empty()) {
      std::cerr << "Error: " << err << std::endl;
    }

    if (!fileread) {
      std::cerr << "Failed to parse glTF" << std::endl;
      return BimInfo(); // Return empty BimInfo if failed to parse glTF
    }

    BimInfo bimInfo;

    // Scene의 모든 root node 탐색
    for (size_t s = 0; s < model.scenes.size(); ++s) {
      const tinygltf::Scene& scene = model.scenes[s];
      for (int nodeIdx : scene.nodes) {
        TraverseNode(model, nodeIdx, bimInfo);
      }
    }
    
    return bimInfo;
  }

  BimInfo read(std::string fileName, std::vector<mesh::Face>& faces, std::vector<pctree::XYZPoint>& vertices)
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
          return BimInfo(); // Return empty BimInfo if unsupported file format
      }

      if (!warn.empty()) {
          std::cerr << "Warning: " << warn << std::endl;
      }

      if (!err.empty()) {
          std::cerr << "Error: " << err << std::endl;
      }

      if (!fileread) {
          std::cerr << "Failed to parse glTF" << std::endl;
          return BimInfo(); // Return empty BimInfo if failed to parse glTF
      }

      BimInfo bimInfo;       

      // Scene 전체 탐색
      for (size_t s = 0; s < model.scenes.size(); ++s) {
        const tinygltf::Scene& scene = model.scenes[s];
        for (int nodeIdx : scene.nodes) {
          TraverseNodeFaces(model, nodeIdx, bimInfo, faces, vertices);
        }
      }

      return bimInfo;
  }

private:
  Point computeNormal(const Point& v1, const Point& v2, const Point& v3)
  {
    // Calculate the vectors representing two sides of the triangle
    Point side1 = v2 - v1;
    Point side2 = v3 - v1;

    // Compute the cross product of the two sides to get the normal vector
    Point normal = side1.crossProduct(side2).normalize();

    return normal;
  }

  // node의 transform을 계산하는 함수
  Matrix4f GetNodeTransform(const tinygltf::Node& node) {
    Matrix4f m = Matrix4f::Identity();

    if (!node.matrix.empty()) {
      for (int i = 0; i < 16; i++)
        m(i / 4, i % 4) = static_cast<float>(node.matrix[i]);
    }
    else {
      // translation
      if (!node.translation.empty()) {
        m.block<3, 1>(0, 3) = Eigen::Vector3f(
          node.translation[0], node.translation[1], node.translation[2]);
      }

      // rotation (quaternion)
      if (!node.rotation.empty()) {
        Eigen::Quaternionf q(
          node.rotation[3], node.rotation[0],
          node.rotation[1], node.rotation[2]);
        m.block<3, 3>(0, 0) = q.toRotationMatrix();
      }

      // scale
      if (!node.scale.empty()) {
        m(0, 0) *= node.scale[0];
        m(1, 1) *= node.scale[1];
        m(2, 2) *= node.scale[2];
      }
    }
    return m;
  }

  // 부모까지 포함한 최종 transform
  Matrix4f ComputeWorldTransform(const tinygltf::Model& model, int nodeIndex) {
    Matrix4f m = Matrix4f::Identity();
    const tinygltf::Node* node = &model.nodes[nodeIndex];

    std::vector<int> hierarchy;
    while (true) {
      hierarchy.push_back(nodeIndex);

      // 부모 탐색 (gltf는 parent 저장 안 해서 직접 찾아야 함)
      int parent = -1;
      for (size_t i = 0; i < model.nodes.size(); i++) {
        for (int child : model.nodes[i].children) {
          if (child == nodeIndex) {
            parent = static_cast<int>(i);
            break;
          }
        }
      }
      if (parent == -1) break;
      nodeIndex = parent;
      node = &model.nodes[nodeIndex];
    }

    // 루트 → 자식 순서로 곱하기
    for (auto it = hierarchy.rbegin(); it != hierarchy.rend(); ++it) {
      m = m * GetNodeTransform(model.nodes[*it]);
    }
    return m;
  }

  // 문자열 끝 비교
  bool EndsWith(const std::string& str, const std::string& suffix) {
    return str.size() >= suffix.size() &&
      str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
  }

  // 노드 탐색 함수 
  void TraverseNode(const tinygltf::Model& model, int nodeIndex, BimInfo& bimInfo) {
    const tinygltf::Node& node = model.nodes[nodeIndex];
    Matrix4f world = ComputeWorldTransform(model, nodeIndex);

    if (node.mesh >= 0) {
      const tinygltf::Mesh& mesh = model.meshes[node.mesh];

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

        // 삼각형 생성
        for (int i = 0; i < numIndices; i += 3) {
          int idx1 = indices[i];
          int idx2 = indices[i + 1];
          int idx3 = indices[i + 2];

          Vector4f v1 = world * Vector4f(positions[idx1 * 3], positions[idx1 * 3 + 1], positions[idx1 * 3 + 2], 1.0f);
          Vector4f v2 = world * Vector4f(positions[idx2 * 3], positions[idx2 * 3 + 1], positions[idx2 * 3 + 2], 1.0f);
          Vector4f v3 = world * Vector4f(positions[idx3 * 3], positions[idx3 * 3 + 1], positions[idx3 * 3 + 2], 1.0f);

          Vector4f n1 = world * Vector4f(normals[idx1 * 3], normals[idx1 * 3 + 1], normals[idx1 * 3 + 2], 0.0f);

          bimInfo.addFace(
            Point(n1.x(), n1.y(), n1.z()),
            Point(v1.x(), v1.y(), v1.z()),
            Point(v2.x(), v2.y(), v2.z()),
            Point(v3.x(), v3.y(), v3.z())
          );
        }
      }
    }

    // 자식 노드도 탐색
    for (int child : node.children) {
      TraverseNode(model, child, bimInfo);
    }
  }


  // 노드 탐색 함수 (faces/vertices 반영)
  void TraverseNodeFaces(const tinygltf::Model& model, int nodeIndex,
    BimInfo& bimInfo,
    std::vector<mesh::Face>& faces,
    std::vector<pctree::XYZPoint>& vertices)
  {
    const tinygltf::Node& node = model.nodes[nodeIndex];
    Matrix4f world = ComputeWorldTransform(model, nodeIndex);

    if (node.mesh >= 0) {
      const tinygltf::Mesh& mesh = model.meshes[node.mesh];

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

        int numVertices = posAcc.count;
        int numIndices = idxAcc.count;

        // ---- Vertex 저장 ----
        size_t baseIndex = vertices.size();
        for (int i = 0; i < numVertices; ++i) {
          Vector4f v = world * Vector4f(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2], 1.0f);
          pctree::XYZPoint p(v.x(), v.y(), v.z());
          vertices.push_back(p);
        }

        // ---- Face 저장 + BimInfo ----
        size_t fid = 0;
        for (int i = 0; i < numIndices; i += 3) {
          int idx1 = indices[i] + baseIndex;
          int idx2 = indices[i + 1] + baseIndex;
          int idx3 = indices[i + 2] + baseIndex;

          faces.emplace_back(idx1, idx2, idx3);

          Vector4f v1 = Vector4f(vertices[idx1].x(), vertices[idx1].y(), vertices[idx1].z(), 1.0f);
          Vector4f v2 = Vector4f(vertices[idx2].x(), vertices[idx2].y(), vertices[idx2].z(), 1.0f);
          Vector4f v3 = Vector4f(vertices[idx3].x(), vertices[idx3].y(), vertices[idx3].z(), 1.0f);

          Vector4f n1 = world * Vector4f(normals[indices[i] * 3],
            normals[indices[i] * 3 + 1],
            normals[indices[i] * 3 + 2],
            0.0f);

          bimInfo.addFace(
            Point(n1.x(), n1.y(), n1.z()),
            Point(v1.x(), v1.y(), v1.z()),
            Point(v2.x(), v2.y(), v2.z()),
            Point(v3.x(), v3.y(), v3.z())
          );

          // Vertex → Face 관계 저장
          vertices[idx1].fIds.push_back(fid);
          vertices[idx2].fIds.push_back(fid);
          vertices[idx3].fIds.push_back(fid);

          ++fid;
        }
      }
    }

    // 자식 노드도 탐색
    for (int child : node.children) {
      TraverseNodeFaces(model, child, bimInfo, faces, vertices);
    }
  }
};

