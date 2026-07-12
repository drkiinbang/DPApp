#pragma once

#pragma warning(disable:4828)

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

/// tinygltf 라이브러리를 이용해 GLTF/GLB 파일을 읽고, 각 노드(node)의 메시(mesh)를
/// 월드 좌표계로 변환한 뒤 BimInfo(면 목록)로 적재하는 로더 클래스.
/// 노드 계층 구조(부모->자식 변환 행렬 누적), 인덱스 버퍼 기반 삼각형 구성,
/// (옵션) 정점-면 인접관계(vertex-to-face) 추출까지 담당한다.
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
			return BimInfo(); // 지원하지 않는 파일 형식이면 빈 BimInfo 반환
		}

		if (!warn.empty()) {
			std::cerr << "Warning: " << warn << std::endl;
		}

		if (!err.empty()) {
			std::cerr << "Error: " << err << std::endl;
		}

		if (!fileread) {
			std::cerr << "Failed to parse glTF" << std::endl;
			return BimInfo(); // 파싱 실패 시 빈 BimInfo 반환
		}

		BimInfo bimInfo;

		// 씬(scene)의 모든 루트 노드를 순회
		for (size_t s = 0; s < model.scenes.size(); ++s) {
			const tinygltf::Scene& scene = model.scenes[s];
			for (int nodeIdx : scene.nodes) {
				TraverseNode(model, nodeIdx, bimInfo);
			}
		}

		return bimInfo;
	}

	BimInfo read(std::string fileName, std::vector<chunkbim::Face>& faces, std::vector<pctree::XYZPoint>& vertices)
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
			return BimInfo(); // 지원하지 않는 파일 형식이면 빈 BimInfo 반환
		}

		if (!warn.empty()) {
			std::cerr << "Warning: " << warn << std::endl;
		}

		if (!err.empty()) {
			std::cerr << "Error: " << err << std::endl;
		}

		if (!fileread) {
			std::cerr << "Failed to parse glTF" << std::endl;
			return BimInfo(); // 파싱 실패 시 빈 BimInfo 반환
		}

		BimInfo bimInfo;

		// 씬 전체를 순회 (face/vertex 인접관계까지 함께 추출)
		for (size_t s = 0; s < model.scenes.size(); ++s) {
			const tinygltf::Scene& scene = model.scenes[s];
			for (int nodeIdx : scene.nodes) {
				TraverseNodeFaces(model, nodeIdx, bimInfo, faces, vertices);
			}
		}

		return bimInfo;
	}

protected:
	// 부모 노드들의 변환까지 모두 누적한 최종(월드) 변환 행렬을 계산한다.
	// gltf는 노드에 부모 정보를 직접 저장하지 않으므로, 매번 전체 노드를
	// 훑어서 부모를 역으로 찾아 루트까지의 경로(hierarchy)를 구성한 뒤,
	// 루트->자식 순서로 행렬을 곱해 나간다.
	Matrix4f ComputeWorldTransform(const tinygltf::Model& model, int nodeIndex) {
		Matrix4f m = Matrix4f::Identity();
		const tinygltf::Node* node = &model.nodes[nodeIndex];

		std::vector<int> hierarchy;
		while (true) {
			hierarchy.push_back(nodeIndex);

			// 부모 노드 탐색 (gltf는 부모를 저장하지 않으므로 직접 찾아야 한다)
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

		// 루트에서 자식 방향 순서로 행렬 곱셈 (역순 순회)
		for (auto it = hierarchy.rbegin(); it != hierarchy.rend(); ++it) {
			m = m * GetNodeTransform(model.nodes[*it]);
		}
		return m;
	}

private:
	Point computeNormal(const Point& v1, const Point& v2, const Point& v3)
	{
		// 삼각형의 두 변을 나타내는 벡터 계산
		Point side1 = v2 - v1;
		Point side2 = v3 - v1;

		// 두 변의 외적(cross product)으로 법선 벡터를 구한다
		Point normal = side1.crossProduct(side2).normalize();

		return normal;
	}

	// 노드 하나의 로컬 변환 행렬을 계산하는 함수.
	// matrix가 직접 주어져 있으면 그대로 사용하고, 그렇지 않으면
	// translation/rotation(quaternion)/scale을 조합해 행렬을 구성한다.
	Matrix4f GetNodeTransform(const tinygltf::Node& node) {
		Matrix4f m = Matrix4f::Identity();

		if (!node.matrix.empty()) {
			for (int i = 0; i < 16; i++)
				m(i / 4, i % 4) = static_cast<float>(node.matrix[i]);
		}
		else {
			// 이동(translation)
			if (!node.translation.empty()) {
				m.block<3, 1>(0, 3) = Eigen::Vector3f(
					node.translation[0], node.translation[1], node.translation[2]);
			}

			// 회전(quaternion)
			if (!node.rotation.empty()) {
				Eigen::Quaternionf q(
					node.rotation[3], node.rotation[0],
					node.rotation[1], node.rotation[2]);
				m.block<3, 3>(0, 0) = q.toRotationMatrix();
			}

			// 스케일(scale)
			if (!node.scale.empty()) {
				m(0, 0) *= node.scale[0];
				m(1, 1) *= node.scale[1];
				m(2, 2) *= node.scale[2];
			}
		}
		return m;
	}

	// 문자열 접미사(suffix) 비교
	bool EndsWith(const std::string& str, const std::string& suffix) {
		return str.size() >= suffix.size() &&
			str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
	}

	// 노드 순회 함수 (face만 추출, vertex 인접관계는 필요 없는 경우)
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

		// 자식 노드도 재귀적으로 순회
		for (int child : node.children) {
			TraverseNode(model, child, bimInfo);
		}
	}


	// 노드 순회 함수 (face/vertex 인접관계 포함 버전)
	void TraverseNodeFaces(const tinygltf::Model& model, int nodeIndex,
		BimInfo& bimInfo,
		std::vector<chunkbim::Face>& faces,
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

				// ---- 정점(vertex) 저장 ----
				size_t baseIndex = vertices.size();
				for (int i = 0; i < numVertices; ++i) {
					Vector4f v = world * Vector4f(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2], 1.0f);
					pctree::XYZPoint p(v.x(), v.y(), v.z());
					vertices.push_back(p);
				}

				// ---- face + BimInfo 저장 ----
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

					// 정점->face 인접관계 저장 (이 정점이 어느 face들에 속하는지)
					vertices[idx1].fIds.push_back(fid);
					vertices[idx2].fIds.push_back(fid);
					vertices[idx3].fIds.push_back(fid);

					++fid;
				}
			}
		}

		// 자식 노드도 재귀적으로 순회
		for (int child : node.children) {
			TraverseNodeFaces(model, child, bimInfo, faces, vertices);
		}
	}
};
