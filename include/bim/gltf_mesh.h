#pragma once

#include "gltf.h"
#include "BimMeshInfo.h"

#include "../bimtree/dataStructs.hpp"
//#include "./MeshChunk.h"

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
					world = ComputeWorldTransform(model, static_cast<int>(nodeIdx));
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

				size_t numIndices = idxAcc.count;

				for (size_t i = 0; i < numIndices; i += 3) {
					size_t idx1 = indices[i];
					size_t idx2 = indices[i + 1];
					size_t idx3 = indices[i + 2];

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

namespace mesh {
	/// column-major 4x4 matrix
	struct Mat4 {
		float m[16];
	};

	Mat4 Mat4Identity() {
		Mat4 r{};
		r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
		return r;
	}

	Mat4 Mat4Multiply(const Mat4& a, const Mat4& b) {
		Mat4 r{};
		for (int col = 0; col < 4; ++col) {
			for (int row = 0; row < 4; ++row) {
				r.m[col * 4 + row] =
					a.m[0 * 4 + row] * b.m[col * 4 + 0] +
					a.m[1 * 4 + row] * b.m[col * 4 + 1] +
					a.m[2 * 4 + row] * b.m[col * 4 + 2] +
					a.m[3 * 4 + row] * b.m[col * 4 + 3];
			}
		}
		return r;
	}

	pctree::XYZPoint TransformPoint(const Mat4& m, const pctree::XYZPoint& v) {
		float x = v[0], y = v[1], z = v[2];
		float rx = m.m[0] * x + m.m[4] * y + m.m[8] * z + m.m[12];
		float ry = m.m[1] * x + m.m[5] * y + m.m[9] * z + m.m[13];
		float rz = m.m[2] * x + m.m[6] * y + m.m[10] * z + m.m[14];
		float rw = m.m[3] * x + m.m[7] * y + m.m[11] * z + m.m[15];
		if (rw != 0.0f) {
			rx /= rw; ry /= rw; rz /= rw;
		}
		return pctree::XYZPoint(rx, ry, rz);
	}

	pctree::XYZPoint RotatePoint(const Mat4& m, const pctree::XYZPoint& v) {
		float x = v[0], y = v[1], z = v[2];
		float rx = m.m[0] * x + m.m[4] * y + m.m[8] * z;
		float ry = m.m[1] * x + m.m[5] * y + m.m[9] * z;
		float rz = m.m[2] * x + m.m[6] * y + m.m[10] * z;
		return pctree::XYZPoint(rx, ry, rz);
	}

	/// quaternion(x,y,z,w) → 4X4 matrix (rotation matrix and zero translatoion)
	Mat4 Mat4FromQuaternion(const std::vector<double>& q) {
		double x = 0, y = 0, z = 0, w = 1;
		if (q.size() == 4) {
			x = q[0]; y = q[1]; z = q[2]; w = q[3];
		}
		double xx = x * x, yy = y * y, zz = z * z;
		double xy = x * y, xz = x * z, yz = y * z;
		double wx = w * x, wy = w * y, wz = w * z;

		Mat4 r{};
		r.m[0] = float(1.0 - 2.0 * (yy + zz));
		r.m[1] = float(2.0 * (xy + wz));
		r.m[2] = float(2.0 * (xz - wy));
		r.m[3] = 0.0f;

		r.m[4] = float(2.0 * (xy - wz));
		r.m[5] = float(1.0 - 2.0 * (xx + zz));
		r.m[6] = float(2.0 * (yz + wx));
		r.m[7] = 0.0f;

		r.m[8] = float(2.0 * (xz + wy));
		r.m[9] = float(2.0 * (yz - wx));
		r.m[10] = float(1.0 - 2.0 * (xx + yy));
		r.m[11] = 0.0f;

		r.m[12] = 0.0f;
		r.m[13] = 0.0f;
		r.m[14] = 0.0f;
		r.m[15] = 1.0f;
		return r;
	}

	Mat4 Mat4FromTRS(const std::vector<double>& t,
		const std::vector<double>& r,
		const std::vector<double>& s)
	{
		/// T
		Mat4 T = Mat4Identity();
		if (t.size() == 3) {
			T.m[12] = float(t[0]);
			T.m[13] = float(t[1]);
			T.m[14] = float(t[2]);
		}

		/// R
		Mat4 R = Mat4FromQuaternion(r);

		/// S
		Mat4 S = Mat4Identity();
		double sx = 1.0, sy = 1.0, sz = 1.0;
		if (s.size() == 3) {
			sx = s[0]; sy = s[1]; sz = s[2];
		}
		S.m[0] = float(sx);
		S.m[5] = float(sy);
		S.m[10] = float(sz);

		/// glTF: M = T * R * S
		return Mat4Multiply(Mat4Multiply(T, R), S);
	}

	Mat4 LocalMatrixFromNode(const tinygltf::Node& node) {
		if (node.matrix.size() == 16) {
			Mat4 M{};
			for (int i = 0; i < 16; ++i)
				M.m[i] = float(node.matrix[i]);
			return M;
		}
		else {
			return Mat4FromTRS(node.translation, node.rotation, node.scale);
		}
	}

	class GltfMesh {
	public:
		const int defauMeshID = -1;

		GltfMesh() = default;
		~GltfMesh() = default;

		static bool verifyMeshIdsInFile(const std::string& filename)
		{
			tinygltf::Model model;
			tinygltf::TinyGLTF loader;
			std::string err, warn;

			bool ok = false;

			/// 파일 확장자 확인 및 로드
			const auto dotPos = filename.find_last_of('.');
			if (dotPos == std::string::npos) {
				std::cerr << "Error: No file extension.\n";
				return false;
			}
			std::string ext = filename.substr(dotPos + 1);
			to_lower(ext); /// static 함수 내에서 private static 헬퍼 함수를 가정하고 호출

			if (ext == "glb") {
				ok = loader.LoadBinaryFromFile(&model, &err, &warn, filename);
			}
			else if (ext == "gltf") {
				ok = loader.LoadASCIIFromFile(&model, &err, &warn, filename);
			}
			else {
				std::cerr << "Error: Unsupported extension: " << ext << "\n";
				return false;
			}

			if (!warn.empty()) 
				std::cerr << "Warning: " << warn << "\n";

			if (!err.empty())  
				std::cerr << "Err : " << err << "\n";
			
			if (!ok) {
				std::cerr << "Error: Failed to load glTF file.\n";
				return false;
			}

			std::cout << "\n--- Verification: Mesh IDs in " << filename << " ---\n";

			int total_meshes = (int)model.meshes.size();
			int found_ids = 0;

			/// 모든 Mesh 순회 및 ID 추출
			for (int i = 0; i < total_meshes; ++i) {
				const auto& mesh = model.meshes[i];
				int mesh_id = -1; // Default ID

				/// Mesh 이름이 있으면 사용, 없으면 'Unnamed'
				std::string mesh_name = mesh.name.empty() ? "Unnamed" : mesh.name;

				/// 1. mesh.extras가 JSON Object인지 확인
				if (mesh.extras.IsObject()) {
					// 2. extras 맵을 추출
					const auto& extrasMap = mesh.extras.Get<tinygltf::Value::Object>();

					/// 3. 맵에서 "id" 키 검색
					auto itExtras = extrasMap.find("id");

					if (itExtras != extrasMap.end()) {
						const auto& extValue = itExtras->second;

						/// 4. Int id 확인하고 추출
						if (extValue.IsInt()) {
							mesh_id = extValue.Get<int>();
							found_ids++;
							std::cout << "[" << i << "] Mesh: '" << mesh_name << "' -> ID: " << mesh_id << "\n";
						}
						else {
							std::cout << "[" << i << "] Mesh: '" << mesh_name << "' -> Warning: 'id' in extras is not a number.\n";
						}
					}
					else {
						/// extras 필드는 있으나 "id" 키가 없음
						std::cout << "[" << i << "] Mesh: '" << mesh_name << "' -> ID: Not Found in extras.\n";
					}
				}
				else {
					/// extras 필드가 object가 아니거나 없음
					std::cout << "[" << i << "] Mesh: '" << mesh_name << "' -> ID: Not Found (Extras missing or invalid).\n";
				}
			}

			std::cout << "------------------------------------------\n";
			std::cout << "Summary: Total Meshes: " << total_meshes << ", Meshes with 'id': " << found_ids << "\n";

			return true;
		}

		bool checkExt(const std::string& filename, const std::string extName)
		{
			tinygltf::Model model;
			tinygltf::TinyGLTF loader;
			std::string err, warn;

			bool ok = false;

			const auto dotPos = filename.find_last_of('.');
			if (dotPos == std::string::npos) {
				std::cerr << "No file extension.\n";
				return false;
			}
			std::string ext = filename.substr(dotPos + 1);
			to_lower(ext);
			if (ext == "glb") {
				ok = loader.LoadBinaryFromFile(&model, &err, &warn, filename);
			}
			else if (ext == "gltf") {
				ok = loader.LoadASCIIFromFile(&model, &err, &warn, filename);
			}
			else {
				std::cerr << "Unsupported extension: " << ext << "\n";
				return false;
			}

			if (!warn.empty()) std::cerr << "Warn: " << warn << "\n";
			if (!err.empty())  std::cerr << "Err : " << err << "\n";
			if (!ok) {
				std::cerr << "Failed to load glTF\n";
				return false;
			}
		}

		bool importMeshData(const std::string& filename,
			std::vector<MeshChunk>& meshChunks,
			const int extractionOption = 0)
		{
			tinygltf::Model model;
			tinygltf::TinyGLTF loader;
			std::string err, warn;

			bool ok = false;

			const auto dotPos = filename.find_last_of('.');
			if (dotPos == std::string::npos) {
				std::cerr << "No file extension.\n";
				return false;
			}
			std::string ext = filename.substr(dotPos + 1);
			to_lower(ext);
			if (ext == "glb") {
				ok = loader.LoadBinaryFromFile(&model, &err, &warn, filename);
			}
			else if (ext == "gltf") {
				ok = loader.LoadASCIIFromFile(&model, &err, &warn, filename);
			}
			else {
				std::cerr << "Unsupported extension: " << ext << "\n";
				return false;
			}

			if (!warn.empty()) std::cerr << "Warn: " << warn << "\n";
			if (!err.empty())  std::cerr << "Err : " << err << "\n";
			if (!ok) {
				std::cerr << "Failed to load glTF\n";
				return false;
			}

			bool retval = false;

			switch (extractionOption) {
			case 0:
				retval = ExtractAllMeshes(model, meshChunks);
				break;
			case 1:
				if (model.scenes.empty()) {
					std::cerr << "No scene in " << filename << "\n";
					return false;
				}

				/// Select the default scene, if not exist then, select the first scene
				int sceneIndex = model.defaultScene;
				if (sceneIndex < 0 && !model.scenes.empty()) {
					sceneIndex = 0;
				}

				if (sceneIndex < 0 || sceneIndex >= (int)model.scenes.size()) {
					std::cerr << "No valid scene in " << filename << "\n";
					return false;
				}

				retval = ExtractDefaultSceneNodes(model, meshChunks, sceneIndex);
				break;
			}

			return retval;
		}

	public:
		inline static void to_lower(std::string& ext) {
			std::transform(ext.begin(), ext.end(), ext.begin(),
				[](unsigned char c) { return std::tolower(c); });
		}

	private:
		bool ExtractAllMeshes(tinygltf::Model& model,
			std::vector<MeshChunk>& meshChunks) {
			auto worldMatrix = Mat4Identity();

			/// Traverse all meshes in a file
			for (const auto& mesh : model.meshes) {
				meshChunks.push_back(MeshChunk());
				MeshChunk& currentChunk = meshChunks.back();
				currentChunk.name = mesh.name;
				currentChunk.id = defauMeshID;

				/// Extract id
				if (mesh.extras.IsObject()) {
					const auto& extrasMap = mesh.extras.Get<tinygltf::Value::Object>();
					auto itExtras = extrasMap.find("id");
					if (itExtras != extrasMap.end()) {
						const auto& extValue = itExtras->second;
						if (extValue.IsInt()) {
							currentChunk.id = extValue.Get<int>();
						}
						else {
							std::cerr << "Mesh id is not a number value in the extras\n";
						}
					}
				}

				for (const auto& primitive : mesh.primitives) {
					if (primitive.mode != TINYGLTF_MODE_TRIANGLES)
						continue;

					/// Extract primitive data
					ExtractMeshPrimitive(model, primitive, worldMatrix, currentChunk);
				}
			}

			return true;
		}

		void ExtractMeshPrimitive(
			const tinygltf::Model& model,
			const tinygltf::Primitive& primitive,
			const Mat4& worldMatrix, // World Matrix
			MeshChunk& currentChunk)
		{
			/// --- Find POSITION accessor
			auto itPos = primitive.attributes.find("POSITION");
			if (itPos == primitive.attributes.end())
				return;

			/// After POSITION Accessor, gettting BufferView and Buffer
			const int posAccIdx = itPos->second;
			if (posAccIdx < 0 || posAccIdx >= (int)model.accessors.size()) return;
			const tinygltf::Accessor& posAcc = model.accessors[posAccIdx];
			if (posAcc.type != TINYGLTF_TYPE_VEC3 || posAcc.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT) return;
			if (posAcc.bufferView < 0 || posAcc.bufferView >= (int)model.bufferViews.size()) return;
			const tinygltf::BufferView& posView = model.bufferViews[posAcc.bufferView];
			if (posView.buffer < 0 || posView.buffer >= (int)model.buffers.size()) return;
			const tinygltf::Buffer& posBuf = model.buffers[posView.buffer];

			const unsigned char* posBase = posBuf.data.data() + posView.byteOffset + posAcc.byteOffset;
			int posStride = posAcc.ByteStride(posView) ? posAcc.ByteStride(posView) : sizeof(float) * 3;

			/// Ladmda func reading positions
			auto readPosition = [&](size_t idx) -> pctree::XYZPoint {
				const float* p = reinterpret_cast<const float*>(posBase + posStride * idx);
				/// Vertexs in a local coordinate system
				pctree::XYZPoint p0_local(p[0], p[1], p[2]);
				/// Vertex in a world coordinate system (인자로 받은 WorldMatrix 사용)
				pctree::XYZPoint p0 = TransformPoint(worldMatrix, p0_local);
				return p0;
				};

			/// Index Handling
			/// idxBase, idxStride, idxType, indexCount

			const unsigned char* idxBase = nullptr;
			int idxStride = 0;
			int idxType = -1;
			size_t vertexCount = posAcc.count;
			size_t indexCount = vertexCount;

			if (primitive.indices >= 0) {
				const int idxAccIdx = primitive.indices;
				if (idxAccIdx < 0 || idxAccIdx >= (int)model.accessors.size()) return;
				const tinygltf::Accessor& idxAcc = model.accessors[idxAccIdx];

				if (idxAcc.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE &&
					idxAcc.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT &&
					idxAcc.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) return;

				if (idxAcc.bufferView < 0 || idxAcc.bufferView >= (int)model.bufferViews.size()) return;
				const tinygltf::BufferView& idxView = model.bufferViews[idxAcc.bufferView];

				if (idxView.buffer < 0 || idxView.buffer >= (int)model.buffers.size()) return;
				const tinygltf::Buffer& idxBuf = model.buffers[idxView.buffer];

				idxBase = idxBuf.data.data() + idxView.byteOffset + idxAcc.byteOffset;
				idxStride = idxAcc.ByteStride(idxView);
				idxType = idxAcc.componentType;
				indexCount = idxAcc.count;

				if (idxStride == 0) {
					if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) idxStride = 2;
					else if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) idxStride = 4;
					else idxStride = 1;
				}
			}

			auto readIndex = [&](size_t i) -> size_t {
				if (!idxBase) return i; /// Non-indexed
				const unsigned char* ptr = idxBase + idxStride * i;
				if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) return (size_t)(*reinterpret_cast<const uint16_t*>(ptr));
				if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) return (size_t)(*reinterpret_cast<const uint32_t*>(ptr));
				return (size_t)(*reinterpret_cast<const uint8_t*>(ptr));
				};

			/// Traverse faces and extract vertices
			std::unordered_map<size_t, unsigned int> globalToLocalVtxMap;
			for (size_t i = 0; i + 2 < indexCount; i += 3) {
				unsigned int localIndices[3];
				for (int k = 0; k < 3; ++k) {
					size_t gltfIndex = readIndex(i + k);
					if (gltfIndex >= posAcc.count) continue;

					auto it = globalToLocalVtxMap.find(gltfIndex);
					if (it != globalToLocalVtxMap.end()) {
						localIndices[k] = it->second;
					}
					else {
						pctree::XYZPoint pWorld = readPosition(gltfIndex);
						currentChunk.vertices.push_back(pWorld);

						unsigned int newIndex = (unsigned int)(currentChunk.vertices.size() - 1);
						globalToLocalVtxMap[gltfIndex] = newIndex;
						localIndices[k] = newIndex;
					}
				}

				auto vtx0 = currentChunk.vertices[localIndices[0]];
				auto vtx1 = currentChunk.vertices[localIndices[1]];
				auto vtx2 = currentChunk.vertices[localIndices[2]];
				pctree::XYZPoint u = vtx1 - vtx0;
				pctree::XYZPoint v = vtx2 - vtx0;

				auto faceNormal = u % v;
				faceNormal.normalize();

				currentChunk.faces.push_back(FaceVtx(localIndices[0], localIndices[1], localIndices[2], faceNormal));
			}
		}

		bool ExtractDefaultSceneNodes(tinygltf::Model& model,
			std::vector<MeshChunk>& meshChunks,
			const int sceneIndex) {
			tinygltf::TinyGLTF loader;
			std::string err, warn;

			const tinygltf::Scene& scene = model.scenes[sceneIndex];
			Mat4 identity = Mat4Identity();

			/// Extract recursively geometry for root nodes
			for (int nodeIndex : scene.nodes) {
				if (nodeIndex >= 0 && nodeIndex < (int)model.nodes.size()) {
					ExtractMeshFromNode(model, nodeIndex, identity, meshChunks);
				}
			}

			return true;
		}

		void ExtractMeshFromNode(
			const tinygltf::Model& model,
			int nodeIndex,
			const Mat4& parentWorld,
			std::vector<MeshChunk>& meshChunks)
		{
			const tinygltf::Node& node = model.nodes[nodeIndex];

			/// ---1. Matrix calculation ---
			/// Local matrix for the current node
			Mat4 local = LocalMatrixFromNode(node);
			/// Current node world: Parent world * local
			Mat4 world = Mat4Multiply(parentWorld, local);

			/// if Node has mesh data
			if (node.mesh >= 0 && node.mesh < (int)model.meshes.size()) {
				const tinygltf::Mesh& mesh = model.meshes[node.mesh];

				meshChunks.push_back(MeshChunk());
				MeshChunk& currentChunk = meshChunks.back();
				currentChunk.name = mesh.name;
				currentChunk.id = defauMeshID;
				if (mesh.extras.IsObject()) {
					const auto& extrasMap = mesh.extras.Get<tinygltf::Value::Object>();
					auto itExtras = extrasMap.find("id");
					if (itExtras != extrasMap.end()) {
						const auto& extValue = itExtras->second;
						if (extValue.IsInt()) {
							currentChunk.id = extValue.Get<int>();
						}
						else {
							std::cerr << "Mesh id is not a number value in the extras\n";
						}
					}
				}

				/// Map for collecting global and local indices
				/// Use hash map instead of vector for performance
				std::unordered_map<size_t, unsigned int> globalToLocalVtxMap;

				for (const auto& primitive : mesh.primitives) {
					/// Process only TRIANGLES
					if (primitive.mode != TINYGLTF_MODE_TRIANGLES)
						continue;

					/// --- 2. Find POSITION accessor ---
					auto itPos = primitive.attributes.find("POSITION");
					if (itPos == primitive.attributes.end())
						continue;

					/// Position accesor
					const int posAccIdx = itPos->second;
					if (posAccIdx < 0 || posAccIdx >= (int)model.accessors.size())
						continue;
					const tinygltf::Accessor& posAcc = model.accessors[posAccIdx];

					/// Check POSITION accessor type (VEC3 + FLOAT)
					if (posAcc.type != TINYGLTF_TYPE_VEC3 || posAcc.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT)
						continue;

					if (posAcc.bufferView < 0 || posAcc.bufferView >= (int)model.bufferViews.size())
						continue;
					const tinygltf::BufferView& posView = model.bufferViews[posAcc.bufferView];

					if (posView.buffer < 0 || posView.buffer >= (int)model.buffers.size())
						continue;
					const tinygltf::Buffer& posBuf = model.buffers[posView.buffer];

					const unsigned char* posBase = posBuf.data.data() + posView.byteOffset + posAcc.byteOffset;
					int posStride = posAcc.ByteStride(posView) ? posAcc.ByteStride(posView) : sizeof(float) * 3;

					/// Ladmda func reading positions
					auto readPosition = [&](size_t idx) -> pctree::XYZPoint {
						const float* p = reinterpret_cast<const float*>(posBase + posStride * idx);
						/// Vertexs in a local coordinate system
						pctree::XYZPoint p0_local(p[0], p[1], p[2]);
						/// Vertex in a world coordinate system
						pctree::XYZPoint p0 = TransformPoint(world, p0_local);
						return p0;
						};

					/// --- 3. Index Handling ---
					/// idxBase: first index address
					const unsigned char* idxBase = nullptr;
					int idxStride = 0;
					int idxType = -1;
					size_t vertexCount = posAcc.count;
					size_t indexCount = vertexCount; /// Default to non-indexed

					if (primitive.indices >= 0) {
						const int idxAccIdx = primitive.indices;
						if (idxAccIdx < 0 || idxAccIdx >= (int)model.accessors.size())
							continue;
						const tinygltf::Accessor& idxAcc = model.accessors[idxAccIdx];

						if (idxAcc.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE &&
							idxAcc.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT &&
							idxAcc.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT)
							continue;

						if (idxAcc.bufferView < 0 || idxAcc.bufferView >= (int)model.bufferViews.size())
							continue;
						const tinygltf::BufferView& idxView = model.bufferViews[idxAcc.bufferView];

						if (idxView.buffer < 0 || idxView.buffer >= (int)model.buffers.size())
							continue;
						const tinygltf::Buffer& idxBuf = model.buffers[idxView.buffer];

						idxBase = idxBuf.data.data() + idxView.byteOffset + idxAcc.byteOffset;
						idxStride = idxAcc.ByteStride(idxView);
						idxType = idxAcc.componentType;
						indexCount = idxAcc.count;

						if (idxStride == 0) {
							if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) idxStride = 2;
							else if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) idxStride = 4;
							else idxStride = 1;
						}
					}

					auto readIndex = [&](size_t i) -> size_t {
						if (!idxBase) return i; // Non-indexed
						const unsigned char* ptr = idxBase + idxStride * i;
						if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) return (size_t)(*reinterpret_cast<const uint16_t*>(ptr));
						if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) return (size_t)(*reinterpret_cast<const uint32_t*>(ptr));
						return (size_t)(*reinterpret_cast<const uint8_t*>(ptr));
						};

					/// --- 4. Traverse faces and extract vertices ---
					for (size_t i = 0; i + 2 < indexCount; i += 3) {
						unsigned int localIndices[3];
						/// For 3 vertices of a triangle
						for (int k = 0; k < 3; ++k) {
							size_t gltfIndex = readIndex(i + k);
							if (gltfIndex >= posAcc.count)
								continue;

							/// Duplicate check and insertion using a map
							auto it = globalToLocalVtxMap.find(gltfIndex);
							if (it != globalToLocalVtxMap.end()) {
								/// If the vertex is already registered, reuse its index
								localIndices[k] = it->second;
							}
							else {
								/// Vertex normals are typically used for shading; computing the geometric face normal may be necessary
								/// If the vertex is not registered, register it as a new one
								pctree::XYZPoint pWorld = readPosition(gltfIndex);
								/// add a vertex to chunk data
								currentChunk.vertices.push_back(pWorld);

								unsigned int newIndex = (unsigned int)(currentChunk.vertices.size() - 1);
								globalToLocalVtxMap[gltfIndex] = newIndex; /// Map the global index to the new local index
								localIndices[k] = newIndex;
							}
						}

						/// Compute surface normal (U = p1 - p0, V = p2 - p0)
						auto vtx0 = currentChunk.vertices[localIndices[0]];
						auto vtx1 = currentChunk.vertices[localIndices[1]];
						auto vtx2 = currentChunk.vertices[localIndices[2]];
						pctree::XYZPoint u = vtx1 - vtx0;
						pctree::XYZPoint v = vtx2 - vtx0;

						/// Cross Product
						auto faceNormal = u % v;
						faceNormal.normalize();

						/// add a face to chunk data
						currentChunk.faces.push_back(FaceVtx(localIndices[0], localIndices[1], localIndices[2], faceNormal));
					} /// indexCount
				}
			}

			/// Process child node recursively
			for (int child : node.children) {
				if (child >= 0 && child < (int)model.nodes.size()) {
					ExtractMeshFromNode(model, child, world, meshChunks);
				}
			}
		}
	};
}