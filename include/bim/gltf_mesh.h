#pragma once

#include "gltf.h"
#include "BimMeshInfo.h"

#include "../bimtree/dataStructs.hpp"
#include "./MeshChunk.h"

/// [주의] 이 파일에는 이름이 같은 "GltfMesh" 클래스가 두 개 존재한다 — 서로 다른
/// 클래스이니 혼동하지 말 것.
///   1) 전역 네임스페이스의 ::GltfMesh (바로 아래) — gltf(gltf.h)를 상속하며,
///      Eigen 기반 Matrix4f로 GLTF를 읽어 BimMeshInfo(Point 기반 Face 목록)를 채운다.
///   2) chunkbim::GltfMesh (파일 하단) — Eigen을 쓰지 않고 자체 Mat4(column-major
///      float[16])로 GLTF를 읽어 MeshChunk(FaceVtx 목록, 네트워크 전송용 청크)를 채운다.
/// 서로 완전히 독립적인 구현이며 재사용 관계가 없다.
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
			return false; // 지원하지 않는 파일 형식이면 false 반환(BimMeshInfo는 비워둠)
		}

		if (!warn.empty()) {
			std::cerr << "Warning: " << warn << std::endl;
		}

		if (!err.empty()) {
			std::cerr << "Error: " << err << std::endl;
		}

		if (!fileread) {
			std::cerr << "Failed to parse glTF" << std::endl;
			return false; // 파싱 실패 시 false 반환(BimInfo는 비워둠)
		}

		/// 모든 메시가 공유하는 단일 변환 행렬을 정의
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

		///  주어진 offset을 적용 (좌표 rebase)
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

			/// 여러 primitive로 구성된 새 메시를 추가
			bimInfo.addMesh(mesh.name, normal, v1, v2, v3);
		}

		return true;
	}
};

namespace chunkbim {
	/// column-major 4x4 행렬 (Eigen을 쓰지 않는 이 네임스페이스 전용의 경량 표현)
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

	/// quaternion(x,y,z,w) → 4x4 행렬 (회전만 반영, 이동(translation)은 0)
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
		/// T (이동)
		Mat4 T = Mat4Identity();
		if (t.size() == 3) {
			T.m[12] = float(t[0]);
			T.m[13] = float(t[1]);
			T.m[14] = float(t[2]);
		}

		/// R (회전)
		Mat4 R = Mat4FromQuaternion(r);

		/// S (스케일)
		Mat4 S = Mat4Identity();
		double sx = 1.0, sy = 1.0, sz = 1.0;
		if (s.size() == 3) {
			sx = s[0]; sy = s[1]; sz = s[2];
		}
		S.m[0] = float(sx);
		S.m[5] = float(sy);
		S.m[10] = float(sz);

		/// glTF 규격: M = T * R * S 순서로 합성
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

	/// GLTF/GLB 파일을 읽어 MeshChunk(FaceVtx 목록) 목록으로 변환하는 클래스.
	/// 전역 네임스페이스의 ::GltfMesh(이 파일 상단)와는 이름만 같을 뿐 별개의 구현이며,
	/// Eigen 대신 위에서 정의한 자체 Mat4/TransformPoint 등을 사용한다.
	class GltfMesh {
	public:
///[ToDo] 아래 줄 삭제 예정
#ifdef _DEBUG
		//std::fstream tempfile;
#endif
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
				int mesh_id = -1; // 기본 ID

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

		size_t getNumMeshes(const std::string& filename) {
			tinygltf::Model model;
			tinygltf::TinyGLTF loader;
			std::string err, warn;

			bool ok = false;

			const auto dotPos = filename.find_last_of('.');
			if (dotPos == std::string::npos) {
				std::cerr << "No file extension.\n";
				return size_t(0);
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
				return size_t(0);
			}

			if (!warn.empty()) std::cerr << "Warn: " << warn << "\n";
			if (!err.empty())  std::cerr << "Err : " << err << "\n";
			if (!ok) {
				std::cerr << "Failed to load glTF\n";
				return size_t(0);
			}

			return model.meshes.size();
		}

		bool importMeshData(const std::string& filename, std::vector<MeshChunk>& meshChunks)
		{
///[ToDo] 아래 줄 삭제 예정
#ifdef _DEBUG
			//this->tempfile.open("F:\\out.tmp", std::ios::out);
#endif

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

			//bool retval = false;
			//retval = ExtractAllMeshes(model, meshChunks);

			/// Scene Graph 방식으로 추출 (수정)
			if (model.scenes.empty()) {
				return ExtractAllMeshes(model, meshChunks);  // fallback
			}

			int sceneIndex = model.defaultScene >= 0 ? model.defaultScene : 0;
			const tinygltf::Scene& scene = model.scenes[sceneIndex];
			Mat4 identity = Mat4Identity();

			for (int nodeIndex : scene.nodes) {
				if (nodeIndex >= 0 && nodeIndex < (int)model.nodes.size()) {
					ExtractMeshFromNode(model, nodeIndex, identity, meshChunks);
				}
			}

///[ToDo] 아래 줄 삭제 예정
#ifdef _DEBUG
			//this->tempfile.close();
#endif

			/// [ToDo] 사용되지 않음; 추후 삭제
			/*
			switch (extractionOption) {
			case 0:
				retval = ExtractAllMeshes(model, meshChunks);
				break;
			case 1:
				if (model.scenes.empty()) {
					std::cerr << "No scene in " << filename << "\n";
					return false;
				}

				/// 기본(default) 씬을 선택, 없으면 첫 번째 씬을 선택
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
			*/

			return !meshChunks.empty();
		}

	public:
		inline static void to_lower(std::string& ext) {
			std::transform(ext.begin(), ext.end(), ext.begin(),
				[](unsigned char c) { return std::tolower(c); });
		}

	private:
		/// mesh.extras에서 "centerline" 키(로컬 mesh 좌표계 기준 [x1,y1,z1,x2,y2,z2]
		/// 6개 숫자 배열)를 읽어, 이 mesh가 속한 노드의 world 변환을 동일하게 적용한 뒤
		/// chunk.centerlineStart/End에 채운다. 성공하면 chunk.hasCenterline을 true로
		/// 설정하고 true를 반환한다. 키가 없거나 형식이 올바르지 않으면 아무것도
		/// 바꾸지 않고 false를 반환한다 -- 호출부는 이 경우 PCA 축 추정으로 폴백해야 한다.
		/// [참고] 이 스키마는 아직 어떤 Revit/GLTF export 파이프라인도 채워 보내지
		/// 않는, 이번에 새로 정의한 잠정적인 규격이다. 실제 export 쪽 구현이 준비되면
		/// 키 이름/형식을 그에 맞게 조정해야 할 수 있다.
		static bool tryReadCenterlineFromExtras(const tinygltf::Value& extras, const Mat4& world, MeshChunk& chunk)
		{
			if (!extras.IsObject()) return false;
			const auto& extrasMap = extras.Get<tinygltf::Value::Object>();
			auto it = extrasMap.find("centerline");
			if (it == extrasMap.end()) return false;

			const auto& v = it->second;
			if (!v.IsArray() || v.ArrayLen() != 6) {
				std::cerr << "Mesh centerline extras is present but is not a 6-element array\n";
				return false;
			}

			double raw[6];
			for (size_t i = 0; i < 6; ++i) {
				const auto& elem = v.Get(i);
				if (!elem.IsNumber()) {
					std::cerr << "Mesh centerline extras contains a non-numeric element\n";
					return false;
				}
				raw[i] = elem.GetNumberAsDouble();
			}

			/// 정점과 동일하게, 로컬 좌표를 world 변환에 통과시켜 MeshChunk::faces와
			/// 같은(월드/절대) 좌표계로 맞춘다.
			pctree::XYZPoint p0Local(static_cast<float>(raw[0]), static_cast<float>(raw[1]), static_cast<float>(raw[2]));
			pctree::XYZPoint p1Local(static_cast<float>(raw[3]), static_cast<float>(raw[4]), static_cast<float>(raw[5]));
			pctree::XYZPoint p0World = TransformPoint(world, p0Local);
			pctree::XYZPoint p1World = TransformPoint(world, p1Local);

			chunk.centerlineStart = { p0World.x(), p0World.y(), p0World.z() };
			chunk.centerlineEnd = { p1World.x(), p1World.y(), p1World.z() };
			chunk.hasCenterline = true;
			return true;
		}

		bool ExtractAllMeshes(tinygltf::Model& model,
			std::vector<MeshChunk>& meshChunks) {
			auto worldMatrix = Mat4Identity();

			/// 파일 내 모든 메시를 순회
			for (const auto& mesh : model.meshes) {
				meshChunks.push_back(MeshChunk());
				MeshChunk& currentChunk = meshChunks.back();
				currentChunk.name = mesh.name;
				currentChunk.id = defauMeshID;

				/// id 추출
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

				/// centerline 추출 (제공되는 경우) -- 없으면 나중에 PCA로 축을 추정한다
				tryReadCenterlineFromExtras(mesh.extras, worldMatrix, currentChunk);

				for (const auto& primitive : mesh.primitives) {
					if (primitive.mode != TINYGLTF_MODE_TRIANGLES)
						continue;

					/// primitive 데이터 추출
					ExtractMeshPrimitive(model, primitive, worldMatrix, currentChunk);
				}
			}

			return true;
		}

		void ExtractMeshPrimitive(
			const tinygltf::Model& model,
			const tinygltf::Primitive& primitive,
			const Mat4& worldMatrix,
			MeshChunk& currentChunk)
		{
			/// --- POSITION accessor 탐색 ---
			auto itPos = primitive.attributes.find("POSITION");
			if (itPos == primitive.attributes.end())
				return;

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

			/// --- 인덱스 처리 ---
			if (primitive.indices < 0) return;  /// 인덱스가 없는 메시는 여기서 지원하지 않음

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

			const unsigned char* idxBase = idxBuf.data.data() + idxView.byteOffset + idxAcc.byteOffset;
			int idxStride = idxAcc.ByteStride(idxView);
			int idxType = idxAcc.componentType;
			size_t indexCount = idxAcc.count;

			if (idxStride == 0) {
				if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) idxStride = 2;
				else if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) idxStride = 4;
				else idxStride = 1;
			}

			/// 람다: 인덱스 읽기
			auto readIndex = [&](size_t i) -> size_t {
				const unsigned char* ptr = idxBase + idxStride * i;
				if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT)
					return (size_t)(*reinterpret_cast<const uint16_t*>(ptr));
				if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT)
					return (size_t)(*reinterpret_cast<const uint32_t*>(ptr));
				return (size_t)(*reinterpret_cast<const uint8_t*>(ptr));
				};

			/// 람다: 위치를 읽어 월드 좌표로 변환
			auto readPosition = [&](size_t idx) -> pctree::XYZPoint {
				const float* p = reinterpret_cast<const float*>(posBase + posStride * idx);
				pctree::XYZPoint p_local(p[0], p[1], p[2]);
				return TransformPoint(worldMatrix, p_local);
				};

			/// --- 삼각형을 바로 생성 ---
			for (size_t i = 0; i + 2 < indexCount; i += 3) {
				size_t idx0 = readIndex(i);
				size_t idx1 = readIndex(i + 1);
				size_t idx2 = readIndex(i + 2);

				if (idx0 >= posAcc.count || idx1 >= posAcc.count || idx2 >= posAcc.count)
					continue;

				/// 정점 위치를 바로 읽기
				pctree::XYZPoint vtx0 = readPosition(idx0);
				pctree::XYZPoint vtx1 = readPosition(idx1);
				pctree::XYZPoint vtx2 = readPosition(idx2);

///[ToDo] 아래 줄 삭제 예정
#ifdef _DEBUG
				//this->tempfile.precision(6);
				//this->tempfile << vtx0[0] << "\t" << vtx0[1] << "\t" << vtx0[2] << "\n";
				//this->tempfile << vtx1[0] << "\t" << vtx1[1] << "\t" << vtx1[2] << "\n";
				//this->tempfile << vtx2[0] << "\t" << vtx2[1] << "\t" << vtx2[2] << "\n";
#endif

				/// face normal 계산 (외적)
				pctree::XYZPoint u = vtx1 - vtx0;
				pctree::XYZPoint v = vtx2 - vtx0;
				pctree::XYZPoint faceNormal = u % v;
				faceNormal = faceNormal.normalize();

				/// face를 바로 추가 (정점을 별도로 저장하지 않음)
				currentChunk.faces.push_back(FaceVtx(vtx0, vtx1, vtx2, faceNormal));
			}
		}

		inline pctree::XYZPoint TransformNormal(const Mat4& m, const pctree::XYZPoint& n) {
			/// 3x3 회전 성분만 추출해 적용 (법선 벡터이므로 이동은 반영하지 않음)
			double nx = m.m[0] * n[0] + m.m[4] * n[1] + m.m[8] * n[2];
			double ny = m.m[1] * n[0] + m.m[5] * n[1] + m.m[9] * n[2];
			double nz = m.m[2] * n[0] + m.m[6] * n[1] + m.m[10] * n[2];

			/// 정규화
			double len = std::sqrt(nx * nx + ny * ny + nz * nz);
			if (len > 1e-10) {
				return pctree::XYZPoint(nx / len, ny / len, nz / len);
			}
			return pctree::XYZPoint(0, 0, 1);  /// 실패 시 대체값(fallback)
		}

		void ExtractMeshFromNode(
			const tinygltf::Model& model,
			int nodeIndex,
			const Mat4& parentWorld,
			std::vector<MeshChunk>& meshChunks)
		{
			const tinygltf::Node& node = model.nodes[nodeIndex];

			/// ---1. 행렬 계산 ---
			Mat4 local = LocalMatrixFromNode(node);
			Mat4 world = Mat4Multiply(parentWorld, local);

			/// 노드에 메시 데이터가 있으면
			if (node.mesh >= 0 && node.mesh < (int)model.meshes.size()) {
				const tinygltf::Mesh& mesh = model.meshes[node.mesh];

				meshChunks.push_back(MeshChunk());
				MeshChunk& currentChunk = meshChunks.back();
				currentChunk.name = mesh.name;
				currentChunk.id = defauMeshID;

				/// extras에서 id 추출
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

				/// centerline 추출 (제공되는 경우) -- 없으면 나중에 PCA로 축을 추정한다
				tryReadCenterlineFromExtras(mesh.extras, world, currentChunk);

				/// 각 primitive 처리
				for (const auto& primitive : mesh.primitives) {
					if (primitive.mode != TINYGLTF_MODE_TRIANGLES)
						continue;

					/// --- POSITION accessor 탐색 ---
					auto itPos = primitive.attributes.find("POSITION");
					if (itPos == primitive.attributes.end())
						continue;

					const int posAccIdx = itPos->second;
					if (posAccIdx < 0 || posAccIdx >= (int)model.accessors.size())
						continue;
					const tinygltf::Accessor& posAcc = model.accessors[posAccIdx];

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

					/// --- NORMAL accessor 탐색 (선택적) ---
					const unsigned char* normalBase = nullptr;
					int normalStride = 0;
					bool hasNormals = false;

					auto itNormal = primitive.attributes.find("NORMAL");
					if (itNormal != primitive.attributes.end()) {
						const int normalAccIdx = itNormal->second;
						if (normalAccIdx >= 0 && normalAccIdx < (int)model.accessors.size()) {
							const tinygltf::Accessor& normalAcc = model.accessors[normalAccIdx];

							if (normalAcc.type == TINYGLTF_TYPE_VEC3 &&
								normalAcc.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT &&
								normalAcc.bufferView >= 0 &&
								normalAcc.bufferView < (int)model.bufferViews.size()) {

								const tinygltf::BufferView& normalView = model.bufferViews[normalAcc.bufferView];

								if (normalView.buffer >= 0 && normalView.buffer < (int)model.buffers.size()) {
									const tinygltf::Buffer& normalBuf = model.buffers[normalView.buffer];
									normalBase = normalBuf.data.data() + normalView.byteOffset + normalAcc.byteOffset;
									normalStride = normalAcc.ByteStride(normalView) ? normalAcc.ByteStride(normalView) : sizeof(float) * 3;
									hasNormals = true;
								}
							}
						}
					}

					/// --- 인덱스 처리 ---
					if (primitive.indices < 0)
						continue;

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

					const unsigned char* idxBase = idxBuf.data.data() + idxView.byteOffset + idxAcc.byteOffset;
					int idxStride = idxAcc.ByteStride(idxView);
					int idxType = idxAcc.componentType;
					size_t indexCount = idxAcc.count;

					if (idxStride == 0) {
						if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) idxStride = 2;
						else if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) idxStride = 4;
						else idxStride = 1;
					}

					/// 람다: 인덱스 읽기
					auto readIndex = [&](size_t i) -> size_t {
						const unsigned char* ptr = idxBase + idxStride * i;
						if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT)
							return (size_t)(*reinterpret_cast<const uint16_t*>(ptr));
						if (idxType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT)
							return (size_t)(*reinterpret_cast<const uint32_t*>(ptr));
						return (size_t)(*reinterpret_cast<const uint8_t*>(ptr));
						};

					/// 람다: 위치를 읽어 월드 좌표로 변환
					auto readPosition = [&](size_t idx) -> pctree::XYZPoint {
						const float* p = reinterpret_cast<const float*>(posBase + posStride * idx);
						pctree::XYZPoint p_local(p[0], p[1], p[2]);
						return TransformPoint(world, p_local);
						};

					/// 람다: 법선을 읽어 월드 좌표로 변환 (회전만 적용)
					auto readNormal = [&](size_t idx) -> pctree::XYZPoint {
						const float* n = reinterpret_cast<const float*>(normalBase + normalStride * idx);
						pctree::XYZPoint n_local(n[0], n[1], n[2]);
						/// 법선 변환 (회전만 적용, 이동은 반영하지 않음)
						return TransformNormal(world, n_local);
						};

					/// --- 삼각형을 바로 생성 ---
					for (size_t i = 0; i + 2 < indexCount; i += 3) {
						size_t idx0 = readIndex(i);
						size_t idx1 = readIndex(i + 1);
						size_t idx2 = readIndex(i + 2);

						if (idx0 >= posAcc.count || idx1 >= posAcc.count || idx2 >= posAcc.count)
							continue;

						/// 정점 위치를 바로 읽기
						pctree::XYZPoint vtx0 = readPosition(idx0);
						pctree::XYZPoint vtx1 = readPosition(idx1);
						pctree::XYZPoint vtx2 = readPosition(idx2);

						double x0 = static_cast<double>(vtx0[0]);
						double y0 = static_cast<double>(vtx0[1]);
						double z0 = static_cast<double>(vtx0[2]);

						double x1 = static_cast<double>(vtx1[0]);
						double y1 = static_cast<double>(vtx1[1]);
						double z1 = static_cast<double>(vtx1[2]);

						double x2 = static_cast<double>(vtx2[0]);
						double y2 = static_cast<double>(vtx2[1]);
						double z2 = static_cast<double>(vtx2[2]);

						double ux = x1 - x0, uy = y1 - y0, uz = z1 - z0;
						double vx = x2 - x0, vy = y2 - y0, vz = z2 - z0;

						double nx = uy * vz - uz * vy;
						double ny = uz * vx - ux * vz;
						double nz = ux * vy - uy * vx;

						double lengthSq = nx * nx + ny * ny + nz * nz;

						pctree::XYZPoint faceNormal;
						constexpr double EPSILON_SQ = 1e-12;
						if (lengthSq > EPSILON_SQ) {
							/// 정규화 후 float로 변환
							double invLen = 1.0 / std::sqrt(lengthSq);
							faceNormal = pctree::XYZPoint(
								static_cast<float>(nx * invLen),
								static_cast<float>(ny * invLen),
								static_cast<float>(nz * invLen)
							);
						}
						else if (false) {
						//else if (hasNormals) {
							/// 퇴화 삼각형(면적이 0에 가까움) - 메시에 저장된 정점 법선으로 대체
							/// [참고] 위 조건이 `if (false)`로 비활성화되어 있어, 이 블록은 현재
							/// 절대 실행되지 않는다. 원래는 `hasNormals`였을 것으로 보이며(바로
							/// 아래 주석 처리된 줄), 의도적으로 막아둔 것인지 실수인지는 불명확
							/// 하다 — 현재 동작(퇴화 삼각형은 무조건 스킵)은 유지한다.
							pctree::XYZPoint n0 = readNormal(idx0);
							pctree::XYZPoint n1 = readNormal(idx1);
							pctree::XYZPoint n2 = readNormal(idx2);

							/// 정점 법선들의 평균 (double 정밀도)
							double avgX = (static_cast<double>(n0[0]) + n1[0] + n2[0]) / 3.0;
							double avgY = (static_cast<double>(n0[1]) + n1[1] + n2[1]) / 3.0;
							double avgZ = (static_cast<double>(n0[2]) + n1[2] + n2[2]) / 3.0;

							double avgLengthSq = avgX * avgX + avgY * avgY + avgZ * avgZ;
							if (avgLengthSq > EPSILON_SQ) {
								double invLen = 1.0 / std::sqrt(avgLengthSq);
								faceNormal = pctree::XYZPoint(
									static_cast<float>(avgX * invLen),
									static_cast<float>(avgY * invLen),
									static_cast<float>(avgZ * invLen)
								);
							}
							else {
								/// 두 방법 모두 실패 - 이 face는 건너뜀
								continue;
							}
						}
						else {
							/// 사용 가능한 메시 법선이 없음 - 퇴화 face는 건너뜀
							continue;
						}

						/// face를 바로 추가 (정점을 별도로 저장하지 않음)
						currentChunk.faces.push_back(FaceVtx(vtx0, vtx1, vtx2, faceNormal));
					}
				}
			}

			/// 자식 노드도 재귀적으로 처리
			for (int child : node.children) {
				if (child >= 0 && child < (int)model.nodes.size()) {
					ExtractMeshFromNode(model, child, world, meshChunks);
				}
			}
		}
	};

}
