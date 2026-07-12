#pragma once

#include <string>

#include "dataStructs.hpp"
#include "pc3d.h"
#include "PseudoPtsInMesh.hpp"
#include "xyzpoint.hpp"

namespace chunkbim /// 이 클래스를 사용할 것 (MeshData가 이 네임스페이스의 대표 메시 표현임)
{
	class MeshData
	{
	public:
		std::vector<Face> faces;
		pctree::PointCloudType3D pc;
		std::shared_ptr<pctree::kd_tree3D> meshTree;
		//size_t numVertices;
		//FILE* idxFile;

	private:
		/// XYZPoint 직렬화
		void serializeXYZPoint(std::ostream& out, const pctree::XYZPoint& point) const {
			out.write(reinterpret_cast<const char*>(&point.xyz), sizeof(point.xyz));
			size_t fIdsSize = point.fIds.size();
			out.write(reinterpret_cast<const char*>(&fIdsSize), sizeof(fIdsSize));
			out.write(reinterpret_cast<const char*>(point.fIds.data()), fIdsSize * sizeof(size_t));
		}

		/// XYZPoint 역직렬화
		void deserializeXYZPoint(std::istream& in, pctree::XYZPoint& point) {
			in.read(reinterpret_cast<char*>(&point.xyz), sizeof(point.xyz));
			size_t fIdsSize;
			in.read(reinterpret_cast<char*>(&fIdsSize), sizeof(fIdsSize));
			point.fIds.resize(fIdsSize);
			in.read(reinterpret_cast<char*>(point.fIds.data()), fIdsSize * sizeof(size_t));
		}

		/// MatrixForMesh 직렬화
		void serializeMatrixForMesh(std::ostream& out, const pctree::MatrixForMesh& matrix) const {
			out.write(reinterpret_cast<const char*>(&matrix.r), sizeof(matrix.r));
		}

		/// MatrixForMesh 역직렬화
		void deserializeMatrixForMesh(std::istream& in, pctree::MatrixForMesh& matrix) {
			in.read(reinterpret_cast<char*>(&matrix.r), sizeof(matrix.r));
		}

		/// TempBaryCentVar 직렬화
		void serializeTempBaryCentVar(std::ostream& out, const geo::TempBaryCentVar& baryTemp) const {
			serializeXYZPoint(out, baryTemp.v0);
			serializeXYZPoint(out, baryTemp.v1);
			out.write(reinterpret_cast<const char*>(&baryTemp.dot00), sizeof(baryTemp.dot00));
			out.write(reinterpret_cast<const char*>(&baryTemp.dot11), sizeof(baryTemp.dot11));
			out.write(reinterpret_cast<const char*>(&baryTemp.dot01), sizeof(baryTemp.dot01));
			out.write(reinterpret_cast<const char*>(&baryTemp.invDenom), sizeof(baryTemp.invDenom));
		}

		/// TempBaryCentVar 역직렬화
		void deserializeTempBaryCentVar(std::istream& in, geo::TempBaryCentVar& baryTemp) {
			deserializeXYZPoint(in, baryTemp.v0);
			deserializeXYZPoint(in, baryTemp.v1);
			in.read(reinterpret_cast<char*>(&baryTemp.dot00), sizeof(baryTemp.dot00));
			in.read(reinterpret_cast<char*>(&baryTemp.dot11), sizeof(baryTemp.dot11));
			in.read(reinterpret_cast<char*>(&baryTemp.dot01), sizeof(baryTemp.dot01));
			in.read(reinterpret_cast<char*>(&baryTemp.invDenom), sizeof(baryTemp.invDenom));
		}

		/// Face 직렬화
		void serializeFace(std::ostream& out, const Face& face) const {
			out.write(reinterpret_cast<const char*>(&face.partId), sizeof(face.partId));
			out.write(reinterpret_cast<const char*>(face.vtxIds.data()), face.vtxIds.size() * sizeof(int));

			size_t pseudoPtsIdsSize = face.pseudoPtsIds.size();
			out.write(reinterpret_cast<const char*>(&pseudoPtsIdsSize), sizeof(pseudoPtsIdsSize));
			out.write(reinterpret_cast<const char*>(face.pseudoPtsIds.data()), pseudoPtsIdsSize * sizeof(int));

			serializeMatrixForMesh(out, face.planeOri);
			serializeTempBaryCentVar(out, face.baryTemp);

			out.write(reinterpret_cast<const char*>(&face.rotAvailability), sizeof(face.rotAvailability));
			out.write(reinterpret_cast<const char*>(&face.availability), sizeof(face.availability));
			out.write(reinterpret_cast<const char*>(&face.baryAvailability), sizeof(face.baryAvailability));
		}

		/// Face 역직렬화
		void deserializeFace(std::istream& in, Face& face) {
			in.read(reinterpret_cast<char*>(&face.partId), sizeof(face.partId));
			in.read(reinterpret_cast<char*>(face.vtxIds.data()), face.vtxIds.size() * sizeof(int));

			size_t pseudoPtsIdsSize;
			in.read(reinterpret_cast<char*>(&pseudoPtsIdsSize), sizeof(pseudoPtsIdsSize));
			face.pseudoPtsIds.resize(pseudoPtsIdsSize);
			in.read(reinterpret_cast<char*>(face.pseudoPtsIds.data()), pseudoPtsIdsSize * sizeof(int));

			deserializeMatrixForMesh(in, face.planeOri);
			deserializeTempBaryCentVar(in, face.baryTemp);

			in.read(reinterpret_cast<char*>(&face.rotAvailability), sizeof(face.rotAvailability));
			in.read(reinterpret_cast<char*>(&face.availability), sizeof(face.availability));
			in.read(reinterpret_cast<char*>(&face.baryAvailability), sizeof(face.baryAvailability));
		}

	public:

		/// 기존 메시에 새 face/vertex들을 추가한다. 추가되는 데이터의 인덱스는
		/// 항상 0부터 시작하므로, 기존에 이미 들어있던 개수만큼 오프셋을
		/// 더해줘야 전체 메시 기준의 올바른 인덱스가 된다.
		void addMeshData(std::vector<Face>& addedFaces, std::vector<pctree::XYZPoint>& addedVertices)
		{
			/// 추가되는 데이터의 id를 갱신 (기존 데이터 개수만큼 오프셋)
			unsigned int lastNumPts = static_cast<int>(this->pc.points.size());
			unsigned int lastNumFaces = static_cast<int>(this->faces.size());

			if (lastNumPts > 0) {
				for (auto& f : addedFaces) {
					f.vtxIds[0] += lastNumPts;
					f.vtxIds[1] += lastNumPts;
					f.vtxIds[2] += lastNumPts;

					for (auto& p : f.pseudoPtsIds)
						p += lastNumPts;
				}
			}

			if (lastNumFaces > 0) {
				for (auto& p : addedVertices) {
					for (auto& fid : p.fIds)
						fid += lastNumFaces;
				}
			}

			/// 삽입
			this->faces.insert(this->faces.end(), addedFaces.begin(), addedFaces.end());
			this->pc.points.insert(this->pc.points.end(), addedVertices.begin(), addedVertices.end());

#ifdef _DEBUG
			std::cout << addedFaces.size() << " faces and " << pc.points.size() << " vertices are added\n";
			std::cout << "For now, " << this->faces.size() << " faces and " << this->pc.points.size() << " vertices\n";
#endif
		}

		/// 포인트클라우드(pc) 저장
		bool savePc(const std::string& filename) const {
			std::ofstream ofs(filename, std::ios::binary | std::ios::out);
			if (!ofs) {
				std::cerr << "Error opening file for writing: " << filename << std::endl;
				return false;
			}

			const std::vector<pctree::XYZPoint>& points = this->pc.points;

			size_t numStructs = points.size();
			ofs.write(reinterpret_cast<const char*>(&numStructs), sizeof(numStructs));

			for (const auto& data : points) {
				ofs.write(reinterpret_cast<const char*>(data.xyz), sizeof(data.xyz));

				size_t size = data.fIds.size();
				ofs.write(reinterpret_cast<const char*>(&size), sizeof(size));
				ofs.write(reinterpret_cast<const char*>(data.fIds.data()), size * sizeof(size_t));
			}

			ofs.close();

			return true;
		}

		/// 포인트클라우드(pc) 로딩
		bool loadPc(const std::string& filename) {
			std::ifstream ifs(filename, std::ios::binary | std::ios::in);
			if (!ifs) {
				std::cerr << "Error opening file for reading: " << filename << std::endl;
				return false;
			}

			size_t numStructs = 0;
			ifs.read(reinterpret_cast<char*>(&numStructs), sizeof(numStructs));

			std::vector<pctree::XYZPoint>& points = this->pc.points;
			points.resize(numStructs);
			for (auto& data : points) {
				ifs.read(reinterpret_cast<char*>(data.xyz), sizeof(data.xyz));

				size_t size = 0;
				ifs.read(reinterpret_cast<char*>(&size), sizeof(size));
				data.fIds.resize(size);
				ifs.read(reinterpret_cast<char*>(data.fIds.data()), size * sizeof(size_t));
			}

			ifs.close();
			return true;
		}

		/// face 목록 저장
		bool saveFaces(const std::string& filename) const {
			std::ofstream outFile(filename, std::ios::binary);
			if (!outFile) {
				std::cerr << "Failed to open output file" << std::endl;
				return false;
			}

			size_t numFaces = faces.size();
			outFile.write(reinterpret_cast<const char*>(&numFaces), sizeof(numFaces));

			std::streamsize bufferSize = 1024 * 1024; // 1MB buffer
			char* buffer = new char[bufferSize];
			std::streamsize bufferIndex = 0;

			for (const auto& face : faces) {
				std::ostringstream oss;
				serializeFace(oss, face);
				std::string data = oss.str();
				std::streamsize dataSize = data.size();

				if (bufferIndex + dataSize > bufferSize) {
					outFile.write(buffer, bufferIndex);
					bufferIndex = 0;
				}

				std::memcpy(buffer + bufferIndex, data.data(), dataSize);
				bufferIndex += dataSize;
			}

			if (bufferIndex > 0) {
				outFile.write(buffer, bufferIndex);
			}

			delete[] buffer;
			outFile.close();

			return true;
		}

		/// face 목록 로딩
		bool loadFaces(const std::string& filename) {
			std::ifstream inFile(filename, std::ios::binary);
			if (!inFile) {
				std::cerr << "Failed to open input file" << std::endl;
				return false;
			}

			size_t numFaces;
			inFile.read(reinterpret_cast<char*>(&numFaces), sizeof(numFaces));
			this->faces.resize(numFaces);

			for (auto& face : this->faces) {
				deserializeFace(inFile, face);
			}

			inFile.close();
		}

		/// 가상점(pseudo point) 생성
		void generatePseudoPts(const float lengthTh)
		{
			geo::generatePseudoGridPts(this->faces, this->pc.points, lengthTh);
		}

		/// KD-Tree 구축
		bool buildTree()
		{
			if (!pctree::buildKdtree3D(this->pc, meshTree))
				return false;
			else
			{
#ifdef _DEBUG
				std::cout << "num pts in tree: " << meshTree->size() << "\n";
#endif
				return true;
			}
		}

		bool exportTree(const std::string& treePath)
		{
			std::clog << "Exporting tree...\n";
			if (pctree::saveTree3D(this->meshTree, treePath))
				return true;
			else
				return false;
		}

		bool loadTree(const std::string& treePath)
		{
			if (pctree::loadKdtree3D(this->meshTree, this->pc, treePath))
				return true;
			else
				return false;

		}

		/// 트리와 메시를 함께 내보내기(export)
		bool exportTreeAndMesh(const std::string& treePath)
		{
			//pctree::
			std::clog << "Exporting tree...\n";
			if (!pctree::saveTree3D(this->meshTree, treePath))
				return false;


			if (!savePc(treePath + std::string(".meshpc"))) {
				std::cerr << "Failure in saving point cloud in mesh.\n";
				return false;
			}

			if (!saveFaces(treePath + std::string(".meshfaces"))) {
				std::cerr << "Failure in saving point cloud in mesh.\n";
				return false;
			}

			return true;
		}

		/// queryPt에 가장 가까운 face를 찾는다. 1) KD-Tree로 queryPt에 가까운 정점
		/// N개를 찾고, 2) 그 정점들이 속한 (아직 방문하지 않은) face들을 후보로
		/// 모은 뒤, 3) 각 후보 face의 평면으로 queryPt를 회전 투영해 평면까지의
		/// 수직 거리가 가장 작은 face를 선택한다. N(템플릿 매개변수)이 클수록
		/// 더 넓은 범위의 face를 후보로 검토하지만 그만큼 느려진다.
		template<std::size_t N>
		bool searchFace_closest(const pctree::XYZPoint& queryPt,
			size_t& foundFaceId, 
			float& minDistToFace, 
			std::array<float, N>& squareDist, 
			std::array<std::size_t, N>& indices) const
		{
			size_t num = squareDist.size();
			this->meshTree->knnSearch(queryPt.xyz, num, indices.data(), squareDist.data());

			std::vector<size_t> foundFaces;
			for (int i = 0; i < num; ++i) {
				auto idx = indices[i];
				auto pt0 = this->pc[idx];

				for (const auto& fidx : pt0.fIds) {
					bool duplicated = false;

					for (const auto id : foundFaces) {
						if (id == fidx) {
							duplicated = true;
							break;
						}
					}

					if (duplicated) continue;

					if (!this->faces[fidx].availability) continue;

					foundFaces.emplace_back(fidx);
				}
			}

			minDistToFace = std::numeric_limits<float>::max();
			foundFaceId = std::numeric_limits<std::size_t>::max();;

			for (int i = 0; i < foundFaces.size(); ++i) {
				auto fidx = foundFaces[i];
				auto vtx0 = this->pc.points[this->faces[fidx].vtxIds[0]];
				auto vtx1 = this->pc.points[this->faces[fidx].vtxIds[1]];
				auto vtx2 = this->pc.points[this->faces[fidx].vtxIds[2]];
				auto a = pctree::XYZPoint(0, 0, 0);
				auto b = vtx1 - vtx0;
				auto c = vtx2 - vtx0;
				auto p = queryPt - vtx0;

				pctree::MatrixForMesh planeRot;
				if (!math::getPlaneRotationMatrix(a, b, c, planeRot))
					continue;

				auto p0 = planeRot % p;
				auto d2Plane = static_cast<float>(fabs(p0.xyz[2]));

				if (d2Plane > minDistToFace)
					continue;

				minDistToFace = static_cast<float>(d2Plane);
				foundFaceId = fidx;

				continue;
			}

			if (foundFaceId < faces.size() && foundFaceId >= 0)
				return true;
			else
				return false;
		}
	};
}
