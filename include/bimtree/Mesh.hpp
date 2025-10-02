#pragma once

#include <string>

#include "dataStructs.hpp"
#include "pc3d.h"
#include "PseudoPtsInMesh.hpp"
#include "xyzpoint.hpp"

namespace mesh /// Use this one
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
		/// Serialize XYZPoint
		void serializeXYZPoint(std::ostream& out, const pctree::XYZPoint& point) const {
			out.write(reinterpret_cast<const char*>(&point.xyz), sizeof(point.xyz));
			size_t fIdsSize = point.fIds.size();
			out.write(reinterpret_cast<const char*>(&fIdsSize), sizeof(fIdsSize));
			out.write(reinterpret_cast<const char*>(point.fIds.data()), fIdsSize * sizeof(size_t));
		}

		/// Deserialize XYZPoint
		void deserializeXYZPoint(std::istream& in, pctree::XYZPoint& point) {
			in.read(reinterpret_cast<char*>(&point.xyz), sizeof(point.xyz));
			size_t fIdsSize;
			in.read(reinterpret_cast<char*>(&fIdsSize), sizeof(fIdsSize));
			point.fIds.resize(fIdsSize);
			in.read(reinterpret_cast<char*>(point.fIds.data()), fIdsSize * sizeof(size_t));
		}

		/// Serialize MatrixForMesh
		void serializeMatrixForMesh(std::ostream& out, const pctree::MatrixForMesh& matrix) const {
			out.write(reinterpret_cast<const char*>(&matrix.r), sizeof(matrix.r));
		}

		/// Deserialize MatrixForMesh
		void deserializeMatrixForMesh(std::istream& in, pctree::MatrixForMesh& matrix) {
			in.read(reinterpret_cast<char*>(&matrix.r), sizeof(matrix.r));
		}

		/// Serialize TempBaryCentVar
		void serializeTempBaryCentVar(std::ostream& out, const geo::TempBaryCentVar& baryTemp) const {
			serializeXYZPoint(out, baryTemp.v0);
			serializeXYZPoint(out, baryTemp.v1);
			out.write(reinterpret_cast<const char*>(&baryTemp.dot00), sizeof(baryTemp.dot00));
			out.write(reinterpret_cast<const char*>(&baryTemp.dot11), sizeof(baryTemp.dot11));
			out.write(reinterpret_cast<const char*>(&baryTemp.dot01), sizeof(baryTemp.dot01));
			out.write(reinterpret_cast<const char*>(&baryTemp.invDenom), sizeof(baryTemp.invDenom));
		}

		/// Deserialize TempBaryCentVar
		void deserializeTempBaryCentVar(std::istream& in, geo::TempBaryCentVar& baryTemp) {
			deserializeXYZPoint(in, baryTemp.v0);
			deserializeXYZPoint(in, baryTemp.v1);
			in.read(reinterpret_cast<char*>(&baryTemp.dot00), sizeof(baryTemp.dot00));
			in.read(reinterpret_cast<char*>(&baryTemp.dot11), sizeof(baryTemp.dot11));
			in.read(reinterpret_cast<char*>(&baryTemp.dot01), sizeof(baryTemp.dot01));
			in.read(reinterpret_cast<char*>(&baryTemp.invDenom), sizeof(baryTemp.invDenom));
		}

		/// Serialize Face
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

		/// Deserialize Face
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
		
		void addMeshData(std::vector<Face>& addedFaces, std::vector<pctree::XYZPoint>& addedVertices)
		{
			/// update ids for added data
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

			/// insert
			this->faces.insert(this->faces.end(), addedFaces.begin(), addedFaces.end());
			this->pc.points.insert(this->pc.points.end(), addedVertices.begin(), addedVertices.end());

#ifdef _DEBUG
			std::cout << addedFaces.size() << " faces and " << pc.points.size() << " vertices are added\n";
			std::cout << "For now, " << this->faces.size() << " faces and " << this->pc.points.size() << " vertices\n";
#endif
		}

		/// Save pc
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

		/// Load pc
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

		/// Save faces
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

		/// Load faces
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

		/// Generate pseudo pts
		void generatePseudoPts(const float lengthTh)
		{
			geo::generatePseudoGridPts(this->faces, this->pc.points, lengthTh);
		}

		/// Build tree
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

		/// export tree
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
