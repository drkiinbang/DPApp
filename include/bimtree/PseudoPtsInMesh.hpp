#pragma once

#include "barycentric.hpp"
#include "dataStructs.hpp"
#include "lineintersection.hpp"
#include "rotation.hpp"
#include "xyzpoint.hpp"
#include <omp.h>

namespace geo
{
	bool definePseudoInTri(
		const pctree::XYZPoint& vtx0, 
		const pctree::XYZPoint& vtx1,
		const pctree::XYZPoint& vtx2,
		pctree::MatrixForMesh& planeRot,
		geo::TempBaryCentVar& baryTemp,
		std::vector<pctree::XYZPoint>& gridPoints,
		std::vector<pctree::XYZPoint>& pointsOnLines,
		const double gridRes)
	{
		auto a0 = pctree::XYZPoint(0.f, 0.f, 0.f);
		auto b0 = vtx1 - vtx0;
		auto c0 = vtx2 - vtx0;

		auto abD = sqrt(b0.getSquareSum());
		auto acD = sqrt(c0.getSquareSum());
		auto bcD = sqrt((c0 - b0).getSquareSum());

		if (abD <= gridRes && acD <= gridRes && bcD <= gridRes)
			return true; /// no pseudo pt (return true but no additional pt)
		
		if (!math::getPlaneRotationMatrix(a0, b0, c0, planeRot))
			return false;

		//if (planeRot.getCols() != 3 || planeRot.getRows() != 3)
		//	return false; /// non-avaiable face

		auto a = planeRot % a0;
		auto b = planeRot % b0;
		auto c = planeRot % c0;

		a[2] = b[2] = c[2] = 0.0;

		/// Bounding box
		double minX = std::min({ a.xyz[0], b.xyz[0], c.xyz[0] });
		double minY = std::min({ a.xyz[1], b.xyz[1], c.xyz[1] });

		double maxX = std::max({ a.xyz[0], b.xyz[0], c.xyz[0] });
		double maxY = std::max({ a.xyz[1], b.xyz[1], c.xyz[1] });

		gridPoints.clear();
		pointsOnLines.clear();

		gridPoints.reserve(static_cast<size_t>((maxX - minX) / gridRes * (maxY - minY) / gridRes + 1.0));
		pointsOnLines.reserve(static_cast<size_t>((abD + acD + bcD) / gridRes * 3.0 + 1.0));

		pctree::XYZPoint v0, v1;
		if (!geo::computeBarycentricCoord_step1(a, b, c, baryTemp))
			return false; /// non-avaiable face

		/// Generating Grid points in a triangle
		pctree::XYZPoint p(0.0f, 0.0f, 0.0f);
		for (double x = minX + gridRes; x < maxX; x += gridRes)
		{
			p[0] = static_cast<float>(x);

			for (double y = minY + gridRes; y < maxY; y += gridRes)
			{
				/// Check if the p(x, y, z) is inside (Barycentric coordinate)
				p[1] = static_cast<float>(y);
				pctree::XYZPoint uvw = geo::computeBarycentricCoord_step2(p, a, baryTemp);

				if (geo::checkInside(uvw))
					gridPoints.emplace_back(p);
			}
		}

		double intersPt[3] = { 0., 0., 0. };

		geo::IntersectionRetVal res;
		/// Generating intersection points
		const double y0 = minY - gridRes;
		const double y1 = maxY + gridRes;
		for (double x = minX + gridRes; x < maxX; x += gridRes)
		{
			/// grid line
			res = geo::Line2DIntersect(x, y0, x, y1, a[0], a[1], b[0], b[1], intersPt[0], intersPt[1]);
			if (res == geo::IntersectionRetVal::FOUND)
				pointsOnLines.emplace_back(intersPt[0], intersPt[1], intersPt[2]);
			res = geo::Line2DIntersect(x, y0, x, y1, a[0], a[1], c[0], c[1], intersPt[0], intersPt[1]);
			if (res == geo::IntersectionRetVal::FOUND)
				pointsOnLines.emplace_back(intersPt[0], intersPt[1], intersPt[2]);
			res = geo::Line2DIntersect(x, y0, x, y1, b[0], b[1], c[0], c[1], intersPt[0], intersPt[1]);
			if (res == geo::IntersectionRetVal::FOUND)
				pointsOnLines.emplace_back(intersPt[0], intersPt[1], intersPt[2]);
		}

		const double x0 = minX - gridRes;
		const double x1 = maxX + gridRes;
		for (double y = minY + gridRes; y < maxY; y += gridRes)
		{
			/// grid line
			res = geo::Line2DIntersect(x0, y, x1, y, a[0], a[1], b[0], b[1], intersPt[0], intersPt[1]);
			if (res == geo::IntersectionRetVal::FOUND)
				pointsOnLines.emplace_back(intersPt[0], intersPt[1], intersPt[2]);
			res = geo::Line2DIntersect(x0, y, x1, y, a[0], a[1], c[0], c[1], intersPt[0], intersPt[1]);
			if (res == geo::IntersectionRetVal::FOUND)
				pointsOnLines.emplace_back(intersPt[0], intersPt[1], intersPt[2]);
			res = geo::Line2DIntersect(x0, y, x1, y, b[0], b[1], c[0], c[1], intersPt[0], intersPt[1]);
			if (res == geo::IntersectionRetVal::FOUND)
				pointsOnLines.emplace_back(intersPt[0], intersPt[1], intersPt[2]);
		}

		gridPoints.shrink_to_fit();
		pointsOnLines.shrink_to_fit();

#ifdef _DEBUG
		if (false)
		{
			std::cout << "vertices0:\n";
			std::cout << "vtx0: " << a[0] << "\t" << a[1] << "\t" << 0.0 << "\n";
			std::cout << "vtx1: " << b[0] << "\t" << b[1] << "\t" << 0.0 << "\n";
			std::cout << "vtx2: " << c[0] << "\t" << c[1] << "\t" << 0.0 << "\n";
			std::cout << "created pseudo poitns\n";
			for (size_t i = 0; i < pointsOnLines.size(); ++i)
				std::cout << "xyz: " << pointsOnLines[i][0] << "\t" << pointsOnLines[i][1] << "\t" << 0.0 << "\n";
		}
#endif
		/// inverse_rotate and shift to transform back to the original frame
		auto ptRotTrans = planeRot.transpose();
		pctree::MatrixForMesh* rotTrans = (pctree::MatrixForMesh*)&ptRotTrans;

		for (size_t i = 0; i < gridPoints.size(); ++i)
			gridPoints[i] = (*rotTrans) % gridPoints[i] + vtx0;

		for (size_t i = 0; i < pointsOnLines.size(); ++i)
			pointsOnLines[i] = (*rotTrans) % pointsOnLines[i] + vtx0;

		return true;
	}

	
	bool definePseudoInTri_parallel(
		const pctree::XYZPoint& vtx0,
		const pctree::XYZPoint& vtx1,
		const pctree::XYZPoint& vtx2,
		pctree::MatrixForMesh& planeRot,
		geo::TempBaryCentVar& baryTemp,
		std::vector<pctree::XYZPoint>& gridPoints,
		std::vector<pctree::XYZPoint>& pointsOnLines,
		double gridRes,
		size_t maxPoints = 500000
	) {
		const double MIN_GRID_RES = 1e-6;
		if (gridRes < MIN_GRID_RES) gridRes = MIN_GRID_RES;

		auto a0 = pctree::XYZPoint(0.f, 0.f, 0.f);
		auto b0 = vtx1 - vtx0;
		auto c0 = vtx2 - vtx0;

		auto abD = sqrt(b0.getSquareSum());
		auto acD = sqrt(c0.getSquareSum());
		auto bcD = sqrt((c0 - b0).getSquareSum());

		if (abD <= gridRes && acD <= gridRes && bcD <= gridRes)
			return true;

		if (!math::getPlaneRotationMatrix(a0, b0, c0, planeRot))
			return false;

		auto a = planeRot % a0;
		auto b = planeRot % b0;
		auto c = planeRot % c0;

		a[2] = b[2] = c[2] = 0.0;

		double minX = std::min({ a.xyz[0], b.xyz[0], c.xyz[0] });
		double minY = std::min({ a.xyz[1], b.xyz[1], c.xyz[1] });

		double maxX = std::max({ a.xyz[0], b.xyz[0], c.xyz[0] });
		double maxY = std::max({ a.xyz[1], b.xyz[1], c.xyz[1] });

		gridPoints.clear();
		pointsOnLines.clear();

		if (!geo::computeBarycentricCoord_step1(a, b, c, baryTemp))
			return false;

		// ===== 병렬 그리드 스캔 =====
		std::vector<std::vector<pctree::XYZPoint>> localGrids(omp_get_max_threads());
		size_t threadLimit = maxPoints / omp_get_max_threads();

#pragma omp parallel
		{
			int tid = omp_get_thread_num();
			auto& localVec = localGrids[tid];
			pctree::XYZPoint p(0.0f, 0.0f, 0.0f);

			for (double x = minX + gridRes; x < maxX; x += gridRes) {
				p[0] = x;
				for (double y = minY + gridRes; y < maxY; y += gridRes) {
					p[1] = y;
					pctree::XYZPoint uvw = geo::computeBarycentricCoord_step2(p, a, baryTemp);
					if (geo::checkInside(uvw)) {
						localVec.emplace_back(p);
						if (localVec.size() > threadLimit) break;
					}
				}
				if (localVec.size() > threadLimit) break;
			}
		}

		// merge local grids
		for (auto& vec : localGrids) {
			gridPoints.insert(gridPoints.end(), vec.begin(), vec.end());
		}

		if (gridPoints.size() > maxPoints) {
			gridPoints.resize(maxPoints);
		}

		// ===== 기존 라인 교차 계산 (단일 스레드 유지) =====
		double intersPt[3] = { 0., 0., 0. };
		geo::IntersectionRetVal res;
		const double y0 = minY - gridRes;
		const double y1 = maxY + gridRes;
		for (double x = minX + gridRes; x < maxX; x += gridRes) {
			res = geo::Line2DIntersect(x, y0, x, y1, a[0], a[1], b[0], b[1], intersPt[0], intersPt[1]);
			if (res == geo::IntersectionRetVal::FOUND) pointsOnLines.emplace_back(intersPt[0], intersPt[1], intersPt[2]);
			res = geo::Line2DIntersect(x, y0, x, y1, a[0], a[1], c[0], c[1], intersPt[0], intersPt[1]);
			if (res == geo::IntersectionRetVal::FOUND) pointsOnLines.emplace_back(intersPt[0], intersPt[1], intersPt[2]);
			res = geo::Line2DIntersect(x, y0, x, y1, b[0], b[1], c[0], c[1], intersPt[0], intersPt[1]);
			if (res == geo::IntersectionRetVal::FOUND) pointsOnLines.emplace_back(intersPt[0], intersPt[1], intersPt[2]);
		}

		const double x0 = minX - gridRes;
		const double x1 = maxX + gridRes;
		for (double y = minY + gridRes; y < maxY; y += gridRes) {
			res = geo::Line2DIntersect(x0, y, x1, y, a[0], a[1], b[0], b[1], intersPt[0], intersPt[1]);
			if (res == geo::IntersectionRetVal::FOUND) pointsOnLines.emplace_back(intersPt[0], intersPt[1], intersPt[2]);
			res = geo::Line2DIntersect(x0, y, x1, y, a[0], a[1], c[0], c[1], intersPt[0], intersPt[1]);
			if (res == geo::IntersectionRetVal::FOUND) pointsOnLines.emplace_back(intersPt[0], intersPt[1], intersPt[2]);
			res = geo::Line2DIntersect(x0, y, x1, y, b[0], b[1], c[0], c[1], intersPt[0], intersPt[1]);
			if (res == geo::IntersectionRetVal::FOUND) pointsOnLines.emplace_back(intersPt[0], intersPt[1], intersPt[2]);
		}

		// ===== 좌표 복원 =====
		auto ptRotTrans = planeRot.transpose();
		pctree::MatrixForMesh* rotTrans = (pctree::MatrixForMesh*)&ptRotTrans;
		for (auto& gp : gridPoints) gp = (*rotTrans) % gp + vtx0;
		for (auto& lp : pointsOnLines) lp = (*rotTrans) % lp + vtx0;

		return true;
	}

	std::vector<pctree::XYZPoint> definePseudoInTri(chunkbim::Face& face,
		const pctree::XYZPoint& vtx0,
		const pctree::XYZPoint& vtx1,
		const pctree::XYZPoint& vtx2,
		const double gridSpacing)
	{
		std::vector<pctree::XYZPoint> gridPoints;
		std::vector<pctree::XYZPoint> pointsOnLines;

		//if (definePseudoInTri(vtx0, vtx1, vtx2, face.planeOri, face.baryTemp, gridPoints, pointsOnLines, gridSpacing))
		if (definePseudoInTri_parallel(vtx0, vtx1, vtx2, face.planeOri, face.baryTemp, gridPoints, pointsOnLines, gridSpacing))
		{
			/// merge pts
			gridPoints.insert(gridPoints.end(), pointsOnLines.begin(), pointsOnLines.end());
			face.baryAvailability = true;
			face.rotAvailability = true;
		}
		else
			face.availability = false;

		return gridPoints;
	}

	// 삼각형 면적 계산
	double calcTriangleArea(const pctree::XYZPoint& p1,
		const pctree::XYZPoint& p2,
		const pctree::XYZPoint& p3)
	{
		// 두 변 벡터
		pctree::XYZPoint u = p2 - p1;
		pctree::XYZPoint v = p3 - p1;

		// 외적
		pctree::XYZPoint cross = u % v;

		// 외적의 길이의 절반 = 삼각형 면적
		double len2 = static_cast<double>(cross.getSquareSum());
		if (len2 <= std::numeric_limits<double>::epsilon()) {
			return 0.0; // 사실상 면적 없음
		}
		return 0.5 * std::sqrt(len2);

	}

	void generatePseudoGridPts(std::vector<chunkbim::Face>& faces, std::vector<pctree::XYZPoint>& points, const float lengthTh)
	{
		/// Faces
		const auto numFaces = faces.size();
		const auto numVertices = points.size();

		std::array<size_t, 3> vIds;
		size_t lastId = points.size();

		for (size_t fi = 0; fi < numFaces; ++fi)
		{
			vIds[0] = faces[fi].vtxIds[0];
			vIds[1] = faces[fi].vtxIds[1];
			vIds[2] = faces[fi].vtxIds[2];

			double area = calcTriangleArea(points[vIds[0]], points[vIds[1]], points[vIds[2]]);
			if (area < 1e-8)
				continue;

			/// Generate pseudo points
			faces[fi].planeOri.del();
			std::vector<pctree::XYZPoint> pseudoPoints = definePseudoInTri(faces[fi],
				points[vIds[0]],
				points[vIds[1]],
				points[vIds[2]],
				double(lengthTh));

			auto numPseudo = pseudoPoints.size();

			if (numPseudo < 1)
				continue;

			faces[fi].pseudoPtsIds.reserve(numPseudo);

			for (size_t i = 0; i < numPseudo; ++i) {
				pseudoPoints[i].fIds.push_back(fi);
				faces[fi].pseudoPtsIds.push_back(i + lastId);
			}

			/// Insert pseudo points
			points.insert(points.end(), pseudoPoints.begin(), pseudoPoints.end());

			lastId += numPseudo;
		}
	}

}
