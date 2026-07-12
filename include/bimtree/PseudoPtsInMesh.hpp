#pragma once

/// ============================================================================
/// PseudoPtsInMesh.hpp - 삼각형 face 내부에 격자 형태의 가상점(pseudo point) 생성
/// ============================================================================
///
/// Mesh.hpp의 MeshData::generatePseudoPts()가 사용하는 저수준 구현으로,
/// PseudoPointGenerator.hpp(ICP의 target 가상점 생성, barycentric 격자 샘플링
/// 방식)와는 별개의 알고리즘이다. 여기서는 각 삼각형을 2차원 평면에 투영한 뒤,
/// 그 평면 위에서 실제 격자선(x=const, y=const)과 삼각형 변의 교차점 및 격자
/// 내부점을 계산하여, 3차원 좌표로 되돌리는 방식을 사용한다 (BimImport의 메시
/// 전처리 단계에서 호출됨).
/// ============================================================================

#include "barycentric.hpp"
#include "dataStructs.hpp"
#include "lineintersection.hpp"
#include "rotation.hpp"
#include "xyzpoint.hpp"
#include <omp.h>

namespace geo
{
	/// 삼각형(vtx0, vtx1, vtx2) 내부에 격자 간격 gridRes로 가상점을 생성한다
	/// (단일 스레드 버전). 삼각형을 평면 좌표계로 회전시킨 뒤(planeRot), 그
	/// 평면 위에서 격자점 및 격자선-변 교차점을 계산하고, 다시 원래 3차원
	/// 좌표계로 역회전+이동시켜 되돌린다. 세 변이 모두 gridRes보다 작으면
	/// (너무 작은 삼각형) 가상점 없이 true를 반환한다.
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
			return true; /// 가상점 없음 (true를 반환하지만 추가 점은 없음)

		if (!math::getPlaneRotationMatrix(a0, b0, c0, planeRot))
			return false;

		//if (planeRot.getCols() != 3 || planeRot.getRows() != 3)
		//	return false; /// non-avaiable face

		auto a = planeRot % a0;
		auto b = planeRot % b0;
		auto c = planeRot % c0;

		a[2] = b[2] = c[2] = 0.0;

		/// 바운딩 박스
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
			return false; /// 사용할 수 없는 face

		/// 삼각형 내부에 격자점 생성
		pctree::XYZPoint p(0.0f, 0.0f, 0.0f);
		for (double x = minX + gridRes; x < maxX; x += gridRes)
		{
			p[0] = static_cast<float>(x);

			for (double y = minY + gridRes; y < maxY; y += gridRes)
			{
				/// p(x, y, z)가 삼각형 내부에 있는지 확인 (barycentric 좌표 이용)
				p[1] = static_cast<float>(y);
				pctree::XYZPoint uvw = geo::computeBarycentricCoord_step2(p, a, baryTemp);

				if (geo::checkInside(uvw))
					gridPoints.emplace_back(p);
			}
		}

		double intersPt[3] = { 0., 0., 0. };

		geo::IntersectionRetVal res;
		/// 교차점 생성
		const double y0 = minY - gridRes;
		const double y1 = maxY + gridRes;
		for (double x = minX + gridRes; x < maxX; x += gridRes)
		{
			/// 격자선
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
			/// 격자선
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
		/// 역회전 및 이동으로 원래 좌표계로 되돌림
		auto ptRotTrans = planeRot.transpose();
		pctree::MatrixForMesh* rotTrans = (pctree::MatrixForMesh*)&ptRotTrans;

		for (size_t i = 0; i < gridPoints.size(); ++i)
			gridPoints[i] = (*rotTrans) % gridPoints[i] + vtx0;

		for (size_t i = 0; i < pointsOnLines.size(); ++i)
			pointsOnLines[i] = (*rotTrans) % pointsOnLines[i] + vtx0;

		return true;
	}


	/// definePseudoInTri()의 병렬(OpenMP) 버전. 격자 내부점 생성 단계만 스레드별로
	/// 나눠 처리하고(각 스레드가 자신의 로컬 벡터에 쌓은 뒤 병합), 격자선-변
	/// 교차점 계산은 상대적으로 저비용이라 단일 스레드로 유지한다. maxPoints로
	/// 전체 격자점 개수를 제한해 극단적으로 큰 face에서 메모리가 폭주하지 않게 한다.
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

		// merge local grids (스레드별 로컬 결과를 하나로 병합)
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

	/// face 하나에 대해 가상점을 생성하고, 격자 내부점과 격자선 교차점을
	/// 하나의 벡터로 합쳐 반환한다. 성공하면 face의 baryAvailability/rotAvailability를
	/// true로, 실패(퇴화 face 등)하면 face.availability를 false로 표시해 이후
	/// 처리 단계에서 이 face를 건너뛸 수 있게 한다.
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
			/// 점들을 병합
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

	/// 메시 전체(faces/points)에 대해 각 face마다 가상점을 생성하고, 생성된
	/// 가상점들을 points 배열 끝에 이어붙인다. 각 face의 pseudoPtsIds에는
	/// 그 face가 만들어낸 가상점들의 (points 배열 기준) 인덱스를 기록해 둔다.
	/// 면적이 사실상 0인(퇴화) face는 건너뛴다.
	void generatePseudoGridPts(std::vector<chunkbim::Face>& faces, std::vector<pctree::XYZPoint>& points, const float lengthTh)
	{
		/// face들
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

			/// 가상점 생성
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

			/// 가상점 삽입
			points.insert(points.end(), pseudoPoints.begin(), pseudoPoints.end());

			lastId += numPseudo;
		}
	}

}
