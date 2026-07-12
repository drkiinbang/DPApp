#pragma once

#include <iostream>
#include <vector>

#include "nanoflann.hpp"
#include "xyzpoint.hpp"

namespace pctree
{
	/// nanoflann과 함께 사용하기 위한 포인트클라우드 어댑터. IcpCore.hpp의
	/// PointCloudAdaptor(std::array<double,3> 벡터를 감싸는 버전)와 달리, 이쪽은
	/// XYZPoint(각 점이 자신이 속한 face id 목록까지 들고 있는 타입)를 감싸며,
	/// Mesh.hpp의 MeshData::meshTree(메시 정점 검색용 KD-Tree)에 쓰인다.
	template<typename T, size_t n>
	struct PointCloud
	{
		typedef T data_t; // 사용할 기본 타입
		const static size_t dim = n; // 차원 수

		std::vector<XYZPoint> points;

		// 데이터 점의 개수를 반환해야 함
		inline std::size_t kdtree_get_point_count() const { return points.size(); }

		// 벡터 "p1[0:size-1]"과 클래스에 저장된 인덱스 "idx_p2"의 데이터 점 사이의 거리 제곱을 반환:
		inline T kdtree_distance(const T* p1, const size_t idx_p2, size_t size) const
		{
			double squareSum = 0.0;
			for (size_t i = 0; i < size; ++i)
			{
				auto d = p1[i] - points[idx_p2].xyz[i];
				squareSum += static_cast<double>(d * d);
			}

			return (static_cast<T>(squareSum));
		}

		// 클래스 내 idx번째 점의 dim번째 성분을 반환:
		// 이 함수는 inline이고 "dim" 인자가 보통 즉시값(immediate value)이므로,
		// if/else 분기는 실제로 컴파일 타임에 해석된다.
		inline T kdtree_get_pt(const size_t idx, int dim) const
		{
			return points[idx][dim];
		}

		// 선택적 바운딩 박스 계산: false를 반환하면 표준 bbox 계산 루프를 기본으로 사용한다.
		//   클래스가 이미 bbox를 계산해서 "bb"에 담아 반환했다면 true를 반환해 재계산을 피할 수 있다.
		//   bb.size()를 보면 기대되는 차원 수를 알 수 있다 (포인트클라우드의 경우 보통 2 또는 3)
		template <class BBOX>
		bool kdtree_get_bbox(BBOX& bb) const { return false; }

		//! @returns 포인트클라우드의 중심점(centroid)
		XYZPoint getCentroid() const
		{
			XYZPoint result;
			size_t numP = points.size();
			double sumX = 0.0;
			double sumY = 0.0;
			double sumZ = 0.0;

			for (size_t i = 0; i < numP; ++i)
			{
				sumX += points[i].x();
				sumY += points[i].y();
				sumZ += points[i].z();
			}

			result[0] = sumX / static_cast<double>(numP);
			result[1] = sumY / static_cast<double>(numP);
			result[2] = sumZ / static_cast<double>(numP);

			return result;
		}

		//! 포인트클라우드의 모든 점에서 offset을 뺀다(이동시킨다).
		void shiftPointCloud(const XYZPoint& offsetPoint)
		{
			size_t numP = points.size();
			for (size_t i = 0; i < numP; ++i)
			{
				points[i][0] -= offsetPoint[0];
				points[i][1] -= offsetPoint[1];
				points[i][2] -= offsetPoint[2];
			}
		}

		XYZPoint& operator[] (const size_t idx)
		{
			return points[idx];
		}

		const XYZPoint& operator[] (const size_t idx) const
		{
			return points[idx];
		}

		size_t size() { return points.size(); }
		void clear() { points.clear(); }
		void reserve(const size_t size) { points.reserve(size); }
		void shrink_to_fit() { points.shrink_to_fit(); }
		void resize(const size_t size) { points.resize(size); }
		void push_back(const XYZPoint& p) { points.push_back(p); }
	};

	typedef PointCloud<float, 3> PointCloudType3D;
	using kd_tree3D = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<PointCloudType3D::data_t, PointCloudType3D>, PointCloudType3D, PointCloudType3D::dim>;

	const unsigned int maxLeaf = 50;

	bool buildKdtree3D(const pctree::PointCloudType3D& pc, std::shared_ptr<pctree::kd_tree3D>& tree);

	bool saveTree3D(const std::shared_ptr<pctree::kd_tree3D>& tree, const std::string& treePath);

	bool buildAndSaveKdtree3D(const pctree::PointCloudType3D& pc, const std::string& treePath);

	bool loadKdtree3D(std::shared_ptr<pctree::kd_tree3D>& tree, const pctree::PointCloudType3D& pc, const std::string& treePath);
}
