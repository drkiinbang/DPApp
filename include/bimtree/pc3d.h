#pragma once

#include <iostream>
#include <vector>

#include "nanoflann.hpp"
#include "xyzpoint.hpp"

namespace pctree
{
	template<typename T, size_t n>
	struct PointCloud
	{
		typedef T data_t; // Base type used
		const static size_t dim = n; // dimension

		std::vector<XYZPoint> points;

		// Must return the number of data points
		inline std::size_t kdtree_get_point_count() const { return points.size(); }

		// Returns the squared_distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
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

		// Returns the dim'th component of the idx'th point in the class:
		// Since this is inlined and the "dim" argument is typically an immediate value, the
		//  "if/else's" are actually solved at compile time.
		inline T kdtree_get_pt(const size_t idx, int dim) const
		{
			return points[idx][dim];
		}

		// Optional bounding-box computation: return false to default to a standard bbox computation loop.
		//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
		//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
		template <class BBOX>
		bool kdtree_get_bbox(BBOX& bb) const { return false; }

		//! @returns Centroid of the point cloud
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

		//! Remove an offset from all points in the point cloud.
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