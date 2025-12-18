#pragma once

#include "../bimtree/xyzpoint.hpp"

#include <vector>
#include <string>
#include <algorithm> // for std::min, std::max
#include <cstdint>   // for uint64_t, uint32_t


namespace chunkbim {
	struct FaceVtx {
		pctree::XYZPoint normal;
		unsigned int idxVtx[3];

		FaceVtx() = default;
		FaceVtx(
			const size_t idx0,
			const size_t idx1,
			const size_t idx2,
			const pctree::XYZPoint& n)
			: normal(n) {
			idxVtx[0] = static_cast<unsigned int>(idx0);
			idxVtx[1] = static_cast<unsigned int>(idx1);
			idxVtx[2] = static_cast<unsigned int>(idx2);
		}
	};

	struct MeshChunk {
		std::vector<pctree::XYZPoint> vertices;
		std::vector<FaceVtx> faces;
		std::string name;
		int id;
	};
}

namespace chunkpc {
    struct PointCloudHeader {
        std::string filename;
        uint64_t point_count;
        double min_x, min_y, min_z;
        double max_x, max_y, max_z;
        std::string coordinate_system;

        PointCloudHeader() : point_count(0), min_x(0), min_y(0), min_z(0), max_x(0), max_y(0), max_z(0) {}
    };

    struct PointCloudChunk {
        std::vector<pctree::XYZPoint> points;
        uint32_t chunk_id;
        PointCloudHeader header;

        /// Bounding box
        double min_x, min_y, min_z;
        double max_x, max_y, max_z;

        PointCloudChunk() : chunk_id(0), min_x(0), min_y(0), min_z(0), max_x(0), max_y(0), max_z(0) {}

        /// Compute bounding box
        void calculateBounds() {
            if (points.empty()) {
                min_x = min_y = min_z = 0;
                max_x = max_y = max_z = 0;
                return;
            }

            min_x = max_x = points[0][0];
            min_y = max_y = points[0][1];
            min_z = max_z = points[0][2];

            for (const auto& point : points) {
                min_x = (std::min)(min_x, static_cast<double>(point[0]));
                min_y = (std::min)(min_y, static_cast<double>(point[1]));
                min_z = (std::min)(min_z, static_cast<double>(point[2]));

                max_x = (std::max)(max_x, static_cast<double>(point[0]));
                max_y = (std::max)(max_y, static_cast<double>(point[1]));
                max_z = (std::max)(max_z, static_cast<double>(point[2]));
            }
        }

        /// Get chunk size in byte
        size_t getSize() const {
            return sizeof(PointCloudChunk) + points.size() * sizeof(pctree::XYZPoint);
        }

        size_t getNumPts() const {
            return points.size();
        }
    };
}