#pragma once

#include "../bimtree/xyzpoint.hpp"

#include <vector>
#include <string>
#include <algorithm> // for std::min, std::max
#include <cstdint>   // for uint64_t, uint32_t


namespace chunkbim {
	struct FaceVtx {
		pctree::XYZPoint normal;
        pctree::XYZPoint vertices[3];

		FaceVtx() = default;
		FaceVtx(
			const pctree::XYZPoint vertex0,
			const pctree::XYZPoint vertex1,
			const pctree::XYZPoint vertex2,
			const pctree::XYZPoint& n)
			: normal(n) {
            vertices[0] = vertex0;
            vertices[1] = vertex1;
            vertices[2] = vertex2;
		}
	};

	struct MeshChunk {
		std::vector<FaceVtx> faces;
		std::string name;
		int id;

        /// Bounding box
        float min_x, min_y, min_z;
        float max_x, max_y, max_z;

        void calculateBounds() {
            if (faces.empty()) {
                min_x = min_y = min_z = 0;
                max_x = max_y = max_z = 0;
                return;
            }

            /// Initialize with first vertex of first face
            min_x = max_x = faces[0].vertices[0][0];
            min_y = max_y = faces[0].vertices[0][1];
            min_z = max_z = faces[0].vertices[0][2];

            for (const auto& face : faces) {
                for (int v = 0; v < 3; ++v) {
                    min_x = (std::min)(min_x, face.vertices[v][0]);
                    min_y = (std::min)(min_y, face.vertices[v][1]);
                    min_z = (std::min)(min_z, face.vertices[v][2]);
                    max_x = (std::max)(max_x, face.vertices[v][0]);
                    max_y = (std::max)(max_y, face.vertices[v][1]);
                    max_z = (std::max)(max_z, face.vertices[v][2]);
                }
            }
        }
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