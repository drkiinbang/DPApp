#pragma once

#include "../bimtree/xyzpoint.hpp"

namespace mesh {
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