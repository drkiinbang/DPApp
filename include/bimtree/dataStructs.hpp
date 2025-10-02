#pragma once

#include <array>
#include <vector>

#include "barycentric.hpp"
#include "xyzpoint.hpp"

namespace mesh
{
	const double deg2rad = acos(-1.0) / 180.0;
	const double rad2deg = 180.0 / acos(-1.0);

	struct Face
	{
		/// part id (model file id)
		int partId;
		/// Ids of corresponding vertices
		std::array<unsigned int, 3> vtxIds;
		/// Ids of corresponding pseudo pts
		std::vector<unsigned int> pseudoPtsIds;

		pctree::MatrixForMesh planeOri;
		geo::TempBaryCentVar baryTemp;

		bool rotAvailability = false;
		/// face availability
		bool availability = true;
		/// Barycentric temp availability
		bool baryAvailability = false;

		Face() {}
		Face(const std::array<unsigned int, 3>& i) { vtxIds = i; }
		Face(const std::array<unsigned int, 3>& i, const std::vector<unsigned int>& j, const unsigned int id) { vtxIds = i; pseudoPtsIds = j; partId = id; }
		Face(const std::array<unsigned int, 3>& i, const std::vector<unsigned int>& j) { vtxIds = i; pseudoPtsIds = j; }
		Face(const unsigned int i0, const unsigned int i1, const unsigned int i2) { vtxIds[0] = i0; vtxIds[1] = i1; vtxIds[2] = i2; }
		Face(const Face& val) { copy(val); }

		void copy(const Face& val) {
			this->partId = val.partId;
			this->vtxIds = val.vtxIds;
			this->pseudoPtsIds = val.pseudoPtsIds;
			this->rotAvailability = val.rotAvailability;
			this->availability = val.availability;
			this->baryAvailability = val.baryAvailability;
		}

		Face operator=(const Face& val) { copy(val); return *this; }
	};
}