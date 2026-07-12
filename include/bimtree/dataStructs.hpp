#pragma once

#include <array>
#include <vector>

#include "barycentric.hpp"
#include "xyzpoint.hpp"

namespace chunkbim
{
	const double deg2rad = acos(-1.0) / 180.0;
	const double rad2deg = 180.0 / acos(-1.0);

	struct Face
	{
		/// 부재(part) id (모델 파일 id)
		int partId;
		/// 대응하는 정점(vertex)들의 id
		std::array<unsigned int, 3> vtxIds;
		/// 대응하는 가상점(pseudo point)들의 id
		std::vector<unsigned int> pseudoPtsIds;

		pctree::MatrixForMesh planeOri;
		geo::TempBaryCentVar baryTemp;

		bool rotAvailability = false;
		/// face 유효 여부
		bool availability = true;
		/// barycentric 임시 변수 유효 여부
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