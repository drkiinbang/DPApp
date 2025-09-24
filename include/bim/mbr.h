#pragma once

#include <algorithm>
#include <vector>
#include <array>

//#include "../../change_detection_slave/change_detection_slave/ChangeDetection.h"
#include "XYZ.h"

constexpr auto NUM_MAX = 1000000.0f;
constexpr auto NUM_MIN = -9999999.0f;

class MBR
{
public:
	MBR() = default;
	MBR(const XYZ& from, const XYZ& to) : from(from), to(to) {}

	bool is_intersect(MBR& other) const
	{
		if ((from.x <= other.from.x && to.x >= other.from.x) &&
			(from.y <= other.from.y && to.y >= other.from.y) &&
			(from.z <= other.from.z && to.z >= other.from.z))
		{
			return true;
		}

		if ((from.x <= other.to.x && to.x >= other.to.x) &&
			(from.y <= other.to.y && to.y >= other.to.y) &&
			(from.z <= other.to.z && to.z >= other.to.z))
		{
			return true;
		}

		return false;
	}

	std::array<float, 3> get_min() const{
		return { from.x, from.y, from.z };
	}
	std::array<float, 3> get_max() const{
		return { to.x, to.y, to.z };
	}

private:
	XYZ from;
	XYZ to;
};

