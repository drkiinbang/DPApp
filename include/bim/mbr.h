#pragma once

#include <algorithm>
#include <vector>
#include <array>

//#include "../../change_detection_slave/change_detection_slave/ChangeDetection.h"
#include "XYZ.h"

constexpr auto NUM_MAX = 1000000.0f;
constexpr auto NUM_MIN = -9999999.0f;

/// MBR(Minimum Bounding Rectangle/Box) — 축 정렬 바운딩 박스를 [from, to] 두 모서리
/// 좌표(XYZ)로 표현하는 클래스. BIM 부재(BimInfo::get_mbr)나 청크의 대략적인
/// 공간 범위를 나타내고, is_intersect()로 두 박스가 겹치는지 빠르게 판별하는 데 쓰인다.
class MBR
{
public:
	MBR() = default;
	MBR(const XYZ& from, const XYZ& to) : from(from), to(to) {}

	/// 이 박스와 other 박스가 서로 겹치는지 검사한다.
	/// [주의] 아래는 other의 두 모서리점(from/to)이 this 박스 안에 들어오는지만
	/// 확인하는 방식이라, this의 모서리점이 other 내부에 있는 경우(예: this가
	/// other보다 훨씬 작고 완전히 그 안에 포함된 경우)는 놓칠 수 있다(비대칭 검사).
	/// 완전한 AABB 교차 판정이 필요하면 두 방향 모두 확인해야 한다.
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

