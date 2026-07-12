#pragma once

/// ============================================================================
/// lineintersection.hpp - 2차원 선분 교차 판정
/// ============================================================================
///
/// 2차원 평면 위 두 선분(A: P0-P1, B: P0-P1)의 교차 여부와 교차점을 계산한다.
/// PseudoPtsInMesh.hpp에서 삼각형을 2차원 평면에 투영한 뒤, 격자선이 삼각형
/// 변과 만나는 지점을 찾는 데 사용된다.
/// ============================================================================

#include <iostream>

namespace geo
{
	/// 교차 판정 결과: PARALLEL(평행, 교차 없음), SAME(두 선이 사실상 같은 직선),
	/// NOTLINE(입력이 선이 아니라 점), OUTOFRANGE(직선끼리는 만나지만 선분 범위
	/// 밖), FOUND(선분 범위 내에서 교차점을 찾음), ENDPOINT(끝점에서 교차, 현재
	/// 미사용)
	enum class IntersectionRetVal { PARALLEL = -3, SAME = -2, NOTLINE = -1, OUTOFRANGE = 0, FOUND = 1, ENDPOINT = 2 };

	/// 두 2차원 선분(A: (x1,y1)-(x2,y2), B: (x3,y3)-(x4,y4))의 교차 여부를 판정하고,
	/// 교차점이 있으면 (x, y)에 담아 반환한다. 축에 평행한 특수 케이스들을 각각
	/// 분기 처리한 뒤, 일반적인 경우에는 매개변수 sa(선분 A를 따라가는 비율)를
	/// 구해 교차점을 계산한다.
	IntersectionRetVal Line2DIntersect(const double x1, const double y1, ///LINE_0_P0
		const double x2, const double y2, ///LINE_0_P1
		const double x3, const double y3, ///LINE_1_P0
		const double x4, const double y4, ///LINE_1_P1
		double& x, double& y)
	{
		double dxA = x2 - x1;
		double dyA = y2 - y1;
		double dxB = x4 - x3;
		double dyB = y4 - y3;

		/// 선분 A가 사실상 한 점인 경우
		double distA = sqrt(dxA * dxA + dyA * dyA);
		if (fabs(distA) < std::numeric_limits<double>::epsilon())
			return IntersectionRetVal::NOTLINE;

		/// 선분 B가 사실상 한 점인 경우
		double distB = sqrt(dxB * dxB + dyB * dyB);
		if (fabs(distB) < std::numeric_limits<double>::epsilon())
			return IntersectionRetVal::NOTLINE;

		/// 선분 A가 y축에 평행한가?
		if (fabs(dxA) < std::numeric_limits<double>::epsilon())
		{
			/// 선분 B도 y축에 평행한가?
			if (fabs(dxB) < std::numeric_limits<double>::epsilon())
			{
				/// x 좌표가 같은가?
				if (fabs(x1 - x3) < std::numeric_limits<double>::epsilon())
					return IntersectionRetVal::SAME;
				else
					return IntersectionRetVal::PARALLEL;
			}
			/// 선분 B가 x축에 평행한가?
			else if (fabs(dyB) < std::numeric_limits<double>::epsilon())
			{
				x = x1;
				y = y3;
			}
			else
			{
				x = x1;
				y = y3 + (x - x3) * (y4 - y3) / (x4 - x3);
			}

			if ((x3 - x) * (x4 - x) < 0.0 && (y1 - y) * (y2 - y) < 0.0)
				return IntersectionRetVal::FOUND;
			else
				return IntersectionRetVal::OUTOFRANGE;
		}
		/// 선분 A가 x축에 평행한가?
		else if (fabs(dyA) < std::numeric_limits<double>::epsilon())
		{
			/// 선분 B가 y축에 평행한가?
			if (fabs(dxB) < std::numeric_limits<double>::epsilon())
			{
				y = y1;
				x = x3;
			}
			/// 선분 B가 x축에 평행한가?
			else if (fabs(dyB) < std::numeric_limits<double>::epsilon())
			{
				/// y 좌표가 같은가?
				if (fabs(y1 - y3) < std::numeric_limits<double>::epsilon())
					return IntersectionRetVal::SAME;
				else
					return IntersectionRetVal::PARALLEL;
			}
			else
			{
				y = y1;
				x = x3 + (y - y3) * (x4 - x3) / (y4 - y3);
			}

			if ((x2 - x) * (x1 - x) < 0.0 && (y4 - y) * (y3 - y) < 0.0)
				return IntersectionRetVal::FOUND;
			else
				return IntersectionRetVal::OUTOFRANGE;
		}
		else
		{
			/// 선분 B가 y축에 평행한가?
			if (fabs(dxB) < std::numeric_limits<double>::epsilon())
			{
				x = x3;
				y = y1 + (x - x1) * (y2 - y1) / (x2 - x1);

				if ((x2 - x) * (x1 - x) < 0.0 && (y4 - y) * (y3 - y) < 0.0)
					return IntersectionRetVal::FOUND;
				else
					return IntersectionRetVal::OUTOFRANGE;
			}
			/// 선분 B가 x축에 평행한가?
			else if (fabs(dyB) < std::numeric_limits<double>::epsilon())
			{
				y = y3;
				x = x1 + (y - y1) * (x2 - x1) / (y2 - y1);

				if ((x4 - x) * (x3 - x) < 0.0 && (y2 - y) * (y1 - y) < 0.0)
					return IntersectionRetVal::FOUND;
				else
					return IntersectionRetVal::OUTOFRANGE;
			}
			else
			{
				/// 일반적인 경우: 선분 A를 매개변수화(sa)하여 교차점을 계산
				auto saN = (y1 - y3) * (x4 - x3) - (x1 - x3) * (y4 - y3);
				auto saD = (x2 - x1) * (y4 - y3) - (y2 - y1) * (x4 - x3);
				auto sa = saN / saD;

				x = x1 + sa * (x2 - x1);
				y = y1 + sa * (y2 - y1);

				if ((x2 - x) * (x1 - x) < 0.0 && (y2 - y) * (y1 - y) < 0.0 && (x4 - x) * (x3 - x) < 0.0 && (y4 - y) * (y3 - y) < 0.0)
					return IntersectionRetVal::FOUND;
				else
					return IntersectionRetVal::OUTOFRANGE;
			}
		}
	}
}
