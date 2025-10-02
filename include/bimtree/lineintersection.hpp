#pragma once

#include <iostream>

namespace geo
{
	enum class IntersectionRetVal { PARALLEL = -3, SAME = -2, NOTLINE = -1, OUTOFRANGE = 0, FOUND = 1, ENDPOINT = 2 };

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

		/// It's a point (A)
		double distA = sqrt(dxA * dxA + dyA * dyA);
		if (fabs(distA) < std::numeric_limits<double>::epsilon())
			return IntersectionRetVal::NOTLINE;

		/// It's a point (B)
		double distB = sqrt(dxB * dxB + dyB * dyB);
		if (fabs(distB) < std::numeric_limits<double>::epsilon())
			return IntersectionRetVal::NOTLINE;

		/// Is the line A parallel to the y axis ?
		if (fabs(dxA) < std::numeric_limits<double>::epsilon())
		{
			/// Is the line B parallel to the y axis ?
			if (fabs(dxB) < std::numeric_limits<double>::epsilon())
			{
				/// Same x coordinate?
				if (fabs(x1 - x3) < std::numeric_limits<double>::epsilon())
					return IntersectionRetVal::SAME;
				else
					return IntersectionRetVal::PARALLEL;
			}
			/// Is the line B parallel to the x axis ?
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
		/// Is the line A parallel to the x axis ?
		else if (fabs(dyA) < std::numeric_limits<double>::epsilon())
		{
			/// Is the line B parallel to the y axis ?
			if (fabs(dxB) < std::numeric_limits<double>::epsilon())
			{
				y = y1;
				x = x3;
			}
			/// Is the line B parallel to the x axis ?
			else if (fabs(dyB) < std::numeric_limits<double>::epsilon())
			{
				/// Same x coordinate?
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
			/// Is the line B parallel to the y axis ?
			if (fabs(dxB) < std::numeric_limits<double>::epsilon())
			{
				x = x3;
				y = y1 + (x - x1) * (y2 - y1) / (x2 - x1);

				if ((x2 - x) * (x1 - x) < 0.0 && (y4 - y) * (y3 - y) < 0.0)
					return IntersectionRetVal::FOUND;
				else
					return IntersectionRetVal::OUTOFRANGE;
			}
			/// Is the line B parallel to the x axis ?
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
