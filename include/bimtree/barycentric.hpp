#pragma once

#include "xyzpoint.hpp"

namespace geo
{
	struct TempBaryCentVar
	{
		pctree::XYZPoint v0;
		pctree::XYZPoint v1;
		double dot00, dot11, dot01;
		double invDenom;
	};

	/// Barycentric coordinates
	pctree::XYZPoint computeBarycentricCoord(const pctree::XYZPoint& P, const pctree::XYZPoint& A, const pctree::XYZPoint& B, const pctree::XYZPoint& C)
	{
		pctree::XYZPoint v0(C - A);
		pctree::XYZPoint v1(B - A);
		pctree::XYZPoint v2(P - A);

		/// Dot product
		auto dot00 = v0 * v0;
		auto dot01 = v0 * v1;
		auto dot02 = v0 * v2;
		auto dot11 = v1 * v1;
		auto dot12 = v1 * v2;

		/// Barycentric coordinates
		pctree::XYZPoint retval;
		double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
		retval[0] = static_cast<float>((dot11 * dot02 - dot01 * dot12) * invDenom);
		retval[1] = static_cast<float>((dot00 * dot12 - dot01 * dot02) * invDenom);
		retval[2] = static_cast<float>(1.0 - retval[0] - retval[1]);

		return retval;
	}

	/// Barycentric coordinates
	bool computeBarycentricCoord_step1(
		const pctree::XYZPoint& a, 
		const pctree::XYZPoint& b, 
		const pctree::XYZPoint& c,
		TempBaryCentVar& interMedia)
	{
		interMedia.v0 = c - a;
		interMedia.v1 = b - a;

		/// Dot product
		interMedia.dot00 = interMedia.v0 * interMedia.v0;
		interMedia.dot01 = interMedia.v0 * interMedia.v1;
		interMedia.dot11 = interMedia.v1 * interMedia.v1;

		double temp = (interMedia.dot00 * interMedia.dot11 - interMedia.dot01 * interMedia.dot01);
		if (fabs(temp) < std::numeric_limits<double>::epsilon())
			return false;

		interMedia.invDenom = 1.0 / (interMedia.dot00 * interMedia.dot11 - interMedia.dot01 * interMedia.dot01);
		return true;
	}

	/// Barycentric coordinates
	pctree::XYZPoint computeBarycentricCoord_step2(const pctree::XYZPoint& p,
		const pctree::XYZPoint& a,
		const TempBaryCentVar& interMedia)
	{
		pctree::XYZPoint v2(p - a);
		/// Dot product
		double dot02 = interMedia.v0 * v2;
		double dot12 = interMedia.v1 * v2;

		/// Barycentric coordinates
		pctree::XYZPoint retval;
		retval[0] = static_cast<float>((interMedia.dot11 * dot02 - interMedia.dot01 * dot12) * interMedia.invDenom);
		retval[1] = static_cast<float>((interMedia.dot00 * dot12 - interMedia.dot01 * dot02) * interMedia.invDenom);
		retval[2] = static_cast<float>(1.0 - retval[0] - retval[1]);

		return retval;
	}

	/// Check inside
	bool checkInside(const pctree::XYZPoint& uvw)
	{
		if (uvw[0] >= 0 && uvw[1] >= 0 && (uvw[0] + uvw[1]) <= 1)
			return true;
		else
			return false;
	}

	/// Check inside
	bool checkInside(const pctree::XYZPoint& p, const pctree::XYZPoint& a, const pctree::XYZPoint& b, const pctree::XYZPoint& c)
	{
		auto uvw = computeBarycentricCoord(p, a, b, c);
		return checkInside(uvw);
	}
}
