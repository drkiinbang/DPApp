#pragma once

#include "xyzpoint.hpp"

namespace math
{
	bool getPlaneRotationMatrix(const pctree::XYZPoint& a, const pctree::XYZPoint& b, const pctree::XYZPoint& c, pctree::MatrixForMesh& R)
	{
		auto& v1 = a.xyz;
		auto& v2 = b.xyz;
		auto& v3 = c.xyz;

		double v2x, v2y, v2z;
		v2x = static_cast<double>(v2[0] - v1[0]);
		v2y = static_cast<double>(v2[1] - v1[1]);
		v2z = static_cast<double>(v2[2] - v1[2]);

		double v3x, v3y, v3z;

		v3x = v3[0] - v1[0];
		v3y = v3[1] - v1[1];
		v3z = v3[2] - v1[2];

		double Dx = sqrt(v2x * v2x + v2y * v2y + v2z * v2z);

		if (fabs(Dx) < std::numeric_limits<double>::epsilon())
			return false;

		//X axis
		double XX1 = v2x / Dx;
		double XX2 = v2y / Dx;
		double XX3 = v2z / Dx;

		//Z axis
		double ZZ1 = v2y * v3z - v2z * v3y;
		double ZZ2 = v2z * v3x - v2x * v3z;
		double ZZ3 = v2x * v3y - v2y * v3x;

		double Dz = sqrt(ZZ1 * ZZ1 + ZZ2 * ZZ2 + ZZ3 * ZZ3);

		if (fabs(Dz) < std::numeric_limits<double>::epsilon())
			return false;

		ZZ1 = ZZ1 / Dz;
		ZZ2 = ZZ2 / Dz;
		ZZ3 = ZZ3 / Dz;

		//Y axis

		double YY1 = ZZ2 * XX3 - ZZ3 * XX2;
		double YY2 = ZZ3 * XX1 - ZZ1 * XX3;
		double YY3 = ZZ1 * XX2 - ZZ2 * XX1;

		double Dy = sqrt(YY1 * YY1 + YY2 * YY2 + YY3 * YY3);

		if (fabs(Dy) < std::numeric_limits<double>::epsilon())
			return false;

		YY1 = YY1 / Dy;
		YY2 = YY2 / Dy;
		YY3 = YY3 / Dy;

		R.r[0] = XX1; R.r[1] = XX2; R.r[2] = XX3;
		R.r[3] = YY1; R.r[4] = YY2; R.r[5] = YY3;
		R.r[6] = ZZ1; R.r[7] = ZZ2; R.r[8] = ZZ3;

		return true;
	}
}
