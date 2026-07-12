#pragma once

/// ============================================================================
/// rotation.hpp - 회전 행렬 유틸리티
/// ============================================================================
///
/// 사진측량학(photogrammetry)에서 흔히 쓰는 omega(오메가)/phi(파이)/kappa(카파)
/// 오일러 각도 표현으로부터 회전 행렬을 만들고, 그 편미분(각 각도에 대한
/// 회전 행렬의 변화율)까지 계산한다. getRMat/getPartial_dRdO/dRdP/dRdK는
/// IcpCore.hpp의 미완성/미사용 NLLS(비선형 최소제곱) 실험 코드에서만 쓰인다.
/// getPlaneRotationMatrix는 별개의 함수로, 삼각형 세 점으로부터 그 평면의
/// 정규직교 기저(orthonormal basis)를 구성하며 Mesh.hpp/PseudoPtsInMesh.hpp의
/// barycentric 계산에 실제로 쓰인다.
/// ============================================================================

#include "xyzpoint.hpp"

namespace math
{
	/// omega/phi/kappa 각도로부터 3x3 회전 행렬 R을 계산 (사진측량학의 표준
	/// 회전 관례 R = Rx(omega)*Ry(phi)*Rz(kappa))
	inline math::Matrixd getRMat(const double omega, const double phi, const double kappa)
	{
		math::Matrixd Rmatrix(3, 3);
		Rmatrix(0, 0) = cos(phi) * cos(kappa);
		Rmatrix(1, 0) = sin(omega) * sin(phi) * cos(kappa) + cos(omega) * sin(kappa);
		Rmatrix(2, 0) = -cos(omega) * sin(phi) * cos(kappa) + sin(omega) * sin(kappa);

		Rmatrix(0, 1) = -cos(phi) * sin(kappa);
		Rmatrix(1, 1) = -sin(omega) * sin(phi) * sin(kappa) + cos(omega) * cos(kappa);
		Rmatrix(2, 1) = cos(omega) * sin(phi) * sin(kappa) + sin(omega) * cos(kappa);

		Rmatrix(0, 2) = sin(phi);
		Rmatrix(1, 2) = -sin(omega) * cos(phi);
		Rmatrix(2, 2) = cos(omega) * cos(phi);

		return Rmatrix;
	}

	/// R을 omega에 대해 편미분한 행렬 dR/domega
	inline math::Matrixd getPartial_dRdO(double omega, double phi, double kappa)
	{
		math::Matrixd Rmatrix(3, 3);

		Rmatrix(0, 0) = 0;
		Rmatrix(0, 1) = 0;
		Rmatrix(0, 2) = 0;

		Rmatrix(1, 0) = cos(omega) * sin(phi) * cos(kappa) - sin(omega) * sin(kappa);
		Rmatrix(1, 1) = -cos(omega) * sin(phi) * sin(kappa) - sin(omega) * cos(kappa);
		Rmatrix(1, 2) = -cos(omega) * cos(phi);

		Rmatrix(2, 0) = sin(omega) * sin(phi) * cos(kappa) + cos(omega) * sin(kappa);
		Rmatrix(2, 1) = -sin(omega) * sin(phi) * sin(kappa) + cos(omega) * cos(kappa);
		Rmatrix(2, 2) = -sin(omega) * cos(phi);

		return Rmatrix;
	}

	/// R을 phi에 대해 편미분한 행렬 dR/dphi
	inline math::Matrixd getPartial_dRdP(double omega, double phi, double kappa)
	{
		math::Matrixd Rmatrix(3, 3);

		Rmatrix(0, 0) = -sin(phi) * cos(kappa);
		Rmatrix(0, 1) = sin(phi) * sin(kappa);
		Rmatrix(0, 2) = cos(phi);

		Rmatrix(1, 0) = sin(omega) * cos(phi) * cos(kappa);
		Rmatrix(1, 1) = -sin(omega) * cos(phi) * sin(kappa);
		Rmatrix(1, 2) = sin(omega) * sin(phi);

		Rmatrix(2, 0) = -cos(omega) * cos(phi) * cos(kappa);
		Rmatrix(2, 1) = cos(omega) * cos(phi) * sin(kappa);
		Rmatrix(2, 2) = -cos(omega) * sin(phi);

		return Rmatrix;
	}

	/// R을 kappa에 대해 편미분한 행렬 dR/dkappa
	inline math::Matrixd getPartial_dRdK(double omega, double phi, double kappa)
	{
		math::Matrixd Rmatrix(3, 3);

		Rmatrix(0, 0) = -cos(phi) * sin(kappa);
		Rmatrix(0, 1) = -cos(phi) * cos(kappa);
		Rmatrix(0, 2) = 0.;

		Rmatrix(1, 0) = -sin(omega) * sin(phi) * sin(kappa) + cos(omega) * cos(kappa);
		Rmatrix(1, 1) = -sin(omega) * sin(phi) * cos(kappa) - cos(omega) * sin(kappa);
		Rmatrix(1, 2) = 0.;

		Rmatrix(2, 0) = cos(omega) * sin(phi) * sin(kappa) + sin(omega) * cos(kappa);
		Rmatrix(2, 1) = cos(omega) * sin(phi) * cos(kappa) - sin(omega) * sin(kappa);
		Rmatrix(2, 2) = 0.;

		return Rmatrix;
	}

	/// 삼각형 세 점 (a, b, c)로부터 그 평면의 정규직교(orthonormal) 기저를 구성한다:
	/// X축은 a→b 방향, Z축은 삼각형의 normal(a→b와 a→c의 외적) 방향, Y축은 Z×X로
	/// 나머지 축을 완성한다. 세 점이 일직선(면적 0)이면 false를 반환한다.
	inline bool getPlaneRotationMatrix(const pctree::XYZPoint& a, const pctree::XYZPoint& b, const pctree::XYZPoint& c, pctree::MatrixForMesh& R)
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

		//X축
		double XX1 = v2x / Dx;
		double XX2 = v2y / Dx;
		double XX3 = v2z / Dx;

		//Z축
		double ZZ1 = v2y * v3z - v2z * v3y;
		double ZZ2 = v2z * v3x - v2x * v3z;
		double ZZ3 = v2x * v3y - v2y * v3x;

		double Dz = sqrt(ZZ1 * ZZ1 + ZZ2 * ZZ2 + ZZ3 * ZZ3);

		if (fabs(Dz) < std::numeric_limits<double>::epsilon())
			return false;

		ZZ1 = ZZ1 / Dz;
		ZZ2 = ZZ2 / Dz;
		ZZ3 = ZZ3 / Dz;

		//Y축

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
