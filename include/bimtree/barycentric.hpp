#pragma once

#include "xyzpoint.hpp"

namespace geo
{
	/// computeBarycentricCoord_step1/step2로 나눠 계산할 때, 삼각형(a,b,c)에만
	/// 의존하는 중간값(step1 결과)을 보관해 두는 캐시. 같은 삼각형에 대해 여러
	/// 점 p의 barycentric 좌표를 반복 계산할 때(예: 메시 표면에 가상점을 여러 개
	/// 뿌릴 때) 삼각형 쪽 연산을 매번 다시 하지 않도록 하기 위함이다.
	struct TempBaryCentVar
	{
		pctree::XYZPoint v0;
		pctree::XYZPoint v1;
		double dot00, dot11, dot01;
		double invDenom;
	};

	/// Barycentric 좌표 계산 (삼각형 A,B,C에 대한 점 P의 무게중심좌표를 한 번에 계산)
	pctree::XYZPoint computeBarycentricCoord(const pctree::XYZPoint& P, const pctree::XYZPoint& A, const pctree::XYZPoint& B, const pctree::XYZPoint& C)
	{
		pctree::XYZPoint v0(C - A);
		pctree::XYZPoint v1(B - A);
		pctree::XYZPoint v2(P - A);

		/// 내적(dot product)
		auto dot00 = v0 * v0;
		auto dot01 = v0 * v1;
		auto dot02 = v0 * v2;
		auto dot11 = v1 * v1;
		auto dot12 = v1 * v2;

		/// Barycentric 좌표
		pctree::XYZPoint retval;
		double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
		retval[0] = static_cast<float>((dot11 * dot02 - dot01 * dot12) * invDenom);
		retval[1] = static_cast<float>((dot00 * dot12 - dot01 * dot02) * invDenom);
		retval[2] = static_cast<float>(1.0 - retval[0] - retval[1]);

		return retval;
	}

	/// Barycentric 좌표 계산 - 1단계: 삼각형(a,b,c)에만 의존하는 부분을 미리
	/// 계산해 interMedia에 캐싱한다. 삼각형이 퇴화(면적 0)되어 있으면 false 반환.
	bool computeBarycentricCoord_step1(
		const pctree::XYZPoint& a,
		const pctree::XYZPoint& b,
		const pctree::XYZPoint& c,
		TempBaryCentVar& interMedia)
	{
		interMedia.v0 = c - a;
		interMedia.v1 = b - a;

		/// 내적(dot product)
		interMedia.dot00 = interMedia.v0 * interMedia.v0;
		interMedia.dot01 = interMedia.v0 * interMedia.v1;
		interMedia.dot11 = interMedia.v1 * interMedia.v1;

		double temp = (interMedia.dot00 * interMedia.dot11 - interMedia.dot01 * interMedia.dot01);
		if (fabs(temp) < std::numeric_limits<double>::epsilon())
			return false;

		interMedia.invDenom = 1.0 / (interMedia.dot00 * interMedia.dot11 - interMedia.dot01 * interMedia.dot01);
		return true;
	}

	/// Barycentric 좌표 계산 - 2단계: step1에서 캐싱해 둔 삼각형 쪽 값을 재사용해,
	/// 점 p 하나의 barycentric 좌표만 계산한다.
	pctree::XYZPoint computeBarycentricCoord_step2(const pctree::XYZPoint& p,
		const pctree::XYZPoint& a,
		const TempBaryCentVar& interMedia)
	{
		pctree::XYZPoint v2(p - a);
		/// 내적(dot product)
		double dot02 = interMedia.v0 * v2;
		double dot12 = interMedia.v1 * v2;

		/// Barycentric 좌표
		pctree::XYZPoint retval;
		retval[0] = static_cast<float>((interMedia.dot11 * dot02 - interMedia.dot01 * dot12) * interMedia.invDenom);
		retval[1] = static_cast<float>((interMedia.dot00 * dot12 - interMedia.dot01 * dot02) * interMedia.invDenom);
		retval[2] = static_cast<float>(1.0 - retval[0] - retval[1]);

		return retval;
	}

	/// barycentric 좌표(uvw)가 삼각형 내부에 있는지 확인
	bool checkInside(const pctree::XYZPoint& uvw)
	{
		if (uvw[0] >= 0 && uvw[1] >= 0 && (uvw[0] + uvw[1]) <= 1)
			return true;
		else
			return false;
	}

	/// 점 p가 삼각형(a,b,c) 내부에 있는지 확인 (barycentric 좌표를 계산한 뒤 판정)
	bool checkInside(const pctree::XYZPoint& p, const pctree::XYZPoint& a, const pctree::XYZPoint& b, const pctree::XYZPoint& c)
	{
		auto uvw = computeBarycentricCoord(p, a, b, c);
		return checkInside(uvw);
	}
}
