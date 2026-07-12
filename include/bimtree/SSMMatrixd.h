/*
* Copyright(c) 2001-2025 KI IN Bang
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files(the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions :

* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#pragma once

#include <string>
#include "SSMMatDatad.h"

namespace math
{
	/// @class Matrixd
	/// @brief 행렬(Matrix) 연산을 위한 수학 클래스. IcpCore.hpp의 미사용 NLLS
	/// 실험 코드(estimateRigidTransformPointToPlane_NLLS, runLSM 등)와
	/// rotation.hpp의 회전행렬 편미분 계산에서 사용된다. Eigen과는 별개의,
	/// 이 프로젝트 이전부터 있던 자체 행렬 라이브러리다.
	class Matrixd : public MatDatad
	{
		template<size_t r, size_t c>
		friend class FixedSizeMatd;

		template<size_t d>
		friend class VectorMatd;

	public:
		/// 기본 생성자.
		Matrixd();

		/// 행/열 크기를 지정하는 생성자.
		Matrixd(const unsigned int r, const unsigned int c);

		/// 행/열 크기와 초기값을 지정하는 생성자.
		Matrixd(const unsigned int r, const unsigned int c, double val);

		/// 복사 생성자.
		Matrixd(const Matrixd& copy);

		/// 이동 생성자.
		Matrixd(Matrixd&& other) noexcept;

		/// 소멸자.
		virtual ~Matrixd();

		/// 기존 행렬 아래에 행(row)을 이어붙인다.
		/// |A| addRows |B| = |A|
		///                   |B|
		void addRows(const Matrixd& copy);

		/// 기존 행렬 오른쪽에 열(column)을 이어붙인다.
		/// |A| addCols |B| = |A||B|
		void addCols(const Matrixd& copy);

		/// 두 행렬을 대각선 방향으로 결합한다.
		/// |A| add |B| = |A| 0
		///               0 |B|
		void addRowsCols(const Matrixd& copy);

		/// 지정된 위치에 행렬을 삽입한다.
		/// 삽입 범위가 기존 행렬을 벗어나면 크기를 늘린다.
		void insert(const unsigned int rIdx, const unsigned int cIdx, const Matrixd& copy);

		/// 지정된 위치에 행렬 값을 누적(+=)한다.
		/// 범위를 벗어나면 오류를 던진다.
		void insertPlus(const unsigned int rIdx, const unsigned int cIdx, const Matrixd& copy);

		/// 지정된 위치에 행렬 값을 누적(+=)한다.
		/// 범위를 벗어나면 크기를 늘린다.
		void insertPlusResize(const unsigned int rIdx, const unsigned int cIdx, const Matrixd& copy);

		/// 기존 행렬에서 부분 행렬(submatrix)을 추출한다.
		Matrixd getSubset(const unsigned int rIdx, const unsigned int cIdx, const unsigned int size_r, const unsigned int size_c) const;

		/// 절대값이 가장 큰 원소를 반환.
		double getMaxabsElement() const;

		/// 가장 큰 원소를 반환.
		double getMaxElement() const;

		/// 절대값이 가장 작은 원소를 반환.
		double getMinabsElement() const;

		/// 가장 작은 원소를 반환.
		double getMinElement() const;

		/// 행/열 인덱스로부터 1차원 배열 인덱스를 계산.
		unsigned int findIndex(const unsigned int rIdx, const unsigned int cIdx) const;

		/// 단위 행렬(identity matrix)을 만든다.
		void makeIdentityMat(const double& init_val = 1.0);

		/// 전치 행렬(transpose)을 생성.
		Matrixd transpose() const;

		/// 역행렬(inverse)을 생성.
		Matrixd inverse() const;

		// 연산자
	public:
		/// 대입 연산자.
		Matrixd& operator=(const Matrixd& copy);

		/// 이동 대입 연산자.
		Matrixd& operator=(Matrixd&& other) noexcept;

		/// 연산자 + (행렬 + 행렬).
		Matrixd operator+(const Matrixd& mat) const;

		/// 연산자 += (행렬 += 행렬).
		void operator+=(const Matrixd& mat);

		/// 연산자 + (행렬 + 스칼라).
		Matrixd operator+(const double& val) const;

		/// 연산자 += (행렬 += 스칼라).
		void operator+=(const double& val);

		/// 연산자 - (단항 부호 반전).
		Matrixd operator-() const;

		/// 연산자 - (행렬 - 행렬).
		Matrixd operator-(const Matrixd& mat) const;

		/// 연산자 -= (행렬 -= 행렬).
		void operator-=(const Matrixd& mat);

		/// 연산자 - (행렬 - 스칼라).
		Matrixd operator-(const double& val) const;

		/// 연산자 -= (행렬 -= 스칼라).
		void operator-=(const double& val);

		/// 연산자 * (원소별 곱셈).
		Matrixd operator*(const Matrixd& mat) const;

		/// 연산자 *= (원소별 곱셈).
		void operator*=(const Matrixd& mat);

		/// 연산자 * (스칼라 곱셈).
		Matrixd operator*(const double& val) const;

		/// 연산자 *= (스칼라 곱셈).
		void operator*=(const double& val);

		/// 연산자 / (원소별 나눗셈).
		Matrixd operator/(const Matrixd& mat) const;

		/// 연산자 /= (원소별 나눗셈).
		void operator/=(const Matrixd& mat);

		/// 연산자 / (스칼라 나눗셈).
		Matrixd operator/(const double& val) const;

		/// 연산자 /= (스칼라 나눗셈).
		void operator/=(const double& val);

		/// 연산자 % (행렬 곱셈, matrix multiplication). 이 라이브러리에서는
		/// operator*가 원소별 곱셈이므로, 진짜 행렬곱은 %로 구분해서 제공한다.
		Matrixd operator%(const Matrixd& mat) const;

		/// 모든 원소의 합을 반환.
		double getSum() const;

		/// 모든 원소의 제곱합을 반환.
		double getSumOfSquares() const;

		/// 연산자 (): 원소 접근 (0-based 인덱스).
		double& operator()(const unsigned int rIdx, const unsigned int cIdx);
		const double& operator()(const unsigned int rIdx, const unsigned int cIdx) const;

		/// 연산자 []: 선형 인덱스로 원소 접근.
		double& operator[](const unsigned int idx);
		const double& operator[](const unsigned int idx) const;

		/// 연산자 == 비교.
		bool operator==(const Matrixd& copy) const;

		/// 연산자 != 비교.
		bool operator!=(const Matrixd& copy) const;

	private:
		/// LU 분해.
		Matrixd LUDcompose();

		/// LU 분해를 이용한 역행렬 계산.
		Matrixd LUInvert(Matrixd& perm);

		/// LU 분해를 이용한 선형방정식 풀이.
		Matrixd LUSolve(Matrixd& perm, Matrixd& b);
	};

	/// 분할 행렬(partitioned matrix)의 역행렬 계산.
	Matrixd inversePartitionedMatrixd(const Matrixd& a, const Matrixd& b, const Matrixd& c, const Matrixd& d);

	/// 행렬을 문자열로 출력.
	std::string matrixout(const Matrixd& mat);
}
