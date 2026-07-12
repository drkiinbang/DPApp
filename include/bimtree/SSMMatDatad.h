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

#include <vector>
#include <stdexcept>

namespace math
{
	/// @class MatDatad
	/// @brief std::vector로 관리되는, double 타입 기반 행렬 연산용 데이터 클래스.
	/// Matrixd(SSMMatrixd.h)의 기반(base) 클래스로, 실제 저장소와 크기 관리만 담당한다.
	class MatDatad
	{
	public:
		/// 기본 생성자.
		MatDatad();

		/// 크기를 지정하는 생성자.
		MatDatad(const unsigned int r, const unsigned int c);

		/// 크기와 초기값을 지정하는 생성자.
		MatDatad(const unsigned int r, const unsigned int c, const double val);

		/// 복사 생성자.
		MatDatad(const MatDatad& copy);

		/// 이동 생성자.
		MatDatad(MatDatad&& other) noexcept;

		/// 소멸자.
		virtual ~MatDatad();

		/// 복사 대입 연산자.
		MatDatad& operator=(const MatDatad& copy);

		/// 이동 대입 연산자.
		MatDatad& operator=(MatDatad&& other) noexcept;

		/// 데이터를 비우고 크기를 초기화.
		void del();

		/// 행(row) 개수 반환.
		unsigned int getRows() const;

		/// 열(column) 개수 반환.
		unsigned int getCols() const;

		/// 전체 데이터 크기 반환 (rows * cols).
		unsigned int getSize() const;

		/// 데이터에 대한 포인터 반환 (수정 가능).
		double* getDataHandle();

		/// 데이터에 대한 const 포인터 반환 (읽기 전용).
		const double* getAccessData() const;

		/// 행렬 크기 변경.
		void resize(const unsigned int r, const unsigned int c, double val = 0.0);

	protected:
		/// 메모리 할당 (내부용, vector resize를 감쌈).
		void allocateMem(const unsigned int r, const unsigned int c);

		/// 데이터를 특정 값으로 초기화.
		void initialize(double val);

	protected:
		std::vector<double> data; /// 행렬 원소를 담는 컨테이너.
		unsigned int rows;        /// 행 개수.
		unsigned int cols;        /// 열 개수.
	};

	/// @class Arrayd
	/// @brief 1차원 배열(열 벡터)을 위한 헬퍼 클래스.
	class Arrayd : public MatDatad
	{
	public:
		Arrayd() : MatDatad() {}

		Arrayd(const unsigned int r) : MatDatad(r, 1) {}

		Arrayd(const unsigned int r, const double val) : MatDatad(r, 1, val) {}

		Arrayd(const Arrayd& copy) : MatDatad(copy) {}

		virtual ~Arrayd() = default;

		Arrayd& operator=(const Arrayd& copy)
		{
			if (this != &copy) {
				MatDatad::operator=(copy);
			}
			return *this;
		}

		double& operator[](unsigned int idx)
		{
			return data[idx];
		}

		const double& operator[](unsigned int idx) const
		{
			return data[idx];
		}

		/// 기본값을 지정해 배열 크기 변경.
		void resize(const unsigned int r, double val = 0.0)
		{
			MatDatad::resize(r, 1, val);
		}
	};
}