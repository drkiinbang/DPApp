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
	/// @brief Mathematics class for manipulating a Matrixd.
	class Matrixd : public MatDatad
	{
		template<size_t r, size_t c>
		friend class FixedSizeMatd;

		template<size_t d>
		friend class VectorMatd;

	public:
		/// Default constructor.
		Matrixd();

		/// Constructor with dimensions.
		Matrixd(const unsigned int r, const unsigned int c);

		/// Constructor with dimensions and initial value.
		Matrixd(const unsigned int r, const unsigned int c, double val);

		/// Copy constructor.
		Matrixd(const Matrixd& copy);

		/// Move constructor.
		Matrixd(Matrixd&& other) noexcept;

		/// Destructor.
		virtual ~Matrixd();

		/// Append rows to the existing matrix.
		/// |A| addRows |B| = |A|
		///                   |B|
		void addRows(const Matrixd& copy);

		/// Append columns to the existing matrix.
		/// |A| addCols |B| = |A||B|
		void addCols(const Matrixd& copy);

		/// Bind two matrices diagonally.
		/// |A| add |B| = |A| 0
		///               0 |B|
		void addRowsCols(const Matrixd& copy);

		/// Insert a matrix at a specific position.
		/// Resizes the matrix if the insertion goes out of bounds.
		void insert(const unsigned int rIdx, const unsigned int cIdx, const Matrixd& copy);

		/// Accumulate a matrix at a specific position (+=).
		/// Throws error if out of bounds.
		void insertPlus(const unsigned int rIdx, const unsigned int cIdx, const Matrixd& copy);

		/// Accumulate a matrix at a specific position (+=).
		/// Resizes the matrix if out of bounds.
		void insertPlusResize(const unsigned int rIdx, const unsigned int cIdx, const Matrixd& copy);

		/// Get a subset (submatrix) from the existing matrix.
		Matrixd getSubset(const unsigned int rIdx, const unsigned int cIdx, const unsigned int size_r, const unsigned int size_c) const;

		/// Get the maximum absolute value element.
		double getMaxabsElement() const;

		/// Get the maximum element.
		double getMaxElement() const;

		/// Get the minimum absolute value element.
		double getMinabsElement() const;

		/// Get the minimum element.
		double getMinElement() const;

		/// Calculate 1D array index from row and column indices.
		unsigned int findIndex(const unsigned int rIdx, const unsigned int cIdx) const;

		/// Make an identity matrix.
		void makeIdentityMat(const double& init_val = 1.0);

		/// Create a transposed matrix.
		Matrixd transpose() const;

		/// Create an inverse matrix.
		Matrixd inverse() const;

		// Operators
	public:
		/// Assignment operator.
		Matrixd& operator=(const Matrixd& copy);

		/// Move assignment operator.
		Matrixd& operator=(Matrixd&& other) noexcept;

		/// Operator + (Matrix + Matrix).
		Matrixd operator+(const Matrixd& mat) const;

		/// Operator += (Matrix += Matrix).
		void operator+=(const Matrixd& mat);

		/// Operator + (Matrix + Scalar).
		Matrixd operator+(const double& val) const;

		/// Operator += (Matrix += Scalar).
		void operator+=(const double& val);

		/// Operator - (Unary Negation).
		Matrixd operator-() const;

		/// Operator - (Matrix - Matrix).
		Matrixd operator-(const Matrixd& mat) const;

		/// Operator -= (Matrix -= Matrix).
		void operator-=(const Matrixd& mat);

		/// Operator - (Matrix - Scalar).
		Matrixd operator-(const double& val) const;

		/// Operator -= (Matrix -= Scalar).
		void operator-=(const double& val);

		/// Operator * (Element-wise multiplication).
		Matrixd operator*(const Matrixd& mat) const;

		/// Operator *= (Element-wise multiplication).
		void operator*=(const Matrixd& mat);

		/// Operator * (Scalar multiplication).
		Matrixd operator*(const double& val) const;

		/// Operator *= (Scalar multiplication).
		void operator*=(const double& val);

		/// Operator / (Element-wise division).
		Matrixd operator/(const Matrixd& mat) const;

		/// Operator /= (Element-wise division).
		void operator/=(const Matrixd& mat);

		/// Operator / (Scalar division).
		Matrixd operator/(const double& val) const;

		/// Operator /= (Scalar division).
		void operator/=(const double& val);

		/// Operator % (Matrix Multiplication).
		Matrixd operator%(const Matrixd& mat) const;

		/// Get the sum of all elements.
		double getSum() const;

		/// Get the sum of squares of all elements.
		double getSumOfSquares() const;

		/// Operator (): Access an element (0-based index).
		double& operator()(const unsigned int rIdx, const unsigned int cIdx);
		const double& operator()(const unsigned int rIdx, const unsigned int cIdx) const;

		/// Operator []: Access an element by linear index.
		double& operator[](const unsigned int idx);
		const double& operator[](const unsigned int idx) const;

		/// Operator == check.
		bool operator==(const Matrixd& copy) const;

		/// Operator != check.
		bool operator!=(const Matrixd& copy) const;

	private:
		/// LU Decompose.
		Matrixd LUDcompose();

		/// LU Invert.
		Matrixd LUInvert(Matrixd& perm);

		/// LU Solve.
		Matrixd LUSolve(Matrixd& perm, Matrixd& b);
	};

	/// Inverse partitioned matrix.
	Matrixd inversePartitionedMatrixd(const Matrixd& a, const Matrixd& b, const Matrixd& c, const Matrixd& d);

	/// Output matrix as string.
	std::string matrixout(const Matrixd& mat);
}