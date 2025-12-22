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

#include "SSMMatrixd.h"
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <limits>
#include <cfloat>

namespace math
{
	Matrixd::Matrixd() : MatDatad() {}

	Matrixd::Matrixd(const unsigned int r, const unsigned int c) : MatDatad(r, c) {}

	Matrixd::Matrixd(const unsigned int r, const unsigned int c, double val) : MatDatad(r, c, val) {}

	Matrixd::Matrixd(const Matrixd& copy) : MatDatad(copy) {}

	Matrixd::Matrixd(Matrixd&& other) noexcept : MatDatad(std::move(other)) {}

	Matrixd::~Matrixd() = default;

	/// Append rows to the existing matrix.
	void Matrixd::addRows(const Matrixd& copy)
	{
		if (this->getCols() != copy.getCols())
			throw std::runtime_error("addRows: column count mismatch.");

		unsigned int oldRows = this->getRows();
		unsigned int addedRows = copy.getRows();

		// Resize underlying vector
		this->allocateMem(oldRows + addedRows, this->getCols());

		// Copy new data
		// Since std::vector is contiguous, we can use std::copy or loops.
		for (unsigned int i = 0; i < addedRows; ++i)
		{
			for (unsigned int j = 0; j < this->getCols(); ++j)
			{
				this->operator()(oldRows + i, j) = copy(i, j);
			}
		}
	}

	/// Append columns to the existing matrix.
	void Matrixd::addCols(const Matrixd& copy)
	{
		if (this->getRows() != copy.getRows())
			throw std::runtime_error("addCols: row count mismatch.");

		Matrixd newMat(this->getRows(), this->getCols() + copy.getCols());

		for (unsigned int i = 0; i < this->getRows(); ++i)
		{
			for (unsigned int j = 0; j < this->getCols(); ++j)
			{
				newMat(i, j) = this->operator()(i, j);
			}
			for (unsigned int j = 0; j < copy.getCols(); ++j)
			{
				newMat(i, this->getCols() + j) = copy(i, j);
			}
		}

		*this = std::move(newMat);
	}

	/// Bind two matrices diagonally.
	void Matrixd::addRowsCols(const Matrixd& copy)
	{
		Matrixd newMat(this->getRows() + copy.getRows(), this->getCols() + copy.getCols(), 0.0);

		// Copy A
		for (unsigned int i = 0; i < this->getRows(); ++i)
		{
			for (unsigned int j = 0; j < this->getCols(); ++j)
			{
				newMat(i, j) = this->operator()(i, j);
			}
		}

		// Copy B
		for (unsigned int i = 0; i < copy.getRows(); ++i)
		{
			for (unsigned int j = 0; j < copy.getCols(); ++j)
			{
				newMat(this->getRows() + i, this->getCols() + j) = copy(i, j);
			}
		}

		*this = std::move(newMat);
	}

	/// Insert a matrix at a specific position.
	void Matrixd::insert(const unsigned int rIdx, const unsigned int cIdx, const Matrixd& copy)
	{
		if ((this->getRows() < (rIdx + copy.getRows())) || (this->getCols() < (cIdx + copy.getCols())))
		{
			unsigned int newrows = std::max(this->getRows(), rIdx + copy.getRows());
			unsigned int newcols = std::max(this->getCols(), cIdx + copy.getCols());

			Matrixd backup = std::move(*this);
			this->resize(newrows, newcols, 0.0);

			for (unsigned int i = 0; i < backup.getRows(); ++i)
			{
				for (unsigned int j = 0; j < backup.getCols(); ++j)
				{
					this->operator()(i, j) = backup(i, j);
				}
			}
		}

		for (unsigned int i = 0; i < copy.getRows(); ++i)
		{
			for (unsigned int j = 0; j < copy.getCols(); ++j)
			{
				this->operator()(rIdx + i, cIdx + j) = copy(i, j);
			}
		}
	}

	/// Accumulate a matrix at a specific position (+=).
	void Matrixd::insertPlus(const unsigned int rIdx, const unsigned int cIdx, const Matrixd& copy)
	{
		if ((this->getRows() < (rIdx + copy.getRows())) || (this->getCols() < (cIdx + copy.getCols())))
		{
			throw std::runtime_error("insertPlus: Inserted matrix is too large.");
		}

		for (unsigned int i = 0; i < copy.getRows(); ++i)
		{
			for (unsigned int j = 0; j < copy.getCols(); ++j)
			{
				this->operator()(rIdx + i, cIdx + j) += copy(i, j);
			}
		}
	}

	/// Accumulate a matrix at a specific position (+=) with resize.
	void Matrixd::insertPlusResize(const unsigned int rIdx, const unsigned int cIdx, const Matrixd& copy)
	{
		if ((this->getRows() < (rIdx + copy.getRows())) || (this->getCols() < (cIdx + copy.getCols())))
		{
			unsigned int newrows = std::max(this->getRows(), rIdx + copy.getRows());
			unsigned int newcols = std::max(this->getCols(), cIdx + copy.getCols());

			Matrixd backup = std::move(*this);
			this->resize(newrows, newcols, 0.0);

			for (unsigned int i = 0; i < backup.getRows(); ++i)
			{
				for (unsigned int j = 0; j < backup.getCols(); ++j)
				{
					this->operator()(i, j) = backup(i, j);
				}
			}
		}
		insertPlus(rIdx, cIdx, copy);
	}

	/// Get a subset from an existing matrix.
	Matrixd Matrixd::getSubset(const unsigned int rIdx, const unsigned int cIdx, const unsigned int size_r, const unsigned int size_c) const
	{
		if ((this->getRows() < (rIdx + size_r)) || (this->getCols() < (cIdx + size_c)))
		{
			throw std::runtime_error("getSubset: Out of range.");
		}

		Matrixd ret(size_r, size_c);
		for (unsigned int i = 0; i < size_r; ++i)
		{
			for (unsigned int j = 0; j < size_c; ++j)
			{
				ret(i, j) = operator()(rIdx + i, cIdx + j);
			}
		}
		return ret;
	}

	/// Get a max absolute value element.
	double Matrixd::getMaxabsElement() const
	{
		double val = 0.0;
		for (double d : data)
		{
			if (std::abs(d) > val) val = std::abs(d);
		}
		return val;
	}

	/// Get a max value element.
	double Matrixd::getMaxElement() const
	{
		double val = -DBL_MAX;
		for (double d : data)
		{
			if (d > val) val = d;
		}
		return val;
	}

	/// Get a min absolute value element.
	double Matrixd::getMinabsElement() const
	{
		if (data.empty()) return 0.0;
		double val = std::abs(data[0]);
		for (double d : data)
		{
			if (std::abs(d) < val) val = std::abs(d);
		}
		return val;
	}

	/// Get a min value element.
	double Matrixd::getMinElement() const
	{
		double val = DBL_MAX;
		for (double d : data)
		{
			if (d < val) val = d;
		}
		return val;
	}

	/// Find 1d array index from row and col indices.
	unsigned int Matrixd::findIndex(const unsigned int rIdx, const unsigned int cIdx) const
	{
		// Removed exceptions for performance, but can be added back for debug
		return (rIdx * cols) + cIdx;
	}

	/// Make an identity matrix.
	void Matrixd::makeIdentityMat(const double& init_val)
	{
		if (this->getCols() != this->getRows())
			throw std::runtime_error("makeIdentityMat: Not a square matrix.");

		std::fill(data.begin(), data.end(), 0.0);
		for (unsigned int i = 0; i < this->getRows(); ++i)
		{
			this->operator()(i, i) = init_val;
		}
	}

	/// Make a transpose matrix.
	Matrixd Matrixd::transpose() const
	{
		Matrixd result(this->getCols(), this->getRows());
		// Optimized loop order for write locality?
		// Actually for transpose, one side will always be non-sequential.
		for (unsigned int i = 0; i < this->getRows(); ++i)
		{
			for (unsigned int j = 0; j < this->getCols(); ++j)
			{
				result(j, i) = this->operator()(i, j);
			}
		}
		return result;
	}

	/// Make an inverse matrix.
	Matrixd Matrixd::inverse() const
	{
		if (this->getCols() != this->getRows())
			throw std::runtime_error("inverse: Not a square matrix.");

		Matrixd copy(*this);

		if (this->getCols() == 1)
		{
			if (std::abs(copy[0]) < DBL_EPSILON)
				throw std::runtime_error("inverse: Divide by zero.");
			copy[0] = 1.0 / copy[0];
			return copy;
		}
		else
		{
			Matrixd LUD = copy.LUDcompose();
			return copy.LUInvert(LUD);
		}
	}

	// --- Operators ---

	Matrixd& Matrixd::operator=(const Matrixd& copy)
	{
		if (this != &copy) // Self-assignment check
		{
			MatDatad::operator=(copy);
		}
		return *this;
	}

	Matrixd& Matrixd::operator=(Matrixd&& other) noexcept
	{
		if (this != &other) // Self-assignment check
		{
			MatDatad::operator=(std::move(other));
		}
		return *this;
	}

	/// Operator +
	Matrixd Matrixd::operator+(const Matrixd& mat) const
	{
		if (rows != mat.getRows() || cols != mat.getCols())
			throw std::runtime_error("operator+: Size mismatch.");

		Matrixd result(rows, cols);
		for (size_t i = 0; i < data.size(); ++i)
			result.data[i] = this->data[i] + mat.data[i];

		return result; // RVO optimization
	}

	/// Operator +=
	void Matrixd::operator+=(const Matrixd& mat)
	{
		if (rows != mat.getRows() || cols != mat.getCols())
			throw std::runtime_error("operator+=: Size mismatch.");

		for (size_t i = 0; i < data.size(); ++i)
			this->data[i] += mat.data[i];
	}

	/// Operator + (scalar)
	Matrixd Matrixd::operator+(const double& val) const
	{
		Matrixd result(*this);
		for (double& d : result.data) d += val;
		return result;
	}

	/// Operator += (scalar)
	void Matrixd::operator+=(const double& val)
	{
		for (double& d : data) d += val;
	}

	/// Operator - (Unary)
	Matrixd Matrixd::operator-() const
	{
		Matrixd result(*this);
		for (double& d : result.data) d = -d;
		return result;
	}

	/// Operator -
	Matrixd Matrixd::operator-(const Matrixd& mat) const
	{
		if (rows != mat.getRows() || cols != mat.getCols())
			throw std::runtime_error("operator-: Size mismatch.");

		Matrixd result(rows, cols);
		for (size_t i = 0; i < data.size(); ++i)
			result.data[i] = this->data[i] - mat.data[i];
		return result;
	}

	/// Operator -=
	void Matrixd::operator-=(const Matrixd& mat)
	{
		if (rows != mat.getRows() || cols != mat.getCols())
			throw std::runtime_error("operator-=: Size mismatch.");

		for (size_t i = 0; i < data.size(); ++i)
			this->data[i] -= mat.data[i];
	}

	/// Operator - (scalar)
	Matrixd Matrixd::operator-(const double& val) const
	{
		Matrixd result(*this);
		for (double& d : result.data) d -= val;
		return result;
	}

	/// Operator -= (scalar)
	void Matrixd::operator-=(const double& val)
	{
		for (double& d : data) d -= val;
	}

	/// Operator * (Element-wise)
	Matrixd Matrixd::operator*(const Matrixd& mat) const
	{
		if (rows != mat.getRows() || cols != mat.getCols())
			throw std::runtime_error("operator*: Size mismatch.");

		Matrixd result(rows, cols);
		for (size_t i = 0; i < data.size(); ++i)
			result.data[i] = this->data[i] * mat.data[i];
		return result;
	}

	/// Operator *= (Element-wise)
	void Matrixd::operator*=(const Matrixd& mat)
	{
		if (rows != mat.getRows() || cols != mat.getCols())
			throw std::runtime_error("operator*=: Size mismatch.");

		for (size_t i = 0; i < data.size(); ++i)
			this->data[i] *= mat.data[i];
	}

	/// Operator * (scalar)
	Matrixd Matrixd::operator*(const double& val) const
	{
		Matrixd result(*this);
		for (double& d : result.data) d *= val;
		return result;
	}

	/// Operator *= (scalar)
	void Matrixd::operator*=(const double& val)
	{
		for (double& d : data) d *= val;
	}

	/// Operator / (Element-wise)
	Matrixd Matrixd::operator/(const Matrixd& mat) const
	{
		if (rows != mat.getRows() || cols != mat.getCols())
			throw std::runtime_error("operator/: Size mismatch.");

		Matrixd result(rows, cols);
		for (size_t i = 0; i < data.size(); ++i)
		{
			if (std::abs(mat.data[i]) < DBL_EPSILON) throw std::runtime_error("operator/: Divide by zero.");
			result.data[i] = this->data[i] / mat.data[i];
		}
		return result;
	}

	/// Operator /= (Element-wise)
	void Matrixd::operator/=(const Matrixd& mat)
	{
		if (rows != mat.getRows() || cols != mat.getCols())
			throw std::runtime_error("operator/=: Size mismatch.");

		for (size_t i = 0; i < data.size(); ++i)
		{
			if (std::abs(mat.data[i]) < DBL_EPSILON) throw std::runtime_error("operator/=: Divide by zero.");
			this->data[i] /= mat.data[i];
		}
	}

	/// Operator / (scalar)
	Matrixd Matrixd::operator/(const double& val) const
	{
		if (std::abs(val) < DBL_EPSILON)
			throw std::runtime_error("operator/(scalar): Divide by zero.");

		Matrixd result(*this);
		for (double& d : result.data) d /= val;
		return result;
	}

	/// Operator /= (scalar)
	void Matrixd::operator/=(const double& val)
	{
		if (std::abs(val) < DBL_EPSILON)
			throw std::runtime_error("operator/=(scalar): Divide by zero.");

		for (double& d : data) d /= val;
	}

	/// Matrix Multiplication (%)
	/// Optimization: Transpose the second matrix to improve cache locality.
	Matrixd Matrixd::operator%(const Matrixd& mat) const
	{
		if (this->getCols() != mat.getRows())
			throw std::runtime_error("operator%: Dimension mismatch.");

		Matrixd result(this->getRows(), mat.getCols(), 0.0);

		// Optimization: Transpose mat to allow sequential access
		Matrixd matT = mat.transpose();

		const unsigned int M = this->getRows();
		const unsigned int N = mat.getCols();
		const unsigned int K = this->getCols();

		// Use raw pointers for maximum speed in inner loops
		const double* A_ptr = this->data.data();
		const double* B_ptr = matT.data.data();
		double* R_ptr = result.getDataHandle();

		for (unsigned int i = 0; i < M; ++i)
		{
			for (unsigned int j = 0; j < N; ++j)
			{
				double sum = 0.0;
				for (unsigned int k = 0; k < K; ++k)
				{
					// Access A by row i, Access B_T by row j (which is col j of B)
					sum += A_ptr[i * K + k] * B_ptr[j * K + k];
				}
				R_ptr[i * N + j] = sum;
			}
		}
		return result;
	}

	/// Get sum of elements.
	double Matrixd::getSum() const
	{
		double sum = 0.0;
		for (double d : data) sum += d;
		return sum;
	}

	/// Get sum of squares.
	double Matrixd::getSumOfSquares() const
	{
		// Note: Original code did (*this) * (*this) which is element-wise mult, 
		// then getSum(). We can do it in one loop.
		double sum = 0.0;
		for (double d : data) sum += (d * d);
		return sum;
	}

	double& Matrixd::operator()(const unsigned int rIdx, const unsigned int cIdx)
	{
		return data[(rIdx * cols) + cIdx];
	}

	const double& Matrixd::operator()(const unsigned int rIdx, const unsigned int cIdx) const
	{
		return data[(rIdx * cols) + cIdx];
	}

	double& Matrixd::operator[](const unsigned int idx)
	{
		return data[idx];
	}

	const double& Matrixd::operator[](const unsigned int idx) const
	{
		return data[idx];
	}

	bool Matrixd::operator==(const Matrixd& copy) const
	{
		if (this->getRows() != copy.getRows() || this->getCols() != copy.getCols())
			return false;

		// Vector equality check is fast and optimized
		return this->data == copy.data;
		// Note: Exact float equality. If epsilon check is needed, keep loop.
	}

	bool Matrixd::operator!=(const Matrixd& copy) const
	{
		return !(*this == copy);
	}

	/// LU Decompose.
	Matrixd Matrixd::LUDcompose()
	{
		unsigned int i, j, k, k2, t;
		double p, temp;
		Matrixd perm(this->getRows(), 1);

		for (i = 0; i < this->getRows(); ++i)
			perm(i, 0) = static_cast<double>(i);

		for (k = 0; k < (this->getRows() - 1); ++k)
		{
			p = 0.0;
			k2 = k; // Initialize k2 to avoid warning

			for (i = k; i < this->getRows(); ++i)
			{
				temp = std::abs((*this)(i, k));
				if (temp > p)
				{
					p = temp;
					k2 = i;
				}
			}

			if (p == 0.0)
				throw std::runtime_error("LUDcompose: Singular Matrix.");

			// Exchange rows
			t = static_cast<unsigned int>(perm(k, 0));
			perm(k, 0) = perm(k2, 0);
			perm(k2, 0) = static_cast<double>(t);

			// Swap rows in this matrix
			for (i = 0; i < this->getCols(); ++i) // Changed from Rows to Cols for correctness, though square
			{
				std::swap((*this)(k, i), (*this)(k2, i));
			}

			for (i = k + 1; i < this->getRows(); ++i)
			{
				(*this)(i, k) /= (*this)(k, k);

				for (j = k + 1; j < this->getRows(); ++j)
					(*this)(i, j) -= (*this)(i, k) * (*this)(k, j);
			}
		}
		return perm;
	}

	/// LU Invert.
	Matrixd Matrixd::LUInvert(Matrixd& perm)
	{
		unsigned int i, j;
		Matrixd p(this->getRows(), 1, 0.0);
		Matrixd result(this->getRows(), this->getRows(), 0.0);

		for (j = 0; j < this->getRows(); ++j)
		{
			for (i = 0; i < this->getRows(); ++i) p(i, 0) = 0.0;
			p(j, 0) = 1.0;

			Matrixd colSol = LUSolve(perm, p);

			for (i = 0; i < this->getRows(); ++i)
				result(i, j) = colSol(i, 0);
		}
		return result;
	}

	/// LU Solve.
	Matrixd Matrixd::LUSolve(Matrixd& perm, Matrixd& b)
	{
		unsigned int i, j;
		double sum;
		Matrixd y(this->getRows(), 1, 0.0), x(this->getRows(), 1, 0.0);

		// Forward substitution
		for (i = 0; i < this->getRows(); ++i)
		{
			sum = 0.0;
			for (j = 0; j < i; ++j)
			{
				sum += (*this)(i, j) * y(j, 0);
			}
			y(i, 0) = b(static_cast<unsigned int>(perm(i, 0)), 0) - sum;
		}

		// Backward substitution
		for (int i_idx = this->getRows() - 1; i_idx >= 0; --i_idx)
		{
			i = static_cast<unsigned int>(i_idx);
			sum = 0.0;
			for (j = i + 1; j < this->getRows(); ++j)
				sum += (*this)(i, j) * x(j, 0);

			x(i, 0) = (y(i, 0) - sum) / (*this)(i, i);
		}

		return x;
	}

	/// Inverse partitioned matrix.
	Matrixd inversePartitionedMatrixd(const Matrixd& a, const Matrixd& b, const Matrixd& c, const Matrixd& d)
	{
		if (a.getCols() != c.getCols() || a.getRows() != b.getRows() ||
			c.getRows() != d.getRows() || b.getCols() != d.getCols())
		{
			throw std::runtime_error("inversePartitionedMatrixd: Size mismatch.");
		}

		// Schur complement: M/A
		auto aInv = a.inverse();
		auto ma = d - ((c % aInv) % b); // Using % for matrix mult
		auto result22 = ma.inverse();
		auto temp = ((aInv % b) % result22);
		auto result12 = -temp;
		auto result11 = aInv + ((temp % c) % aInv);
		auto result21 = ((-result22) % c) % aInv;

		Matrixd result(a.getRows() + c.getRows(), a.getCols() + b.getCols(), 0.0);
		result.insertPlus(0, 0, result11);
		result.insertPlus(0, result11.getCols(), result12);
		result.insertPlus(result11.getRows(), 0, result21);
		result.insertPlus(result11.getRows(), result11.getCols(), result22);

		return result;
	}

	std::string matrixout(const Matrixd& mat)
	{
		std::stringstream strStream;
		strStream.precision(9);

		for (unsigned int r = 0; r < mat.getRows(); ++r)
		{
			for (unsigned int c = 0; c < mat.getCols(); ++c)
			{
				strStream << std::to_string(mat(r, c)) << "\t";
			}
			strStream << std::endl;
		}
		return strStream.str();
	}
}