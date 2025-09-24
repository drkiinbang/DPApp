/*
* Copyright(c) 2001-2019 KI IN Bang
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
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#pragma once

#include "SSMMatrixd.h"

#include <iostream>
#include <sstream>
#include <stdexcept>

namespace math
{
	
	/**constructor*/
	Matrixd::Matrixd() : MatDatad() {}

	/**constructor*/
	Matrixd::Matrixd(const unsigned int r, const unsigned int c) : MatDatad(r, c) {}

	/**constructor*/
	Matrixd::Matrixd(const unsigned int r, const unsigned int c, double val) : MatDatad(r, c, val) {}

	/**copy constructor*/
	Matrixd::Matrixd(const Matrixd& copy)
		: MatDatad(copy.getRows(), copy.getCols())
	{
		for (unsigned int i = 0; i < this->getSize(); ++i)
		{
			this->getDataHandle()[i] = copy.getAccessData()[i];
		}
	}

	/**destructor*/
	Matrixd::~Matrixd() {}

	/**Add rows to existing matrix<br>
	*|A|addCols|B|=|A|
	*              |B|.
	*/
	void Matrixd::addRows(const Matrixd& copy)
	{
		if (this->getCols() != copy.getCols())
			throw std::runtime_error("addRows produces an error since the numbers of cols are not same.");

		Matrixd newMat(this->getRows() + copy.getRows(), this->getCols(), double(0));

		unsigned int i, j;

		for (i = 0; i < this->getRows(); ++i)
		{
			for (j = 0; j < this->getCols(); ++j)
			{
				newMat(i, j) = this->operator()(i, j);
			}
		}

		unsigned int size_r = copy.getRows();
		unsigned int size_c = copy.getCols();

		for (i = 0; i < size_r; ++i)
		{
			for (j = 0; j < size_c; ++j)
			{
				newMat(this->getRows() + i, j) = copy(i, j);
			}
		}

		*this = newMat;
	}

	/**Add columns to an existing matrix<br>
	*|A|addCols|B|=|A||B|.
	*/
	void Matrixd::addCols(const Matrixd& copy)
	{
		if (this->getRows() != copy.getRows())
			throw std::runtime_error("addCols produces an error since the numbers of rows are not same.");

		Matrixd newMat(this->getRows(), this->getCols() + copy.getCols(), double(0));

		unsigned int i, j;

		for (i = 0; i < this->getRows(); ++i)
		{
			for (j = 0; j < this->getCols(); ++j)
			{
				newMat(i, j) = this->operator()(i, j);
			}
		}

		unsigned int size_r = copy.getRows();
		unsigned int size_c = copy.getCols();

		for (i = 0; i < size_r; ++i)
		{
			for (j = 0; j < size_c; ++j)
			{
				newMat(i, this->getCols() + j) = copy(i, j);
			}
		}

		*this = newMat;
	}

	/**bind two matrices<br>
	*|A|add|B|=|A||0|<br>
	*          |0||B| = C.
	*/
	void Matrixd::addRowsCols(const Matrixd& copy)
	{
		Matrixd newMat(this->getRows() + copy.getRows(), this->getCols() + copy.getCols(), double(0));

		unsigned int i, j;

		for (i = 0; i < this->getRows(); ++i)
		{
			for (j = 0; j < this->getCols(); ++j)
			{
				newMat(i, j) = this->operator()(i, j);
			}
		}

		unsigned int size_r = copy.getRows();
		unsigned int size_c = copy.getCols();

		for (i = 0; i < size_r; ++i)
		{
			for (j = 0; j < size_c; ++j)
			{
				newMat(this->getRows() + i, this->getCols() + j) = copy(i, j);
			}
		}

		*this = newMat;
	}

	/**change specific part with given matrix<br>
	*if the size of an existing matrix is not available, resize the matrix.
	*/
	void Matrixd::insert(const unsigned int rIdx, const unsigned int cIdx, const Matrixd& copy)
	{
		if ((this->getRows() < (rIdx + copy.getRows())) || (this->getCols() < (cIdx + copy.getCols())))
		{
			unsigned int newrows = this->getRows(), newcols = this->getCols();
			if (this->getRows() < (rIdx + copy.getRows())) newrows = rIdx + copy.getRows();
			if (this->getCols() < (cIdx + copy.getCols())) newcols = cIdx + copy.getCols();

			Matrixd backup = *this;

			this->resize(newrows, newcols);

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

	/**using the given matrix and position accumulate existing matrix<br>
	*if the existing matrix size is not proper, throw error.
	*/
	void Matrixd::insertPlus(const unsigned int rIdx, const unsigned int cIdx, const Matrixd& copy)
	{
		if ((this->getRows() < (rIdx + copy.getRows())) || (this->getCols() < (cIdx + copy.getCols())))
		{
			throw std::runtime_error("The inserted matrix is too large.");
		}
		else
		{
			for (unsigned int i = 0; i < copy.getRows(); ++i)
			{
				for (unsigned int j = 0; j < copy.getCols(); ++j)
				{
					operator()(rIdx + i, cIdx + j) += copy(i, j);
				}
			}
		}
	}

	/**using the given matrix and position accumulate existing matrix<br>
	*if the existing matrix size is not proper, resize the matrix.
	*/
	void Matrixd::insertPlusResize(const unsigned int rIdx, const unsigned int cIdx, const Matrixd& copy)
	{
		if ((this->getRows() < (rIdx + copy.getRows())) || (this->getCols() < (cIdx + copy.getCols())))
		{
			unsigned int newrows = this->getRows(), newcols = this->getCols();
			if (this->getRows() < (rIdx + copy.getRows())) newrows = rIdx + copy.getRows();
			if (this->getCols() < (cIdx + copy.getCols())) newcols = cIdx + copy.getCols();

			Matrixd backup = *this;

			this->resize(newrows, newcols);

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
				this->operator()(rIdx + i, cIdx + j) += copy(i, j);
			}
		}
	}

	/**get a subset from an existing matrix*/
	Matrixd Matrixd::getSubset(const unsigned int rIdx, const unsigned int cIdx, const unsigned int size_r, const unsigned int size_c) const
	{
		if ((this->getRows() < (rIdx + size_r)) || (this->getCols() < (cIdx + size_c)))
		{
			throw std::runtime_error("Out of range error from getSubset");
		}
		else
		{
			Matrixd ret(size_r, size_c, double(0));

			for (unsigned int i = 0; i < size_r; ++i)
			{
				for (unsigned int j = 0; j < size_c; ++j)
				{
					ret(i, j) = operator()(rIdx + i, cIdx + j);
				}
			}

			return ret;
		}
	}

	/**Get a abs-max value.*/
	double Matrixd::getMaxabsElement() const
	{
		double val = fabs(static_cast<double>(this->getAccessData()[0]));

		for (unsigned int i = 0; i < this->getSize(); ++i)
		{
			if (val < (fabs(static_cast<double>(this->getAccessData()[i]))))
				val = fabs(static_cast<double>(this->getAccessData()[i]));
		}

		return val;
	}

	/**Get a max value.*/
	double Matrixd::getMaxElement() const
	{
		double val = this->getAccessData()[0];

		for (unsigned int i = 0; i < this->getSize(); ++i)
		{
			if (val < this->getAccessData()[i])
				val = this->getAccessData()[i];
		}

		return val;
	}

	/**Get a abs-min value.*/
	double Matrixd::getMinabsElement() const
	{
		double val = fabs(this->getAccessData()[0]);

		for (unsigned int i = 0; i < this->getSize(); ++i)
		{
			if (val > (fabs(static_cast<double>(this->getAccessData()[i]))))
				val = fabs(static_cast<double>(this->getAccessData()[i]));
		}

		return val;
	}

	/**Get a min value.*/
	double Matrixd::getMinElement() const
	{
		double val = this->getAccessData()[0];

		for (unsigned int i = 0; i < this->getSize(); ++i)
		{
			if (val > this->getAccessData()[i])
				val = this->getAccessData()[i];
		}

		return val;
	}

	/**find 1d array index from row and col indices*/
	unsigned int Matrixd::findIndex(const unsigned int rIdx, const unsigned int cIdx) const
	{
		if ((this->getRows() <= rIdx) || (this->getCols() <= cIdx))
			throw std::runtime_error("OUt of range error from findIndex");

		unsigned int index = (rIdx * (this->getCols())) + cIdx;
		return index;
	}

	/**make an identity matrix*/
	void Matrixd::makeIdentityMat(const double& init_val)
	{
		if (this->getCols() != this->getRows())
			throw std::runtime_error("It is not a square matrix.");

		for (unsigned int i = 0; i < this->getRows(); ++i)
		{
			for (unsigned int j = 0; j < this->getCols(); ++j)
			{
				if (i == j) this->operator()(i, j) = init_val;
				else this->operator()(i, j) = double(0);
			}
		}
	};

	/**make a transpose matrix*/
	Matrixd Matrixd::transpose() const
	{
		Matrixd result(this->getCols(), this->getRows());

		for (unsigned int i = 0; i < this->getRows(); ++i)
		{
			for (unsigned int j = 0; j < this->getCols(); ++j)
			{
				result(j, i) = this->operator()(i, j);
			}
		}

		return result;
	};

	/**make a inverse matrix*/
	Matrixd Matrixd::inverse() const
	{
		if (this->getCols() != this->getRows())
			throw std::runtime_error("It is not a square matrix.");

		Matrixd copy(*this);

		if (this->getCols() == 1)
		{
			try
			{
				copy.getDataHandle()[0] = 1.0 / this->getAccessData()[0];
			}
			catch (...)
			{
				throw std::runtime_error("Divided by near zero in inverse function");
			}

			return copy;
		}
		else
		{
			Matrixd LUD;
			Matrixd mat_inv;

			try
			{
				LUD = copy.LUDcompose();
			}
			catch (...)
			{
				throw std::runtime_error("Error from Matrixd::LUDcompose()");
			}

			mat_inv = copy.LUInvert(LUD);

			return mat_inv;
		}

	}

	//operators

	Matrixd Matrixd::operator = (const Matrixd& copy)
	{
		this->resize(copy.getRows(), copy.getCols());

		for (unsigned int i = 0; i < this->getSize(); ++i)
		{
			this->getDataHandle()[i] = copy.getAccessData()[i];
		}

		return *this;
	}

	/**+ operator*/
	Matrixd Matrixd::operator + (const Matrixd& mat) const//Matrixd + Matrixd
	{
		if ((this->getRows() != mat.getRows()) || (this->getCols() != mat.getCols()))
			throw std::runtime_error("Different size error from operator +");

		Matrixd result(this->getRows(), this->getCols());

		for (unsigned int i = 0; i < this->getSize(); ++i)
			result.getDataHandle()[i] = this->getAccessData()[i] + mat.getAccessData()[i];

		return result;
	}

	/**+= operator*/
	void Matrixd::operator += (const Matrixd& mat) //Matrixd += Matrixd
	{
		if ((this->getRows() != mat.getRows()) || (this->getCols() != mat.getCols()))
			throw std::runtime_error("Different size error from operator +=");

		for (unsigned int i = 0; i < this->getSize(); ++i)
			this->getDataHandle()[i] += mat.getAccessData()[i];
	}

	/**+ operator*/
	Matrixd Matrixd::operator + (const double& val) const//Matrixd + val
	{
		Matrixd result(*this);

		for (unsigned int i = 0; i < this->getSize(); ++i)
			result.getDataHandle()[i] += val;

		return result;
	}

	/**+= operator*/
	void Matrixd::operator += (const double& val) //Matrixd += val
	{
		for (unsigned int i = 0; i < this->getSize(); ++i)
			this->getDataHandle()[i] += val;
	}

	/**- operator*/
	Matrixd Matrixd::operator - () const//-Matrixd
	{
		Matrixd result(*this);

		for (unsigned int i = 0; i < this->getSize(); ++i)
			result.getDataHandle()[i] *= -1.0;

		return result;
	}

	/**- operator*/
	Matrixd Matrixd::operator - (const Matrixd& mat) const//Matrixd - Matrixd
	{
		if ((this->getRows() != mat.getRows()) || (this->getCols() != mat.getCols()))
			throw std::runtime_error("Different size error from operator -");

		Matrixd result(*this);

		for (unsigned int i = 0; i < this->getSize(); ++i)
			result.getDataHandle()[i] -= mat.getAccessData()[i];

		return result;
	}

	/**-= operator*/
	void Matrixd::operator -= (const Matrixd& mat) //Matrixd -= Matrixd
	{
		if ((this->getRows() != mat.getRows()) || (this->getCols() != mat.getCols()))
			throw std::runtime_error("Different size error from operator -=");

		for (unsigned int i = 0; i < this->getSize(); ++i)
			this->getDataHandle()[i] -= mat.getAccessData()[i];
	}

	/**- operator*/
	Matrixd Matrixd::operator - (const double& val) const//Matrixd - val
	{
		Matrixd result(*this);

		for (unsigned int i = 0; i < this->getSize(); ++i)
			result.getDataHandle()[i] -= val;

		return result;
	}

	/**-= operator*/
	void Matrixd::operator -= (const double& val) //Matrixd -= val
	{
		for (unsigned int i = 0; i < this->getSize(); ++i)
			this->getDataHandle()[i] -= val;
	}

	/** * operator*/
	Matrixd Matrixd::operator * (const Matrixd& mat) const//Matrixd * Matrixd
	{
		if ((this->getRows() != mat.getRows()) || (this->getCols() != mat.getCols()))
			throw std::runtime_error("Different size error from operator *");

		Matrixd result(*this);

		for (unsigned int i = 0; i < this->getSize(); ++i)
			result.getDataHandle()[i] *= mat.getAccessData()[i];

		return result;
	}

	/** *= operator*/
	void Matrixd::operator *= (const Matrixd& mat) //Matrixd *= Matrixd
	{
		if ((this->getRows() != mat.getRows()) || (this->getCols() != mat.getCols()))
			throw std::runtime_error("Different size error from operator *=");

		for (unsigned int i = 0; i < this->getSize(); ++i)
			this->getDataHandle()[i] *= mat.getAccessData()[i];
	}

	/** * operator*/
	Matrixd Matrixd::operator * (const double& val) const//Matrixd * val
	{
		Matrixd result(*this);

		for (unsigned int i = 0; i < this->getSize(); ++i)
			result.getDataHandle()[i] *= val;

		return result;
	}

	/** *= operator*/
	void Matrixd::operator *= (const double& val) //Matrixd *= val
	{
		for (unsigned int i = 0; i < this->getSize(); ++i)
			this->getDataHandle()[i] *= val;
	}

	/** / operator*/
	Matrixd Matrixd::operator / (const Matrixd& mat) const//Matrixd / Matrixd
	{
		if ((this->getRows() != mat.getRows()) || (this->getCols() != mat.getCols()))
			throw std::runtime_error("Different size error from operator /");

		Matrixd result(*this);
		try
		{
			for (unsigned int i = 0; i < this->getSize(); ++i)
				result.getDataHandle()[i] /= mat.getAccessData()[i];
		}
		catch (...)
		{
			throw std::runtime_error("Divided by near zero in operator /");
		}

		return result;
	}

	/** /= operator*/
	void Matrixd::operator /= (const Matrixd& mat) //Matrixd /= Matrixd
	{
		if ((this->getRows() != mat.getRows()) || (this->getCols() != mat.getCols()))
			throw std::runtime_error("Different size error from operator /=");

		try
		{
			for (unsigned int i = 0; i < this->getSize(); ++i)
				this->getDataHandle()[i] /= mat.getAccessData()[i];
		}
		catch (...)
		{
			throw std::runtime_error("Divided by near zero in operator /=");
		}
	}

	/** / operator*/
	Matrixd Matrixd::operator / (const double& val) const//Matrixd / val
	{
		if (static_cast<double>(val) < DBL_EPSILON)
			throw std::runtime_error("Error from operator /(double val): Devided by near zero");

		Matrixd result(*this);

		try
		{
			for (unsigned int i = 0; i < this->getSize(); ++i)
				result.getDataHandle()[i] /= val;
		}
		catch (...)
		{
			throw std::runtime_error("Divided by near zero in operator /");
		}

		return result;
	}

	/**scalar /= operator*/
	void Matrixd::operator /= (const double& val) //Matrixd / val
	{
		if (static_cast<double>(val) < DBL_EPSILON)
			throw std::runtime_error("Error from operator /=(double val): Devided by near zero");

		try
		{
			for (unsigned int i = 0; i < this->getSize(); ++i)
				this->getDataHandle()[i] /= val;
		}
		catch (...)
		{
			throw std::runtime_error("Divided by near zero in operator /=");
		}
	}

	/**matrix multiplication*/
	Matrixd Matrixd::operator % (const Matrixd& mat) const//Matrixd % Matrixd
	{
		if (this->getCols() != mat.getRows())
			throw std::runtime_error("The numbers of cols and rows are not correct for % operation.");

		double sum;
		Matrixd result(this->getRows(), mat.getCols());

		for (unsigned int i = 0; i < this->getRows(); ++i)
		{
			for (unsigned int j = 0; j < mat.getCols(); ++j)
			{
				sum = double(0);
				for (unsigned int k = 0; k < this->getCols(); ++k)
				{
					sum += operator()(i, k) * mat(k, j);
				}
				result(i, j) = sum;
			}
		}
		return result;
	}

	/**Get square sum*/
	double Matrixd::getSum() const
	{
		double sum = 0.0;
		for (unsigned int i = 0; i < this->getSize(); ++i)
			sum += this->getAccessData()[i];

		return sum;
	}

	/**Get square sum*/
	double Matrixd::getSumOfSquares() const
	{
		Matrixd temp = (*this) * (*this);
		return temp.getSum();
	}

	/**() operator: access a element of matrix*/
	double& Matrixd::operator () (const unsigned int rIdx, const unsigned int cIdx) const
	{
		unsigned int i;
		i = findIndex(rIdx, cIdx);
		return this->getDataHandle()[i];
	}

	/**[] operator: access a element of matrix*/
	double& Matrixd::operator [] (const unsigned int idx) const
	{
		return this->getDataHandle()[idx];
	}

	bool Matrixd::operator == (const Matrixd& copy)
	{
		if (this->getRows() != copy.getRows()) return false;
		if (this->getCols() != copy.getCols()) return false;

		for (unsigned int i = 0; i < this->getSize(); ++i)
		{
			if (fabs(this->getAccessData()[i] - copy.getAccessData()[i]) > std::numeric_limits<double>::epsilon())
				return false;
		}

		return true;
	}

	bool Matrixd::operator != (const Matrixd& copy)
	{
		if (this->operator == (copy))
			return false;
		else
			return true;
	}

	/**LU Decompose*/
	Matrixd Matrixd::LUDcompose()
	{
		// LU decomposition
		unsigned int i, j, k, k2, t;
		double p, temp;
		// permutation matrix
		Matrixd perm(this->getRows(), 1);

		// initialize permutation
		for (i = 0; i < this->getRows(); ++i)
			perm(i, 0) = static_cast<double>(i);

		for (k = 0; k < (this->getRows() - 1); ++k)
		{
			p = double(0);

			for (i = k; i < this->getRows(); ++i)
			{
				temp = (this->getAccessData()[findIndex(i, k)] < 0 ? (-this->getAccessData()[findIndex(i, k)]) : this->getAccessData()[findIndex(i, k)]);

				if (temp > p)
				{
					p = temp;
					k2 = i;
				}
			}

			if (p == double(0))
				throw std::runtime_error("Inverse failed: SINGULAR_MATRIX");

			// exchange rows
			t = static_cast<unsigned int>(perm(k, 0));
			perm(k, 0) = perm(k2, 0);
			perm(k2, 0) = static_cast<double>(t);

			for (i = 0; i < this->getRows(); ++i)
			{
				temp = this->getAccessData()[findIndex(k, i)];
				this->getDataHandle()[findIndex(k, i)] = this->getAccessData()[findIndex(k2, i)];
				this->getDataHandle()[findIndex(k2, i)] = temp;
			}

			for (i = k + 1; i < this->getRows(); ++i)
			{
				this->getDataHandle()[findIndex(i, k)] /= this->getAccessData()[findIndex(k, k)];

				for (j = k + 1; j < this->getRows(); ++j)
					this->getDataHandle()[findIndex(i, j)] -= this->getAccessData()[findIndex(i, k)] * this->getAccessData()[findIndex(k, j)];
			}
		}
		return perm;
	}

	/**LU Invert*/
	Matrixd Matrixd::LUInvert(Matrixd& perm)
	{
		unsigned int i, j;
		Matrixd p(this->getRows(), 1, double(0));

		Matrixd result(this->getRows(), this->getRows(), double(0));

		for (j = 0; j < this->getRows(); ++j)
		{
			for (i = 0; i < this->getRows(); ++i)
				p(i, 0) = double(0);

			p(j, 0) = double(1);

			p = LUSolve(perm, p);

			for (i = 0; i < this->getRows(); ++i)
				result(i, j) = p(i, 0);
		}

		return result;
	}

	/**LU solve*/
	Matrixd Matrixd::LUSolve(Matrixd& perm, Matrixd& b)
	{
		unsigned int i, j, j2;
		double sum, u;
		Matrixd y(this->getRows(), 1, double(0)), x(this->getRows(), 1, double(0));

		for (i = 0; i < this->getRows(); ++i)
		{
			sum = double(0);
			j2 = 0;

			for (j = 1; j <= i; ++j)
			{
				sum += this->getAccessData()[findIndex(i, j2)] * y(j2, 0);
				++j2;
			}
			y(i, 0) = b(static_cast<unsigned int>(perm(i, 0)), 0) - sum;
		}

		i = this->getRows() - 1;

		while (1)
		{
			sum = double(0);
			u = this->getAccessData()[findIndex(i, i)];

			for (j = i + 1; j < this->getRows(); ++j)
				sum += this->getAccessData()[findIndex(i, j)] * x(j, 0);

			x(i, 0) = (y(i, 0) - sum) / u;

			if (i == 0)
				break;

			--i;
		}

		return x;
	}

	Matrixd inversePartitionedMatrixd(const Matrixd& a, const Matrixd& b, const Matrixd& c, const Matrixd& d)
	{
		if (a.getCols() != c.getCols()
			|| a.getRows() != b.getRows()
			|| c.getRows() != d.getRows()
			|| b.getCols() != d.getCols())
		{
			std::cout << "Error caused by improper matrix sizes in inversePartitionedMatrixd" << std::endl;
			throw std::runtime_error("Error caused by improper matrix sizes in inversePartitionedMatrixd");
		}
		/**
		* http://www.cfm.brown.edu/people/dobrush/cs52/python/Part2/partition.html
		*/

		// Schur complement: M/A
		auto aInv = a.inverse();
		auto ma = d - c % aInv % b;
		auto result22 = ma.inverse();//(M/A).inverse()
		auto temp = (aInv % b % result22);
		auto result12 = -temp;
		auto result11 = aInv + (temp % c % aInv);
		auto result21 = -result22 % c % aInv;

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
				strStream << std::to_string(mat(r, c)) << std::string("\t");
			}

			strStream << std::endl;
		}
		return strStream.str();
	}
}