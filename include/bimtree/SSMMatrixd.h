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

#include <string>

#include "SSMMatDatad.h"

namespace math
{
	/**
	*@class Matrixd Class
	*@brief Mathematics class for manipulating a Matrixd
	*/
	class Matrixd : public MatDatad
	{
		template<size_t r, size_t c>
		friend class FixedSizeMatd;

		template<size_t d>
		friend class VectorMatd;

	public:

		/**constructor*/
		Matrixd();

		/**constructor*/
		Matrixd(const unsigned int r, const unsigned int c);

		/**constructor*/
		Matrixd(const unsigned int r, const unsigned int c, double val);

		/**copy constructor*/
		Matrixd(const Matrixd& copy);

		/**destructor*/
		virtual ~Matrixd();

		/**Add rows to existing matrix<br>
		*|A|addCols|B|=|A|
		*              |B|.
		*/
		void addRows(const Matrixd& copy);

		/**Add columns to an existing matrix<br>
		*|A|addCols|B|=|A||B|.
		*/
		void addCols(const Matrixd& copy);

		/**bind two matrices<br>
		*|A|add|B|=|A||0|<br>
		*          |0||B| = C.
		*/
		void addRowsCols(const Matrixd& copy);

		/**change specific part with given matrix<br>
		*if the size of an existing matrix is not available, resize the matrix.
		*/
		void insert(const unsigned int rIdx, const unsigned int cIdx, const Matrixd& copy);

		/**using the given matrix and position accumulate existing matrix<br>
		*if the existing matrix size is not proper, throw error.
		*/
		void insertPlus(const unsigned int rIdx, const unsigned int cIdx, const Matrixd& copy);

		/**using the given matrix and position accumulate existing matrix<br>
		*if the existing matrix size is not proper, resize the matrix.
		*/
		void insertPlusResize(const unsigned int rIdx, const unsigned int cIdx, const Matrixd& copy);

		/**get a subset from an existing matrix*/
		Matrixd getSubset(const unsigned int rIdx, const unsigned int cIdx, const unsigned int size_r, const unsigned int size_c) const;

		/**Get a abs-max value.*/
		double getMaxabsElement() const;

		/**Get a max value.*/
		double getMaxElement() const;

		/**Get a abs-min value.*/
		double getMinabsElement() const;

		/**Get a min value.*/
		double getMinElement() const;

		/**find 1d array index from row and col indices*/
		unsigned int findIndex(const unsigned int rIdx, const unsigned int cIdx) const;

		/**make an identity matrix*/
		void makeIdentityMat(const double& init_val = double(1));

		/**make a transpose matrix*/
		Matrixd transpose() const;

		/**make a inverse matrix*/
		Matrixd inverse() const;

	//operators
	public:
		/**= operator*/
		Matrixd operator = (const Matrixd& copy);

		/**+ operator*/
		Matrixd operator + (const Matrixd& mat) const;//Matrixd + Matrixd

		/**+= operator*/
		void operator += (const Matrixd& mat); //Matrixd += Matrixd

		/**+ operator*/
		Matrixd operator + (const double& val) const;//Matrixd + val

		/**+= operator*/
		void operator += (const double& val); //Matrixd += val

		/**- operator*/
		Matrixd operator - () const;//-Matrixd

		/**- operator*/
		Matrixd operator - (const Matrixd& mat) const;//Matrixd - Matrixd

		/**-= operator*/
		void operator -= (const Matrixd& mat); //Matrixd -= Matrixd

		/**- operator*/
		Matrixd operator - (const double& val) const;//Matrixd - val

		/**-= operator*/
		void operator -= (const double& val); //Matrixd -= val

		/** * operator*/
		Matrixd operator * (const Matrixd& mat) const;//Matrixd * Matrixd

		/** *= operator*/
		void operator *= (const Matrixd& mat); //Matrixd *= Matrixd

		/** * operator*/
		Matrixd operator * (const double& val) const;//Matrixd * val

		/** *= operator*/
		void operator *= (const double& val); //Matrixd *= val

		/** / operator*/
		Matrixd operator / (const Matrixd& mat) const;//Matrixd / Matrixd

		/** /= operator*/
		void operator /= (const Matrixd& mat); //Matrixd /= Matrixd

		/** / operator*/
		Matrixd operator / (const double& val) const;//Matrixd / val

		/**scalar /= operator*/
		void operator /= (const double& val); //Matrixd / val

		/**matrix multiplication*/
		Matrixd operator % (const Matrixd& mat) const;//Matrixd % Matrixd

		/**Get square sum*/
		double getSum() const;

		/**Get square sum*/
		double getSumOfSquares() const;

		/**() operator: access a element of matrix*/
		double& operator () (const unsigned int rIdx, const unsigned int cIdx = 0) const;

		/**[] operator: access a element of matrix*/
		double& operator [] (const unsigned int idx) const;

		bool operator == (const Matrixd& copy);

		bool operator != (const Matrixd& copy);

	private:
		/**LU Decompose*/
		Matrixd LUDcompose();

		/**LU Invert*/
		Matrixd LUInvert(Matrixd& perm);

		/**LU solve*/
		Matrixd LUSolve(Matrixd& perm, Matrixd& b);

	};

	Matrixd inversePartitionedMatrixd(const Matrixd& a, const Matrixd& b, const Matrixd& c, const Matrixd& d);

	std::string matrixout(const Matrixd& mat);
}