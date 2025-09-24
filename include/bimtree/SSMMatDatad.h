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

namespace math
{
	/**
	*@class Matrix Data Class with double type data
	*@brief data for Matrix
	*/
	class MatDatad
	{
	public:
		MatDatad();

		MatDatad(const unsigned int r, const unsigned int c);

		MatDatad(const unsigned int r, const unsigned int c, const double val);

		MatDatad(const MatDatad& copy);

		virtual ~MatDatad();

		MatDatad operator = (const MatDatad& copy);

		void del();

		/**get the number of rows*/
		unsigned int getRows() const;

		/**get the number of cols*/
		unsigned int getCols() const;

		/**get the size of data*/
		unsigned int getSize() const;

		/**get the pointer of data*/
		double* getDataHandle() const;

		/**get the const pointer of data*/
		const double* getAccessData() const;

		/**resize a matrix with a default value*/
		void resize(const unsigned int r, const unsigned int c, double val = 0.0);

	private:
		void allocateMem(const unsigned int r, const unsigned int c);

		void initialize(double val);

	private:
		double* data;
		unsigned int rows;
		unsigned int cols;
		unsigned int size;
	};

	class Arrayd : public MatDatad
	{
	public:
		Arrayd() : MatDatad() {}

		Arrayd(const unsigned int r) : MatDatad(r, 1) {}

		Arrayd(const unsigned int r, const double val) : MatDatad(r, 1, val) {}

		Arrayd(const Arrayd& copy) : MatDatad(copy) {}

		virtual ~Arrayd() {}

		Arrayd operator = (const Arrayd& copy)
		{
			MatDatad::operator=(copy);
			return *this;
		}

		double& operator [] (unsigned int idx) const
		{
			return this->getDataHandle()[idx];
		}

		/**resize a matrix with a default value*/
		void resize(const unsigned int r, double val = 0.0)
		{
			MatDatad::resize(r, 1, val);
		}
	};
}