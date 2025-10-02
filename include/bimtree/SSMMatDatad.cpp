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

#include "SSMMatDatad.h"

#include <iostream>

namespace math
{
	MatDatad::MatDatad()
	{
		this->size = 0;
		resize(1, 1);
	}

	MatDatad::MatDatad(const unsigned int r, const unsigned int c)
	{
		this->size = 0;
		resize(r, c);
	}

	MatDatad::MatDatad(const unsigned int r, const unsigned int c, const double val)
	{
		this->size = 0;
		allocateMem(r, c);
		initialize(val);
	}

	MatDatad::MatDatad(const MatDatad& copy)
	{
		this->size = 0;
		allocateMem(copy.getRows(), copy.getCols());

		for (unsigned int i = 0; i < size; i++)
		{
			data[i] = copy.data[i];
		}
	}

	MatDatad::~MatDatad() { del(); }

	MatDatad MatDatad::operator = (const MatDatad& copy)
	{
		del();

		allocateMem(copy.getRows(), copy.getCols());

		for (unsigned int i = 0; i < size; i++)
		{
			data[i] = copy.getAccessData()[i];
		}

		return *this;
	}

	void MatDatad::del()
	{
		try
		{
			if (size >= 1 && data != nullptr)
			{
				delete[] data;
			}
		}
		catch (...)
		{
			throw std::runtime_error("Error in deleting memory in MatDatad");
		}

		rows = 0;
		cols = 0;
		size = 0;
		data = nullptr;
	}

	/**get the number of rows*/
	unsigned int MatDatad::getRows() const { return rows; }

	/**get the number of cols*/
	unsigned int MatDatad::getCols() const { return cols; }

	/**get the size of data*/
	unsigned int MatDatad::getSize() const { return size; }

	/**get the pointer of data*/
	double* MatDatad::getDataHandle() const { return data; }

	/**get the const pointer of data*/
	const double* MatDatad::getAccessData() const { return data; }

	/**resize a matrix with a default value*/
	void MatDatad::resize(const unsigned int r, const unsigned int c, double val)
	{
		del();
		allocateMem(r, c);
		initialize(val);
	}

	void MatDatad::allocateMem(const unsigned int r, const unsigned int c)
	{
		rows = r;
		cols = c;
		size = rows * cols;

		data = new double[size];
	}

	void MatDatad::initialize(double val)
	{
		for (unsigned int i = 0; i < size; i++)
		{
			data[i] = val;
		}
	}
}