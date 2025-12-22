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

#include "SSMMatDatad.h"
#include <algorithm> // for std::fill

namespace math
{
	MatDatad::MatDatad() : rows(0), cols(0)
	{
	}

	MatDatad::MatDatad(const unsigned int r, const unsigned int c)
	{
		resize(r, c);
	}

	MatDatad::MatDatad(const unsigned int r, const unsigned int c, const double val)
	{
		resize(r, c, val);
	}

	MatDatad::MatDatad(const MatDatad& copy)
		: data(copy.data), rows(copy.rows), cols(copy.cols)
	{
	}

	MatDatad::MatDatad(MatDatad&& other) noexcept
		: data(std::move(other.data)), rows(other.rows), cols(other.cols)
	{
		other.rows = 0;
		other.cols = 0;
	}

	MatDatad::~MatDatad()
	{
		// No try-catch needed. std::vector handles destruction automatically.
	}

	MatDatad& MatDatad::operator=(const MatDatad& copy)
	{
		// Self-assignment check
		if (this == &copy)
			return *this;

		this->data = copy.data; // std::vector deep copy
		this->rows = copy.rows;
		this->cols = copy.cols;

		return *this;
	}

	MatDatad& MatDatad::operator=(MatDatad&& other) noexcept
	{
		// Self-assignment check
		if (this == &other)
			return *this;

		this->data = std::move(other.data); // Move semantics
		this->rows = other.rows;
		this->cols = other.cols;

		other.rows = 0;
		other.cols = 0;

		return *this;
	}

	void MatDatad::del()
	{
		data.clear();
		data.shrink_to_fit();
		rows = 0;
		cols = 0;
	}

	unsigned int MatDatad::getRows() const { return rows; }

	unsigned int MatDatad::getCols() const { return cols; }

	unsigned int MatDatad::getSize() const { return static_cast<unsigned int>(data.size()); }

	double* MatDatad::getDataHandle() { return data.data(); }

	const double* MatDatad::getAccessData() const { return data.data(); }

	void MatDatad::resize(const unsigned int r, const unsigned int c, double val)
	{
		allocateMem(r, c);
		initialize(val);
	}

	void MatDatad::allocateMem(const unsigned int r, const unsigned int c)
	{
		rows = r;
		cols = c;
		// Resize vector; new elements are default-initialized
		data.resize(rows * cols);
	}

	void MatDatad::initialize(double val)
	{
		std::fill(data.begin(), data.end(), val);
	}
}