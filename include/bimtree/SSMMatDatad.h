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
	/// @brief Data class for Matrix operations using double type with std::vector management.
	class MatDatad
	{
	public:
		/// Default constructor.
		MatDatad();

		/// Constructor with size.
		MatDatad(const unsigned int r, const unsigned int c);

		/// Constructor with size and initial value.
		MatDatad(const unsigned int r, const unsigned int c, const double val);

		/// Copy constructor.
		MatDatad(const MatDatad& copy);

		/// Move constructor.
		MatDatad(MatDatad&& other) noexcept;

		/// Destructor.
		virtual ~MatDatad();

		/// Copy assignment operator.
		MatDatad& operator=(const MatDatad& copy);

		/// Move assignment operator.
		MatDatad& operator=(MatDatad&& other) noexcept;

		/// Clear data and reset size.
		void del();

		/// Get the number of rows.
		unsigned int getRows() const;

		/// Get the number of columns.
		unsigned int getCols() const;

		/// Get the total size of data (rows * cols).
		unsigned int getSize() const;

		/// Get the pointer to the data (modifiable).
		double* getDataHandle();

		/// Get the const pointer to the data (read-only).
		const double* getAccessData() const;

		/// Resize the matrix.
		void resize(const unsigned int r, const unsigned int c, double val = 0.0);

	protected:
		/// Allocate memory (internal use, wraps vector resize).
		void allocateMem(const unsigned int r, const unsigned int c);

		/// Initialize data with a value.
		void initialize(double val);

	protected:
		std::vector<double> data; /// Container for matrix elements.
		unsigned int rows;        /// Number of rows.
		unsigned int cols;        /// Number of columns.
	};

	/// @class Arrayd
	/// @brief Helper class for 1D array (column vector).
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

		/// Resize the array with a default value.
		void resize(const unsigned int r, double val = 0.0)
		{
			MatDatad::resize(r, 1, val);
		}
	};
}