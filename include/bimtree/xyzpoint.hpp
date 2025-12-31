#pragma once

#include <vector>

#include "SSMMatrixd.h"

namespace pctree
{
	struct XYZPoint
	{
		double xyz[3];

		inline double x() const { return xyz[0]; }
		inline double y() const { return xyz[1]; }
		inline double z() const { return xyz[2]; }

		/// Ids of corresponding faces
		std::vector<size_t> fIds;

		XYZPoint(const XYZPoint& val) { copy(val); }
		XYZPoint() { xyz[0] = xyz[1] = xyz[2] = 0.0; };
		XYZPoint(const double x, const double y, const double z) { xyz[0] = x; xyz[1] = y; xyz[2] = z; };
		XYZPoint(const double x, const double y, const double z, const size_t fId) { xyz[0] = x; xyz[1] = y; xyz[2] = z; fIds.push_back(fId); };

		inline XYZPoint& operator=(const XYZPoint& val) { copy(val); return *this; }
		inline double& operator[](const int idx) { return xyz[idx]; }
		inline const double& operator[](const int idx) const { return xyz[idx]; }
		inline bool operator==(const XYZPoint& pt) const { return pt.xyz[0] == xyz[0] && pt.xyz[1] == xyz[1] && pt.xyz[2] == xyz[2]; }
		inline bool operator!=(const XYZPoint& pt) const { return pt.xyz[0] != xyz[0] || pt.xyz[1] != xyz[1] || pt.xyz[2] != xyz[2]; }
		inline XYZPoint operator+(const XYZPoint& pt) const { return XYZPoint(xyz[0] + pt.xyz[0], xyz[1] + pt.xyz[1], xyz[2] + pt.xyz[2]); }
		inline XYZPoint& operator+=(const XYZPoint& pt) { for (int i = 0; i < 3; i++) xyz[i] += pt.xyz[i]; return *this; }
		inline XYZPoint operator-(const XYZPoint& pt) const { XYZPoint temp(xyz[0] - pt.xyz[0], xyz[1] - pt.xyz[1], xyz[2] - pt.xyz[2]); return temp; }
		inline XYZPoint& operator-=(const XYZPoint& pt) { for (int i = 0; i < 3; i++) xyz[i] -= pt.xyz[i]; return *this; }
		inline XYZPoint operator*(const double s) const { return XYZPoint(xyz[0] * s, xyz[1] * s, xyz[2] * s); }
		inline XYZPoint& operator*=(const double s) { for (int i = 0; i < 3; i++) xyz[i] *= s; return *this; }
		/// inner product
		inline double operator*(const XYZPoint& pt) const { return (this->xyz[0] * pt.xyz[0] + this->xyz[1] * pt.xyz[1] + this->xyz[2] * pt.xyz[2]); }
		/// cross product
		inline XYZPoint operator%(const XYZPoint& pt) const {
			XYZPoint ret;
			ret[0] = this->xyz[1] * pt.xyz[2] - this->xyz[2] * pt.xyz[1];
			ret[1] = this->xyz[2] * pt.xyz[0] - this->xyz[0] * pt.xyz[2];
			ret[2] = this->xyz[0] * pt.xyz[1] - this->xyz[1] * pt.xyz[0];
			return ret;
		}
		inline XYZPoint operator/(const double s) const { return XYZPoint(xyz[0] / s, xyz[1] / s, xyz[2] / s); }
		inline XYZPoint& operator/=(const double s) { for (int i = 0; i < 3; i++) xyz[i] /= s; return *this; }
		inline XYZPoint normalize() const
		{
			double sum2 = xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2];
			if (sum2 > std::numeric_limits<double>::epsilon()) {
				double length = sqrt(sum2);
				return XYZPoint(xyz[0] / length, xyz[1] / length, xyz[2] / length);
			}
			else
				return XYZPoint(*this);
		}
		inline double getSquareSum() const { return (this->xyz[0] * this->xyz[0] + this->xyz[1] * this->xyz[1] + this->xyz[2] * this->xyz[2]); }

	private:
		inline void copy(const XYZPoint& val) {
			this->xyz[0] = val.xyz[0];
			this->xyz[1] = val.xyz[1];
			this->xyz[2] = val.xyz[2];
			this->fIds = val.fIds;
		}
	};

	struct MatrixForMesh
	{
		double r[9];

		pctree::XYZPoint operator % (const pctree::XYZPoint& p) const
		{
			pctree::XYZPoint retval;
			retval.xyz[0] = r[0] * p.xyz[0] + r[1] * p.xyz[1] + r[2] * p.xyz[2];
			retval.xyz[1] = r[3] * p.xyz[0] + r[4] * p.xyz[1] + r[5] * p.xyz[2];
			retval.xyz[2] = r[6] * p.xyz[0] + r[7] * p.xyz[1] + r[8] * p.xyz[2];

			return retval;
		}

		MatrixForMesh transpose()
		{
			MatrixForMesh retval;

			retval.r[0] = this->r[0];
			retval.r[3] = this->r[1];
			retval.r[6] = this->r[2];

			retval.r[1] = this->r[3];
			retval.r[4] = this->r[4];
			retval.r[7] = this->r[5];

			retval.r[2] = this->r[6];
			retval.r[5] = this->r[7];
			retval.r[8] = this->r[8];

			return retval;
		}

		void del()
		{
			for (int i = 0; i < 9; ++i) {
				r[i] = 0.;
			}
		}
	};
}