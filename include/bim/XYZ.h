#pragma once

struct XYZ {

  float x, y, z;

  XYZ operator+(const XYZ& rhs) const
  {
    return XYZ{ x + rhs.x, y + rhs.y, z + rhs.z };
  }
  XYZ operator-(const XYZ& rhs) const
  {
    return XYZ{ x - rhs.x, y - rhs.y, z - rhs.z };
  }
  XYZ operator*(const float s) const
  {
    return XYZ{ x * s, y * s, z * s };
  }
};

typedef XYZ TRANSLATION;