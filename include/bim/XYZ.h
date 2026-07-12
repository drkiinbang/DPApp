#pragma once

/// float 기반의 경량 3차원 벡터. BIM 관련 코드에서 정점/이동량을 표현하는 데 쓰인다
/// (cnvrt::POINT, pctree::XYZPoint와는 별개의, 이 모듈 전용의 단순한 타입).
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