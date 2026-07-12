//#ifndef CHANGE_DECTION_SRC_DOMAIN_POINT_H_
//#define CHANGE_DECTION_SRC_DOMAIN_POINT_H_
#pragma once

#include <iostream>
#include <cmath>

/// float 기반의 범용 3차원 점/벡터 클래스. include/bim 하위(Face, BimInfo, gltf 등)에서
/// 정점 좌표와 법선 벡터를 표현하는 데 널리 쓰인다. XYZ(include/bim/XYZ.h)와 유사하지만
/// 벡터 연산(외적/내적/정규화 등)까지 자체적으로 제공한다는 점이 다르다.
class Point {

public:
    Point() {
      coords[0] = 0.0f;
      coords[1] = 0.0f;
      coords[2] = 0.0f;
    }
    Point(float x, float y, float z) {
      coords[0] = x;
      coords[1] = y;
      coords[2] = z;
    }

    Point(float* coords) {
      memcpy(this->coords, coords, sizeof(float) * 3);
    }

    Point(const Point& other) {
      assign(other);
    }
    
    float* getCoords() { return coords; }

    inline auto getX() const { return coords[0]; }
    inline auto getY() const { return coords[1]; }
    inline auto getZ() const { return coords[2]; }

    inline void setX(float x) { coords[0] = x;  }
    inline void setY(float y) { coords[1] = y;  }
    inline void setZ(float z) { coords[2] = z;  }

    inline bool isEmpty() {
      if (coords[0] == 0.0 && coords[1] == 0.0 && coords[2] == 0.0) {
        return true;
      } else {
        return false;
      }
    }

    inline float at(size_t idx) const {
      if (idx > 2) {
        std::cerr << "index over Z" << std::endl;
        return 0.0;
      }
      return coords[idx];
    }

    Point& operator=(const Point& other) {
      if (this == &other) {
        return *this;
      }
      assign(other);
      return *this;
    }  


    // 벡터 덧셈
    Point operator+(const Point& other) const {
      return Point(coords[0] + other.coords[0], coords[1] + other.coords[1], coords[2] + other.coords[2]);
    }

    // 벡터 뺄셈
    Point operator-(const Point& other) const {
      return Point(coords[0] - other.coords[0], coords[1] - other.coords[1], coords[2] - other.coords[2]);
    }

    // 스칼라 곱(scale)
    Point operator*(const float scale) const {
        return Point(this->coords[0] * scale, this->coords[1] * scale, this->coords[2] * scale);
    }

    // 벡터 외적(cross product)
    Point crossProduct(const Point& other) const {
      return Point(
        coords[1] * other.coords[2] - coords[2] * other.coords[1],
        coords[2] * other.coords[0] - coords[0] * other.coords[2],
        coords[0] * other.coords[1] - coords[1] * other.coords[0]
      );
    }

    // 벡터 내적(dot product)
    float dotProduct(const Point& other) const {
      return coords[0] * other.coords[0] + coords[1] * other.coords[1] + coords[2] * other.coords[2];
    }

    // 벡터 정규화(단위 벡터로 변환).
    // [주의] length가 0인 경우(영벡터를 정규화하려는 경우)에 대한 방어 코드가 없어
    // 0으로 나누기(결과는 NaN/Inf)가 발생할 수 있다. 호출부에서 길이가 0이 아님을
    // 보장해야 한다(예: computeNormal()에서 퇴화 삼각형이 아닌 경우에만 호출).
    Point normalize() const {
      float length = std::sqrt(coords[0] * coords[0] + coords[1] * coords[1] + coords[2] * coords[2]);
      return Point(coords[0] / length, coords[1] / length, coords[2] / length);
    }

   protected:
     void assign(const Point& other) {
       memcpy(coords, other.coords, sizeof(float) * 3);
     }

   private:
    float coords[3];
};

//#endif //CHANGE_DECTION_SRC_DOMAIN_POINT_H_
