//#ifndef CHANGE_DECTION_SRC_DOMAIN_POINT_H_
//#define CHANGE_DECTION_SRC_DOMAIN_POINT_H_
#pragma once

#include <iostream>
#include <cmath>

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


    // Vector addition
    Point operator+(const Point& other) const {
      return Point(coords[0] + other.coords[0], coords[1] + other.coords[1], coords[2] + other.coords[2]);
    }

    // Vector subtraction
    Point operator-(const Point& other) const {
      return Point(coords[0] - other.coords[0], coords[1] - other.coords[1], coords[2] - other.coords[2]);
    }

    // scale
    Point operator*(const float scale) const {
        return Point(this->coords[0] * scale, this->coords[1] * scale, this->coords[2] * scale);
    }

    // Vector cross product
    Point crossProduct(const Point& other) const {
      return Point(
        coords[1] * other.coords[2] - coords[2] * other.coords[1],
        coords[2] * other.coords[0] - coords[0] * other.coords[2],
        coords[0] * other.coords[1] - coords[1] * other.coords[0]
      );
    }

    // Vector dot product
    float dotProduct(const Point& other) const {
      return coords[0] * other.coords[0] + coords[1] * other.coords[1] + coords[2] * other.coords[2];
    }

    // Vector normalization
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
