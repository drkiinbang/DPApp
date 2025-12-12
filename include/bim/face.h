#pragma once

#include "Point.h"

class Face {
 public:
  Face() = default;
  Face(Point normal, Point v1, Point v2, Point v3) :
        normal(normal),    v1(v1),    v2(v2),    v3(v3) {  }

  inline auto getNormal() const { return normal; }
  inline auto getv1() const { return v1; }
  inline auto getv2() const { return v2; }
  inline auto getv3() const { return v3; }

  inline auto getVertex(size_t idx) const {
    switch (idx) {
      case 0:return v1;
      case 1:return v2;
      case 2:return v3;
      default:return v3;
    }
  }

  /// Generate psuedo points on a triangular face
  void generate_pseudo_points(const double grid_size)
  {
      pseudoPts.clear();

      if (grid_size <= 0.0)
          return;

      /// Define two vectors
      Point edge1(v1.getX() - v2.getX(), v1.getY() - v2.getY(), v1.getZ() - v2.getZ());
      Point edge2(v2.getX() - v2.getX(), v2.getY() - v2.getY(), v2.getZ() - v2.getZ());

      /// Calculate lengths
      double len1 = std::sqrt(
          edge1.getX() * edge1.getX() +
          edge1.getY() * edge1.getY() +
          edge1.getZ() * edge1.getZ());
      double len2 = std::sqrt(
          edge2.getX() * edge2.getX() +
          edge2.getY() * edge2.getY() +
          edge2.getZ() * edge2.getZ());

      if (len1 < std::numeric_limits<double>::epsilon() || len2 < std::numeric_limits<double>::epsilon())
          return;

      if (len1 < grid_size && len2 < grid_size)
          return;

      /// Inner product of the two vectors
      double dotproduct =
          edge1.getX() * edge2.getX() +
          edge1.getY() * edge2.getY() +
          edge1.getZ() * edge2.getZ();

      /// Angle between edge1 and edge2
      double angle = fabs(acos(dotproduct / (len1 * len2)));
      auto sinAngle = sin(angle);

      double slant_grid_size = grid_size;
      if (sinAngle < std::numeric_limits<double>::epsilon())
          slant_grid_size = grid_size / sin(angle);

      /// Number of samples
      int num_steps1 = (std::max)(1, static_cast<int>(len1 / grid_size));
      int num_steps2 = (std::max)(1, static_cast<int>(len2 / slant_grid_size));

      const double inv_steps1 = 1.0 / static_cast<double>(num_steps1);
      const double inv_steps2 = 1.0 / static_cast<double>(num_steps2);

      /// Generate pseudo grid points
      pseudoPts.reserve(static_cast<size_t>(num_steps1 * num_steps2 * 0.5));
      for (int i = 0; i <= num_steps1; ++i) {
          double u = static_cast<double>(i) * inv_steps1;

          for (int j = 0; j <= num_steps2; ++j) {
              double v = static_cast<double>(j) * inv_steps2;

              /// Check inner points (triangle inside)
              if (u + v <= 1.0) {
                  Point p(
                      static_cast<double>(v2.getX()) + u * static_cast<double>(edge1.getX()) + v * static_cast<double>(edge2.getX()),
                      static_cast<double>(v2.getY()) + u * static_cast<double>(edge1.getY()) + v * static_cast<double>(edge2.getY()),
                      static_cast<double>(v2.getZ()) + u * static_cast<double>(edge1.getZ()) + v * static_cast<double>(edge2.getZ())
                  );
                  pseudoPts.emplace_back(v2.getX() + u * edge1.getX() + v * edge2.getX(),
                      v2.getY() + u * edge1.getY() + v * edge2.getY(),
                      v2.getZ() + u * edge1.getZ() + v * edge2.getZ());
              }
          }
      }

      return;
  }

 protected:
 private:
  Point normal;
  Point v1;
  Point v2;
  Point v3;
  std::vector<Point> pseudoPts;
};

