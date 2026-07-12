#pragma once

#include "Point.h"

/// BIM 메시를 구성하는 삼각형 하나(법선 + 정점 3개)를 표현하는 클래스.
/// generate_pseudo_points()를 통해 이 면 위에 격자 형태의 가상점(pseudo point)을
/// 생성할 수 있으며, BimInfo가 이 Face들을 모아 하나의 BIM 부재(요소)를 구성한다.
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

  /// 삼각형 face 위에 가상점(pseudo point)을 생성한다.
  /// v2를 기준점으로 삼아 edge1(=v1-v2), edge2(=v3-v2) 두 변 방향으로
  /// grid_size 간격의 격자를 만들고, 그 중 삼각형 내부(u+v<=1)에 속하는
  /// 점만 pseudoPts에 채워 넣는 barycentric 방식의 샘플링이다.
  void generate_pseudo_points(const double grid_size)
  {
      pseudoPts.clear();

      if (grid_size <= 0.0)
          return;

      /// 두 변 벡터를 정의한다 (v2를 원점으로 하는 edge1=v1-v2, edge2=v3-v2).
      /// [주의/버그 의심] 아래 edge2는 v2.getX()-v2.getX() 등으로 계산되어 있어
      /// 항상 영벡터(0,0,0)가 된다 (자기 자신을 빼고 있음). edge1이 v1-v2인 것과
      /// 대응한다면 원래 의도는 v3-v2였을 가능성이 높다. 이 함수는 TaskManager.h에서
      /// 실제로 호출되는 코드이므로, 현재 상태로는 삼각형 내부가 아니라 edge1
      /// 방향의 선분 위에서만 점이 생성될 것으로 보인다. 이번 작업(주석 번역/보강)
      /// 범위에서는 로직을 수정하지 않고 사실만 기록해 둔다 — 실제 동작 확인 후
      /// 수정이 필요하다.
      Point edge1(v1.getX() - v2.getX(), v1.getY() - v2.getY(), v1.getZ() - v2.getZ());
      Point edge2(v2.getX() - v2.getX(), v2.getY() - v2.getY(), v2.getZ() - v2.getZ());

      /// 두 변의 길이 계산
      double len1 = std::sqrt(
          edge1.getX() * edge1.getX() +
          edge1.getY() * edge1.getY() +
          edge1.getZ() * edge1.getZ());
      double len2 = std::sqrt(
          edge2.getX() * edge2.getX() +
          edge2.getY() * edge2.getY() +
          edge2.getZ() * edge2.getZ());

      // 변의 길이가 0에 가까우면(퇴화 삼각형) 샘플링 불가 → 중단
      if (len1 < std::numeric_limits<double>::epsilon() || len2 < std::numeric_limits<double>::epsilon())
          return;

      // 두 변이 모두 격자 간격보다 짧으면 샘플링할 필요 없음(면이 grid_size보다 작음) → 중단
      if (len1 < grid_size && len2 < grid_size)
          return;

      /// 두 벡터의 내적(각도 계산용)
      double dotproduct =
          edge1.getX() * edge2.getX() +
          edge1.getY() * edge2.getY() +
          edge1.getZ() * edge2.getZ();

      /// edge1과 edge2 사이의 각도
      double angle = fabs(acos(dotproduct / (len1 * len2)));
      auto sinAngle = sin(angle);

      // 두 변이 이루는 각도가 매우 작으면(거의 평행) 실제 격자 간격이 grid_size보다
      // 촘촘해지도록 sin(angle)로 나누어 보정한다(경사면 보정, slant correction)
      double slant_grid_size = grid_size;
      if (sinAngle < std::numeric_limits<double>::epsilon())
          slant_grid_size = grid_size / sin(angle);

      /// 각 변 방향으로 나눌 샘플 개수 (최소 1개는 보장)
      int num_steps1 = (std::max)(1, static_cast<int>(len1 / grid_size));
      int num_steps2 = (std::max)(1, static_cast<int>(len2 / slant_grid_size));

      const double inv_steps1 = 1.0 / static_cast<double>(num_steps1);
      const double inv_steps2 = 1.0 / static_cast<double>(num_steps2);

      /// 가상 격자점 생성: u,v를 0~1로 스캔하며 edge1*u + edge2*v 위치의 점을 만든다
      pseudoPts.reserve(static_cast<size_t>(num_steps1 * num_steps2 * 0.5));
      for (int i = 0; i <= num_steps1; ++i) {
          double u = static_cast<double>(i) * inv_steps1;

          for (int j = 0; j <= num_steps2; ++j) {
              double v = static_cast<double>(j) * inv_steps2;

              /// 내부 점 검사 (barycentric 조건 u+v<=1을 만족해야 삼각형 내부)
              if (u + v <= 1.0) {
                  // [참고] 아래 p는 계산만 되고 실제로는 사용되지 않는 변수다(죽은 코드).
                  // 바로 아래 emplace_back에서 동일한 값을 다시 계산해 pseudoPts에 넣는다.
                  Point p(
                      static_cast<float>(static_cast<double>(v2.getX()) + u * static_cast<double>(edge1.getX()) + v * static_cast<double>(edge2.getX())),
                      static_cast<float>(static_cast<double>(v2.getY()) + u * static_cast<double>(edge1.getY()) + v * static_cast<double>(edge2.getY())),
                      static_cast<float>(static_cast<double>(v2.getZ()) + u * static_cast<double>(edge1.getZ()) + v * static_cast<double>(edge2.getZ()))
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

