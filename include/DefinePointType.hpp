#pragma once

/// 포맷 변환(cnvrt) 유틸리티가 사용하는 경량 점/바운딩박스 타입 모음.
/// icp::Transform4x4나 pctree::XYZPoint와는 별개의, 이 네임스페이스 전용 타입이다.
#include <string>

namespace cnvrt
{
    constexpr double UNDER_POINT[] = {
                0.1,
                0.01,
                0.001,
                0.0001,
                0.00001,
                0.000001,
                0.0000001,
                0.00000001,
                0.000000001,
                0.0000000001,
                0.00000000001,
                0.000000000001 };

    constexpr size_t NUM_POINT_IN_BUF = 64 * 1024 * 1024;

    struct POINT {
        float x, y, z;
        POINT operator+(const POINT& rhs) const
        {
            return POINT{ x + rhs.x, y + rhs.y, z + rhs.z };
        }
        POINT operator-(const POINT& rhs) const
        {
            return POINT{ x - rhs.x, y - rhs.y, z - rhs.z };
        }
        POINT operator*(const float s) const
        {
            return POINT{ x * s, y * s, z * s };
        }
    };
    typedef POINT TRANSLATION;


    struct POINT_DOUBLE {
        double x, y, z;
    };

    class BOUNDING_BOX {
        POINT_DOUBLE min_p, max_p;
    public:
        BOUNDING_BOX() : min_p(POINT_DOUBLE{ (std::numeric_limits<double>::max)(), (std::numeric_limits<double>::max)(), (std::numeric_limits<double>::max)() }),
            max_p(POINT_DOUBLE{ -(std::numeric_limits<double>::max)(), -(std::numeric_limits<double>::max)(), -(std::numeric_limits<double>::max)() }) {
        }
        void merge(const BOUNDING_BOX& bb)
        {
            min_p.x = (std::min)(min_p.x, bb.min_p.x);
            min_p.y = (std::min)(min_p.y, bb.min_p.y);
            min_p.z = (std::min)(min_p.z, bb.min_p.z);
            max_p.x = (std::max)(max_p.x, bb.max_p.x);
            max_p.y = (std::max)(max_p.y, bb.max_p.y);
            max_p.z = (std::max)(max_p.z, bb.max_p.z);
        }
        void merge_point(const POINT_DOUBLE& p)
        {
            min_p.x = (std::min)(min_p.x, p.x);
            min_p.y = (std::min)(min_p.y, p.y);
            min_p.z = (std::min)(min_p.z, p.z);
            max_p.x = (std::max)(max_p.x, p.x);
            max_p.y = (std::max)(max_p.y, p.y);
            max_p.z = (std::max)(max_p.z, p.z);
        }

        void merge_points(const POINT_DOUBLE points[], const size_t count)
        {
            for (size_t i = 0; i < count; ++i) {
                min_p.x = (std::min)(min_p.x, points[i].x);
                min_p.y = (std::min)(min_p.y, points[i].y);
                min_p.z = (std::min)(min_p.z, points[i].z);
                max_p.x = (std::max)(max_p.x, points[i].x);
                max_p.y = (std::max)(max_p.y, points[i].y);
                max_p.z = (std::max)(max_p.z, points[i].z);
            }
        }

        /// 바운딩 박스를 (가장 긴 변에 맞춘) 정육면체로 확장한다.
        /// [주의] z_center 대신 x_center로 z의 min/max를 계산하고 있어(복사-붙여넣기
        /// 오류로 보임) z축 결과가 잘못될 수 있다. 현재 이 함수는 코드베이스 어디에서도
        /// 호출되지 않는 미사용 함수이므로 아직 실제 영향은 없지만, 향후 사용하기 전에
        /// 반드시 수정할 것.
        void make_cube()
        {
            double x_span = max_p.x - min_p.x;
            double y_span = max_p.y - min_p.y;
            double z_span = max_p.z - min_p.z;

            double max_span = (std::max)(x_span, (std::max)(y_span, z_span));

            double x_center = (min_p.x + max_p.x) / 2;
            double y_center = (min_p.y + max_p.y) / 2;
            double z_center = (min_p.z + max_p.z) / 2;

            min_p.x = x_center - max_span / 2;
            min_p.y = y_center - max_span / 2;
            min_p.z = x_center - max_span / 2;
            max_p.x = x_center + max_span / 2;
            max_p.y = y_center + max_span / 2;
            max_p.z = x_center + max_span / 2;
        }

        POINT_DOUBLE get_center()
        {
            return POINT_DOUBLE{ (min_p.x + max_p.x) / 2, (min_p.y + max_p.y) / 2, (min_p.z + max_p.z) / 2, };
        }

        POINT get_center_single()
        {
            return POINT{ static_cast<float>((min_p.x + max_p.x) / 2), static_cast<float>((min_p.y + max_p.y) / 2), static_cast<float>((min_p.z + max_p.z) / 2) };
        }

        POINT get_min_single()
        {
            return POINT{ static_cast<float>(min_p.x), static_cast<float>(min_p.y), static_cast<float>(min_p.z) };
        }

        POINT get_max_single()
        {
            return POINT{ static_cast<float>(max_p.x), static_cast<float>(max_p.y), static_cast<float>(max_p.z) };
        }

        std::string dump()
        {
            std::string d = std::format("AABB min ({}, {}, {})\n", min_p.x, min_p.y, min_p.z);
            d += std::format("AABB max ({}, {}, {})", max_p.x, max_p.y, max_p.z);
            return d;
        }
    };
}