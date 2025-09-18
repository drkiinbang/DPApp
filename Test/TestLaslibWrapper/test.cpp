#include "pch.h"

#include "LaslibWrapper.hpp"

#include <type_traits>
using namespace las;

// ---------- 도우미 ----------
static PointData makePoint(double x, double y, double z,
    uint16_t intensity = 0,
    uint8_t classification = 0,
    uint8_t rn = 1, uint8_t nr = 1,
    int8_t sa = 0, uint8_t ud = 0,
    uint16_t psi = 0, double gps = 0.0,
    std::array<uint16_t, 3> rgb = { 0,0,0 })
{
    PointData p;
    p.x = x; p.y = y; p.z = z;
    p.intensity = intensity;
    p.classification = classification;
    p.return_number = rn;
    p.number_of_returns = nr;
    p.scan_angle = sa;
    p.user_data = ud;
    p.point_source_id = psi;
    p.gps_time = gps;
    p.rgb = rgb;
    return p;
}


// ---------- PointData 유틸리티 ----------
TEST(PointDataTest, HasRGBAndGPSAndDistance)
{
    PointData p0 = makePoint(0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0.0, { 0,0,0 });
    EXPECT_FALSE(p0.hasRGB());
    EXPECT_FALSE(p0.hasGPS());

    PointData p1 = makePoint(3, 4, 12, 0, 0, 1, 1, 0, 0, 0, 123.45, { 10,0,0 });
    EXPECT_TRUE(p1.hasRGB());
    EXPECT_TRUE(p1.hasGPS());

    // 2D: (3,4) → 거리 5
    EXPECT_DOUBLE_EQ(p1.distance2D(p0), 5.0);

    // 3D: (3,4,12) → 거리 13
    EXPECT_DOUBLE_EQ(p1.distance3D(p0), 13.0);

    // 비교 연산자
    PointData p2 = makePoint(3, 4, 12);
    EXPECT_TRUE(p1 == p2);
    PointData p3 = makePoint(3, 4, 13);
    EXPECT_TRUE(p1 < p3);
}

// ---------- HeaderInfo::BoundingBox ----------
TEST(HeaderInfoTest, BoundingBoxContainsAndDims)
{
    HeaderInfo::BoundingBox bb;
    bb.min_x = -10; bb.max_x = 10;
    bb.min_y = -5;  bb.max_y = 5;
    bb.min_z = 0;  bb.max_z = 20;

    PointData inside = makePoint(0, 0, 10);
    PointData edge1 = makePoint(-10, -5, 0);
    PointData edge2 = makePoint(10, 5, 20);
    PointData outside = makePoint(11, 0, 10);

    EXPECT_TRUE(bb.contains(inside));
    EXPECT_TRUE(bb.contains(edge1));
    EXPECT_TRUE(bb.contains(edge2));
    EXPECT_FALSE(bb.contains(outside));

    EXPECT_DOUBLE_EQ(bb.width(), 20.0); // 10 - (-10)
    EXPECT_DOUBLE_EQ(bb.height(), 10.0); // 5 - (-5)
    EXPECT_DOUBLE_EQ(bb.depth(), 20.0); // 20 - 0
}

// ---------- 예외 메시지 프리픽스 ----------
TEST(ExceptionTest, LASExceptionHasPrefix)
{
    try {
        throw LASException("something bad");
    }
    catch (const std::runtime_error& e) {
        // 생성자가 "LAS Error: " 프리픽스를 붙임
        std::string msg = e.what();
        EXPECT_NE(msg.find("LAS Error: "), std::string::npos);
        EXPECT_NE(msg.find("something bad"), std::string::npos);
    }
}

// ---------- LASToolsReader의 초기 상태/이동 가능성 ----------
TEST(ReaderTest, DefaultStateAndMoveSemantics)
{
    // 컴파일 타임 특성
    static_assert(!std::is_copy_constructible<LASToolsReader>::value, "Should not be copyable");
    static_assert(std::is_nothrow_move_constructible<LASToolsReader>::value, "Should be noexcept move-constructible");

    LASToolsReader reader;
    EXPECT_FALSE(reader.isFileOpen());
    EXPECT_EQ(reader.getPointCount(), static_cast<size_t>(0));
    EXPECT_EQ(reader.getLoadedPointCount(), static_cast<size_t>(0));
    EXPECT_FALSE(reader.getHeaderInfo().has_value());

    // move 생성자 동작(빈 상태에서의 안전성 확인)
    LASToolsReader moved(std::move(reader));
    EXPECT_FALSE(moved.isFileOpen());
    EXPECT_EQ(moved.getPointCount(), static_cast<size_t>(0));
    EXPECT_EQ(moved.getLoadedPointCount(), static_cast<size_t>(0));
    EXPECT_FALSE(moved.getHeaderInfo().has_value());
}

// ---------- 문자열 파싱 유틸 ----------
TEST(UtilTest, ParseExportFormat)
{
    EXPECT_EQ(LASToolsReader::parseFormat("xyz"), ExportFormat::XYZ);
    EXPECT_EQ(LASToolsReader::parseFormat("xyzi"), ExportFormat::XYZI);
    EXPECT_EQ(LASToolsReader::parseFormat("xyzic"), ExportFormat::XYZIC);

    // 정의 외 문자열 → FULL
    EXPECT_EQ(LASToolsReader::parseFormat("full"), ExportFormat::FULL);
    EXPECT_EQ(LASToolsReader::parseFormat("XYZ"), ExportFormat::FULL);
    EXPECT_EQ(LASToolsReader::parseFormat(""), ExportFormat::FULL);
}

// ---------- 기본 템플릿 확인 ----------
TEST(SmokeTest, TemplateStillCompiles)
{
    EXPECT_EQ(1, 1);
    EXPECT_TRUE(true);
}