#include "pch.h"

#include "LaslibReader.hpp"
#include "LaslibWriter.hpp"

#include <filesystem>
#include <array>
#include <vector>
#include <string>
#include <random>
#include <limits>
#include <cmath>
#include <algorithm>

using namespace las;
using std::filesystem::path;
using std::filesystem::temp_directory_path;

// ---------- Helpers ----------
static PointData mk(double x, double y, double z,
    uint16_t intensity = 0,
    uint8_t classification = 0,
    uint8_t rn = 1, uint8_t nr = 1,
    int8_t sa = 0, uint8_t ud = 0,
    uint16_t psi = 0, double gps = 0.0,
    std::array<uint16_t, 3> rgb = { 0,0,0 })
{
    PointData p{};
    p.x = x; p.y = y; p.z = z;
    p.intensity = intensity; p.classification = classification;
    p.return_number = rn; p.number_of_returns = nr;
    p.scan_angle = sa; p.user_data = ud; p.point_source_id = psi;
    p.gps_time = gps; p.rgb = rgb;
    return p;
}

static void remove_if_exists(const path& p) {
    std::error_code ec; std::filesystem::remove(p, ec);
}

static void basic_file_checks(const path& p) {
    ASSERT_TRUE(std::filesystem::exists(p));
    ASSERT_GT(std::filesystem::file_size(p), 0u);
}

static std::vector<PointData> make_sample_points(std::size_t n)
{
    std::vector<PointData> pts;
    pts.reserve(n);
    // 작은 그리드
    const int g = 5;
    for (int i = 0; i < g; i++) for (int j = 0; j < g; j++) {
        double x = 10.0 + i * 0.5;
        double y = 20.0 + j * 0.5;
        double z = 30.0 + (i - j) * 0.1;
        pts.push_back(mk(x, y, z,
            100 + i + j,
            static_cast<uint8_t>((i + j) % 5),
            1, 1,
            static_cast<int8_t>(i - j),
            static_cast<uint8_t>(i * j),
            static_cast<uint16_t>(100 + i * 10 + j),
            1234.0 + i * 10 + j,
            { static_cast<uint16_t>(i * 50),
             static_cast<uint16_t>(j * 40),
             static_cast<uint16_t>((i + j) * 30) }));
    }
    // 재현성 있는 난수 점
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> ux(-5.0, 5.0);
    for (std::size_t k = pts.size(); k < n; ++k) {
        double x = 12.0 + ux(rng);
        double y = 22.0 + ux(rng);
        double z = 28.0 + ux(rng);
        pts.push_back(mk(x, y, z, 200,
            static_cast<uint8_t>(k % 7),
            static_cast<uint8_t>((k % 3) + 1),
            3,
            static_cast<int8_t>((k % 11) - 5),
            static_cast<uint8_t>(k % 255),
            static_cast<uint16_t>(300 + k % 100),
            2000.0 + k,
            { static_cast<uint16_t>((k % 10) * 100),
             static_cast<uint16_t>((k % 15) * 50),
             static_cast<uint16_t>((k % 12) * 80) }));
    }
    return pts;
}

static void save_points_as(const std::string& filename,
    const std::vector<PointData>& pts,
    double scale = 0.01,
    double ox = 0.0, double oy = 0.0, double oz = 0.0)
{
    HeaderInfo hdr{};
    hdr.version_major = 1;
    hdr.version_minor = 2;
    hdr.system_identifier = "UnitTest";
    hdr.generating_software = "GTest";
    hdr.coordinate_transform = { scale,scale,scale, ox,oy,oz };
    hdr.point_data_record_length = 34; // pf=3: 34 bytes 명시 (경고 억제)
    ASSERT_TRUE(LASToolsWriter::save(filename, pts, /*pf=*/3, hdr));
}

// ======================================================
// PointData tests
// ======================================================
TEST(PointDataTest, HasRGBAndGPSAndDistance)
{
    PointData p0 = mk(0, 0, 0);
    EXPECT_FALSE(p0.hasRGB());
    EXPECT_FALSE(p0.hasGPS());

    PointData p1 = mk(3, 4, 12, 0, 0, 1, 1, 0, 0, 0, 123.45, { 10,0,0 });
    EXPECT_TRUE(p1.hasRGB());
    EXPECT_TRUE(p1.hasGPS());

    EXPECT_DOUBLE_EQ(p1.distance2D(p0), 5.0);
    EXPECT_DOUBLE_EQ(p1.distance3D(p0), 13.0);

    PointData p2 = mk(3, 4, 12);
    EXPECT_TRUE(p1 == p2);
    PointData p3 = mk(3, 4, 13);
    EXPECT_TRUE(p1 < p3);
}

// ======================================================
// HeaderInfo::BoundingBox tests
// ======================================================
TEST(HeaderInfoTest, BoundingBoxContainsAndDims)
{
    HeaderInfo::BoundingBox bb;
    bb.min_x = -10; bb.max_x = 10;
    bb.min_y = -5;  bb.max_y = 5;
    bb.min_z = 0;  bb.max_z = 20;

    PointData inside = mk(0, 0, 10);
    PointData edge1 = mk(-10, -5, 0);
    PointData edge2 = mk(10, 5, 20);
    PointData outside = mk(11, 0, 10);

    EXPECT_TRUE(bb.contains(inside));
    EXPECT_TRUE(bb.contains(edge1));
    EXPECT_TRUE(bb.contains(edge2));
    EXPECT_FALSE(bb.contains(outside));

    EXPECT_DOUBLE_EQ(bb.width(), 20.0);
    EXPECT_DOUBLE_EQ(bb.height(), 10.0);
    EXPECT_DOUBLE_EQ(bb.depth(), 20.0);
}

// ======================================================
// Exception test
// ======================================================
TEST(ExceptionTest, LASExceptionHasPrefix)
{
    try {
        throw LASException("something bad");
    }
    catch (const std::runtime_error& e) {
        std::string msg = e.what();
        EXPECT_NE(msg.find("LAS Error: "), std::string::npos);
        EXPECT_NE(msg.find("something bad"), std::string::npos);
    }
}

// ======================================================
// Reader default/move semantics
// ======================================================
TEST(ReaderTest, DefaultStateAndMoveSemantics)
{
    static_assert(!std::is_copy_constructible<LASToolsReader>::value, "Should not be copyable");
    static_assert(std::is_nothrow_move_constructible<LASToolsReader>::value, "Should be noexcept move-constructible");

    LASToolsReader reader;
    EXPECT_FALSE(reader.isFileOpen());
    EXPECT_EQ(reader.getPointCount(), static_cast<size_t>(0));
    EXPECT_EQ(reader.getLoadedPointCount(), static_cast<size_t>(0));
    EXPECT_FALSE(reader.getHeaderInfo().has_value());

    LASToolsReader moved(std::move(reader));
    EXPECT_FALSE(moved.isFileOpen());
    EXPECT_EQ(moved.getPointCount(), static_cast<size_t>(0));
    EXPECT_EQ(moved.getLoadedPointCount(), static_cast<size_t>(0));
    EXPECT_FALSE(moved.getHeaderInfo().has_value());
}

// ======================================================
// Util parse
// ======================================================
TEST(UtilTest, ParseExportFormat)
{
    EXPECT_EQ(LASToolsReader::parseFormat("xyz"), ExportFormat::XYZ);
    EXPECT_EQ(LASToolsReader::parseFormat("xyzi"), ExportFormat::XYZI);
    EXPECT_EQ(LASToolsReader::parseFormat("xyzic"), ExportFormat::XYZIC);
    EXPECT_EQ(LASToolsReader::parseFormat("full"), ExportFormat::FULL);
    EXPECT_EQ(LASToolsReader::parseFormat("XYZ"), ExportFormat::FULL);
    EXPECT_EQ(LASToolsReader::parseFormat(""), ExportFormat::FULL);
}

// ======================================================
// Writer tests (round-trip with small datasets)
// ======================================================
TEST(WriterTest, WriteLasAndReadBack)
{
    const path out = temp_directory_path() / "ut_lastools_writer_out.las";
    remove_if_exists(out);

    std::vector<PointData> pts = {
        mk(100.12, 200.34,  50.56,  120, 2, 1, 1,  0,  9,  1, 12345.0, {255,128, 64}),
        mk(101.12, 201.34,  51.56,  220, 6, 2, 3, -2, 10,  1, 12346.0, {  0, 16,255}),
        mk(-10.0,  -20.0,  -30.0, 1000, 1, 1, 1,  1, 20, 42, 0.0,      { 10, 20, 30}),
    };

    HeaderInfo hdr{};
    hdr.version_major = 1; hdr.version_minor = 2;
    hdr.system_identifier = "UnitTest"; hdr.generating_software = "GTest";
    hdr.coordinate_transform = { 0.01,0.01,0.01, 0,0,0 };
    hdr.point_data_record_length = 34;

    ASSERT_TRUE(LASToolsWriter::save(out.string(), pts, 3, hdr));

    LASToolsReader r;
    ASSERT_TRUE(r.open(out.string()));
    ASSERT_TRUE(r.loadAllPoints());
    const auto& back = r.getLoadedPoints();
    ASSERT_EQ(back.size(), pts.size());

    const double tol = 0.015;
    for (size_t i = 0; i < pts.size(); ++i) {
        EXPECT_NEAR(pts[i].x, back[i].x, tol);
        EXPECT_NEAR(pts[i].y, back[i].y, tol);
        EXPECT_NEAR(pts[i].z, back[i].z, tol);
        EXPECT_EQ(pts[i].intensity, back[i].intensity);
        EXPECT_EQ(pts[i].classification, back[i].classification);
        EXPECT_EQ(pts[i].return_number, back[i].return_number);
        EXPECT_EQ(pts[i].number_of_returns, back[i].number_of_returns);
        EXPECT_EQ(pts[i].scan_angle, back[i].scan_angle);
        EXPECT_EQ(pts[i].user_data, back[i].user_data);
        EXPECT_EQ(pts[i].point_source_id, back[i].point_source_id);
        EXPECT_NEAR(pts[i].gps_time, back[i].gps_time, 1e-6);
        EXPECT_EQ(pts[i].rgb[0], back[i].rgb[0]);
        EXPECT_EQ(pts[i].rgb[1], back[i].rgb[1]);
        EXPECT_EQ(pts[i].rgb[2], back[i].rgb[2]);
    }

    // 경계 포함성 (양자화 고려)
    auto hi = r.getHeaderInfo(); ASSERT_TRUE(hi.has_value());
    const auto& bb = hi->bounds;
    const double eps = 0.015; // scale=0.01 → 1.5cm
    for (const auto& p : pts) {
        EXPECT_LE(bb.min_x - eps, p.x); EXPECT_GE(bb.max_x + eps, p.x);
        EXPECT_LE(bb.min_y - eps, p.y); EXPECT_GE(bb.max_y + eps, p.y);
        EXPECT_LE(bb.min_z - eps, p.z); EXPECT_GE(bb.max_z + eps, p.z);
    }

    r.close(); remove_if_exists(out);
}

TEST(WriterTest, WriteLazAndReadBack)
{
    const path out = temp_directory_path() / "ut_lastools_writer_out.laz";
    remove_if_exists(out);

    std::vector<PointData> pts;
    for (int i = 0; i < 10; ++i) {
        pts.push_back(mk(10 + i * 0.1, -5 + i * 0.2, 2 + i * 0.3,
            50 + i, static_cast<uint8_t>(i % 5),
            (i % 3) + 1, 3,
            static_cast<int8_t>(i - 5), static_cast<uint8_t>(i),
            static_cast<uint16_t>(100 + i), 1000.0 + i,
            { static_cast<uint16_t>(i * 10), static_cast<uint16_t>(i * 20), static_cast<uint16_t>(i * 30) }));
    }

    HeaderInfo hdr{};
    hdr.version_major = 1; hdr.version_minor = 2;
    hdr.coordinate_transform = { 0.001,0.001,0.001, 0,0,0 };
    hdr.point_data_record_length = 34;

    ASSERT_TRUE(LASToolsWriter::save(out.string(), pts, 3, hdr));

    LASToolsReader r;
    ASSERT_TRUE(r.open(out.string()));
    ASSERT_TRUE(r.loadAllPoints());
    const auto& back = r.getLoadedPoints();
    ASSERT_EQ(back.size(), pts.size());

    const double tol = 0.002; // 2 mm
    for (size_t i = 0; i < pts.size(); ++i) {
        EXPECT_NEAR(pts[i].x, back[i].x, tol);
        EXPECT_NEAR(pts[i].y, back[i].y, tol);
        EXPECT_NEAR(pts[i].z, back[i].z, tol);
    }
    r.close(); remove_if_exists(out);
}

TEST(WriterTest, WriterStateAndMisuseGuards)
{
    LASToolsWriter w;
    EXPECT_FALSE(w.isOpen());
    EXPECT_EQ(w.writtenCount(), (size_t)0);

    EXPECT_FALSE(w.writePoint(mk(0, 0, 0))); // open 전에 실패

    const path out = temp_directory_path() / "ut_lastools_writer_state.las";
    remove_if_exists(out);

    HeaderInfo hdr{};
    hdr.version_major = 1; hdr.version_minor = 2;
    hdr.coordinate_transform = { 0.01,0.01,0.01,0,0,0 };
    hdr.point_data_record_length = 34;

    ASSERT_TRUE(w.open(out.string(), 3, hdr));
    EXPECT_TRUE(w.isOpen());
    EXPECT_TRUE(w.writePoint(mk(1, 2, 3, 100, 1, 1, 1, 0, 0, 0, 0.0, { 0,0,0 })));
    EXPECT_EQ(w.writtenCount(), (size_t)1);
    EXPECT_FALSE(w.open(out.string(), 3, hdr)); // 중복 open 방지
    w.close(); w.close(); // idem

    LASToolsReader r;
    ASSERT_TRUE(r.open(out.string()));
    EXPECT_EQ(r.getPointCount(), (size_t)1);
    r.close();

    remove_if_exists(out);
}

TEST(WriterTest, HeaderPresetBoundsAndScalePersist)
{
    const path out = temp_directory_path() / "ut_lastools_writer_header.las";
    remove_if_exists(out);

    std::vector<PointData> pts = { mk(10,10,10), mk(20,10,5), mk(15,25,-5) };

    HeaderInfo hdr{};
    hdr.version_major = 1; hdr.version_minor = 2;
    hdr.coordinate_transform = { 0.01,0.01,0.01, 1000.0,2000.0,-3000.0 };
    hdr.point_data_record_length = 34;
    hdr.bounds = { 0.0,30.0, 0.0,30.0, -10.0,30.0 };

    ASSERT_TRUE(LASToolsWriter::save(out.string(), pts, 3, hdr));

    LASToolsReader r;
    ASSERT_TRUE(r.open(out.string()));
    auto hi = r.getHeaderInfo(); ASSERT_TRUE(hi.has_value());

    EXPECT_DOUBLE_EQ(hi->coordinate_transform.x_scale, 0.01);
    EXPECT_DOUBLE_EQ(hi->coordinate_transform.y_scale, 0.01);
    EXPECT_DOUBLE_EQ(hi->coordinate_transform.z_scale, 0.01);
    EXPECT_DOUBLE_EQ(hi->coordinate_transform.x_offset, 1000.0);
    EXPECT_DOUBLE_EQ(hi->coordinate_transform.y_offset, 2000.0);
    EXPECT_DOUBLE_EQ(hi->coordinate_transform.z_offset, -3000.0);

    // 경계는 보통 인벤토리 기반으로 기록됨(= 실제 min/max)
    const auto& bb = hi->bounds;
    const double tol = 0.015;
    const double px_min = (std::min)({ pts[0].x, pts[1].x, pts[2].x });
    const double px_max = (std::max)({ pts[0].x, pts[1].x, pts[2].x });
    const double py_min = (std::min)({ pts[0].y, pts[1].y, pts[2].y });
    const double py_max = (std::max)({ pts[0].y, pts[1].y, pts[2].y });
    const double pz_min = (std::min)({ pts[0].z, pts[1].z, pts[2].z });
    const double pz_max = (std::max)({ pts[0].z, pts[1].z, pts[2].z });

    EXPECT_NEAR(bb.min_x, px_min, tol);
    EXPECT_NEAR(bb.max_x, px_max, tol);
    EXPECT_NEAR(bb.min_y, py_min, tol);
    EXPECT_NEAR(bb.max_y, py_max, tol);
    EXPECT_NEAR(bb.min_z, pz_min, tol);
    EXPECT_NEAR(bb.max_z, pz_max, tol);

    r.close(); remove_if_exists(out);
}

// ======================================================
// Integration tests (quantization-aware header checks)
// ======================================================
TEST(WriterIntegrationTest, CreateAndReadLAS)
{
    const path out = temp_directory_path() / "ut_writer_integration.las";
    remove_if_exists(out);

    auto pts = make_sample_points(64);
    save_points_as(out.string(), pts, /*scale*/0.01);
    basic_file_checks(out);

    LASToolsReader r;
    ASSERT_TRUE(r.open(out.string()));
    ASSERT_TRUE(r.loadAllPoints());
    const auto& back = r.getLoadedPoints();
    ASSERT_EQ(back.size(), pts.size());

    // 대표 인덱스 비교
    const double tol = 0.015;
    for (int i : {0, 7, 18, 31, 63}) {
        EXPECT_NEAR(pts[i].x, back[i].x, tol);
        EXPECT_NEAR(pts[i].y, back[i].y, tol);
        EXPECT_NEAR(pts[i].z, back[i].z, tol);
    }

    auto hi = r.getHeaderInfo(); ASSERT_TRUE(hi.has_value());
    const auto& ct = hi->coordinate_transform;
    const double sx = ct.x_scale, sy = ct.y_scale, sz = ct.z_scale;
    const double ox = ct.x_offset, oy = ct.y_offset, oz = ct.z_offset;

    // 정수 그리드로 양자화 후 기대 경계 재구성
    auto qx = [&](double v) { return std::llround((v - ox) / sx); };
    auto qy = [&](double v) { return std::llround((v - oy) / sy); };
    auto qz = [&](double v) { return std::llround((v - oz) / sz); };

    long long mx_q = (std::numeric_limits<long long>::max)();
    long long my_q = (std::numeric_limits<long long>::max)();
    long long mz_q = (std::numeric_limits<long long>::max)();
    long long Mx_q = (std::numeric_limits<long long>::min)();
    long long My_q = (std::numeric_limits<long long>::min)();
    long long Mz_q = (std::numeric_limits<long long>::min)();

    for (const auto& p : back) {
        mx_q = (std::min)(mx_q, qx(p.x));
        my_q = (std::min)(my_q, qy(p.y));
        mz_q = (std::min)(mz_q, qz(p.z));
        Mx_q = (std::max)(Mx_q, qx(p.x));
        My_q = (std::max)(My_q, qy(p.y));
        Mz_q = (std::max)(Mz_q, qz(p.z));
    }

    const double exp_min_x = mx_q * sx + ox;
    const double exp_max_x = Mx_q * sx + ox;
    const double exp_min_y = my_q * sy + oy;
    const double exp_max_y = My_q * sy + oy;
    const double exp_min_z = mz_q * sz + oz;
    const double exp_max_z = Mz_q * sz + oz;

    const auto& bb = hi->bounds;
    const double qtol = (std::max)({ sx,sy,sz }) * 1.1; // 스케일 수준 허용
    EXPECT_NEAR(bb.min_x, exp_min_x, qtol);
    EXPECT_NEAR(bb.max_x, exp_max_x, qtol);
    EXPECT_NEAR(bb.min_y, exp_min_y, qtol);
    EXPECT_NEAR(bb.max_y, exp_max_y, qtol);
    EXPECT_NEAR(bb.min_z, exp_min_z, qtol);
    EXPECT_NEAR(bb.max_z, exp_max_z, qtol);

    r.close(); remove_if_exists(out);
}

TEST(WriterIntegrationTest, CreateAndReadLAZ)
{
    const path out = temp_directory_path() / "ut_writer_integration.laz";
    remove_if_exists(out);

    auto pts = make_sample_points(128);
    save_points_as(out.string(), pts, /*scale*/0.001);
    basic_file_checks(out);

    LASToolsReader r;
    ASSERT_TRUE(r.open(out.string()));
    ASSERT_TRUE(r.loadAllPoints());
    const auto& back = r.getLoadedPoints();
    ASSERT_EQ(back.size(), pts.size());

    const double tol = 0.002; // scale 0.001 → 2mm
    for (int i : {0, 13, 29, 57, 101, 127}) {
        EXPECT_NEAR(pts[i].x, back[i].x, tol);
        EXPECT_NEAR(pts[i].y, back[i].y, tol);
        EXPECT_NEAR(pts[i].z, back[i].z, tol);
    }
    r.close(); remove_if_exists(out);
}

TEST(WriterIntegrationTest, HeaderAndCountsAreConsistent)
{
    const path out = temp_directory_path() / "ut_writer_header_counts.las";
    remove_if_exists(out);

    auto pts = make_sample_points(50);

    HeaderInfo hdr{};
    hdr.version_major = 1; hdr.version_minor = 2;
    hdr.system_identifier = "UnitTest";
    hdr.generating_software = "GTest";
    hdr.coordinate_transform = { 0.01, 0.02, 0.05, 1000.0, 2000.0, -3000.0 };
    hdr.point_data_record_length = 34;

    LASToolsWriter w;
    ASSERT_TRUE(w.open(out.string(), /*pf=*/3, hdr));
    ASSERT_TRUE(w.writeAll(pts));
    w.close();

    basic_file_checks(out);

    LASToolsReader r;
    ASSERT_TRUE(r.open(out.string()));
    EXPECT_EQ(r.getPointCount(), pts.size());
    ASSERT_TRUE(r.loadAllPoints());

    auto hi = r.getHeaderInfo(); ASSERT_TRUE(hi.has_value());
    EXPECT_DOUBLE_EQ(hi->coordinate_transform.x_scale, 0.01);
    EXPECT_DOUBLE_EQ(hi->coordinate_transform.y_scale, 0.02);
    EXPECT_DOUBLE_EQ(hi->coordinate_transform.z_scale, 0.05);
    EXPECT_DOUBLE_EQ(hi->coordinate_transform.x_offset, 1000.0);
    EXPECT_DOUBLE_EQ(hi->coordinate_transform.y_offset, 2000.0);
    EXPECT_DOUBLE_EQ(hi->coordinate_transform.z_offset, -3000.0);

    r.close(); remove_if_exists(out);
}