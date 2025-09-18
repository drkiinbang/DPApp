#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <string_view>
#include <iomanip>
#include <algorithm>
#include <memory>
#include <optional>
#include <unordered_map>
#include <array>
#include <exception>
#include <stdexcept>
#include <cstdint>
#include <sstream>
#include <cmath>

// LAStools headers
#include "lasreader.hpp"
#include "laswriter.hpp"
#include "lasdefinitions.hpp"

#include "LaslibReader.hpp"

namespace las {

    class LASToolsWriter {
    private:
        LASwriter* writer_{ nullptr };
        LASwriteOpener write_opener_;
        LASheader header_{};
        bool is_open_{ false };
        std::size_t written_{ 0 };

        static void applyScaleOffset(LASheader& h, const HeaderInfo::ScaleOffset& so) {
            if (so.x_scale != 0.0) h.x_scale_factor = so.x_scale;
            if (so.y_scale != 0.0) h.y_scale_factor = so.y_scale;
            if (so.z_scale != 0.0) h.z_scale_factor = so.z_scale;
            h.x_offset = so.x_offset;
            h.y_offset = so.y_offset;
            h.z_offset = so.z_offset;
        }

        static void applyBox(LASheader& h, const HeaderInfo::BoundingBox& box) {
            h.min_x = box.min_x; h.max_x = box.max_x;
            h.min_y = box.min_y; h.max_y = box.max_y;
            h.min_z = box.min_z; h.max_z = box.max_z;
        }

        static int defaultRecordLengthForFormat(int pf) {
            switch (pf) {
            case 0: return 20;
            case 1: return 28;
            case 2: return 26;
            case 3: return 34;
            case 6: return 30;
            case 7: return 36;
            case 8: return 38;
            default: return 34;
            }
        }

        static void setPointFromData(LASpoint& p, const PointData& d, int point_format) {
            p.set_x(d.x); p.set_y(d.y); p.set_z(d.z);
            p.set_intensity(d.intensity);
            p.set_return_number(d.return_number);
            p.set_number_of_returns(d.number_of_returns);
            p.set_classification(d.classification);
            p.set_scan_angle(d.scan_angle);
            p.set_user_data(d.user_data);
            p.set_point_source_ID(d.point_source_id);
            if (point_format == 1 || point_format == 3 || point_format == 6 ||
                point_format == 7 || point_format == 8) {
                p.set_gps_time(d.gps_time);
            }
            if (point_format == 2 || point_format == 3 || point_format == 7 || point_format == 8) {
                p.set_R(d.rgb[0]); p.set_G(d.rgb[1]); p.set_B(d.rgb[2]);
            }
        }

    public:
        LASToolsWriter() = default;
        ~LASToolsWriter() { close(); }

        bool isOpen() const noexcept { return is_open_; }
        std::size_t writtenCount() const noexcept { return written_; }
        const LASheader& header() const noexcept { return header_; }

        bool open(std::string_view filename,
            int point_format = 3,
            const std::optional<HeaderInfo>& header_info = std::nullopt)
        {
            if (is_open_) return false;

            header_.clean();
            header_.file_signature[0] = 'L'; header_.file_signature[1] = 'A';
            header_.file_signature[2] = 'S'; header_.file_signature[3] = 'F';
            header_.version_major = header_info ? header_info->version_major : 1;
            header_.version_minor = header_info ? header_info->version_minor : 2;
            header_.point_data_format = static_cast<U8>(point_format);
            header_.point_data_record_length =
                header_info ? header_info->point_data_record_length
                : defaultRecordLengthForFormat(point_format);

            if (header_info) {
                std::snprintf(header_.system_identifier, 32, "%s",
                    header_info->system_identifier.c_str());
                std::snprintf(header_.generating_software, 32, "%s",
                    header_info->generating_software.c_str());
            }
            else {
                std::snprintf(header_.system_identifier, 32, "LasLibWrapper");
                std::snprintf(header_.generating_software, 32, "LASToolsWriter");
            }

            if (header_info) applyScaleOffset(header_, header_info->coordinate_transform);
            if (header_info) applyBox(header_, header_info->bounds);

            // Open file (LAS/LAZ decided by extension and build)
            write_opener_.set_file_name(std::string(filename).c_str());
            writer_ = write_opener_.open(&header_);
            if (!writer_) {
                throw LASException("Failed to open writer for: " + std::string(filename));
            }

            is_open_ = true;
            written_ = 0;
            return true;
        }

        bool writePoint(const PointData& d) {
            if (!is_open_ || !writer_) return false;
            LASpoint p;
            p.init(&header_, header_.point_data_format, header_.point_data_record_length);
            setPointFromData(p, d, header_.point_data_format);
            const BOOL ok = writer_->write_point(&p);
            if (!ok) return false;
            writer_->update_inventory(&p);
            ++written_;
            return true;
        }

        bool writeAll(const std::vector<PointData>& pts) {
            if (!is_open_ || !writer_) return false;
            for (const auto& d : pts) {
                if (!writePoint(d)) return false;
            }
            return true;
        }

        void close() noexcept {
            if (writer_) {
                writer_->update_header(&header_, TRUE);
                delete writer_;
                writer_ = nullptr;
            }
            is_open_ = false;
        }

        static bool save(std::string_view filename,
            const std::vector<PointData>& pts,
            int point_format = 3,
            const std::optional<HeaderInfo>& header_info = std::nullopt)
        {
            LASToolsWriter w;
            if (!w.open(filename, point_format, header_info)) return false;
            if (!w.writeAll(pts)) { w.close(); return false; }
            w.close();
            return true;
        }
    };

} // namespace las