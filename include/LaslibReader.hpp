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

namespace las {

    enum class ExportFormat : std::uint8_t {
        XYZ,
        XYZI,
        XYZIC,
        FULL
    };

    enum class FileType : std::uint8_t {
        LAS,
        LAZ,
        UNKNOWN
    };

    struct PointData {
        double x{}, y{}, z{};
        std::uint16_t intensity{};
        std::uint8_t classification{};
        std::uint8_t return_number{};
        std::uint8_t number_of_returns{};
        std::int8_t scan_angle{};
        std::uint8_t user_data{};
        std::uint16_t point_source_id{};
        double gps_time{};
        std::array<std::uint16_t, 3> rgb{};

        // Utility methods
        [[nodiscard]] bool hasRGB() const noexcept {
            return rgb[0] != 0 || rgb[1] != 0 || rgb[2] != 0;
        }

        [[nodiscard]] bool hasGPS() const noexcept {
            return gps_time != 0.0;
        }

        [[nodiscard]] double distance2D(const PointData& other) const noexcept {
            const double dx = x - other.x;
            const double dy = y - other.y;
            return std::sqrt(dx * dx + dy * dy);
        }

        [[nodiscard]] double distance3D(const PointData& other) const noexcept {
            const double dx = x - other.x;
            const double dy = y - other.y;
            const double dz = z - other.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        // Comparison operators for sorting/searching
        bool operator==(const PointData& other) const noexcept {
            return x == other.x && y == other.y && z == other.z;
        }

        bool operator<(const PointData& other) const noexcept {
            if (x != other.x) return x < other.x;
            if (y != other.y) return y < other.y;
            return z < other.z;
        }
    };

    struct HeaderInfo {
        std::string file_signature;
        std::uint8_t version_major{};
        std::uint8_t version_minor{};
        std::string system_identifier;
        std::string generating_software;
        std::uint16_t file_creation_day{};
        std::uint16_t file_creation_year{};
        std::uint32_t number_of_point_records{};
        std::uint8_t point_data_format{};
        std::uint16_t point_data_record_length{};

        struct BoundingBox {
            double min_x{}, max_x{};
            double min_y{}, max_y{};
            double min_z{}, max_z{};

            [[nodiscard]] bool contains(const PointData& point) const noexcept {
                return point.x >= min_x && point.x <= max_x &&
                    point.y >= min_y && point.y <= max_y &&
                    point.z >= min_z && point.z <= max_z;
            }

            [[nodiscard]] double width() const noexcept { return max_x - min_x; }
            [[nodiscard]] double height() const noexcept { return max_y - min_y; }
            [[nodiscard]] double depth() const noexcept { return max_z - min_z; }
        } bounds;

        struct ScaleOffset {
            double x_scale{ 1.0 }, y_scale{ 1.0 }, z_scale{ 1.0 };
            double x_offset{}, y_offset{}, z_offset{};
        } coordinate_transform;

        std::array<std::uint32_t, 5> points_by_return{};
    };

    // Custom exception for LAS operations
    class LASException : public std::runtime_error {
    public:
        explicit LASException(const std::string& message)
            : std::runtime_error("LAS Error: " + message) {}
    };

    class LASToolsReader {
    private:
        LASreader* reader_{ nullptr };
        bool is_open_{ false };
        std::vector<PointData> loaded_points_;
        std::optional<HeaderInfo> header_info_;

        static constexpr std::size_t PROGRESS_INTERVAL = 50000;
        static constexpr int MAX_CLASSIFICATION = 256;

        // Static classification names using function to avoid initialization order issues
        [[nodiscard]] static const std::unordered_map<int, std::string_view>& getClassificationNames() noexcept {
            static const std::unordered_map<int, std::string_view> classification_names = {
                {0, "Created, never classified"},
                {1, "Unclassified"},
                {2, "Ground"},
                {3, "Low Vegetation"},
                {4, "Medium Vegetation"},
                {5, "High Vegetation"},
                {6, "Building"},
                {7, "Low Point (noise)"},
                {8, "Model Key-point"},
                {9, "Water"},
                {10, "Reserved"},
                {11, "Reserved"},
                {12, "Reserved"}
            };
            return classification_names;
        }

        [[nodiscard]] static FileType detectFileType(std::string_view filename) noexcept {
            if (const auto dot_pos = filename.find_last_of('.'); dot_pos != std::string_view::npos) {
                const auto extension = filename.substr(dot_pos);
                if (extension == ".las") return FileType::LAS;
                if (extension == ".laz") return FileType::LAZ;
            }
            return FileType::UNKNOWN;
        }

        void extractHeaderInfo() {
            if (!reader_) return;

            const LASheader& header = reader_->header;
            HeaderInfo info{};

            info.file_signature = std::string(header.file_signature, 4);
            info.version_major = header.version_major;
            info.version_minor = header.version_minor;
            info.system_identifier = header.system_identifier;
            info.generating_software = header.generating_software;
            info.file_creation_day = header.file_creation_day;
            info.file_creation_year = header.file_creation_year;
            info.number_of_point_records = header.number_of_point_records;
            info.point_data_format = header.point_data_format;
            info.point_data_record_length = header.point_data_record_length;

            info.bounds.min_x = header.min_x;
            info.bounds.max_x = header.max_x;
            info.bounds.min_y = header.min_y;
            info.bounds.max_y = header.max_y;
            info.bounds.min_z = header.min_z;
            info.bounds.max_z = header.max_z;

            info.coordinate_transform.x_scale = header.x_scale_factor;
            info.coordinate_transform.y_scale = header.y_scale_factor;
            info.coordinate_transform.z_scale = header.z_scale_factor;
            info.coordinate_transform.x_offset = header.x_offset;
            info.coordinate_transform.y_offset = header.y_offset;
            info.coordinate_transform.z_offset = header.z_offset;

            std::copy(std::begin(header.number_of_points_by_return),
                std::end(header.number_of_points_by_return),
                info.points_by_return.begin());

            header_info_ = std::move(info);
        }

        [[nodiscard]] PointData extractPointData(const LASpoint& point) const noexcept {
            PointData data{};

            data.x = point.get_x();
            data.y = point.get_y();
            data.z = point.get_z();
            data.intensity = point.get_intensity();
            data.classification = point.get_classification();
            data.return_number = point.get_return_number();
            data.number_of_returns = point.get_number_of_returns();
            data.scan_angle = static_cast<std::int8_t>(point.get_scan_angle());
            data.user_data = point.get_user_data();
            data.point_source_id = point.get_point_source_ID();

            if (point.have_gps_time) {
                data.gps_time = point.get_gps_time();
            }

            if (point.have_rgb) {
                data.rgb[0] = point.rgb[0];
                data.rgb[1] = point.rgb[1];
                data.rgb[2] = point.rgb[2];
            }

            return data;
        }

        void displayProgress(std::size_t current, std::size_t total, std::string_view operation) const {
            if (current % PROGRESS_INTERVAL == 0 || current == total) {
                const auto progress = static_cast<double>(current) / static_cast<double>(total) * 100.0;
                std::cout << '\r' << operation << " progress: "
                    << std::fixed << std::setprecision(1) << progress
                    << "% (" << current << "/" << total << ")" << std::flush;
            }
        }

        // Helper function to replace std::exchange
        template<typename T, typename U>
        T exchange(T& obj, U&& new_value) noexcept {
            T old_value = std::move(obj);
            obj = std::forward<U>(new_value);
            return old_value;
        }

        std::size_t getAvailableMemoryMB() const {
            MEMORYSTATUSEX memInfo;
            memInfo.dwLength = sizeof(MEMORYSTATUSEX);
            GlobalMemoryStatusEx(&memInfo);

            return static_cast<std::size_t>(memInfo.ullAvailPhys / (1024 * 1024));
        }

        std::size_t calculateOptimalReserveSize(std::size_t total_points,
            std::size_t estimated_filtered) const {
            const std::size_t max_memory_mb = getAvailableMemoryMB();
            const std::size_t point_size = sizeof(PointData);
            const std::size_t max_points = (max_memory_mb * 1024 * 1024) / point_size / 2; /// 50%

            return (std::min)({
                estimated_filtered,
                max_points,
                total_points
                });
        }

    public:
        LASToolsReader() = default;

        ~LASToolsReader() {
            close();
        }

        // Non-copyable but movable
        LASToolsReader(const LASToolsReader&) = delete;
        LASToolsReader& operator=(const LASToolsReader&) = delete;

        LASToolsReader(LASToolsReader&& other) noexcept
            : reader_(exchange(other.reader_, nullptr))
            , is_open_(exchange(other.is_open_, false))
            , loaded_points_(std::move(other.loaded_points_))
            , header_info_(std::move(other.header_info_)) {
        }

        LASToolsReader& operator=(LASToolsReader&& other) noexcept {
            if (this != &other) {
                close();
                reader_ = exchange(other.reader_, nullptr);
                is_open_ = exchange(other.is_open_, false);
                loaded_points_ = std::move(other.loaded_points_);
                header_info_ = std::move(other.header_info_);
            }
            return *this;
        }

        [[nodiscard]] bool open(const std::string& filename) {
            close();

            try {
                LASreadOpener opener;
                opener.set_file_name(filename.c_str());

                reader_ = opener.open();
                if (!reader_) {
                    throw LASException("Failed to open file: " + filename);
                }

                is_open_ = true;
                extractHeaderInfo();

                const auto file_type = detectFileType(filename);
                std::string file_type_str;
                switch (file_type) {
                case FileType::LAZ: file_type_str = "LAZ (compressed)"; break;
                case FileType::LAS: file_type_str = "LAS (uncompressed)"; break;
                default: file_type_str = "Unknown"; break;
                }

                std::cout << "File opened successfully: " << filename << '\n';
                std::cout << "File type: " << file_type_str << '\n';
                return true;

            }
            catch (const std::exception& e) {
                std::cerr << "Exception occurred: " << e.what() << '\n';
                return false;
            }
        }

        void close() noexcept {
            if (reader_) {
                reader_->close();
                delete reader_;
                reader_ = nullptr;
            }
            is_open_ = false;
            loaded_points_.clear();
            loaded_points_.shrink_to_fit();
            header_info_.reset();
        }

        std::string printHeader() const {
            if (!reader_ || !header_info_) {
                std::cerr << "File is not open\n";
                return std::string("");
            }

            const auto& info = *header_info_;
            std::stringstream printoutmsg;

            printoutmsg << "\n========== File Header Information ==========\n";
            printoutmsg << "File signature: " << info.file_signature << '\n';
            printoutmsg << "LAS version: " << static_cast<int>(info.version_major)
                << '.' << static_cast<int>(info.version_minor) << '\n';
            printoutmsg << "System identifier: " << info.system_identifier << '\n';
            printoutmsg << "Generating software: " << info.generating_software << '\n';
            printoutmsg << "Creation date: " << info.file_creation_day
                << "/" << info.file_creation_year << '\n';

            printoutmsg << "\nPoint Information:\n";
            printoutmsg << "Point count: " << info.number_of_point_records << '\n';
            printoutmsg << "Point data format: " << static_cast<int>(info.point_data_format) << '\n';
            printoutmsg << "Point record length: " << info.point_data_record_length << " bytes\n";

            printoutmsg << "\nBounding Box:\n";
            printoutmsg << std::fixed << std::setprecision(2);
            printoutmsg << "X: [" << info.bounds.min_x << ", " << info.bounds.max_x
                << "] (width: " << info.bounds.width() << ")\n";
            printoutmsg << "Y: [" << info.bounds.min_y << ", " << info.bounds.max_y
                << "] (height: " << info.bounds.height() << ")\n";
            printoutmsg << "Z: [" << info.bounds.min_z << ", " << info.bounds.max_z
                << "] (depth: " << info.bounds.depth() << ")\n";

            printoutmsg << "\nCoordinate Transform:\n";
            printoutmsg << std::scientific << std::setprecision(6);
            printoutmsg << "X scale: " << info.coordinate_transform.x_scale
                << ", offset: " << info.coordinate_transform.x_offset << '\n';
            printoutmsg << "Y scale: " << info.coordinate_transform.y_scale
                << ", offset: " << info.coordinate_transform.y_offset << '\n';
            printoutmsg << "Z scale: " << info.coordinate_transform.z_scale
                << ", offset: " << info.coordinate_transform.z_offset << '\n';

            printoutmsg << "\nPoints by return:\n";
            for (std::size_t i = 0; i < info.points_by_return.size(); ++i) {
                if (info.points_by_return[i] > 0) {
                    printoutmsg << "Return " << (i + 1) << ": " << info.points_by_return[i] << '\n';
                }
            }

            std::cout << printoutmsg.str();

            return printoutmsg.str();
        }

        [[nodiscard]] bool loadAllPoints() {
            if (!reader_) {
                std::cerr << "File is not open\n";
                return false;
            }

            loaded_points_.clear();
            const auto total_points = static_cast<std::size_t>(reader_->npoints);
            loaded_points_.reserve(total_points);

            std::cout << "\n========== Loading all points... ==========\n";

            reader_->seek(0);
            std::size_t count = 0;

            while (reader_->read_point()) {
                loaded_points_.emplace_back(extractPointData(reader_->point));
                displayProgress(++count, total_points, "Loading");
            }

            std::cout << "\nLoaded " << loaded_points_.size() << " points.\n";
            return true;
        }

        [[nodiscard]] bool loadPointRange(std::size_t start_idx, std::size_t end_idx) {
            if (!reader_) {
                std::cerr << "File is not open\n";
                return false;
            }

            if (start_idx >= end_idx) {
                std::cerr << "Invalid range: start_idx must be less than end_idx\n";
                return false;
            }

            const auto total_points = static_cast<std::size_t>(reader_->npoints);
            if (end_idx > total_points) {
                std::cerr << "end_idx exceeds total points\n";
                return false;
            }

            loaded_points_.clear();
            loaded_points_.reserve(end_idx - start_idx);

            I64 seek_index = static_cast<I64>(start_idx);

            if (!reader_->seek(seek_index)) {
                std::cerr << "reader_->seek(" << seek_index << "\n";
                return false;
            }

            std::size_t points_to_read = end_idx - start_idx;
            std::size_t count = 0;

            while (reader_->read_point() && count < points_to_read) {
                loaded_points_.emplace_back(extractPointData(reader_->point));
                ++count;

                if ((start_idx + count) % PROGRESS_INTERVAL == 0 || count == points_to_read) {
                    displayProgress(start_idx + count, end_idx, "Loading range (random access)");
                }
            }

            std::cout << "\nLoaded " << loaded_points_.size()
                << " points (range: " << start_idx << "-" << end_idx << ")\n";
            return true;
        }

        /// Load points with filter using template and SFINAE
        template<typename Filter, typename = std::enable_if_t<
            std::is_invocable_r_v<bool, Filter, const PointData&>>>
            [[nodiscard]] bool loadFilteredPoints(Filter&& filter) {
            if (!reader_) {
                std::cerr << "File is not open\n";
                return false;
            }

            const auto total_points = static_cast<std::size_t>(reader_->npoints);
            const auto reserve_size = calculateOptimalReserveSize(total_points, total_points / 10);

            loaded_points_.clear();
            loaded_points_.reserve(reserve_size);

            reader_->seek(0);
            std::size_t count = 0, total_count = 0;
            const auto total_points = static_cast<std::size_t>(reader_->npoints);

            while (reader_->read_point()) {
                const auto point_data = extractPointData(reader_->point);
                if (filter(point_data)) {
                    loaded_points_.emplace_back(point_data);
                    ++count;
                }
                displayProgress(++total_count, total_points, "Filtering");
            }

            loaded_points_.shrink_to_fit();
            std::cout << "\nLoaded " << count << " filtered points from "
                << total_points << " total.\n";
            return true;
        }

        [[nodiscard]] bool exportToCSV(std::string_view filename,
            ExportFormat format = ExportFormat::FULL) const {
            if (loaded_points_.empty()) {
                std::cerr << "Points not loaded. Call loadAllPoints() first.\n";
                return false;
            }

            std::ofstream file(filename.data());
            if (!file.is_open()) {
                std::cerr << "Cannot open output file: " << filename << '\n';
                return false;
            }

            std::cout << "\n========== Exporting to CSV... ==========\n";

            // Write header
            switch (format) {
            case ExportFormat::XYZ:
                file << "X,Y,Z\n";
                break;
            case ExportFormat::XYZI:
                file << "X,Y,Z,Intensity\n";
                break;
            case ExportFormat::XYZIC:
                file << "X,Y,Z,Intensity,Classification\n";
                break;
            case ExportFormat::FULL:
                file << "X,Y,Z,Intensity,Classification,ReturnNumber,NumberOfReturns,"
                    "ScanAngle,UserData,PointSourceID,GPSTime,R,G,B\n";
                break;
            }

            // Write data with batch processing for better performance
            constexpr std::size_t BATCH_SIZE = 10000;
            std::ostringstream batch_buffer;

            for (std::size_t i = 0; i < loaded_points_.size(); ++i) {
                const auto& point = loaded_points_[i];

                batch_buffer << std::fixed << std::setprecision(3);

                switch (format) {
                case ExportFormat::XYZ:
                    batch_buffer << point.x << ',' << point.y << ',' << point.z << '\n';
                    break;
                case ExportFormat::XYZI:
                    batch_buffer << point.x << ',' << point.y << ',' << point.z << ','
                        << point.intensity << '\n';
                    break;
                case ExportFormat::XYZIC:
                    batch_buffer << point.x << ',' << point.y << ',' << point.z << ','
                        << point.intensity << ',' << static_cast<int>(point.classification) << '\n';
                    break;
                case ExportFormat::FULL:
                    batch_buffer << point.x << ',' << point.y << ',' << point.z << ','
                        << point.intensity << ',' << static_cast<int>(point.classification) << ','
                        << static_cast<int>(point.return_number) << ',' << static_cast<int>(point.number_of_returns) << ','
                        << static_cast<int>(point.scan_angle) << ',' << static_cast<int>(point.user_data) << ','
                        << point.point_source_id << ',' << std::setprecision(6) << point.gps_time << ','
                        << point.rgb[0] << ',' << point.rgb[1] << ',' << point.rgb[2] << '\n';
                    break;
                }

                if (i % BATCH_SIZE == 0 || i == loaded_points_.size() - 1) {
                    file << batch_buffer.str();
                    batch_buffer.str("");
                    batch_buffer.clear();
                }

                if (i % PROGRESS_INTERVAL == 0) {
                    displayProgress(i, loaded_points_.size(), "Exporting");
                }
            }

            std::cout << '\n' << loaded_points_.size() << " points saved to " << filename << ".\n";
            return true;
        }

        void generateClassificationStats() const {
            if (loaded_points_.empty()) {
                std::cerr << "Points not loaded.\n";
                return;
            }

            std::cout << "\n========== Classification Statistics ==========\n";

            std::array<std::size_t, MAX_CLASSIFICATION> counts{};
            for (const auto& point : loaded_points_) {
                ++counts[point.classification];
            }

            const auto& classification_names = getClassificationNames();
            const auto total_points = loaded_points_.size();

            std::cout << "Total points: " << total_points << "\n\nClassification distribution:\n";

            for (std::size_t i = 0; i < counts.size(); ++i) {
                if (counts[i] == 0) continue;

                const auto it = classification_names.find(static_cast<int>(i));
                const auto class_name = (it != classification_names.end()) ?
                    it->second : "User Defined";

                const auto percentage = static_cast<double>(counts[i]) /
                    static_cast<double>(total_points) * 100.0;

                std::cout << "Class " << i << " (" << class_name << "): "
                    << counts[i] << " (" << std::fixed << std::setprecision(1)
                    << percentage << "%)\n";
            }
        }

        // Getters
        [[nodiscard]] std::size_t getPointCount() const noexcept {
            return reader_ ? static_cast<std::size_t>(reader_->npoints) : 0;
        }

        [[nodiscard]] std::size_t getLoadedPointCount() const noexcept {
            return loaded_points_.size();
        }

        [[nodiscard]] bool isFileOpen() const noexcept {
            return is_open_ && reader_;
        }

        [[nodiscard]] const std::optional<HeaderInfo>& getHeaderInfo() const noexcept {
            return header_info_;
        }

        [[nodiscard]] const std::vector<PointData>& getLoadedPoints() const noexcept {
            return loaded_points_;
        }

        // Enhanced utility methods
        [[nodiscard]] std::vector<PointData> getPointsByClassification(std::uint8_t classification) const {
            std::vector<PointData> result;
            std::copy_if(loaded_points_.begin(), loaded_points_.end(),
                std::back_inserter(result),
                [classification](const auto& point) {
                    return point.classification == classification;
                });
            return result;
        }

        [[nodiscard]] std::vector<PointData> getPointsInBounds(double min_x, double min_y,
            double max_x, double max_y) const {
            std::vector<PointData> result;
            std::copy_if(loaded_points_.begin(), loaded_points_.end(),
                std::back_inserter(result),
                [=](const auto& point) {
                    return point.x >= min_x && point.x <= max_x &&
                        point.y >= min_y && point.y <= max_y;
                });
            return result;
        }

        // Backward compatibility
        [[nodiscard]] static ExportFormat parseFormat(std::string_view format_str) noexcept {
            if (format_str == "xyz") return ExportFormat::XYZ;
            if (format_str == "xyzi") return ExportFormat::XYZI;
            if (format_str == "xyzic") return ExportFormat::XYZIC;
            return ExportFormat::FULL;
        }
    };

} // namespace las