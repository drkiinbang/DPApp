#pragma once

// Windows의 min/max 매크로가 std::min/max와 충돌하지 않도록 방지
#ifdef _WIN32
#define NOMINMAX
#endif

#define MAX_NUM_PTS_CHUNK 100000

// TaskManagerTypes.h에서 기본 타입 정의를 가져옴
#include "TaskManagerTypes.h"

#include <queue>
#include <condition_variable>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <thread>
#include <exception>

#include "NetworkManager.h"
#include "ReadPointFile.h"
#include "bim/gltf_mesh.h"
#include "bim/mbr.h"
#include "bim/BimInfo.h"
#include "DefinePointType.hpp"
#include "LaslibReader.hpp"
#include "Pc2Reader.hpp"
#include "bimtree/nanoflann.hpp"
#include "bimtree/IcpCore.hpp"
#include "bimtree/PseudoPointGenerator.hpp"
#include "../../BimImport/BimImport.h"
#include "bim/MeshChunk.h"

namespace fs = std::filesystem;

const std::size_t CHUNK_POINTS = 1000000;

const double _PI_ = acos(-1.0);

namespace DPApp {

    // =====================================================
    // 아래 타입들은 TaskManagerTypes.h에 정의되어 있음:
    // - enum class TaskStatus
    // - enum class TaskPriority  
    // - struct TaskInfo
    // - struct SlaveWorkerInfo
    // - class TaskManager
    // - class ProcessingStats
    // =====================================================

    /**
     * @brief 포인트클라우드 로딩 유틸리티
     */
    namespace PointCloudLoader {
        std::vector<std::shared_ptr<PointCloudChunk>> loadPointCloudFileInChunks(
            const std::string& file_path,
            uint32_t max_points_per_chunk = MAX_NUM_PTS_CHUNK);

        std::vector<std::shared_ptr<PointCloudChunk>> loadXYZFileStreaming(
            const std::string& file_path, uint32_t max_points_per_chunk);

        std::vector<std::shared_ptr<PointCloudChunk>> loadLasFile(
            const std::string& file_path, uint32_t max_points_per_chunk);
    }

    /**
     * @brief 개별 워커(Slave 노드)를 위한 태스크 처리기 클래스
     * 2가지 태스크 타입 고정: CONVERT_PTS와 BIM_DISTANCE_CALCULATION
     */
    class TaskProcessor {
    public:
        TaskProcessor() {
            std::cout << "TaskProcessor initialized" << std::endl;
        }

        ~TaskProcessor() {
            std::cout << "TaskProcessor destroyed" << std::endl;
        }

        // 태스크 처리 - 지원하는 두 태스크 타입을 내부에서 모두 처리
        ProcessingResult processTask(const ProcessingTask& task, const PointCloudChunk& chunk);

        /// BimPcChunk 처리
        BimPcResult processTask(const ProcessingTask& task, const BimPcChunk& chunk);

        // 지원하는 태스크 타입 조회
        std::vector<TaskType> getSupportedTaskTypes() const;
        bool supportsTaskType(const TaskType task_type) const;

        // 통계
        uint32_t getProcessedTaskCount() const { return stats_.getProcessedCount(); }
        uint32_t getFailedTaskCount() const { return stats_.getFailedCount(); }
        double getAverageProcessingTime() const { return stats_.getAverageProcessingTime(); }

    private:
        ProcessingStats stats_;  /// 안전을 위해 atomic 변수 대신 사용
    };

    /**
     * @brief 단순 포인트클라우드 처리 함수들
     */
    namespace PointCloudProcessors {
        /// pts 포맷 변환
        ProcessingResult convertPts(const ProcessingTask& task, const PointCloudChunk& chunk);

        /// 단순 거리 계산 (BIM 기능을 단순화한 버전)
        ProcessingResult calculateDistance(const ProcessingTask& task, const PointCloudChunk& chunk);
    }

    namespace BimPcProcessors {
        BimPcResult processBimPc(const ProcessingTask& task, const BimPcChunk& chunk);
    }

    ///
    /// 구현부
    ///

    // 포인트클라우드 로딩 유틸리티 구현
    namespace PointCloudLoader {

        std::vector<std::shared_ptr<PointCloudChunk>> loadPointCloudFileInChunks(
            const std::string& file_path, uint32_t max_points_per_chunk) {

            std::vector<std::shared_ptr<PointCloudChunk>> chunks;

            try {
                std::cout << "Loading point cloud file: " << file_path << std::endl;

                /// 파일 확장자 확인
                std::string extension = std::filesystem::path(file_path).extension().string();
                std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

                if (extension == ".xyz" || extension == ".pts") {
                    chunks = loadXYZFileStreaming(file_path, max_points_per_chunk);
                }
                else if (extension == ".las" || extension == ".laz") {
                    chunks = loadLasFile(file_path, max_points_per_chunk);
                }
                else {
                    std::cerr << "Unsupported file format: " << extension << std::endl;
                    return chunks;
                }

                std::cout << "Created " << chunks.size() << " chunks" << std::endl;
            }
            catch (const std::exception& e) {
                std::cerr << "Error loading file: " << e.what() << std::endl;
            }

            return chunks;
        }

        std::vector<std::shared_ptr<PointCloudChunk>> loadXYZFileStreaming(const std::string& file_path, const uint32_t max_points_per_chunk) {

            std::vector<std::shared_ptr<PointCloudChunk>> chunks;

            try {
                std::ifstream file(file_path);
                if (!file.is_open()) {
                    std::cerr << "Cannot open XYZ file: " << file_path << std::endl;
                    return chunks;
                }

                auto current_chunk = std::make_shared<PointCloudChunk>();
                current_chunk->chunk_id = 0;
                current_chunk->points.reserve(max_points_per_chunk);

                std::string line;
                size_t line_count = 0;
                size_t points_in_current_chunk = 0;
                size_t total_points_processed = 0;
                uint32_t chunk_counter = 0;

                while (std::getline(file, line)) {
                    line_count++;

                    // 빈 줄이나 주석 줄은 건너뜀
                    if (line.empty() || line[0] == '#') continue;

                    std::istringstream iss(line);
                    Point3D point;

                    if (iss >> point[0] >> point[1] >> point[2]) {
                        // 추가 필드가 있으면 읽지만 사용하지 않음 (파싱 오류 방지)
                        // float temp_intensity = 0;
                        // int temp_r = 128, temp_g = 128, temp_b = 128;
                        // iss >> temp_intensity >> temp_r >> temp_g >> temp_b;

                        current_chunk->points.push_back(point);
                        points_in_current_chunk++;
                        total_points_processed++;

                        // 청크 크기 제한 확인
                        if (points_in_current_chunk >= max_points_per_chunk) {
                            // 청크 완료
                            chunks.push_back(current_chunk);

                            // 새 청크 시작
                            chunk_counter++;
                            current_chunk = std::make_shared<PointCloudChunk>();
                            current_chunk->chunk_id = chunk_counter;
                            current_chunk->points.reserve(max_points_per_chunk);
                            points_in_current_chunk = 0;
                        }
                    }

                    // 진행 상황 (10만 줄마다)
                    if (line_count % 100000 == 0) {
                        std::cout << "Processing line: " << line_count << std::endl;
                    }
                }

                // 마지막 청크
                if (points_in_current_chunk > 0) {
                    chunks.push_back(current_chunk);
                }

                file.close();

                std::cout << "Streaming load completed: " << chunks.size() << " chunks, "
                    << total_points_processed << " total points" << std::endl;

            }
            catch (const std::exception& e) {
                std::cerr << "Error in XYZ streaming: " << e.what() << std::endl;
            }

            return chunks;
        }

        std::vector<std::shared_ptr<PointCloudChunk>> loadLasFile(const std::string& file_path, const uint32_t max_points_per_chunk)
        {
            std::vector<std::shared_ptr<PointCloudChunk>> chunks;

            try {
                las::LASToolsReader lasReader;

                if (!lasReader.open(file_path)) {
                    std::cerr << "Failed to open input file: " << file_path << std::endl;
                    return chunks;
                }

                size_t total_num_points = lasReader.getPointCount();
                chunks.reserve(size_t(total_num_points / max_points_per_chunk) + 1);
                size_t total_points_loaded = 0;
                uint32_t chunk_counter = 0;

                for (size_t start = 0; start < total_num_points; start += max_points_per_chunk) {
                    std::size_t end = (std::min)(start + max_points_per_chunk, total_num_points);
                    if (!lasReader.loadPointRange(start, end)) {
                        std::cerr << "Failed in loadPointRange(" << start << ", " << end << ")\n";
                        break;
                    }

                    auto current_chunk = std::make_shared<PointCloudChunk>();
                    current_chunk->chunk_id = chunk_counter;
                    current_chunk->points.reserve(lasReader.getLoadedPointCount());

                    for (const auto& p : lasReader.getLoadedPoints()) {
                        current_chunk->points.emplace_back(p.x, p.y, p.z);
                    }

                    total_points_loaded += current_chunk->points.size();
                    chunks.push_back(current_chunk);
                    chunk_counter++;

                    // 진행 상황 보고 (청크마다)
                    if (chunk_counter % 10 == 0) {
                        std::cout << "Loaded chunks: " << chunk_counter
                            << ", points: " << total_points_loaded << std::endl;
                    }
                }

                lasReader.close();

                std::cout << "Streaming load completed: " << chunks.size() << " chunks, "
                    << total_points_loaded << " total points" << std::endl;
            }
            catch (const std::exception& e) {
                std::cerr << "Error in Las streaming: " << e.what() << std::endl;
            }

            return chunks;
        }

        bool loadLasFile(const std::string& file_path, std::vector<Point3D>& pc)
        {
            try {
                las::LASToolsReader lasReader;

                if (!lasReader.open(file_path)) {
                    std::cerr << "Failed to open input file: " << file_path << std::endl;
                    return false;
                }

                size_t total_num_points = lasReader.getPointCount();

                if (!lasReader.loadAllPoints()) {
                    std::cerr << "Failed in loadAllPoints\n";
                    return false;
                }

                size_t gap = static_cast<size_t>(total_num_points * 0.1);

                uint32_t chunk_counter = 0;
                for (const auto& p : lasReader.getLoadedPoints()) {
                    pc.emplace_back(p.x, p.y, p.z);
                    chunk_counter++;

                    // 진행 상황 보고 (청크마다)
                    if (chunk_counter % gap == 0) {
                        std::cout << chunk_counter << " of " << total_num_points << " are loaded.,\n";
                    }
                }

                std::cout << pc.size() << " of " << total_num_points << " are loaded.,\n";

                lasReader.close();

                std::cout << "Streaming load completed" << std::endl;
            }
            catch (const std::exception& e) {
                std::cerr << "Error in Las streaming: " << e.what() << std::endl;
            }

            return true;
        }

    } // namespace PointCloudLoader

    ProcessingResult TaskProcessor::processTask(const ProcessingTask& task, const PointCloudChunk& chunk) {
        auto start_time = std::chrono::steady_clock::now();

        ProcessingResult result;
        result.task_id = task.task_id;
        result.chunk_id = task.chunk_id;

        try {
            /// 태스크 타입에 대해 직접 분기 - 동적 등록이 필요 없음
            switch (task.task_type) {
            case TaskType::CONVERT_PTS:
                result = PointCloudProcessors::convertPts(task, chunk);
                break;

            case TaskType::BIM_DISTANCE_CALCULATION:
                result = PointCloudProcessors::calculateDistance(task, chunk);
                break;

            default:
                result.success = false;
                result.error_message = "Unsupported task type: " + std::string(taskStr(task.task_type));
                stats_.recordFailure();
                return result;
            }
        }
        catch (const std::exception& e) {
            result.success = false;
            result.error_message = "Processing error: " + std::string(e.what());
            stats_.recordFailure();
            return result;
        }

        /// 처리 시간 통계 갱신
        auto end_time = std::chrono::steady_clock::now();
        double processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time).count() / 1000.0;

        if (result.success) {
            stats_.recordSuccess(processing_time);
        }
        else {
            /// 위의 catch 블록이나 default 분기에서 처리되지 않은 논리적 실패
            /// 케이스를 처리한다. 이론적으로는 recordFailure()가 중복 호출되지
            /// 않도록 가드가 필요할 수도 있지만, 현재 제어 흐름 상 이미
            /// recordFailure()를 호출한 모든 경로는 여기 도달하기 전에 조기
            /// 반환한다. 따라서 이 블록에 도달했다는 것은 아직 실패가 기록되지
            /// 않았다는 뜻이므로 여기서 처리해도 안전하다
            if (result.error_message.empty()) {
                /// 오류 메시지가 비어 있다는 것은 위에서 잡히지 않은 실패를 의미함
                stats_.recordFailure();
            }
        }

        return result;
    }

    /// BimPcChunk processTask
    BimPcResult TaskProcessor::processTask(const ProcessingTask& task, const BimPcChunk& chunk) {
        auto start_time = std::chrono::steady_clock::now();

        BimPcResult result;
        result.task_id = task.task_id;
        result.chunk_id = task.chunk_id;

        try {
            if (task.task_type == TaskType::BIM_PC2_DIST) {
                result = BimPcProcessors::processBimPc(task, chunk);
            }
            else {
                result.success = false;
                result.error_message = "Unsupported task type for BimPcChunk: " +
                    std::string(taskStr(task.task_type));
                stats_.recordFailure();
                return result;
            }
        }
        catch (const std::exception& e) {
            result.success = false;
            result.error_message = "Processing error: " + std::string(e.what());
            stats_.recordFailure();
            return result;
            /// 현재 제어 흐름을 고려하면 여기서 반환하는 것은 안전하다 --
            /// recordFailure()가 이미 호출되었으므로 이후 단계에서 실패가
            /// 중복 기록되는 것을 막아준다.
        }

        auto end_time = std::chrono::steady_clock::now();
        double processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time).count() / 1000.0;

        if (result.success) {
            stats_.recordSuccess(processing_time);
        }
        else {
            /// 위 if문이나 catch에서 잡히지 않은 실패 케이스 처리
            if (result.error_message.empty()) {
                stats_.recordFailure();
            }
        }

        return result;
    }

    std::vector<TaskType> TaskProcessor::getSupportedTaskTypes() const {
        return {
            TaskType::CONVERT_PTS,
            TaskType::BIM_DISTANCE_CALCULATION
        };
    }

    bool TaskProcessor::supportsTaskType(const TaskType task_type) const {
        return task_type == TaskType::CONVERT_PTS ||
            task_type == TaskType::BIM_DISTANCE_CALCULATION;
    }

    namespace util {
        std::string wstring_to_utf8(const std::wstring& wstr) {
            int size_needed = WideCharToMultiByte(CP_UTF8, 0, wstr.data(), (int)wstr.size(), NULL, 0, NULL, NULL);
            std::string utf8str(size_needed, 0);
            WideCharToMultiByte(CP_UTF8, 0, wstr.data(), (int)wstr.size(), &utf8str[0], size_needed, NULL, NULL);
            return utf8str;
        }
    }
    /// PointCloudProcessors 구현
    namespace PointCloudProcessors {

        ProcessingResult convertPts(const ProcessingTask& task, const PointCloudChunk& chunk) {
            ProcessingResult result;
            result.task_id = task.task_id;
            result.chunk_id = task.chunk_id;
            result.success = true;

            try {
                std::cout << "Converting PTS chunk " << chunk.chunk_id
                    << " with " << chunk.points.size() << " points" << std::endl;

                // 단순 PTS 변환 - 필요시 포맷 조정만 하고 점들을 그대로 복사
                result.processed_points = chunk.points;

                // 예시: 매개변수에 좌표 offset이 지정되어 있으면 적용
                if (!task.parameters.empty() && task.parameters.size() >= sizeof(double) * 3) {
                    double offset_x, offset_y, offset_z;
                    std::memcpy(&offset_x, task.parameters.data(), sizeof(double));
                    std::memcpy(&offset_y, task.parameters.data() + sizeof(double), sizeof(double));
                    std::memcpy(&offset_z, task.parameters.data() + sizeof(double) * 2, sizeof(double));

                    for (auto& point : result.processed_points) {
                        point[0] += static_cast<float>(offset_x);
                        point[1] += static_cast<float>(offset_y);
                        point[2] += static_cast<float>(offset_z);
                    }

                    std::cout << "Applied offset: (" << offset_x << ", " << offset_y << ", " << offset_z << ")" << std::endl;
                }

                std::cout << "PTS conversion completed for " << result.processed_points.size() << " points" << std::endl;

            }
            catch (const std::exception& e) {
                result.success = false;
                result.error_message = "PTS conversion error: " + std::string(e.what());
            }

            return result;
        }

        ProcessingResult calculateDistance(const ProcessingTask& task, const PointCloudChunk& chunk) {
            ProcessingResult result;
            result.task_id = task.task_id;
            result.chunk_id = task.chunk_id;
            result.success = true;

            try {
                std::cout << "Calculating distances for chunk " << chunk.chunk_id
                    << " with " << chunk.points.size() << " points" << std::endl;

                // 단순 거리 계산 - 각 점의 원점으로부터의 거리를 계산
                result.processed_points = chunk.points;

                // 원점으로부터의 거리를 계산해 intensity 필드에 저장
                for (auto& point : result.processed_points) {
                    float distance = static_cast<float>(std::sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]));
                }

                std::cout << "Distance calculation completed for " << result.processed_points.size() << " points" << std::endl;

            }
            catch (const std::exception& e) {
                result.success = false;
                result.error_message = "Distance calculation error: " + std::string(e.what());
            }

            return result;
        }

    } // namespace PointCloudProcessors

    namespace Pt2MeshProcessors {
        bool convert_gltf2nodes2(const std::string& dataset_name,
            const std::string& bim_folder,
            const std::string& nodes2_folder,
            std::vector<BimMeshInfo>& bimData,
            const bool is_offset_applied,
            const float* offset) {
            std::filesystem::path input_path(bim_folder); // glb 파일이 저장된 폴더 경로
            std::filesystem::path output_path(nodes2_folder); // .nodes2 파일을 저장할 폴더 경로

            if (std::filesystem::exists(input_path) == false) {
                std::cout << "input_folder '" << bim_folder << "' is not exist" << std::endl;
                return false;
            }

            size_t total_gltf_files = std::distance(fs::directory_iterator(input_path), fs::directory_iterator());
            int processed_gltf_files = 0;

            std::string output_file_name = dataset_name + ".nodes2"; // binary file name
            auto full_path = output_path / output_file_name;
            output_file_name = full_path.string();

            float global_bbox_min[3] = { (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::max)() }; // 최대값으로 초기화
            float global_bbox_max[3] = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() }; // 최소값으로 초기화

            // .nodes2 파일로 geometry 정보 출력하기 위한 outputfilestream 생성
            std::ofstream filestream(output_file_name, std::ios::out | std::ios::binary);
            if (!filestream.good()) {
                std::cerr << "file open failure : " << output_file_name << std::endl;
                return false;
            }

            auto numEntries = std::distance(fs::directory_iterator(input_path), fs::directory_iterator{});
            bimData.clear();
            bimData.reserve(numEntries);

            // 디렉터리 내 GLTF 파일들을 순회
            int idx = 1000; // bim_id의 초기값 - 파일명에 Revit Element ID가 없는 경우 사용됨
            for (const auto& entry : std::filesystem::directory_iterator(input_path))
            {
                std::wcout << entry.path().wstring() << std::endl; // unicode string 
                std::cout << entry.path().string() << std::endl; // utf8 string // 영문 windows에서는 주석처리 해야 함
                if (!entry.is_regular_file())
                {
                    std::wcout << entry.path().wstring() << " not regular file" << std::endl;
                    std::cout << entry.path().string() << " not regular file" << std::endl; // utf8 string // 영문 windows에서는 주석처리 해야 함
                    continue;
                }

                auto& data_file = entry.path();
                if (data_file.extension().compare(".gltf") != 0 && data_file.extension().compare(".glb") != 0)
                {
                    if (data_file.extension().compare(".bin") != 0)
                        std::cout << data_file << " not gltf(glb) file" << std::endl;
                    continue;
                }

                // UTF-16을 UTF-8로 변환
                const std::string utf8FileName = util::wstring_to_utf8(data_file);
                std::string gltf_file_name = utf8FileName;

                // 새 gltf_file 객체 생성
                GltfMesh gltf_file;
                bimData.push_back(BimMeshInfo());
                BimMeshInfo& singleFile = bimData.back();
                // gltf 파일을 읽어 BimInfo 객체를 얻음
                if (!gltf_file.read(gltf_file_name, singleFile, is_offset_applied, offset)) {
                    bimData.pop_back();
                    continue;
                }

                auto& bim_info = singleFile;

                // bim_id, node_name 설정
                int bim_id = idx;
                std::string node_name;
                try
                {
                    // 파일 이름에서 확장자를 제외하고 '_'로 분할하여 마지막 문자열을 bim_id로 설정합니다.
                    std::wstring filenameW = data_file.stem().wstring(); // 파일 이름 - convert to unicode   
                    std::string filename = util::wstring_to_utf8(filenameW);

                    std::vector<std::string> parts;
                    std::stringstream ss(filename);
                    std::string item;
                    while (std::getline(ss, item, '_')) {// '_'로 분할
                        parts.push_back(item);
                    }

                    // 문자열의 마지막 부분(Revit Element ID)를 bim_id로 변환합니다.
                    if (!parts.empty()) {
                        //bim_id = boost::lexical_cast<unsigned long>(parts.back());
                        std::string lastPart = parts.back();
                        // 숫자만 남기기 위해 문자 제거
                        lastPart.erase(std::remove_if(lastPart.begin(), lastPart.end(), [](char c) { return !std::isdigit(c); }), lastPart.end());
                        if (!lastPart.empty()) {
                            try {
                                bim_id = std::stoul(lastPart); // boost::lexical_cast 대체
                            }
                            catch (const std::exception& e) {
                                std::cerr << "Error: Failed to convert to number: " << e.what() << std::endl;
                                bim_id = ++idx;
                            }
                        }
                        else {
                            std::cerr << "Error: No numeric part found in the last part of filename" << std::endl;
                            bim_id = ++idx; // 기본값으로 idx를 증가시킴
                        }
                    }
                    else {
                        std::cerr << "Error: Filename does not contain valid parts" << std::endl;
                        bim_id = ++idx; // 기본값으로 idx를 증가시킴
                    }

                    // 마지막 '_'의 위치를 찾고 앞부분(node name)을 가져옵니다.
                    size_t lastUnderscorePos = filename.find_last_of("_");
                    node_name = filename.substr(0, lastUnderscorePos); // node name 
                }
                catch (const std::exception& e)
                {
                    std::cerr << data_file.stem() << std::endl;
                    std::cerr << e.what() << std::endl;
                    bim_id = ++idx;
                }

                // utf8FileName의 파일명만 남기고 나머지는 지우기.
                std::string node_name_utf8 = utf8FileName;
                std::size_t lastSlashPos = node_name_utf8.find_last_of("\\");
                node_name_utf8 = node_name_utf8.substr(lastSlashPos + 1);
                // 확장자도 지우기
                std::size_t lastDotPos = node_name_utf8.find_last_of(".");
                node_name_utf8 = node_name_utf8.substr(0, lastDotPos);
                // 마지막 '_'의 위치를 찾고 앞부분(node name)을 가져옵니다.
                size_t lastUnderscorePos = node_name_utf8.find_last_of("_");
                node_name_utf8 = node_name_utf8.substr(0, lastUnderscorePos);
                // node_name 맨 마지막 문자가 '_'이면 ''로 변경
                if (node_name_utf8.back() == '_') { node_name_utf8.pop_back(); }

                //std::cout << bim_id << ": " << node_name << " " << node_name_utf8 << std::endl;

                // vertex buffer(geometry)를 가져옴 (offset 적용 (translation))
                std::size_t buffer_size = 0;
                char* vertex_buffer = bim_info.get_vertex_buffer(is_offset_applied, offset, buffer_size);

                // bounding box, MBR 정보를 가져옴
                MBR bbox = bim_info.get_mbr(is_offset_applied, offset);
                auto bbox_min = bbox.get_min(); // bbox의 최소값을 복사합니다.
                auto bbox_max = bbox.get_max(); // bbox의 최대값을 복사합니다.
                // 전역 bounding box 정보 업데이트
                for (int i = 0; i < 3; ++i) {
                    global_bbox_min[i] = (std::min)(global_bbox_min[i], bbox_min[i]);
                    global_bbox_max[i] = (std::max)(global_bbox_max[i], bbox_max[i]);
                }

                delete[] vertex_buffer;

                // .nodes2 파일에 bim_info 저장
                bim_info.save(bim_id, node_name, is_offset_applied, offset, filestream);

                // 진행 상황 출력
                processed_gltf_files++;
                int progress = (int)((float)processed_gltf_files / total_gltf_files * 100);
                if (processed_gltf_files % 100 == 0 || progress == 100)
                    std::cout << "Processed " << processed_gltf_files << " / " << total_gltf_files << " (" << progress << "%)" << std::endl;
            }

            filestream.close();

            return true;
        }

        std::vector<std::string> split(const std::string& str, const std::string& delims = "\t, ") {
            std::vector<std::string> tokens;
            size_t start = str.find_first_not_of(delims), end = 0;

            while (start != std::string::npos) {
                end = str.find_first_of(delims, start);
                if (end == std::string::npos) {
                    tokens.push_back(str.substr(start));
                    break;
                }
                else {
                    tokens.push_back(str.substr(start, end - start));
                    start = str.find_first_not_of(delims, end);
                }
            }
            return tokens;
        }

        bool convert_pts2pc2(const std::string& input_file_path,
            const std::string& output_file_path,
            const bool is_offset_applied,
            const float* offset)
        {
            auto start_t = std::chrono::system_clock::now();

            std::ifstream pts_file(input_file_path);
            if (!pts_file.is_open()) {
                std::cerr << "Failed to open input file: " << input_file_path << std::endl;
                return false;
            }

            /// 출력 폴더 경로 확인
            std::error_code ec;
            std::string* err = nullptr;
            std::filesystem::path output_path(output_file_path);
            fs::path dir = output_path.parent_path();
            if (!dir.empty() && !fs::exists(dir)) {
                if (!fs::create_directories(dir, ec)) {
                    std::cerr << "Failed in creating directories: " + dir.string();
                    return false;
                }
                else
                    std::cerr << "Create a directory: " + dir.string();
            }

            auto file_open_mode = std::ios::out | std::ios::binary;
            std::ofstream result_point_cloud_file(output_file_path, file_open_mode);
            if (!result_point_cloud_file.is_open()) {
                std::cerr << "Failed to open output binary file: " << output_file_path << std::endl;
                pts_file.close();
                return false;
            }

            /// offset 쓰기
            if (is_offset_applied)
                result_point_cloud_file.write(reinterpret_cast<const char*>(offset), sizeof(float) * 3);
            else {
                float zeroOffset[3] = { 0.f, 0.f, 0.f };
                result_point_cloud_file.write(reinterpret_cast<const char*>(zeroOffset), sizeof(float) * 3);
            }

            std::vector<float> buffer(3 * CHUNK_POINTS);
            std::size_t idx = 0;
            unsigned long long numPts = 0;
            cnvrt::BOUNDING_BOX bb;
            auto prev_t = start_t;

            std::string line_contents;

            /// 첫 줄 읽기
            if (!std::getline(pts_file, line_contents)) {
                std::cerr << "Failed to read the first line from the PTS file." << std::endl;
                pts_file.close();
                result_point_cloud_file.close();
                return false;
            }

            /// 첫 줄이 점 개수인지 데이터인지 확인
            std::vector<std::string> token = split(line_contents, "\t, ");
            unsigned long long total_num_points = 0;

            if (token.size() == 1) {
                /// 첫 줄이 점 개수인 경우
                total_num_points = std::stoull(token[0]);
                std::cout << "Total number of points recored in the first line in the file: " << total_num_points << "\n";
            }
            else if (token.size() >= 3) {
                /// 첫 줄이 데이터인 경우 - 그대로 처리
                for (int i = 0; i < 3; i++) {
                    double value = std::stod(token[i]);
                    buffer[i] = static_cast<float>(value);
                }
                idx = 1;
            }

            /// 나머지 줄들 처리
            while (std::getline(pts_file, line_contents))
            {
                std::vector<std::string> result = split(line_contents, "\t, ");
                if (result.size() < 3) {
                    continue;
                }

                /// 원본 값을 그대로 저장
                for (unsigned int i = 0; i < 3; i++) {
                    double value = std::stod(result[i]);
                    buffer[idx * 3 + i] = static_cast<float>(value);
                }

                /// 로딩된 점의 인덱스 증가
                idx++;

                if (idx == CHUNK_POINTS) {
                    /// 배치 단위로 offset 적용
                    if (is_offset_applied) {
                        for (size_t i = 0; i < idx; ++i) {
                            for (size_t j = 0; j < 3; ++j) {
                                buffer[i * 3 + j] -= offset[j];
                            }
                        }
                    }

                    /// 배치 단위로 바운딩 박스 갱신
                    cnvrt::POINT_DOUBLE point_d;
                    for (size_t i = 0; i < idx; ++i) {
                        point_d.x = buffer[i * 3 + 0];
                        point_d.y = buffer[i * 3 + 1];
                        point_d.z = buffer[i * 3 + 2];
                        bb.merge_point(point_d);
                    }

                    numPts += static_cast<unsigned long long>(idx);
                    result_point_cloud_file.write(reinterpret_cast<const char*>(buffer.data()), 12 * CHUNK_POINTS);

                    auto curr_t = std::chrono::system_clock::now();
                    if ((curr_t - prev_t) > std::chrono::seconds(2)) {
                        std::cout << "Processing " << numPts << " points.\n";
                        prev_t = curr_t;
                    }

                    idx = 0;
                }
            }

            /// 나머지 데이터 처리
            if (idx > 0)
            {
                /// 배치 단위로 offset 적용
                if (is_offset_applied) {
                    for (size_t i = 0; i < idx; ++i) {
                        for (size_t j = 0; j < 3; ++j) {
                            buffer[i * 3 + j] -= offset[j];
                        }
                    }
                }

                /// 배치 단위로 바운딩 박스 갱신
                cnvrt::POINT_DOUBLE point_d;
                for (size_t i = 0; i < idx; ++i) {
                    point_d.x = buffer[i * 3 + 0];
                    point_d.y = buffer[i * 3 + 1];
                    point_d.z = buffer[i * 3 + 2];
                    bb.merge_point(point_d);
                }

                numPts += static_cast<unsigned long long>(idx);
                result_point_cloud_file.write(reinterpret_cast<const char*>(buffer.data()), 12 * idx);
            }

            std::cout << "Processing " << numPts << " points. All points are processed.\n";

            pts_file.close();
            result_point_cloud_file.close();

            auto end_t = std::chrono::system_clock::now();
            auto exec_t = end_t - start_t;
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(exec_t).count();
            std::cout << "Writing " << output_file_path << " is finished.\n";
            std::cout << "Converting time: " << seconds << " seconds\nTotal number of points: " << numPts << ".\n";

            return true;
        }

        bool convert_las2pc2(const std::string& input_file_path,
            const std::string& output_file_path,
            const bool is_offset_applied,
            const float* offset)
        {
            auto start_t = std::chrono::system_clock::now();

            las::LASToolsReader lasReader;

            if (!lasReader.open(input_file_path)) {
                std::cerr << "Failed to open input file: " << input_file_path << std::endl;
                return false;
            }

            /// 출력 폴더 경로 확인
            std::error_code ec;
            std::string* err = nullptr;
            std::filesystem::path output_path(output_file_path);
            fs::path dir = output_path.parent_path();
            if (!dir.empty() && !fs::exists(dir)) {
                if (!fs::create_directories(dir, ec)) {
                    std::cerr << "Failed in creating directories: " + dir.string();
                    return false;
                }
                else
                    std::cerr << "Create a directory: " + dir.string();
            }

            auto file_open_mode = std::ios::out | std::ios::binary;
            std::ofstream result_point_cloud_file(output_file_path, file_open_mode);
            if (result_point_cloud_file.is_open()) {
                lasReader.printHeader();
            }
            else {
                std::cerr << "Failed to open output binary file: " << output_file_path << std::endl;
                lasReader.close();
                return false;
            }

            /// offset 쓰기
            if (is_offset_applied)
                result_point_cloud_file.write(reinterpret_cast<const char*>(offset), sizeof(float) * 3);
            else {
                float zeroOffset[3] = { 0.f, 0.f, 0.f };
                result_point_cloud_file.write(reinterpret_cast<const char*>(zeroOffset), sizeof(float) * 3);
            }

            std::vector<float> buffer(3 * CHUNK_POINTS);
            unsigned long long numPts = 0;
            cnvrt::BOUNDING_BOX bb;
            auto prev_t = start_t;

            size_t total_num_points = lasReader.getPointCount();
            const std::size_t N = (std::min)(CHUNK_POINTS, total_num_points);

            for (size_t start = 0; start < total_num_points; start += N) {
                std::size_t end = (std::min)(start + N, total_num_points);
                if (!lasReader.loadPointRange(start, end)) {
                    std::cerr << "Failed in loadPointRandge(" << start << ", " << end << ")\n";
                    break;
                }

                cnvrt::POINT_DOUBLE point_d;
                if (is_offset_applied) {
                    size_t idx = 0;
                    for (const auto& p : lasReader.getLoadedPoints()) {
                        buffer[idx * 3 + 0] = static_cast<float>(p.x) - offset[0];
                        buffer[idx * 3 + 1] = static_cast<float>(p.y) - offset[1];
                        buffer[idx * 3 + 2] = static_cast<float>(p.z) - offset[2];

                        /// 배치 단위로 바운딩 박스 갱신
                        point_d.x = buffer[idx * 3 + 0];
                        point_d.y = buffer[idx * 3 + 1];
                        point_d.z = buffer[idx * 3 + 2];
                        bb.merge_point(point_d);

                        ++idx;
                    }
                }
                else {
                    size_t idx = 0;
                    for (const auto& p : lasReader.getLoadedPoints()) {
                        buffer[idx * 3 + 0] = static_cast<float>(p.x);
                        buffer[idx * 3 + 1] = static_cast<float>(p.y);
                        buffer[idx * 3 + 2] = static_cast<float>(p.z);

                        /// 배치 단위로 바운딩 박스 갱신
                        point_d.x = buffer[idx * 3 + 0];
                        point_d.y = buffer[idx * 3 + 1];
                        point_d.z = buffer[idx * 3 + 2];
                        bb.merge_point(point_d);

                        ++idx;
                    }

                }

                numPts += static_cast<unsigned long long>(end - start);
                result_point_cloud_file.write(reinterpret_cast<const char*>(buffer.data()), 12 * (end - start));

                auto curr_t = std::chrono::system_clock::now();
                if ((curr_t - prev_t) > std::chrono::seconds(2)) {
                    std::cout << "Processing " << numPts << " points / " << total_num_points << "\n";
                    prev_t = curr_t;
                }
            }

            lasReader.close();
            result_point_cloud_file.close();

            std::cout << "Processing " << numPts << " points. All points are processed.\n";

            auto end_t = std::chrono::system_clock::now();
            auto exec_t = end_t - start_t;
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(exec_t).count();
            std::cout << "Writing " << output_file_path << " is finished.\n";
            std::cout << "Converting time: " << seconds << " seconds\nTotal number of points: " << numPts << ".\n";

            return true;
        }
    }

    namespace mesh {
        /// 전체 경로로부터 노드 이름을 추출
        void extractNodeInfo(const std::filesystem::path& file_path,
            std::string& nodeName,
            int& bimId,
            int& fallback_idx)
        {
            // 기본값 설정
            nodeName = "unknown";
            bimId = fallback_idx;

            try {
                // UTF-16을 UTF-8로 변환
                std::wstring filenameW = file_path.stem().wstring();
                std::string filename = util::wstring_to_utf8(filenameW);

                if (filename.empty()) {
                    std::cerr << "Error: Empty filename" << std::endl;
                    bimId = ++fallback_idx;
                    return;
                }

                // 마지막 '_' 위치 찾기
                size_t lastUnderscorePos = filename.find_last_of('_');

                if (lastUnderscorePos == std::string::npos) {
                    // '_'가 없는 경우
                    nodeName = filename;
                    bimId = ++fallback_idx;
                    return;
                }

                // 노드 이름 추출 (마지막 '_' 이전)
                nodeName = filename.substr(0, lastUnderscorePos);

                // 노드 이름 마지막 문자가 '_'이면 제거
                if (!nodeName.empty() && nodeName.back() == '_') {
                    nodeName.pop_back();
                }

                // BIM ID 추출 (마지막 '_' 이후)
                std::string idPart = filename.substr(lastUnderscorePos + 1);

                // 숫자만 추출
                idPart.erase(
                    std::remove_if(idPart.begin(), idPart.end(),
                        [](unsigned char c) { return !std::isdigit(c); }
                    ),
                    idPart.end()
                );

                if (idPart.empty()) {
                    std::cerr << "Error: No numeric part found in filename" << std::endl;
                    bimId = ++fallback_idx;
                    return;
                }

                // 문자열을 숫자로 변환
                try {
                    unsigned long temp = std::stoul(idPart);
                    if (temp > static_cast<unsigned long>((std::numeric_limits<int>::max)())) {
                        std::cerr << "Warning: BIM ID too large: " << temp << std::endl;
                        bimId = ++fallback_idx;
                    }
                    else {
                        bimId = static_cast<int>(temp);
                    }
                }
                catch (const std::exception& e) {
                    std::cerr << "Error: Failed to convert to number: " << e.what() << std::endl;
                    bimId = ++fallback_idx;
                }
            }
            catch (const std::exception& e) {
                std::cerr << "Exception in extractNodeInfo: " << e.what() << std::endl;
                bimId = ++fallback_idx;
            }
        }

        /// 지오메트리와 바운딩박스
        struct GeometryInfo {
            MBR bbox;
            size_t buffer_size;
            bool success;
        };

        GeometryInfo processGeometryAndBoundingBox(BimMeshInfo& bim_info,
            std::array<float, 3>& bbox_min,
            std::array<float, 3>& bbox_max)
        {
            GeometryInfo geo_info;
            geo_info.success = false;
            geo_info.buffer_size = 0;

            try {
                bool is_offset_applied = false;
                float offset[3] = { 0.f, 0.f, 0.f };

                /// bim_info의 함수를 이용해 bbox 계산
                geo_info.bbox = bim_info.get_mbr(is_offset_applied, offset);
                auto local_bbox_min = geo_info.bbox.get_min();
                auto local_bbox_max = geo_info.bbox.get_max();

                /// 전역 바운딩 박스 업데이트 (파라미터)
                for (int i = 0; i < 3; ++i) {
                    bbox_min[i] = (std::min)(bbox_min[i], local_bbox_min[i]);
                    bbox_max[i] = (std::max)(bbox_max[i], local_bbox_max[i]);
                }

                geo_info.success = true;
            }
            catch (const std::exception& e) {
                std::cerr << "Error in processGeometryAndBoundingBox: " << e.what() << std::endl;
            }

            return geo_info;
        }

        bool loadGltf(const std::string& bim_folder, std::vector<BimMeshInfo>& bimData)
        {
            std::filesystem::path input_path(bim_folder);

            // 경로 검증
            if (!std::filesystem::exists(input_path)) {
                std::cout << "input_folder '" << bim_folder << "' does not exist" << std::endl;
                return false;
            }

            // 진행 상황 추적
            size_t total_gltf_files = std::distance(
                fs::directory_iterator(input_path),
                fs::directory_iterator()
            );
            int processed_gltf_files = 0;
            int fallback_idx = 1000; // Revit Element ID가 없을 때 사용할 기본 ID

            // 디렉토리 순회
            for (const auto& entry : std::filesystem::directory_iterator(input_path))
            {
                // 디버그 출력
                std::wcout << entry.path().wstring() << std::endl;
                std::cout << entry.path().string() << std::endl; // 영문 Windows에서는 주석 처리

                // 일반 파일 확인
                if (!entry.is_regular_file()) {
                    std::wcout << entry.path().wstring() << " not regular file" << std::endl;
                    continue;
                }

                // GLTF/GLB 파일 확인
                auto& data_file = entry.path();
                if (data_file.extension() != ".gltf" && data_file.extension() != ".glb") {
                    if (data_file.extension() != ".bin") {
                        std::cout << data_file << " not gltf(glb) file" << std::endl;
                    }
                    continue;
                }

                // UTF-16을 UTF-8로 변환
                const std::string utf8FileName = util::wstring_to_utf8(data_file);

                // GLTF 파일 읽기
                bimData.push_back(BimMeshInfo());
                BimMeshInfo& currentGltf = bimData.back();

                bool is_offset_applied = false;
                float offset[3] = { 0.f, 0.f, 0.f };

                GltfMesh gltf_file;
                /// gltf 읽기
                if (!gltf_file.read(utf8FileName, currentGltf, is_offset_applied, offset)) {
                    std::cerr << "Failed to read GLTF file: " << utf8FileName << std::endl;
                    continue;
                }

                extractNodeInfo(utf8FileName, currentGltf.node_name, currentGltf.bim_id, fallback_idx);

                /// 바운딩 박스
                for (auto& min : currentGltf.bbox_min)
                    min = (std::numeric_limits<float>::max)();
                for (auto& max : currentGltf.bbox_max)
                    max = std::numeric_limits<float>::lowest();

                GeometryInfo geometry = processGeometryAndBoundingBox(currentGltf, currentGltf.bbox_min, currentGltf.bbox_max);

                // 진행 상황 출력
                processed_gltf_files++;
                int progress = (int)((float)processed_gltf_files / total_gltf_files * 100);
                if (processed_gltf_files % 100 == 0 || progress == 100) {
                    std::cout << "Processed " << processed_gltf_files
                        << " / " << total_gltf_files
                        << " (" << progress << "%)" << std::endl;
                }
            }

            return true;
        }

        namespace tree {
            /// Point3D를 위한 PointCloud 어댑터
            struct PointCloud3DAdaptor
            {
                const std::vector<Point3D>& points;

                PointCloud3DAdaptor(const std::vector<Point3D>& pts) : points(pts) {}

                /// nanoflann에서 필요한 인터페이스 구현
                inline size_t kdtree_get_point_count() const { return points.size(); }

                /// 특정 포인트의 특정 차원 값 반환
                inline double kdtree_get_pt(const size_t idx, const size_t dim) const
                {
                    if (dim == 0) return points[idx][0];
                    else if (dim == 1) return points[idx][1];
                    else return points[idx][2];
                }

                /// 바운딩 박스 계산 (빠른 빌드를 위해)
                template <class BBOX>
                bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
            };

            struct Point {
                float x, y, z;
            };

            /// Face 포인트 정보를 담는 구조체
            struct FacePointInfo {
                Point point;
                size_t bimIdx;
                size_t faceIdx;
                bool isPseudo;

                FacePointInfo(const Point& p, size_t bIdx, size_t fIdx, bool pseudo = false)
                    : point(p), bimIdx(bIdx), faceIdx(fIdx), isPseudo(pseudo) {
                }
            };

            // nanoflann용 PointCloud 어댑터
            struct FacePointCloudAdaptor {
                const std::vector<FacePointInfo>& points;

                FacePointCloudAdaptor(const std::vector<FacePointInfo>& pts) : points(pts) {}

                // 필수: 포인트 개수 반환
                inline size_t kdtree_get_point_count() const { return points.size(); }

                // 필수: 특정 포인트의 특정 차원 값 반환
                inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
                    const Point& p = points[idx].point;
                    if (dim == 0) return p.x;
                    else if (dim == 1) return p.y;
                    else return p.z;
                }

                // 선택: 바운딩 박스
                template <class BBOX>
                bool kdtree_get_bbox(BBOX&) const { return false; }
            };

            /// KD-tree 타입 정의: KDTree3D
            typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud3DAdaptor>, PointCloud3DAdaptor, 3> KDTree3D;

            /// KD-tree 타입 정의: KDTreeFace
            typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, FacePointCloudAdaptor>, FacePointCloudAdaptor, 3, size_t> KDTreeFace;

            /// KD-tree 빌드 함수
            bool buildKdtree3D(const std::vector<Point3D>& points,
                std::shared_ptr<PointCloud3DAdaptor>& adaptor,
                std::shared_ptr<KDTree3D>& tree,
                int maxLeaf = 10)
            {
                if (points.empty()) {
                    std::cerr << "Error: Point cloud is empty" << std::endl;
                    return false;
                }

                try {
                    /// 어댑터 생성
                    adaptor.reset(new PointCloud3DAdaptor(points));

                    /// KD-tree 생성 및 빌드
                    tree.reset(new KDTree3D(3, *adaptor,
                        nanoflann::KDTreeSingleIndexAdaptorParams(maxLeaf)));
                    tree->buildIndex();
                }
                catch (const std::exception& e) {
                    std::cerr << "Error in buildKdtree3D: " << e.what() << std::endl;
                    return false;
                }
                catch (...) {
                    std::cerr << "Unknown error in buildKdtree3D" << std::endl;
                    return false;
                }

                return true;
            }

            void findNearest3D() {
                std::vector<Point3D> points;
                std::shared_ptr<PointCloud3DAdaptor> adaptor;
                std::shared_ptr<KDTree3D> tree;
                if (!buildKdtree3D(points, adaptor, tree))
                    return;

                size_t numClosest = 2;
                std::vector<size_t> indices(numClosest);
                std::vector<double> distances(numClosest);
                Point3D query(2.0, 3.0, 4.0);
                double queryPt[3] = { 2.0, 3.0, 4.0 };
                //size_t numFound = tree->knnSearch(queryPt, 1, indices.data(), distances.data());

                /*
                size_t num_results = 1;
                std::vector<size_t> ret_index(num_results);
                std::vector<double> out_dist_sqr(num_results);

                nanoflann::KNNResultSet<double> resultSet(num_results);
                resultSet.init(&ret_index[0], &out_dist_sqr[0]);

                double query_pt[3] = { query.x, query.y, query.z };
                tree->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));

                std::cout << "Nearest point index: " << ret_index[0] << std::endl;
                std::cout << "Distance squared: " << out_dist_sqr[0] << std::endl;
                */
            }

            /// Face KD-tree 관리 클래스
            class FaceFinder {
            public:
                FaceFinder() = default;
                ~FaceFinder() = default;

                /*
                /// BimMeshInfo 데이터로부터 KD-tree 구축
                bool buildKdTree(std::vector<BimMeshInfo>& bimData, double grid_size = 0.1) {
                    points.clear();
                    points.reserve(bimData.size() * 1000);

                    std::cout << "Building KD-tree for face search..." << std::endl;

                    /// 모든 BimMeshInfo 순회
                    for (size_t bimIdx = 0; bimIdx < bimData.size(); ++bimIdx) {
                        auto& bimInfo = bimData[bimIdx];
                        auto& faces = bimInfo.getFaces();

                        /// 각 face 처리
                        for (size_t faceIdx = 0; faceIdx < faces.size(); ++faceIdx) {
                            auto& face = faces[faceIdx];

                            // 1. Vertex 포인트 추가
                            points.emplace_back(face.getv1(), bimIdx, faceIdx, false);
                            points.emplace_back(face.getv2(), bimIdx, faceIdx, false);
                            points.emplace_back(face.getv3(), bimIdx, faceIdx, false);

                            // 2. Pseudo 포인트 생성 및 추가
                            std::vector<Point> pseudoPts = generatePseudoPoints(face, grid_size);
                            for (const auto& pt : pseudoPts) {
                                points.emplace_back(pt, bimIdx, faceIdx, true);
                            }
                        }

                        if ((bimIdx + 1) % 100 == 0) {
                            std::cout << "Processed " << (bimIdx + 1) << " / "
                                << bimData.size() << " BIM objects" << std::endl;
                        }
                    }

                    std::cout << "Total points collected: " << points.size() << std::endl;

                    if (points.empty()) {
                        std::cerr << "Error: No points collected for KD-tree" << std::endl;
                        return false;
                    }

                    try {
                        /// KD-tree 빌드
                        adaptor.reset(new FacePointCloudAdaptor(points));
                        kdtree.reset(new KDTreeFace(
                            3,  // 차원
                            *adaptor,
                            nanoflann::KDTreeSingleIndexAdaptorParams(10)  // max leaf
                        ));
                        kdtree->buildIndex();

                        std::cout << "KD-tree built successfully!" << std::endl;
                        return true;
                    }
                    catch (const std::exception& e) {
                        std::cerr << "Error building KD-tree: " << e.what() << std::endl;
                        return false;
                    }
                    catch (...) {
                        std::cerr << "Unknown error building KD-tree" << std::endl;
                        return false;
                    }
                }
                */

                /*
                // 가장 가까운 face 찾기
                bool findNearestFace(const Point& queryPoint,
                    size_t& bimIdx,
                    size_t& faceIdx,
                    double& distance) const {
                    if (!kdtree) {
                        std::cerr << "KD-tree not built yet" << std::endl;
                        return false;
                    }

                    const size_t num_results = 1;
                    size_t ret_index;
                    double out_dist_sqr;

                    nanoflann::KNNResultSet<double> resultSet(num_results);
                    resultSet.init(&ret_index, &out_dist_sqr);

                    double query_pt[3] = { queryPoint.x, queryPoint.y, queryPoint.z };
                    kdtree->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));

                    if (ret_index < points.size()) {
                        const auto& nearestPoint = points[ret_index];
                        bimIdx = nearestPoint.bimIdx;
                        faceIdx = nearestPoint.faceIdx;
                        distance = std::sqrt(out_dist_sqr);
                        return true;
                    }

                    return false;
                }
                */

                /*
                // K개의 가장 가까운 face들 찾기 (중복 제거)
                std::vector<std::tuple<size_t, size_t, double>> findNearestFaces(
                    const Point& queryPoint,
                    size_t k = 5) const {

                    if (!kdtree) {
                        std::cerr << "KD-tree not built yet" << std::endl;
                        return {};
                    }

                    // K개보다 많이 검색 (같은 face의 여러 포인트를 걸러내기 위해)
                    const size_t search_k = (std::min)(k * 10, points.size());
                    std::vector<size_t> ret_indices(search_k);
                    std::vector<double> out_dists_sqr(search_k);

                    nanoflann::KNNResultSet<double> resultSet(search_k);
                    resultSet.init(&ret_indices[0], &out_dists_sqr[0]);

                    double query_pt[3] = { queryPoint.x, queryPoint.y, queryPoint.z };
                    kdtree->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));

                    // Face별로 가장 가까운 거리만 유지 (중복 제거)
                    std::map<std::pair<size_t, size_t>, double> uniqueFaces;

                    for (size_t i = 0; i < search_k && i < ret_indices.size(); ++i) {
                        const auto& pt = points[ret_indices[i]];
                        auto faceKey = std::make_pair(pt.bimIdx, pt.faceIdx);
                        double dist = std::sqrt(out_dists_sqr[i]);

                        if (uniqueFaces.find(faceKey) == uniqueFaces.end() ||
                            uniqueFaces[faceKey] > dist) {
                            uniqueFaces[faceKey] = dist;
                        }

                        if (uniqueFaces.size() >= k) break;
                    }

                    // 결과를 vector로 변환
                    std::vector<std::tuple<size_t, size_t, double>> results;
                    for (const auto& [key, dist] : uniqueFaces) {
                        results.emplace_back(key.first, key.second, dist);
                    }

                    // 거리순 정렬
                    std::sort(results.begin(), results.end(),
                        [](const auto& a, const auto& b) {
                            return std::get<2>(a) < std::get<2>(b);
                        });

                    return results;
                }
                */

                /*
                // 반경 내 face들 찾기
                std::vector<std::tuple<size_t, size_t, double>> findFacesWithinRadius(
                    const Point& queryPoint,
                    double radius) const {

                    if (!kdtree) {
                        std::cerr << "KD-tree not built yet" << std::endl;
                        return {};
                    }

                    std::vector<std::pair<size_t, double>> ret_matches;
                    nanoflann::SearchParams params;
                    params.sorted = true;

                    double query_pt[3] = { queryPoint.x, queryPoint.y, queryPoint.z };
                    const double search_radius_sqr = radius * radius;

                    size_t nMatches = kdtree->radiusSearch(query_pt, search_radius_sqr, ret_matches, params);

                    // Face별로 중복 제거
                    std::map<std::pair<size_t, size_t>, double> uniqueFaces;

                    for (const auto& match : ret_matches) {
                        const auto& pt = points[match.first];
                        auto faceKey = std::make_pair(pt.bimIdx, pt.faceIdx);
                        double dist = std::sqrt(match.second);

                        if (uniqueFaces.find(faceKey) == uniqueFaces.end() ||
                            uniqueFaces[faceKey] > dist) {
                            uniqueFaces[faceKey] = dist;
                        }
                    }

                    std::vector<std::tuple<size_t, size_t, double>> results;
                    for (const auto& [key, dist] : uniqueFaces) {
                        results.emplace_back(key.first, key.second, dist);
                    }

                    std::sort(results.begin(), results.end(),
                        [](const auto& a, const auto& b) {
                            return std::get<2>(a) < std::get<2>(b);
                        });

                    return results;
                }
                */

                size_t getPointCount() const { return points.size(); }

            private:
                // Pseudo 포인트 생성 (Face의 generate_pseudo_points 버그 수정 버전)
                std::vector<Point> generatePseudoPoints(const Face& face, double grid_size) const {
                    std::vector<Point> pseudoPts;

                    if (grid_size <= 0.0) return pseudoPts;

                    Point v1, v2, v3;
                    v1.x = face.getv1().getX();
                    v1.y = face.getv1().getY();
                    v1.z = face.getv1().getZ();
                    v2.x = face.getv2().getX();
                    v2.y = face.getv2().getY();
                    v2.z = face.getv2().getZ();
                    v3.x = face.getv3().getX();
                    v3.y = face.getv3().getY();
                    v3.z = face.getv3().getZ();

                    // 두 edge 벡터 정의 (v2를 기준점으로)
                    Point edge1(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
                    Point edge2(v3.x - v2.x, v3.y - v2.y, v3.z - v2.z);  // 버그 수정

                    // Edge 길이 계산
                    double len1 = std::sqrt(edge1.x * edge1.x + edge1.y * edge1.y + edge1.z * edge1.z);
                    double len2 = std::sqrt(edge2.x * edge2.x + edge2.y * edge2.y + edge2.z * edge2.z);

                    if (len1 < std::numeric_limits<double>::epsilon() ||
                        len2 < std::numeric_limits<double>::epsilon())
                        return pseudoPts;

                    if (len1 < grid_size && len2 < grid_size)
                        return pseudoPts;

                    // 각도 계산
                    double dotproduct = edge1.x * edge2.x + edge1.y * edge2.y + edge1.z * edge2.z;
                    double cosAngle = dotproduct / (len1 * len2);
                    // clamp to [-1, 1]
                    if (cosAngle > 1.0) cosAngle = 1.0;
                    if (cosAngle < -1.0) cosAngle = -1.0;

                    double angle = std::abs(std::acos(cosAngle));
                    double sinAngle = std::sin(angle);

                    double slant_grid_size = grid_size;
                    if (sinAngle > std::numeric_limits<double>::epsilon())
                        slant_grid_size = grid_size / sinAngle;

                    // 샘플링 수 계산
                    int num_steps1 = (std::max)(1, static_cast<int>(len1 / grid_size));
                    int num_steps2 = (std::max)(1, static_cast<int>(len2 / slant_grid_size));

                    const double inv_steps1 = 1.0 / static_cast<double>(num_steps1);
                    const double inv_steps2 = 1.0 / static_cast<double>(num_steps2);

                    // Pseudo 포인트 생성
                    pseudoPts.reserve(num_steps1 * num_steps2 / 2);

                    for (int i = 0; i <= num_steps1; ++i) {
                        double u = static_cast<double>(i) * inv_steps1;

                        for (int j = 0; j <= num_steps2; ++j) {
                            double v = static_cast<double>(j) * inv_steps2;

                            // 삼각형 내부 점만 추가 (Barycentric coordinates)
                            if (u + v <= 1.0) {
                                pseudoPts.emplace_back(
                                    v2.x + u * edge1.x + v * edge2.x,
                                    v2.y + u * edge1.y + v * edge2.y,
                                    v2.z + u * edge1.z + v * edge2.z
                                );
                            }
                        }
                    }

                    return pseudoPts;
                }

                std::vector<FacePointInfo> points;
                std::shared_ptr<FacePointCloudAdaptor> adaptor;
                std::shared_ptr<KDTreeFace> kdtree;
            };
        }

        /*
        /// las 파일 로딩
        bool loadLasFile(const std::string& file_path, std::shared_ptr<tree::KDTree3D>& tree)
        {
            try {
                las::LASToolsReader lasReader;

                if (!lasReader.open(file_path)) {
                    std::cerr << "Failed to open input file: " << file_path << std::endl;
                    return false;
                }

                if (!lasReader.loadAllPoints()) {
                    std::cerr << "Failed in loadAllPoints\n";
                    return false;
                }

                std::vector<Point3D> points;
                points.reserve(lasReader.getLoadedPointCount());

                for (const auto& p : lasReader.getLoadedPoints()) {
                    points.emplace_back(p.x, p.y, p.z);
                }

                /// KD-tree 구축
                std::shared_ptr<tree::PointCloud3DAdaptor> adaptor;

                if (!buildKdtree3D(points, adaptor, tree, 20)) {
                    std::cerr << "Failed in building kd-tree\n";
                    return false;
                }

                lasReader.close();

                std::cout << "Loading pointcloud completed: " << points.size() << " points\n";
            }
            catch (const std::exception& e) {
                std::cerr << "Error in Las loading: " << e.what() << std::endl;
            }

            return true;
        }

        bool loadInputData(const std::string& file_path, const std::string& bim_folder) {

            std::shared_ptr<tree::KDTree3D> tree;

            if (!loadLasFile(file_path, tree)) {
                return false;
            }

            std::vector<BimMeshInfo> bimData;
            if (!loadGltf(bim_folder, bimData)) {
                return false;
            }



            return true;
        }

        */

    }/// namespace mesh

    namespace mesh2 {
        /// Vertex3D: 3차원 정점 데이터
        class Vertex3D
        {
        public:
            Vertex3D()
            {
                xyz[0] = xyz[1] = xyz[2] = 0.0;
            }

            Vertex3D(const double x, const double y, const double z)
            {
                xyz[0] = x;
                xyz[1] = y;
                xyz[2] = z;
            }

            Vertex3D(const Vertex3D& val)
            {
                copy(val);
                faceId = val.faceId;
            }

            inline Vertex3D& operator=(const Vertex3D& val)
            {
                copy(val);
                return *this;
            }

            inline Vertex3D operator+=(const Vertex3D& val)
            {
                this->xyz[0] += val.xyz[0];
                this->xyz[1] += val.xyz[1];
                this->xyz[2] += val.xyz[2];
                return *this;
            }

            inline Vertex3D operator/=(const double& val)
            {
                try
                {
                    this->xyz[0] /= val;
                    this->xyz[1] /= val;
                    this->xyz[2] /= val;
                }
                catch (...)
                {
                    throw std::runtime_error("Error from Vertex3D operator /=");
                }
                return *this;
            }

            inline Vertex3D operator-(const Vertex3D& val) const
            {
                Vertex3D retVal;
                retVal.xyz[0] = this->xyz[0] - val.xyz[0];
                retVal.xyz[1] = this->xyz[1] - val.xyz[1];
                retVal.xyz[2] = this->xyz[2] - val.xyz[2];
                return retVal;
            }

            inline Vertex3D operator+(const Vertex3D& val) const
            {
                Vertex3D retVal;
                retVal.xyz[0] = this->xyz[0] + val.xyz[0];
                retVal.xyz[1] = this->xyz[1] + val.xyz[1];
                retVal.xyz[2] = this->xyz[2] + val.xyz[2];
                return retVal;
            }

            inline Vertex3D operator/(const double val) const
            {
                Vertex3D retVal;

                try {
                    retVal.xyz[0] = this->xyz[0] / val;
                    retVal.xyz[1] = this->xyz[1] / val;
                    retVal.xyz[2] = this->xyz[2] / val;
                }
                catch (...)
                {
                    throw std::runtime_error("Fatal error from Vertex3D operator ""/""");
                }

                return retVal;
            }

            inline Vertex3D operator*(const double val) const
            {
                Vertex3D retVal;

                retVal.xyz[0] = this->xyz[0] * val;
                retVal.xyz[1] = this->xyz[1] * val;
                retVal.xyz[2] = this->xyz[2] * val;

                return retVal;
            }

            inline void copy(const Vertex3D& val)
            {
                ::memcpy(xyz, val.xyz, 3 * sizeof(double));
                faceId = val.faceId;
            }

            inline bool operator==(const Vertex3D& pt) const
            {
                return pt.x() == xyz[0] && pt.y() == xyz[1] && pt.z() == xyz[2];
            }

            inline const double& x() const { return xyz[0]; }
            inline const double& y() const { return xyz[1]; }
            inline const double& z() const { return xyz[2]; }

            inline double& x() { return xyz[0]; }
            inline double& y() { return xyz[1]; }
            inline double& z() { return xyz[2]; }

            inline double& operator[] (const std::size_t idx)
            {
                assert(idx < 3);
                return xyz[idx];
            }

            inline const double& operator[] (const std::size_t idx) const
            {
                try
                {
                    return xyz[idx];
                }
                catch (...)
                {
                    throw std::runtime_error("Error in Vertex3D::operator[]");
                }
            }

            inline void set(const double newX, const double newY, const double newZ)
            {
                xyz[0] = newX;
                xyz[1] = newY;
                xyz[2] = newZ;
            }

            inline const std::vector<int>& getFaceId() const { return faceId; }
            inline std::vector<int>& getFaceId() { return faceId; }
            void setFaceId(const std::vector<int>& newFaceId) { faceId = newFaceId; }
            inline void addFaceId(const int idx)
            {
                faceId.push_back(idx);
            }

        private:
            double xyz[3];
            std::vector<int> faceId;
        };

        ///  MeshData: 메시 데이터를 위한 구조체
        template<class T> struct MeshData
        {
        public:
            typedef T CoordValueType;

            /// 데이터 점의 개수를 반환해야 함
            inline std::size_t kdtree_get_point_count() const { return vertices.size(); }

            /// 벡터 "p1[0:size-1]"과 클래스에 저장된 인덱스 "idx_p2"의 데이터 점 사이의 거리를 반환:
            inline T kdtree_distance(const T* p1, const size_t idx_p2, size_t size) const
            {
                T s = 0;
                for (size_t i = 0; i < size; ++i)
                {
                    const T d = p1[i] - vertices[idx_p2][i];
                    s += d * d;
                }
                return s;
            }

            /// 클래스 내 idx번째 점의 dim번째 성분을 반환:
            inline T kdtree_get_pt(const size_t idx, int dim) const
            {
                return vertices[idx][dim];
            }

            /// 선택적 바운딩 박스 계산: false를 반환하면 표준 bbox 계산 루프를 기본으로 사용한다.
            /// 클래스가 이미 BBOX를 계산해서 "bb"에 담아 반환했다면 true를 반환해 재계산을 피할 수 있다.
            /// bb.size()를 보면 기대되는 차원 수를 알 수 있다 (포인트클라우드의 경우 보통 2 또는 3)
            template <class BBOX> bool kdtree_get_bbox(BBOX& /*bb*/) const
            {
                return false;
            }

        public:
            std::vector<Vertex3D> vertices;
            std::vector<std::vector<unsigned int>> faces;
            std::vector<Vertex3D> fNormals;
        };

        /// 메시 데이터 타입
        using MeshDataType = MeshData<double>;

        /// 메시 노드 데이터에 사용할 KD-Tree 타입
        using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<MeshDataType::CoordValueType, MeshDataType>, MeshDataType, 3>;

        /// 데이터 차원
        const std::size_t dimension = 3;

        /// 가상점(pseudo point) 타입
        enum class PSEUDOTYPE { NONE = 0, CENTROID = 1, MIDPOINT = 2 };

        class SSMFlann
        {
        public:
            SSMFlann(const double maxLength = 0.0, const PSEUDOTYPE pType = PSEUDOTYPE::MIDPOINT);

            /**setNumberOfPoints
            *@brief 정점 데이터 벡터 크기 조정
            *@param size : 정점 데이터 크기 (정점 개수)
            */
            void setNumberOfPoints(const unsigned int size);

            /**setVertex
            *@brief 중복 점 검사 없이 정점 추가
            *@param index : 정점 인덱스
            *@param x
            *@param y
            *@param z
            *@return bool
            */
            bool setVertex(const unsigned int index, const double x, const double y, const double z);

            /**setNumberOfFaces
            *@brief face 데이터 벡터 크기 조정
            *@param size : face 개수
            */
            void setNumberOfFaces(const unsigned int size);

            /**setPseudoType
            *@brief 가상점 생성 방식 설정
            *@param pType: 타입
            */
            void setPseudoType(const PSEUDOTYPE pType);

            /**setFace
            *@brief face 추가
            *@param faceIndex : face 인덱스
            *@param vtxIndex : face를 구성하는 정점들의 인덱스
            *@param fnx : face normal 벡터의 x 좌표
            *@param fny : face normal 벡터의 y 좌표
            *@param fnz : face normal 벡터의 z 좌표
            *@return bool
            */
            bool setFace(const unsigned int faceIndex, const std::vector<unsigned int>& vtxIndex);

            /**setFaceNormal
            *@brief face normal 벡터 추가
            *@param faceIndex : face 인덱스
            *@param fnx : face normal 벡터의 x 좌표
            *@param fny : face normal 벡터의 y 좌표
            *@param fnz : face normal 벡터의 z 좌표
            *@return bool
            */
            bool setFaceNormal(const unsigned int faceIndex, const double fnx, const double fny, const double fnz);

            /**getFaceNormal
            *@brief face normal 벡터 조회
            *@param faceIndex : face 인덱스
            *@return Vertex3D
            */
            Vertex3D getFaceNormal(const unsigned int faceIndex) const;

            /**clearVertexData
            *@brief 정점 데이터 초기화
            */
            void clearVertexData();

            /**buildTree
            *@brief buildTree
            *@param maxLeaf kdtree의 최대 leaf 개수
            */
            void buildTree(const unsigned int maxLeaf = 20);

            /**buildTree
            *@brief 파일로부터 인덱스 정보를 불러옴
            *@param maxLeaf kdtree의 최대 leaf 개수
            *@return bool
            */
            bool buildTree(const std::string& idxFilePath, const unsigned int maxLeaf = 20);

            /**search
            *@brief 가장 가까운 정점들 탐색
            */
            bool search(const double x0, const double y0, const double z0,
                std::vector<double>& distances,
                std::vector<std::size_t>& indices,
                const unsigned int numClosest,
                const double distThreshold = 0.0) const;

            /**raytrace
            *@brief 주어진 광선(ray)을 따라 가장 가까운 점을 찾음
            *@param sx: 광선 시작점의 x 좌표
            *@param sy: 광선 시작점의 y 좌표
            *@param sz: 광선 시작점의 z 좌표
            *@param ex: 광선 끝점의 x 좌표
            *@param ey: 광선 끝점의 y 좌표
            *@param ez: 광선 끝점의 z 좌표
            *@param radius: 탐색 반경
            *@param foundPts: 점 인덱스와 거리, 결과값
            *@return bool
            */
            bool raytrace(const double sx, const double sy, const double sz,
                const double ex, const double ey, const double ez,
                const double radius,
                std::vector<std::pair<size_t, double>>& foundPts);

            /**getVertex
            *@param vtxIndex : 정점 인덱스
            */
            Vertex3D getVertex(const std::size_t vtxIndex) const;

            /**getDistThreshold
            */
            double getDistThreshold() const { return this->maxDist; }

            /**getVertices
            */
            const std::vector<Vertex3D>& getVertices() const { return this->meshData.vertices; }

            /**getPseudoPts
            */
            const std::vector<Vertex3D>& getPseudoPts() const { return this->pseudoPts; }

            /**getFaces
            */
            const std::vector<std::vector<unsigned int>>& getFaces() const { return this->meshData.faces; }

            /**setDistThreshold
            */
            void setDistThreshold(const double newDistTh) { this->maxDist = newDistTh; }

            /**exportIndex
            *@param idxFilePath : 인덱스 파일 경로
            *@return bool
            */
            bool exportIndex(const std::string& idxFilePath);

        private:
            /**defineFace
            *@brief 정점 인덱스들로 face를 정의
            *@param faceIndex face 인덱스
            *@param vertexIndices face를 구성하는 정점 인덱스들
            *@return bool
            */
            bool defineFace(const unsigned int faceIndex, const std::vector<unsigned int>& vertexIndices);

            /**computeCentroid
            *@param vertices : face를 구성하는 정점들
            *@return Vertex3D : 중심점(centroid)의 3차원 좌표
            */
            Vertex3D computeCentroid(const std::vector<Vertex3D>& vertices);

            /**checkDist2Center
            *@param vertices : face를 구성하는 정점들
            *@param centroid : 다각형의 중심점
            *@return bool : [false] face를 정의하는 정점들 중 너무 긴 변(>maxSide)이 있음
            */
            bool checkDist2Center(const std::vector<Vertex3D>& vertices, const Vertex3D& centroid);

            /**defineSubTriangles
            *@brief 하위 다각형(삼각형) 정의
            *@param vertices : face를 구성하는 정점들
            *@param centroid : 다각형의 중심점
            */
            std::vector< std::vector<Vertex3D>> defineSubTriangles(const std::vector<Vertex3D>& vertices, const Vertex3D& centroid);

            /**insertPseudoPoint: 중심점을 이용해 face 내부에 가상점 생성
            *@param vertexIndices : face를 구성하는 정점 인덱스들
            *@param faceIndex : face 인덱스
            */
            void insertPseudoPoint(const std::vector<unsigned int>& vertexIndices, const unsigned int faceIndex);

            /**insertPseudoPointMid: 중심점을 이용해 face 내부에 가상점 생성
            *@param vertexIndices : face를 구성하는 정점 인덱스들
            *@param faceIndex : face 인덱스
            */
            void insertPseudoPointMid(const std::vector<unsigned int>& vertexIndices, const unsigned int faceIndex);

            /**addMidPts: 주어진 선분에 중간점 추가
            *@param p0 : 선분의 시작점
            *@param p1 : 선분의 끝점
            *@param maxLength : 최대 길이 임계값
            *@param pseudoPts : 가상점(midPts)을 담을 버킷
            *@return bool: true면 선분이 김, false면 짧음
            */
            bool addMidPts(const Vertex3D& p0, const Vertex3D& p1, const double maxLength, std::vector<Vertex3D>& pseudoPts);

            /**divideTriangle: 삼각형을 세 개의 하위 삼각형으로 분할
            *@param triVtx : 세 정점
            *@param maxLength : 최대 길이 임계값
            *@param pseudoPts : 가상점(midPts)을 담을 버킷
            *@param centroid : 삼각형의 중심
            *@return bool: true면 삼각형이 큼, false면 작음
            */
            bool divideTriangle(const std::vector<Vertex3D>& triVtx, const double maxLength, std::vector<Vertex3D>& pseudoPts, Vertex3D& centroid);

            /**divideFace: face(다각형) 분할
            *@param faceVtx : face 정점들
            *@param maxLength : 최대 길이 임계값
            *@param pseudoPts : 가상점(midPts)을 담을 버킷
            */
            void divideFace(const std::vector<Vertex3D>& faceVtx, const double maxLength, std::vector<Vertex3D>& pseudoPts);

        private:
            MeshDataType meshData;
            std::vector<Vertex3D> pseudoPts;
            std::shared_ptr<KDTree> tree;
            double maxDist;
            PSEUDOTYPE pseudoType;
        };

        SSMFlann::SSMFlann(const double maxLength, const PSEUDOTYPE pType)
        {
            this->maxDist = maxLength;
            pseudoType = pType;
        };

        void SSMFlann::setNumberOfPoints(const unsigned int size)
        {
            meshData.vertices.resize(size);
        }

        bool SSMFlann::setVertex(const unsigned int index, const double x, const double y, const double z)
        {
            if (!(index < meshData.vertices.size()))
                return false;

            try
            {
                meshData.vertices[index].set(x, y, z);
                return true;
            }
            catch (...)
            {
                std::cerr << "Error in setVertex of SSMFlann" << std::endl;
                return false;
            }
        }

        void SSMFlann::setNumberOfFaces(const unsigned int size)
        {
            meshData.faces.resize(size);
            meshData.fNormals.resize(size);
        }

        /**setPseudoType
        *@brief 가상점 생성 방식 설정
        *@param pType: 타입
        */
        void SSMFlann::setPseudoType(const PSEUDOTYPE pType)
        {
            pseudoType = pType;
        }

        bool SSMFlann::setFace(const unsigned int faceIndex, const std::vector<unsigned int>& vtxIndex)
        {
            if (!(faceIndex < meshData.faces.size()))
            {
                std::cerr << "Error in setFace of SSMFlann: check the index of a face" << std::endl;
                return false;
            }

            if (vtxIndex.size() < 3)
            {
                std::cerr << "Error in setFace of SSMFlann: check the number of vertices consisting a face (should be greater than 2)" << std::endl;
                return false;
            }

            try
            {
                meshData.faces[faceIndex] = vtxIndex;
                return defineFace(faceIndex, vtxIndex);
            }
            catch (...)
            {
                std::cerr << "Error in setFace of SSMFlann" << std::endl;
                return false;
            }
        }

        bool SSMFlann::setFaceNormal(const unsigned int faceIndex, const double fnx, const double fny, const double fnz)
        {
            if (!(faceIndex < meshData.fNormals.size()))
            {
                std::cerr << "Error in setFaceNormal of SSMFlann: check the index of a face-normal" << std::endl;
                return false;
            }

            try
            {
                meshData.fNormals[faceIndex][0] = fnx;
                meshData.fNormals[faceIndex][1] = fny;
                meshData.fNormals[faceIndex][2] = fnz;
            }
            catch (...)
            {
                std::cerr << "Error in setFaceNormal of SSMFlann" << std::endl;
                return false;
            }

            return true;
        }

        Vertex3D SSMFlann::getFaceNormal(const unsigned int faceIndex) const
        {
            try
            {
                return this->meshData.fNormals[faceIndex];
            }
            catch (...)
            {

                std::cerr << "Error in getting a face normal vector" << std::endl;
                Vertex3D temp;
                return temp;
            }
        }

        void SSMFlann::clearVertexData()
        {
            meshData.vertices.clear();
        }

        bool SSMFlann::defineFace(const unsigned int faceIndex, const std::vector<unsigned int>& vertexIndices)
        {
            try
            {
                for (const unsigned int index : vertexIndices)
                {
                    meshData.vertices[index].addFaceId(faceIndex);
                }

                /// face를 하위 삼각형들로 분할
                switch (this->pseudoType)
                {
                case PSEUDOTYPE::CENTROID:
                    insertPseudoPoint(vertexIndices, faceIndex);
                    break;
                case PSEUDOTYPE::MIDPOINT:
                    insertPseudoPointMid(vertexIndices, faceIndex);
                    break;
                default:
                    break;
                }

                return true;
            }
            catch (...)
            {

                std::cerr << "Error in defineFace of SSMFlann" << std::endl;
                return false;
            }
        }

        void SSMFlann::buildTree(const unsigned int maxLeaf)
        {
            try {
                /// 추가 가상점들을 더함
                size_t totalSize = meshData.vertices.size() + pseudoPts.size();
                meshData.vertices.reserve(totalSize);
                meshData.vertices.insert(meshData.vertices.end(), pseudoPts.begin(), pseudoPts.end());

                /// maxLeaf가 클수록 탐색은 빨라지지만 트리 구축에 시간이 오래 걸림
                size_t dimension = 3;
                tree.reset(new KDTree(dimension, meshData, nanoflann::KDTreeSingleIndexAdaptorParams(maxLeaf)));
                tree->buildIndex();
            }
            catch (...) {
                std::cerr << "Error in buildTree of SSMFlann" << std::endl;
            }
        }

        bool SSMFlann::buildTree(const std::string& idxFilePath, const unsigned int maxLeaf)
        {
            try {
                /// 추가 가상점들을 더함
                meshData.vertices.insert(meshData.vertices.end(), pseudoPts.begin(), pseudoPts.end());

                /// maxLeaf가 클수록 탐색은 빨라지지만 트리 구축에 시간이 오래 걸림
                size_t dimension = 3;
                tree.reset(new KDTree(dimension, meshData, nanoflann::KDTreeSingleIndexAdaptorParams(maxLeaf)));

                FILE* idxFile = fopen(idxFilePath.c_str(), "rb");
                if (!idxFile) return false;
                tree->loadIndex(idxFile);
                fclose(idxFile);
            }
            catch (...) {
                std::cerr << "Error in buildTree(import index info from a file) of SSMFlann" << std::endl;
            }

            return true;
        }

        bool SSMFlann::search(const double x0, const double y0, const double z0,
            std::vector<double>& distances,
            std::vector<std::size_t>& indices,
            const unsigned int numClosest,
            const double distThreshold) const
        {
            MeshDataType::CoordValueType searchPt[] = { x0, y0, z0 };
            indices.resize(numClosest, 0);
            distances.resize(numClosest);

            std::vector<double> squareDist(numClosest);
            size_t numFound = tree->knnSearch(searchPt, numClosest, indices.data(), squareDist.data());

            for (size_t i = 0; i < numFound; ++i) {
                distances[i] = std::sqrt(squareDist[i]);
            }

            if (numFound < numClosest) {
                indices.resize(numFound);
                distances.resize(numFound);
            }

            if (distThreshold == 0.0)
                return true;

            const double thresholdSq = distThreshold * distThreshold;
            return (squareDist[0] < thresholdSq);
        }

        /**raytrace
        *@brief 주어진 광선(ray)을 따라 가장 가까운 점을 찾음
        *@param sx: 광선 시작점의 x 좌표
        *@param sy: 광선 시작점의 y 좌표
        *@param sz: 광선 시작점의 z 좌표
        *@param ex: 광선 끝점의 x 좌표
        *@param ey: 광선 끝점의 y 좌표
        *@param ez: 광선 끝점의 z 좌표
        *@param radius: 탐색 반경
        *@param foundPts: 점 인덱스와 거리, 결과값
        *@return bool
        */
        bool SSMFlann::raytrace(const double sx, const double sy, const double sz,
            const double ex, const double ey, const double ez,
            const double radius,
            std::vector<std::pair<size_t, double>>& foundPts)
        {
            double dx = ex - sx;
            double dy = ey - sy;
            double dz = ez - sz;
            double maxDist = sqrt(dx * dx + dy * dy + dz * dz);
            if (maxDist < std::numeric_limits<double>::epsilon())
                return false;
            double nx = dx / maxDist;
            double ny = dy / maxDist;
            double nz = dz / maxDist;

            unsigned int count = 0;
            do
            {
                ++count;
                double fwdDist = radius * count;

                if (fwdDist > maxDist)
                    break;

                MeshDataType::CoordValueType searchPt[] = { sx + nx * fwdDist, sy + ny * fwdDist, sz + nz * fwdDist };
                nanoflann::SearchParams params;
                /// 반경 내에서 가장 가까운 점 탐색
                const size_t nMatches = tree->radiusSearch(&searchPt[0], radius, foundPts, params);
                if (nMatches > 0)
                    return true;

            } while (1);

            return false;
        }

        Vertex3D SSMFlann::getVertex(const std::size_t vtxIndex) const
        {
            return meshData.vertices[vtxIndex];
        }

        /**computeCentroid
        *@param vertices : face를 구성하는 정점들
        *@return Vertex3D : 중심점(centroid)의 3차원 좌표
        */
        Vertex3D SSMFlann::computeCentroid(const std::vector<Vertex3D>& vertices)
        {
            double sumX = 0.0;
            double sumY = 0.0;
            double sumZ = 0.0;
            for (const auto& vtx : vertices)
            {
                sumX += vtx.x();
                sumY += vtx.y();
                sumZ += vtx.z();
            }

            return Vertex3D(sumX / vertices.size(), sumY / vertices.size(), sumZ / vertices.size());
        }

        /**checkDist2Center
        *@param vertices : face를 구성하는 정점들
        *@param centroid : 다각형의 중심점
        *@return bool : [false] face를 정의하는 정점들 중 너무 긴 변(>maxSide)이 있음
        */
        bool SSMFlann::checkDist2Center(const std::vector<Vertex3D>& vertices, const Vertex3D& centroid)
        {
            const double maxDistSq = maxDist * maxDist;
            for (unsigned int i = 0; i < vertices.size(); ++i)
            {
                auto dx = vertices[i].x() - centroid.x();
                auto dy = vertices[i].y() - centroid.y();
                auto dz = vertices[i].z() - centroid.z();
                auto distSq = dx * dx + dy * dy + dz * dz;
                if (distSq < maxDistSq) return true;
            }

            return false;
        }

        /**defineSubTriangles
        *@brief 하위 다각형(삼각형) 정의
        *@param vertices : face를 구성하는 정점들
        *@param centroid : 다각형의 중심점
        */
        std::vector< std::vector<Vertex3D>> SSMFlann::defineSubTriangles(const std::vector<Vertex3D>& vertices, const Vertex3D& centroid)
        {
            std::vector< std::vector<Vertex3D>> triangles(vertices.size());

            for (unsigned int i = 0; i < vertices.size(); ++i)
            {
                unsigned int j = i + 1;
                if (j == vertices.size()) j = 0;

                triangles[i].resize(3);
                triangles[i][0] = centroid;
                triangles[i][1] = vertices[i];
                triangles[i][2] = vertices[j];
            }

            return triangles;
        }

        /**insertPseudoPoint: 중심점을 이용해 face 내부에 가상점 생성
        *@param vertexIndices : face를 구성하는 정점 인덱스들
        *@param faceIndex : face 인덱스
        */
        void SSMFlann::insertPseudoPoint(const std::vector<unsigned int>& vertexIndices, const unsigned int faceIndex)
        {
            if (this->maxDist < std::numeric_limits<double>::epsilon())
                return;

            /// 인덱스들로 정점 수집
            std::vector<std::vector<Vertex3D>> verticesGroup(1);
            verticesGroup[0].reserve(vertexIndices.size());
            for (auto idx : vertexIndices)
                verticesGroup[0].push_back(meshData.vertices[idx]);

            std::vector<std::vector<Vertex3D>> newCollectedVertices;

            /// face 크기를 확인하고 필요하면 중심점을 삽입
            do
            {
                /// 중심점으로 새로 정의되는 다각형들
                newCollectedVertices.clear();

                for (auto vertices : verticesGroup)
                {
                    /// 중심점 계산
                    Vertex3D centroid = computeCentroid(vertices);

                    /// 너무 긴 변이 있는지 확인
                    if (checkDist2Center(vertices, centroid))
                        continue;

                    /// face 인덱스를 부여하고 중심점(pseudoPts)을 저장
                    centroid.addFaceId(faceIndex);
                    pseudoPts.push_back(centroid);

                    /// 새로운 하위 삼각형 정의
                    std::vector<std::vector<Vertex3D>> triangles = defineSubTriangles(vertices, centroid);

                    /// 새로 정의된 삼각형 수집
                    newCollectedVertices.insert(newCollectedVertices.end(), triangles.begin(), triangles.end());
                }

                if (newCollectedVertices.size() < 1)
                    break;
                else
                    verticesGroup = newCollectedVertices;

            } while (1);
        }

        /**insertPseudoPointMid: 가상점 생성
        *@param vertexIndices : face를 구성하는 정점 인덱스들
        *@param faceIndex : face 인덱스
        */
        void SSMFlann::insertPseudoPointMid(const std::vector<unsigned int>& vertexIndices, const unsigned int faceIndex)
        {
            if (this->maxDist < std::numeric_limits<double>::epsilon())
                return;

            /// face를 위한 정점 수집
            std::vector<Vertex3D> faceVtx;
            faceVtx.reserve(vertexIndices.size());
            for (auto idx : vertexIndices)
                faceVtx.push_back(meshData.vertices[idx]);

            /// face를 하위 삼각형들로 나누어 가상점 생성
            std::vector<Vertex3D> midPts;
            midPts.reserve(vertexIndices.size() * 10);
            divideFace(faceVtx, this->maxDist, midPts);
            midPts.shrink_to_fit();
            for (auto& pt : midPts)
                pt.addFaceId(faceIndex);
            pseudoPts.insert(pseudoPts.end(), midPts.begin(), midPts.end());
        }

        /**addMidPts: 주어진 선분에 중간점 추가
        *@param p0 : 선분의 시작점
        *@param p1 : 선분의 끝점
        *@param maxLength : 최대 길이 임계값
        *@param pseudoPts : 가상점(midPts)을 담을 버킷
        *@param faceId: face Id
        *@return bool: true면 선분이 김, false면 짧음
        */
        bool SSMFlann::addMidPts(const Vertex3D& p0, const Vertex3D& p1, const double maxLength, std::vector<Vertex3D>& pseudoPts)
        {
            std::vector<Vertex3D> addedPts;

            /// 거리 확인
            Vertex3D v = p1 - p0;
            auto distSq = v.x() * v.x() + v.y() * v.y() + v.z() * v.z();
            auto maxLengthSq = maxLength * maxLength;

            if (distSq > maxLengthSq)
            {
                auto dist = std::sqrt(distSq);
                Vertex3D dir = v / dist;
                unsigned int num = static_cast<int>(dist / maxLength + 1.0);
                double dLength = dist / static_cast<double>(num);
                addedPts.reserve(num - 1);
                for (unsigned int i = 1; i < num; ++i)
                {
                    Vertex3D tempPt = dir * (dLength * static_cast<double>(i)) + p0;
                    addedPts.push_back(tempPt);
                }
            }

            if (addedPts.size() > 0)
            {
                pseudoPts.insert(pseudoPts.end(), addedPts.begin(), addedPts.end());
                return true;
            }
            else
                return false;
        }

        /**divideTriangle: 삼각형을 세 개의 하위 삼각형으로 분할
        *@param triVtx : 세 정점
        *@param maxLength : 최대 길이 임계값
        *@param pseudoPts : 가상점(midPts)을 담을 버킷
        *@param centroid : 삼각형의 중심
        *@return bool: true면 삼각형이 큼, false면 작음
        */
        bool SSMFlann::divideTriangle(const std::vector<Vertex3D>& triVtx, const double maxLength, std::vector<Vertex3D>& pseudoPts, Vertex3D& centroid)
        {
            /// 중심점 계산
            centroid = Vertex3D((triVtx[0].x() + triVtx[1].x() + triVtx[2].x()) / 3.,
                (triVtx[0].y() + triVtx[1].y() + triVtx[2].y()) / 3.,
                (triVtx[0].z() + triVtx[1].z() + triVtx[2].z()) / 3.);

            std::vector<Vertex3D> midPts;
            bool longSide = true;

            /// 중간점
            if (!addMidPts(triVtx[0], centroid, maxLength, midPts))
                longSide = false;
            if (!addMidPts(triVtx[1], centroid, maxLength, midPts))
                longSide = false;
            if (!addMidPts(triVtx[2], centroid, maxLength, midPts))
                longSide = false;

            pseudoPts.push_back(centroid);
            pseudoPts.insert(pseudoPts.end(), midPts.begin(), midPts.end());

            return longSide;
        }

        /**divideFace: face(다각형) 분할
        *@param faceVtx : face 정점들
        *@param maxLength : 최대 길이 임계값
        *@param pseudoPts : 가상점(midPts)을 담을 버킷
        */
        void SSMFlann::divideFace(const std::vector<Vertex3D>& faceVtx, const double maxLength, std::vector<Vertex3D>& pseudoPts)
        {
            if (faceVtx.size() < 3)
                return;

            Vertex3D centroid; /// 중심점

            /// 변(side) 분할
            for (unsigned int i = 0; i < faceVtx.size(); ++i)
            {
                unsigned int j;
                if (i == faceVtx.size() - 1) j = 0;
                else j = i + 1;

                /// 내부에 가상점 추가
                addMidPts(faceVtx[i], faceVtx[j], maxLength, pseudoPts);

                centroid += faceVtx[i];
            }

            if (pseudoPts.size() < 1)
                return;

            centroid /= static_cast<double>(faceVtx.size());

            bool bigSize = false;

            for (unsigned int i = 0; i < faceVtx.size(); ++i)
            {
                auto dP = centroid - faceVtx[i];
                auto dist = sqrt(dP.x() * dP.y() + dP.x() * dP.y() + dP.z() * dP.z());
                if (dist > maxLength)
                {
                    pseudoPts.push_back(centroid);
                    bigSize = true;
                    break;
                }
            }

            if (!bigSize)
                return;

            /// face로부터의 첫 하위 삼각형들
            std::vector<std::vector<Vertex3D>> subTri(faceVtx.size());

            for (unsigned int i = 0; i < faceVtx.size(); ++i)
            {
                /// 내부에 가상점 추가
                addMidPts(centroid, faceVtx[i], maxLength, pseudoPts);

                /// 변에 가상점 추가
                Vertex3D side;
                unsigned int j;
                if (i == faceVtx.size() - 1) j = 0;
                else j = i + 1;

                std::vector<Vertex3D> tri(3);
                tri[0] = faceVtx[i];
                tri[1] = faceVtx[j];
                tri[2] = centroid;

                subTri[i] = tri;
            }

            /// 하위 삼각형 확인
            unsigned int count = 0;
            do
            {
                ++count;

                auto subTriCopy = subTri;
                subTri.clear();

                for (const auto& sub : subTriCopy)
                {
                    Vertex3D triCenter;
                    if (divideTriangle(sub, maxLength, pseudoPts, triCenter))
                    {
                        std::vector<Vertex3D> dividedTri(3);
                        dividedTri[0] = sub[0];
                        dividedTri[1] = sub[1];
                        dividedTri[2] = triCenter;
                        subTri.push_back(dividedTri);
                        dividedTri[0] = sub[1];
                        dividedTri[1] = sub[2];
                        dividedTri[2] = triCenter;
                        subTri.push_back(dividedTri);
                        dividedTri[0] = sub[2];
                        dividedTri[1] = sub[0];
                        dividedTri[2] = triCenter;
                        subTri.push_back(dividedTri);
                    }
                }

                if (subTri.size() < 1)
                    break;

            } while (1);
        }

        /**exportIndex
        */
        bool SSMFlann::exportIndex(const std::string& idxFilePath)
        {
            if (!tree)
                return false;

            FILE* idxFile = fopen(idxFilePath.c_str(), "wb");
            if (!idxFile) return false;

            tree->saveIndex(idxFile);

            fclose(idxFile);

            return true;
        }

        bool loadGltf(const std::string& bim_folder, std::vector<BimMeshInfo>& bimData)
        {
            std::filesystem::path input_path(bim_folder);

            // 경로 검증
            if (!std::filesystem::exists(input_path)) {
                std::cout << "input_folder '" << bim_folder << "' does not exist" << std::endl;
                return false;
            }

            // 진행 상황 추적
            size_t total_gltf_files = std::distance(
                fs::directory_iterator(input_path),
                fs::directory_iterator()
            );
            int processed_gltf_files = 0;
            int fallback_idx = 1000; // Revit Element ID가 없을 때 사용할 기본 ID

            // 디렉토리 순회
            for (const auto& entry : std::filesystem::directory_iterator(input_path))
            {
                // 디버그 출력
                std::wcout << entry.path().wstring() << std::endl;
                std::cout << entry.path().string() << std::endl; // 영문 Windows에서는 주석 처리

                // 일반 파일 확인
                if (!entry.is_regular_file()) {
                    std::wcout << entry.path().wstring() << " not regular file" << std::endl;
                    continue;
                }

                // GLTF/GLB 파일 확인
                auto& data_file = entry.path();
                if (data_file.extension() != ".gltf" && data_file.extension() != ".glb") {
                    if (data_file.extension() != ".bin") {
                        std::cout << data_file << " not gltf(glb) file" << std::endl;
                    }
                    continue;
                }

                // UTF-16을 UTF-8로 변환
                const std::string utf8FileName = util::wstring_to_utf8(data_file);

                // GLTF 파일 읽기
                bimData.push_back(BimMeshInfo());
                BimMeshInfo& currentGltf = bimData.back();

                bool is_offset_applied = false;
                float offset[3] = { 0.f, 0.f, 0.f };

                GltfMesh gltf_file;
                /// gltf 읽기
                if (!gltf_file.read(utf8FileName, currentGltf, is_offset_applied, offset)) {
                    std::cerr << "Failed to read GLTF file: " << utf8FileName << std::endl;
                    continue;
                }

                mesh::extractNodeInfo(utf8FileName, currentGltf.node_name, currentGltf.bim_id, fallback_idx);

                /// 바운딩 박스
                for (auto& min : currentGltf.bbox_min)
                    min = (std::numeric_limits<float>::max)();
                for (auto& max : currentGltf.bbox_max)
                    max = std::numeric_limits<float>::lowest();

                mesh::GeometryInfo geometry = mesh::processGeometryAndBoundingBox(currentGltf, currentGltf.bbox_min, currentGltf.bbox_max);

                /// 진행 상황 출력
                processed_gltf_files++;
                int progress = (int)((float)processed_gltf_files / total_gltf_files * 100);
                if (processed_gltf_files % 100 == 0 || progress == 100) {
                    std::cout << "Processed " << processed_gltf_files
                        << " / " << total_gltf_files
                        << " (" << progress << "%)" << std::endl;
                }
            }

            return true;
        }
    }

    /// BIM–포인트클라우드 처리 함수들
    namespace BimPcProcessors {

        ///
        /// @brief BIM–포인트클라우드 처리 함수
        ///
        /// @param task 처리할 태스크 정보
        /// @param chunk BimPcChunk 데이터 (포인트클라우드 + 메시)
        /// @return BimPcResult 처리 결과
        ///
        /// 점이 BIM 메시 표면에 "공차 이내(within tolerance)"로 분류되는 기준이 되는
        /// 거리 임계값(미터). 일반적인 시공 품질검사(QC) 공차값으로 선택되었다.
        /// 현재는 ProcessingTask::parameters를 통해 노출되지 않는다 -- 태스크별로
        /// 설정 가능하게 만들 필요가 있다면, icp::IcpChunk에 IcpConfig가 전달되는
        /// 방식과 동일하게 구현하면 된다.
        constexpr double kBimPcWithinThresholdMeters = 0.05;

        /// 최근접 이웃 탐색을 위해 메시 표면을 가상점으로 샘플링할 때 사용하는
        /// 격자 간격(미터). ICP 정합에서 사용하는 기본값과 일치한다
        /// (include/IcpTypes.h의 IcpConfig::pseudoPointGridSize 참고).
        constexpr double kBimPcPseudoPointGridSize = 0.1;

        BimPcResult processBimPc(const ProcessingTask& task, const BimPcChunk& chunk) {
            BimPcResult result;
            result.task_id = task.task_id;
            result.chunk_id = chunk.chunk_id;

            auto start_time = std::chrono::steady_clock::now();

            try {
                std::cout << "=== BimPcProcessors::processBimPc ===" << std::endl;
                std::cout << "Task ID: " << task.task_id << std::endl;
                std::cout << "Chunk ID: " << chunk.chunk_id << std::endl;
                std::cout << "Points: " << chunk.points.size() << std::endl;
                std::cout << "Mesh faces: " << chunk.bim.faces.size() << std::endl;

                result.total_points_processed = static_cast<uint32_t>(chunk.points.size());
                result.total_faces_processed = static_cast<uint32_t>(chunk.bim.faces.size());

                /// chunk.points는 (offsetX, offsetY, offsetZ)만큼 이동된 float로 저장되어
                /// 있다 -- 여기서 딱 한 번 double로 승격시키고 offset을 다시 더해서, 이
                /// 함수의 나머지 부분이 이 offset-재배치 최적화가 도입되기 전과 완전히
                /// 동일하게 chunk.bim(항상 절대좌표, double)과 비교하도록 한다.
                std::vector<std::array<double, 3>> pointsAbsolute;
                pointsAbsolute.reserve(chunk.points.size());
                for (const auto& p : chunk.points) {
                    pointsAbsolute.push_back({
                        static_cast<double>(p[0]) + chunk.offsetX,
                        static_cast<double>(p[1]) + chunk.offsetY,
                        static_cast<double>(p[2]) + chunk.offsetZ });
                }

                if (chunk.points.empty() || chunk.bim.faces.empty()) {
                    result.min_distance = 0.0;
                    result.max_distance = 0.0;
                    result.avg_distance = 0.0;
                    result.std_deviation = 0.0;
                    result.points_within_threshold = 0;
                    result.points_outside_threshold = 0;
                    result.success = true;
                    std::cout << "No points or faces to process; returning empty result." << std::endl;
                }
                else {
                    /// 1. 메시 표면을 가상점으로 샘플링 (ICP 정합에서 사용하는 것과 동일한
                    /// 격자 샘플링 방식; PseudoPointGenerator.hpp 참고).
                    std::vector<std::array<double, 3>> pseudoPoints;
                    std::vector<size_t> faceIdx;
                    std::vector<std::array<double, 3>> facePts;
                    std::vector<std::array<double, 3>> faceNormals;
                    icp::generatePseudoPointsFromMesh(chunk.bim, kBimPcPseudoPointGridSize,
                        pseudoPoints, faceIdx, facePts, faceNormals);

                    if (pseudoPoints.empty()) {
                        throw std::runtime_error(
                            "No pseudo points could be generated from the mesh (degenerate faces?)");
                    }

                    /// 2. 빠른 최근접 이웃 탐색을 위해 가상점들 위에 KD-Tree를 구축.
                    icp::PointCloudAdaptor adaptor(pseudoPoints);
                    icp::KdTree3D tree(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(50));
                    tree.buildIndex();

                    /// 3. 입력된 각 점에 대해, 가상점 위의 KD-Tree를 이용해 후보 face
                    /// 몇 개(그 근처 가상점들이 유래한 face들)로 범위를 좁힌 뒤, 각 후보에
                    /// 대해 *정확한* point-to-triangle 거리를 계산하고 그 중 최솟값을
                    /// 취한다. 이렇게 하면 (샘플링된 가상점 중 가장 가까운 것까지의 거리가
                    /// 아니라) 실제 표면까지의 참(true) 거리를 보고하게 된다 -- 가장 가까운
                    /// 가상점만 사용하면 격자 간격(kBimPcPseudoPointGridSize) 정도만큼
                    /// 거리에 편차가 생길 수 있는데, 이는 바로 아래의 QC 임계값 판정에서
                    /// 중요하게 작용한다.
                    constexpr size_t kCandidateNeighbors = 8;
                    const size_t numCandidates = (std::min)(kCandidateNeighbors, pseudoPoints.size());

                    const size_t n = chunk.points.size();
                    result.point_distances.resize(n);
                    result.nearest_face_ids.resize(n);

                    double sum = 0.0;
                    double sumSq = 0.0;
                    double minDist = (std::numeric_limits<double>::max)();
                    double maxDist = 0.0;
                    uint32_t within = 0;

                    std::vector<size_t> candidateIdx(numCandidates);
                    std::vector<double> candidateDistSq(numCandidates);

                    for (size_t i = 0; i < n; ++i) {
                        const std::array<double, 3>& query = pointsAbsolute[i];

                        nanoflann::KNNResultSet<double> resultSet(numCandidates);
                        resultSet.init(candidateIdx.data(), candidateDistSq.data());
                        tree.findNeighbors(resultSet, query.data(), nanoflann::SearchParams());

                        double bestDistSq = (std::numeric_limits<double>::max)();
                        size_t bestFace = faceIdx[candidateIdx[0]];
                        size_t lastFace = static_cast<size_t>(-1);

                        for (size_t k = 0; k < numCandidates; ++k) {
                            size_t face = faceIdx[candidateIdx[k]];
                            if (face == lastFace) continue; /// 후보들이 같은 face에 속하는 경우가 많으므로 중복은 건너뜀
                            lastFace = face;

                            const auto& f = chunk.bim.faces[face];
                            std::array<double, 3> a = { f.vertices[0][0], f.vertices[0][1], f.vertices[0][2] };
                            std::array<double, 3> b = { f.vertices[1][0], f.vertices[1][1], f.vertices[1][2] };
                            std::array<double, 3> c = { f.vertices[2][0], f.vertices[2][1], f.vertices[2][2] };

                            double distSq = icp::pointToTriangleDistanceSquared(query, a, b, c);
                            if (distSq < bestDistSq) {
                                bestDistSq = distSq;
                                bestFace = face;
                            }
                        }

                        double dist = std::sqrt(bestDistSq);
                        result.point_distances[i] = dist;
                        result.nearest_face_ids[i] = static_cast<int32_t>(bestFace);

                        sum += dist;
                        sumSq += dist * dist;
                        minDist = (std::min)(minDist, dist);
                        maxDist = (std::max)(maxDist, dist);
                        if (dist <= kBimPcWithinThresholdMeters) ++within;
                    }

                    double mean = sum / static_cast<double>(n);
                    double variance = (sumSq / static_cast<double>(n)) - (mean * mean);

                    result.min_distance = minDist;
                    result.max_distance = maxDist;
                    result.avg_distance = mean;
                    result.std_deviation = std::sqrt((std::max)(variance, 0.0));
                    result.points_within_threshold = within;
                    result.points_outside_threshold = static_cast<uint32_t>(n) - within;

                    result.success = true;

                    std::cout << "Processing completed: avg_distance=" << result.avg_distance
                        << "m, min=" << result.min_distance << "m, max=" << result.max_distance
                        << "m, within " << kBimPcWithinThresholdMeters << "m: "
                        << within << "/" << n << std::endl;
                }

                std::cout << "=======================================" << std::endl;
            }
            catch (const std::exception& e) {
                result.success = false;
                result.error_message =
                    "BimPc processing error: " + std::string(e.what());
                std::cerr << "Error: " << result.error_message << std::endl;
            }

            auto end_time = std::chrono::steady_clock::now();
            result.processing_time_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    end_time - start_time).count();

            return result;
        }
    } /// namespace BimPcProcessors
} /// namespace DPApp