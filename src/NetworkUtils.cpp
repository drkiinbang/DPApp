#include "../include/NetworkManager.h"
#include <cstring>
#include <sstream>

namespace DPApp {
namespace NetworkUtils {

std::vector<uint8_t> serializeTask(const ProcessingTask& task) {
    std::vector<uint8_t> data;
    
    // 헤더: task_id, chunk_id, task_type 길이, 파라미터 길이
    data.resize(sizeof(uint32_t) * 3 + sizeof(size_t) * 2);
    size_t offset = 0;
    
    // task_id
    std::memcpy(data.data() + offset, &task.task_id, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // chunk_id
    std::memcpy(data.data() + offset, &task.chunk_id, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // task_type 길이
    size_t task_type_len = task.task_type.length();
    std::memcpy(data.data() + offset, &task_type_len, sizeof(size_t));
    offset += sizeof(size_t);
    
    // 파라미터 길이
    size_t param_len = task.parameters.size();
    std::memcpy(data.data() + offset, &param_len, sizeof(size_t));
    offset += sizeof(size_t);
    
    // task_type 문자열
    data.insert(data.end(), task.task_type.begin(), task.task_type.end());
    
    // 파라미터 데이터
    data.insert(data.end(), task.parameters.begin(), task.parameters.end());
    
    return data;
}

ProcessingTask deserializeTask(const std::vector<uint8_t>& data) {
    if (data.size() < sizeof(uint32_t) * 2 + sizeof(size_t) * 2) {
        throw std::runtime_error("Invalid task data size");
    }
    
    ProcessingTask task;
    size_t offset = 0;
    
    // task_id
    std::memcpy(&task.task_id, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // chunk_id
    std::memcpy(&task.chunk_id, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // task_type 길이
    size_t task_type_len;
    std::memcpy(&task_type_len, data.data() + offset, sizeof(size_t));
    offset += sizeof(size_t);
    
    // 파라미터 길이
    size_t param_len;
    std::memcpy(&param_len, data.data() + offset, sizeof(size_t));
    offset += sizeof(size_t);
    
    // 데이터 크기 검증
    if (offset + task_type_len + param_len > data.size()) {
        throw std::runtime_error("Invalid task data format");
    }
    
    // task_type
    task.task_type = std::string(reinterpret_cast<const char*>(data.data() + offset), task_type_len);
    offset += task_type_len;
    
    // 파라미터
    task.parameters.assign(data.begin() + offset, data.begin() + offset + param_len);
    
    return task;
}

std::vector<uint8_t> serializeResult(const ProcessingResult& result) {
    std::vector<uint8_t> data;
    
    // 헤더 크기 계산
    size_t header_size = sizeof(uint32_t) * 2 +  // task_id, chunk_id
                        sizeof(bool) +            // success
                        sizeof(size_t) * 3;       // error_message_len, points_len, result_data_len
    
    data.resize(header_size);
    size_t offset = 0;
    
    // task_id
    std::memcpy(data.data() + offset, &result.task_id, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // chunk_id
    std::memcpy(data.data() + offset, &result.chunk_id, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // success
    std::memcpy(data.data() + offset, &result.success, sizeof(bool));
    offset += sizeof(bool);
    
    // 길이 정보들
    size_t error_msg_len = result.error_message.length();
    size_t points_len = result.processed_points.size();
    size_t result_data_len = result.result_data.size();
    
    std::memcpy(data.data() + offset, &error_msg_len, sizeof(size_t));
    offset += sizeof(size_t);
    
    std::memcpy(data.data() + offset, &points_len, sizeof(size_t));
    offset += sizeof(size_t);
    
    std::memcpy(data.data() + offset, &result_data_len, sizeof(size_t));
    offset += sizeof(size_t);
    
    // error_message
    data.insert(data.end(), result.error_message.begin(), result.error_message.end());
    
    // processed_points
    for (const auto& point : result.processed_points) {
        const uint8_t* point_bytes = reinterpret_cast<const uint8_t*>(&point);
        data.insert(data.end(), point_bytes, point_bytes + sizeof(Point3D));
    }
    
    // result_data
    data.insert(data.end(), result.result_data.begin(), result.result_data.end());
    
    return data;
}

ProcessingResult deserializeResult(const std::vector<uint8_t>& data) {
    size_t header_size = sizeof(uint32_t) * 2 + sizeof(bool) + sizeof(size_t) * 3;
    
    if (data.size() < header_size) {
        throw std::runtime_error("Invalid result data size");
    }
    
    ProcessingResult result;
    size_t offset = 0;
    
    // task_id
    std::memcpy(&result.task_id, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // chunk_id
    std::memcpy(&result.chunk_id, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // success
    std::memcpy(&result.success, data.data() + offset, sizeof(bool));
    offset += sizeof(bool);
    
    // 길이 정보들
    size_t error_msg_len, points_len, result_data_len;
    
    std::memcpy(&error_msg_len, data.data() + offset, sizeof(size_t));
    offset += sizeof(size_t);
    
    std::memcpy(&points_len, data.data() + offset, sizeof(size_t));
    offset += sizeof(size_t);
    
    std::memcpy(&result_data_len, data.data() + offset, sizeof(size_t));
    offset += sizeof(size_t);
    
    // 데이터 크기 검증
    size_t expected_size = header_size + error_msg_len + 
                          (points_len * sizeof(Point3D)) + result_data_len;
    
    if (data.size() != expected_size) {
        throw std::runtime_error("Invalid result data format");
    }
    
    // error_message
    if (error_msg_len > 0) {
        result.error_message = std::string(reinterpret_cast<const char*>(data.data() + offset), error_msg_len);
        offset += error_msg_len;
    }
    
    // processed_points
    result.processed_points.resize(points_len);
    for (size_t i = 0; i < points_len; ++i) {
        std::memcpy(&result.processed_points[i], data.data() + offset, sizeof(Point3D));
        offset += sizeof(Point3D);
    }
    
    // result_data
    if (result_data_len > 0) {
        result.result_data.assign(data.begin() + offset, data.begin() + offset + result_data_len);
    }
    
    return result;
}

std::vector<uint8_t> serializeChunk(const PointCloudChunk& chunk) {
    std::vector<uint8_t> data;
    
    // 헤더 크기 계산
    size_t header_size = sizeof(uint32_t) +       // chunk_id
                        sizeof(size_t) +          // points count
                        sizeof(double) * 6 +      // bounds
                        sizeof(size_t);           // filename length
    
    data.resize(header_size);
    size_t offset = 0;
    
    // chunk_id
    std::memcpy(data.data() + offset, &chunk.chunk_id, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // points count
    size_t points_count = chunk.points.size();
    std::memcpy(data.data() + offset, &points_count, sizeof(size_t));
    offset += sizeof(size_t);
    
    // bounds
    std::memcpy(data.data() + offset, &chunk.min_x, sizeof(double));
    offset += sizeof(double);
    std::memcpy(data.data() + offset, &chunk.min_y, sizeof(double));
    offset += sizeof(double);
    std::memcpy(data.data() + offset, &chunk.min_z, sizeof(double));
    offset += sizeof(double);
    std::memcpy(data.data() + offset, &chunk.max_x, sizeof(double));
    offset += sizeof(double);
    std::memcpy(data.data() + offset, &chunk.max_y, sizeof(double));
    offset += sizeof(double);
    std::memcpy(data.data() + offset, &chunk.max_z, sizeof(double));
    offset += sizeof(double);
    
    // header filename length
    size_t filename_len = chunk.header.filename.length();
    std::memcpy(data.data() + offset, &filename_len, sizeof(size_t));
    offset += sizeof(size_t);
    
    // header filename
    data.insert(data.end(), chunk.header.filename.begin(), chunk.header.filename.end());
    
    // points data
    for (const auto& point : chunk.points) {
        const uint8_t* point_bytes = reinterpret_cast<const uint8_t*>(&point);
        data.insert(data.end(), point_bytes, point_bytes + sizeof(Point3D));
    }
    
    return data;
}

PointCloudChunk deserializeChunk(const std::vector<uint8_t>& data) {
    size_t header_size = sizeof(uint32_t) + sizeof(size_t) + sizeof(double) * 6 + sizeof(size_t);
    
    if (data.size() < header_size) {
        throw std::runtime_error("Invalid chunk data size");
    }
    
    PointCloudChunk chunk;
    size_t offset = 0;
    
    // chunk_id
    std::memcpy(&chunk.chunk_id, data.data() + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // points count
    size_t points_count;
    std::memcpy(&points_count, data.data() + offset, sizeof(size_t));
    offset += sizeof(size_t);
    
    // bounds
    std::memcpy(&chunk.min_x, data.data() + offset, sizeof(double));
    offset += sizeof(double);
    std::memcpy(&chunk.min_y, data.data() + offset, sizeof(double));
    offset += sizeof(double);
    std::memcpy(&chunk.min_z, data.data() + offset, sizeof(double));
    offset += sizeof(double);
    std::memcpy(&chunk.max_x, data.data() + offset, sizeof(double));
    offset += sizeof(double);
    std::memcpy(&chunk.max_y, data.data() + offset, sizeof(double));
    offset += sizeof(double);
    std::memcpy(&chunk.max_z, data.data() + offset, sizeof(double));
    offset += sizeof(double);
    
    // filename length
    size_t filename_len;
    std::memcpy(&filename_len, data.data() + offset, sizeof(size_t));
    offset += sizeof(size_t);
    
    // 데이터 크기 검증
    size_t expected_size = header_size + filename_len + (points_count * sizeof(Point3D));
    if (data.size() != expected_size) {
        throw std::runtime_error("Invalid chunk data format");
    }
    
    // filename
    if (filename_len > 0) {
        chunk.header.filename = std::string(reinterpret_cast<const char*>(data.data() + offset), filename_len);
        offset += filename_len;
    }
    
    // points
    chunk.points.resize(points_count);
    for (size_t i = 0; i < points_count; ++i) {
        std::memcpy(&chunk.points[i], data.data() + offset, sizeof(Point3D));
        offset += sizeof(Point3D);
    }
    
    return chunk;
}

} // namespace NetworkUtils
} // namespace DPApp