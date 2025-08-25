#include "TaskManager.h"
#include <iostream>
#include <chrono>
#include <random>
#include <algorithm>
#include <cmath>

namespace DPApp {

// TaskProcessor 구현
TaskProcessor::TaskProcessor() 
    : processed_task_count_(0), failed_task_count_(0), avg_processing_time_(0.0) {
    
    // 기본 프로세서들 등록
    registerProcessor("filter", PointCloudProcessors::filterPoints);
    registerProcessor("classify", PointCloudProcessors::classifyPoints);
    registerProcessor("estimate_normals", PointCloudProcessors::estimateNormals);
    registerProcessor("remove_outliers", PointCloudProcessors::removeOutliers);
    registerProcessor("downsample", PointCloudProcessors::downsample);
}

TaskProcessor::~TaskProcessor() {
    std::lock_guard<std::mutex> lock(processors_mutex_);
    processors_.clear();
}

void TaskProcessor::registerProcessor(const std::string& task_type, ProcessorFunction processor) {
    std::lock_guard<std::mutex> lock(processors_mutex_);
    processors_[task_type] = processor;
    std::cout << "Processor registered for task type: " << task_type << std::endl;
}

void TaskProcessor::unregisterProcessor(const std::string& task_type) {
    std::lock_guard<std::mutex> lock(processors_mutex_);
    processors_.erase(task_type);
}

ProcessingResult TaskProcessor::processTask(const ProcessingTask& task, const PointCloudChunk& chunk) {
    auto start_time = std::chrono::steady_clock::now();
    
    ProcessingResult result;
    result.task_id = task.task_id;
    result.chunk_id = task.chunk_id;
    
    try {
        std::lock_guard<std::mutex> lock(processors_mutex_);
        
        auto it = processors_.find(task.task_type);
        if (it == processors_.end()) {
            result.success = false;
            result.error_message = "Unsupported task type: " + task.task_type;
            failed_task_count_++;
            return result;
        }
        
        // 프로세서 실행
        result = it->second(task, chunk);
        
        if (result.success) {
            processed_task_count_++;
        } else {
            failed_task_count_++;
        }
        
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = "Processing error: " + std::string(e.what());
        failed_task_count_++;
    }
    
    // 처리 시간 업데이트
    auto end_time = std::chrono::steady_clock::now();
    auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time).count() / 1000.0;
    
    uint32_t total_tasks = processed_task_count_ + failed_task_count_;
    if (total_tasks > 0) {
        avg_processing_time_ = (avg_processing_time_ * (total_tasks - 1) + processing_time) / total_tasks;
    }
    
    return result;
}

std::vector<std::string> TaskProcessor::getSupportedTaskTypes() {
    std::lock_guard<std::mutex> lock(processors_mutex_);
    
    std::vector<std::string> types;
    for (const auto& [type, processor] : processors_) {
        types.push_back(type);
    }
    
    return types;
}

bool TaskProcessor::supportsTaskType(const std::string& task_type) {
    std::lock_guard<std::mutex> lock(processors_mutex_);
    return processors_.find(task_type) != processors_.end();
}

// 기본 포인트클라우드 처리 함수들 구현
namespace PointCloudProcessors {

ProcessingResult filterPoints(const ProcessingTask& task, const PointCloudChunk& chunk) {
    ProcessingResult result;
    result.task_id = task.task_id;
    result.chunk_id = task.chunk_id;
    result.success = true;
    
    try {
        // 파라미터 파싱 (간단한 예제)
        // 실제로는 더 복잡한 파라미터 구조를 사용할 것
        double min_z = -100.0;
        double max_z = 100.0;
        
        if (task.parameters.size() >= 16) {
            std::memcpy(&min_z, task.parameters.data(), sizeof(double));
            std::memcpy(&max_z, task.parameters.data() + sizeof(double), sizeof(double));
        }
        
        // Z 좌표 필터링
        for (const auto& point : chunk.points) {
            if (point.z >= min_z && point.z <= max_z) {
                result.processed_points.push_back(point);
            }
        }
        
        std::cout << "Filtered " << chunk.points.size() << " points to " 
                  << result.processed_points.size() << " points" << std::endl;
        
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = "Filter error: " + std::string(e.what());
    }
    
    return result;
}

ProcessingResult classifyPoints(const ProcessingTask& task, const PointCloudChunk& chunk) {
    ProcessingResult result;
    result.task_id = task.task_id;
    result.chunk_id = task.chunk_id;
    result.success = true;
    
    try {
        // 간단한 높이 기반 분류 예제
        result.processed_points = chunk.points;
        
        for (auto& point : result.processed_points) {
            if (point.z < 0) {
                point.classification = 2; // Ground
            } else if (point.z < 5) {
                point.classification = 1; // Low vegetation
            } else if (point.z < 20) {
                point.classification = 4; // Medium vegetation
            } else {
                point.classification = 5; // High vegetation
            }
        }
        
        std::cout << "Classified " << result.processed_points.size() << " points" << std::endl;
        
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = "Classification error: " + std::string(e.what());
    }
    
    return result;
}

ProcessingResult estimateNormals(const ProcessingTask& task, const PointCloudChunk& chunk) {
    ProcessingResult result;
    result.task_id = task.task_id;
    result.chunk_id = task.chunk_id;
    result.success = true;
    
    try {
        // 간단한 법선 벡터 추정 (실제로는 더 복잡한 알고리즘 필요)
        result.processed_points = chunk.points;
        
        // 각 포인트에 대해 간단한 법선 벡터 계산
        // 실제로는 k-nearest neighbors나 radius search 사용
        for (size_t i = 0; i < result.processed_points.size(); ++i) {
            // 임시로 Z축을 법선으로 설정
            // 실제로는 주변 점들을 이용한 평면 피팅 필요
            auto& point = result.processed_points[i];
            
            // 법선 정보를 RGB에 저장 (시각화용)
            point.r = 127; // Normal X
            point.g = 127; // Normal Y
            point.b = 255; // Normal Z (위쪽 향함)
        }
        
        std::cout << "Estimated normals for " << result.processed_points.size() << " points" << std::endl;
        
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = "Normal estimation error: " + std::string(e.what());
    }
    
    return result;
}

ProcessingResult removeOutliers(const ProcessingTask& task, const PointCloudChunk& chunk) {
    ProcessingResult result;
    result.task_id = task.task_id;
    result.chunk_id = task.chunk_id;
    result.success = true;
    
    try {
        // 통계적 아웃라이어 제거 (간단한 버전)
        if (chunk.points.empty()) {
            result.success = false;
            result.error_message = "Empty point cloud";
            return result;
        }
        
        // Z 좌표의 평균과 표준편차 계산
        double sum_z = 0.0;
        for (const auto& point : chunk.points) {
            sum_z += point.z;
        }
        double mean_z = sum_z / chunk.points.size();
        
        double sum_sq_diff = 0.0;
        for (const auto& point : chunk.points) {
            double diff = point.z - mean_z;
            sum_sq_diff += diff * diff;
        }
        double std_dev = std::sqrt(sum_sq_diff / chunk.points.size());
        
        // 2 표준편차 범위 내의 점들만 유지
        double threshold = 2.0 * std_dev;
        
        for (const auto& point : chunk.points) {
            if (std::abs(point.z - mean_z) <= threshold) {
                result.processed_points.push_back(point);
            }
        }
        
        std::cout << "Removed outliers: " << chunk.points.size() << " -> " 
                  << result.processed_points.size() << " points" << std::endl;
        
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = "Outlier removal error: " + std::string(e.what());
    }
    
    return result;
}

ProcessingResult downsample(const ProcessingTask& task, const PointCloudChunk& chunk) {
    ProcessingResult result;
    result.task_id = task.task_id;
    result.chunk_id = task.chunk_id;
    result.success = true;
    
    try {
        // 파라미터에서 다운샘플링 비율 읽기
        double sampling_ratio = 0.1; // 기본값: 10%
        
        if (task.parameters.size() >= sizeof(double)) {
            std::memcpy(&sampling_ratio, task.parameters.data(), sizeof(double));
        }
        
        // 무작위 다운샘플링
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        
        for (const auto& point : chunk.points) {
            if (dis(gen) < sampling_ratio) {
                result.processed_points.push_back(point);
            }
        }
        
        std::cout << "Downsampled " << chunk.points.size() << " points to " 
                  << result.processed_points.size() << " points (ratio: " 
                  << sampling_ratio << ")" << std::endl;
        
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = "Downsampling error: " + std::string(e.what());
    }
    
    return result;
}

} // namespace PointCloudProcessors

} // namespace DPApp