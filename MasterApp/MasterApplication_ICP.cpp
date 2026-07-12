/**
 * @file MasterApplication_ICP.cpp
 * @brief ICP REST API 구현
 *
 * 이 파일이 담고 있는 내용:
 * - ICP REST API 라우트 설정
 * - ICP 작업(Job) 관리 (시작, 상태조회, 취소)
 * - Slave로부터 받은 ICP 결과 처리
 */

#include "MasterApplication.h"
#include "../include/MiniJson.h"
#include "../include/bimtree/IcpCore.hpp"
#include "../include/bimtree/IcpSerialization.hpp"
#include "../include/bimtree/PseudoPointGenerator.hpp"
#include "../include/LaslibWriter.hpp"

 /// DLL로부터의 데이터 로딩
#include "../LasImport/LasImport.h"
#include "../BimImport/BimImport.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <iomanip>

/// =========================================
/// ICP REST API 라우트 설정
/// =========================================

void MasterApplication::setupIcpApiRoutes() {
    /// POST /api/icp/start - 새 ICP 작업 시작
    api_server_->POST("/api/icp/start", [this](const HttpRequest& req) {
        return handleIcpStart(req);
        });

    /// GET /api/icp/jobs - 전체 ICP 작업 목록 조회
    api_server_->GET("/api/icp/jobs", [this](const HttpRequest& req) {
        return handleIcpJobs(req);
        });

    /// GET /api/icp/jobs/{id} - 작업 상태 조회
    api_server_->GET("/api/icp/jobs/{id}", [this](const HttpRequest& req) {
        return handleIcpJobStatus(req);
        });

    /// GET /api/icp/jobs/{id}/result - 작업 결과 조회
    api_server_->GET("/api/icp/jobs/{id}/result", [this](const HttpRequest& req) {
        return handleIcpJobResult(req);
        });

    /// POST /api/icp/jobs/{id}/cancel - 작업 취소
    api_server_->POST("/api/icp/jobs/{id}/cancel", [this](const HttpRequest& req) {
        return handleIcpJobCancel(req);
        });

    /// GET /api/icp/jobs/{id}/stats - 작업 통계 조회
    api_server_->GET("/api/icp/jobs/{id}/stats", [this](const HttpRequest& req) {
        return handleIcpJobStats(req);
        });

    ILOG << "[ICP] REST API routes registered";
}

/// =========================================
/// 작업 ID 생성
/// =========================================

std::string MasterApplication::generateIcpJobId() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::tm tm_buf{};
    localtime_s(&tm_buf, &time_t_now);

    std::stringstream ss;
    ss << "icp_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
        << "_" << std::setfill('0') << std::setw(3) << ms.count();

    return ss.str();
}

/// =========================================
/// REST API 핸들러
/// =========================================

HttpResponse MasterApplication::handleIcpStart(const HttpRequest& req) {
    ILOG << "[ICP] POST /api/icp/start";

    try {
        /// JSON 본문 파싱
        auto json_opt = MiniJson::parse_object_ex(req.body);
        if (!json_opt) {
            return createErrorResponse(400, "Invalid JSON body");
        }
        auto& json = *json_opt;

        auto las_file_opt = MiniJson::get_ex<std::string>(json, "las_file");
        auto bim_folder_opt = MiniJson::get_ex<std::string>(json, "bim_folder");

        if (!las_file_opt || !bim_folder_opt) {
            return createErrorResponse(400, "Missing required fields: las_file, bim_folder");
        }

        std::string las_file = *las_file_opt;
        std::string bim_folder = *bim_folder_opt;

        /// 새 ICP 작업 생성
        auto job = std::make_shared<icp::IcpJob>();
        job->jobId = generateIcpJobId();
        job->lasFilePath = las_file;
        job->bimFolderPath = bim_folder;
        job->status = icp::IcpJobStatus::PENDING;

        /// 선택적 출력 경로 (생략 시 las_file 기준으로 자동 생성)
        auto output_opt = MiniJson::get_ex<std::string>(json, "output_path");
        if (output_opt) {
            job->outputPath = *output_opt;
        }
        else {
            size_t dot_pos = las_file.rfind('.');
            if (dot_pos != std::string::npos) {
                job->outputPath = las_file.substr(0, dot_pos) + "_icp_aligned.las";
            }
            else {
                job->outputPath = las_file + "_icp_aligned.las";
            }
        }

        /// 선택적 설정값 재정의
        auto config_opt = MiniJson::get_object(json, "config");
        if (config_opt) {
            auto& cfg = *config_opt;
            auto max_iter = MiniJson::get_ex<double>(cfg, "max_iterations");
            if (max_iter) job->config.maxIterations = static_cast<int>(*max_iter);

            auto conv_thresh = MiniJson::get_ex<double>(cfg, "convergence_threshold");
            if (conv_thresh) job->config.convergenceThreshold = *conv_thresh;

            auto max_dist = MiniJson::get_ex<double>(cfg, "max_correspondence_distance");
            if (max_dist) job->config.maxCorrespondenceDistance = *max_dist;

            auto ds_ratio = MiniJson::get_ex<double>(cfg, "downsample_ratio");
            if (ds_ratio) job->config.downsampleRatio = static_cast<int>(*ds_ratio);

            auto grid_size = MiniJson::get_ex<double>(cfg, "pseudo_point_grid_size");
            if (grid_size) job->config.pseudoPointGridSize = *grid_size;
        }

        /// 선택적 LAS offset (dx, dy, dz)
        auto offset_opt = MiniJson::get_object(json, "offset");
        if (offset_opt) {
            auto& off = *offset_opt;
            auto dx = MiniJson::get_ex<double>(off, "dx");
            if (dx) job->offsetX = *dx;

            auto dy = MiniJson::get_ex<double>(off, "dy");
            if (dy) job->offsetY = *dy;

            auto dz = MiniJson::get_ex<double>(off, "dz");
            if (dz) job->offsetZ = *dz;
        }

        /// 시작 시각 기록
        job->startTimeMs = static_cast<double>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());

        /// 작업 저장
        {
            std::lock_guard<std::mutex> lock(icp_jobs_mutex_);
            icp_jobs_[job->jobId] = job;
        }

        ILOG << "[ICP] Job created: " << job->jobId
            << " (LAS: " << las_file << ", BIM: " << bim_folder << ")";

        /// 백그라운드 스레드에서 처리 시작
        std::thread([this, job]() {
            processIcpJob(job);
            }).detach();

        /// 응답 반환
        std::ostringstream oss;
        oss << "{\n";
        oss << "  \"success\": true,\n";
        oss << "  \"job_id\": \"" << job->jobId << "\",\n";
        oss << "  \"status\": \"" << icp::statusToString(job->status) << "\",\n";
        oss << "  \"message\": \"ICP job started\"\n";
        oss << "}";

        HttpResponse response;
        response.status_code = 200;
        response.body = oss.str();
        return response;
    }
    catch (const std::exception& e) {
        ELOG << "[ICP] handleIcpStart error: " << e.what();
        return createErrorResponse(500, std::string("Error: ") + e.what());
    }
}

HttpResponse MasterApplication::handleIcpJobs(const HttpRequest& req) {
    ILOG << "[ICP] GET /api/icp/jobs";

    try {
        std::ostringstream oss;
        oss << "{\n";
        oss << "  \"success\": true,\n";

        {
            std::lock_guard<std::mutex> lock(icp_jobs_mutex_);
            oss << "  \"count\": " << icp_jobs_.size() << ",\n";
            oss << "  \"jobs\": [\n";

            size_t i = 0;
            for (const auto& pair : icp_jobs_) {
                const auto& job = pair.second;

                oss << "    {\n";
                oss << "      \"job_id\": \"" << job->jobId << "\",\n";
                oss << "      \"status\": \"" << icp::statusToString(job->status) << "\",\n";
                oss << "      \"progress\": " << std::fixed << std::setprecision(2) << job->getProgress() << ",\n";
                oss << "      \"las_file\": \"" << job->lasFilePath << "\",\n";
                oss << "      \"bim_folder\": \"" << job->bimFolderPath << "\",\n";
                oss << "      \"total_chunks\": " << job->totalChunks << ",\n";
                oss << "      \"completed_chunks\": " << job->completedChunks << "\n";
                oss << "    }";

                if (i < icp_jobs_.size() - 1) oss << ",";
                oss << "\n";
                ++i;
            }
            oss << "  ]\n";
        }

        oss << "}";

        HttpResponse response;
        response.status_code = 200;
        response.body = oss.str();
        return response;
    }
    catch (const std::exception& e) {
        return createErrorResponse(500, std::string("Error: ") + e.what());
    }
}

HttpResponse MasterApplication::handleIcpJobStatus(const HttpRequest& req) {
    std::string job_id = req.path_params.count("id") ? req.path_params.at("id") : "";
    ILOG << "[ICP] GET /api/icp/jobs/" << job_id;

    if (job_id.empty()) {
        return createErrorResponse(400, "Missing job_id");
    }

    try {
        std::shared_ptr<icp::IcpJob> job;
        {
            std::lock_guard<std::mutex> lock(icp_jobs_mutex_);
            auto it = icp_jobs_.find(job_id);
            if (it == icp_jobs_.end()) {
                return createErrorResponse(404, "Job not found: " + job_id);
            }
            job = it->second;
        }

        std::ostringstream oss;
        oss << "{\n";
        oss << "  \"success\": true,\n";
        oss << "  \"job_id\": \"" << job->jobId << "\",\n";
        oss << "  \"status\": \"" << icp::statusToString(job->status) << "\",\n";
        oss << "  \"progress\": " << std::fixed << std::setprecision(2) << job->getProgress() << ",\n";
        oss << "  \"las_file\": \"" << job->lasFilePath << "\",\n";
        oss << "  \"bim_folder\": \"" << job->bimFolderPath << "\",\n";
        oss << "  \"output_path\": \"" << job->outputPath << "\",\n";
        oss << "  \"total_chunks\": " << job->totalChunks << ",\n";
        oss << "  \"completed_chunks\": " << job->completedChunks << ",\n";
        oss << "  \"failed_chunks\": " << job->failedChunks;

        if (job->status == icp::IcpJobStatus::FAILED) {
            oss << ",\n  \"error_message\": \"" << job->errorMessage << "\"";
        }

        if (job->isFinished()) {
            oss << ",\n  \"elapsed_time_sec\": " << std::fixed << std::setprecision(2) << job->getElapsedTimeSec();
        }

        oss << "\n}";

        HttpResponse response;
        response.status_code = 200;
        response.body = oss.str();
        return response;
    }
    catch (const std::exception& e) {
        return createErrorResponse(500, std::string("Error: ") + e.what());
    }
}

HttpResponse MasterApplication::handleIcpJobResult(const HttpRequest& req) {
    std::string job_id = req.path_params.count("id") ? req.path_params.at("id") : "";
    ILOG << "[ICP] GET /api/icp/jobs/" << job_id << "/result";

    if (job_id.empty()) {
        return createErrorResponse(400, "Missing job_id");
    }

    try {
        std::shared_ptr<icp::IcpJob> job;
        {
            std::lock_guard<std::mutex> lock(icp_jobs_mutex_);
            auto it = icp_jobs_.find(job_id);
            if (it == icp_jobs_.end()) {
                return createErrorResponse(404, "Job not found: " + job_id);
            }
            job = it->second;
        }

        if (!job->isFinished()) {
            HttpResponse response;
            response.status_code = 202;
            response.body = std::string("{\"success\": false, \"status\": \"")
                + icp::statusToString(job->status)
                + "\", \"message\": \"Job not yet completed\"}";
            return response;
        }

        std::ostringstream oss;
        oss << "{\n";
        oss << "  \"success\": " << (job->status == icp::IcpJobStatus::COMPLETED ? "true" : "false") << ",\n";
        oss << "  \"job_id\": \"" << job->jobId << "\",\n";
        oss << "  \"status\": \"" << icp::statusToString(job->status) << "\"";

        if (job->status == icp::IcpJobStatus::COMPLETED) {
            oss << ",\n  \"final_transform\": [";
            for (int i = 0; i < 16; ++i) {
                oss << job->finalTransform.m[i];
                if (i < 15) oss << ", ";
            }
            oss << "]";
            oss << ",\n  \"final_rmse\": " << job->finalRMSE;
            oss << ",\n  \"coarse_rmse\": " << job->coarseRMSE;
            oss << ",\n  \"output_path\": \"" << job->outputPath << "\"";
            oss << ",\n  \"alignment_output_path\": \"" << job->alignmentOutputPath << "\"";
            oss << ",\n  \"element_count\": " << job->elementResults.size();
            oss << ",\n  \"elapsed_time_sec\": " << std::fixed << std::setprecision(2) << job->getElapsedTimeSec();
            oss << ",\n  \"total_points\": " << job->totalPointCount;
        }
        else if (job->status == icp::IcpJobStatus::FAILED) {
            oss << ",\n  \"error_message\": \"" << job->errorMessage << "\"";
        }

        oss << "\n}";

        HttpResponse response;
        response.status_code = 200;
        response.body = oss.str();
        return response;
    }
    catch (const std::exception& e) {
        return createErrorResponse(500, std::string("Error: ") + e.what());
    }
}

HttpResponse MasterApplication::handleIcpJobCancel(const HttpRequest& req) {
    std::string job_id = req.path_params.count("id") ? req.path_params.at("id") : "";
    ILOG << "[ICP] POST /api/icp/jobs/" << job_id << "/cancel";

    if (job_id.empty()) {
        return createErrorResponse(400, "Missing job_id");
    }

    try {
        std::shared_ptr<icp::IcpJob> job;
        {
            std::lock_guard<std::mutex> lock(icp_jobs_mutex_);
            auto it = icp_jobs_.find(job_id);
            if (it == icp_jobs_.end()) {
                return createErrorResponse(404, "Job not found: " + job_id);
            }
            job = it->second;
        }

        if (job->isFinished()) {
            return createErrorResponse(400, "Job already finished");
        }

        /// 취소 요청
        job->cancelRequested.store(true);
        ILOG << "[ICP] Cancellation requested for job: " << job_id;

        std::ostringstream oss;
        oss << "{\n";
        oss << "  \"success\": true,\n";
        oss << "  \"job_id\": \"" << job_id << "\",\n";
        oss << "  \"message\": \"Cancellation requested\"\n";
        oss << "}";

        HttpResponse response;
        response.status_code = 200;
        response.body = oss.str();
        return response;
    }
    catch (const std::exception& e) {
        return createErrorResponse(500, std::string("Error: ") + e.what());
    }
}

HttpResponse MasterApplication::handleIcpJobStats(const HttpRequest& req) {
    std::string job_id = req.path_params.count("id") ? req.path_params.at("id") : "";
    ILOG << "[ICP] GET /api/icp/jobs/" << job_id << "/stats";

    if (job_id.empty()) {
        return createErrorResponse(400, "Missing job_id");
    }

    try {
        std::shared_ptr<icp::IcpJob> job;
        {
            std::lock_guard<std::mutex> lock(icp_jobs_mutex_);
            auto it = icp_jobs_.find(job_id);
            if (it == icp_jobs_.end()) {
                return createErrorResponse(404, "Job not found: " + job_id);
            }
            job = it->second;
        }

        icp::IcpStatistics stats = icp::IcpStatistics::fromJob(*job);

        std::ostringstream oss;
        oss << "{\n";
        oss << "  \"success\": true,\n";
        oss << "  \"job_id\": \"" << job->jobId << "\",\n";
        oss << "  \"statistics\": {\n";
        oss << "    \"total_source_points\": " << stats.totalSourcePoints << ",\n";
        oss << "    \"processed_source_points\": " << stats.processedSourcePoints << ",\n";
        oss << "    \"total_mesh_triangles\": " << stats.totalMeshTriangles << ",\n";
        oss << "    \"total_pseudo_points\": " << stats.totalPseudoPoints << ",\n";
        oss << "    \"coarse_rmse\": " << stats.coarseRMSE << ",\n";
        oss << "    \"final_rmse\": " << stats.finalRMSE << ",\n";
        oss << "    \"rmse_improvement_percent\": " << stats.rmseImprovement << ",\n";
        oss << "    \"total_chunks\": " << stats.totalChunks << ",\n";
        oss << "    \"successful_chunks\": " << stats.successfulChunks << ",\n";
        oss << "    \"failed_chunks\": " << stats.failedChunks << ",\n";
        oss << "    \"converged_chunks\": " << stats.convergedChunks << ",\n";
        oss << "    \"total_fine_iterations\": " << stats.totalFineIterations << ",\n";
        oss << "    \"average_fine_iterations\": " << std::fixed << std::setprecision(2) << stats.averageFineIterations << ",\n";
        oss << "    \"total_processing_time_sec\": " << std::fixed << std::setprecision(2) << stats.totalProcessingTimeSec << ",\n";
        oss << "    \"coarse_alignment_time_sec\": " << std::fixed << std::setprecision(2) << stats.coarseAlignmentTimeSec << ",\n";
        oss << "    \"fine_alignment_time_sec\": " << std::fixed << std::setprecision(2) << stats.fineAlignmentTimeSec << "\n";
        oss << "  }\n";
        oss << "}";

        HttpResponse response;
        response.status_code = 200;
        response.body = oss.str();
        return response;
    }
    catch (const std::exception& e) {
        return createErrorResponse(500, std::string("Error: ") + e.what());
    }
}

/// =========================================
/// ICP 작업 처리 (백그라운드 스레드)
/// =========================================

/// 여러 청크(예: BIM 부재당 하나씩)의 ICP 결과를 각 청크의 source 점 개수로
/// 가중치를 준 하나의 강체 변환으로 통합한다. 회전은 선형적으로 평균낼 수
/// 없으므로, 가중 평균한 3x3 회전 블록을 SVD로 가장 가까운 올바른 회전 행렬로
/// 재직교화(re-orthogonalize)한다(서로 가까운 회전들을 평균내는 표준적인
/// "chordal L2 mean" 방식 -- 모든 청크가 서로 다른, 공간적으로 국소적인 대응점
/// 부분집합으로부터 동일한 전역 강체 변환을 추정하고 있으므로 이 회전들은
/// 서로 가까울 것으로 기대된다). 결과가 정확히 1개뿐이면 그 결과 자신의
/// 변환으로 그대로 축소된다.
static icp::Transform4x4 aggregateWeightedTransforms(const std::vector<icp::IcpResult>& results) {
    if (results.empty()) {
        return icp::Transform4x4::identity();
    }
    if (results.size() == 1) {
        return results[0].transform;
    }

    double totalWeight = 0.0;
    Eigen::Matrix3d Rsum = Eigen::Matrix3d::Zero();
    Eigen::Vector3d Tsum = Eigen::Vector3d::Zero();

    for (const auto& r : results) {
        double w = static_cast<double>((std::max)(r.sourcePointCount, 1));

        double R[9];
        r.transform.getRotation(R);
        Eigen::Matrix3d Rm;
        Rm << R[0], R[1], R[2],
              R[3], R[4], R[5],
              R[6], R[7], R[8];
        Rsum += w * Rm;

        double tx, ty, tz;
        r.transform.getTranslation(tx, ty, tz);
        Tsum += w * Eigen::Vector3d(tx, ty, tz);

        totalWeight += w;
    }

    if (totalWeight <= 0.0) {
        return results[0].transform;
    }

    Rsum /= totalWeight;
    Tsum /= totalWeight;

    /// 가중 평균한 회전을 SO(3) 위로 재투영 (가장 가까운 올바른 회전 행렬로).
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(Rsum, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d Ravg = U * V.transpose();

    /// 올바른 회전(det = +1) 대신 반사(reflection, det = -1)가 나오는 것을 방지.
    if (Ravg.determinant() < 0.0) {
        V.col(2) *= -1.0;
        Ravg = U * V.transpose();
    }

    double Rout[9] = {
        Ravg(0, 0), Ravg(0, 1), Ravg(0, 2),
        Ravg(1, 0), Ravg(1, 1), Ravg(1, 2),
        Ravg(2, 0), Ravg(2, 1), Ravg(2, 2)
    };

    return icp::Transform4x4::fromRotationTranslation(Rout, Tsum.x(), Tsum.y(), Tsum.z());
}

/// decomposeSwingTwist()가 반환하는, 부재 중심축 기준으로 분해한 회전/이동 성분.
struct SwingTwistDecomposition {
    double twistAngleDeg = 0.0;   /// 축을 중심으로 한 회전(비틀림), 부호 있음
    double swingAngleDeg = 0.0;   /// 축 자체의 기울어짐(비틀림 제외 나머지 회전), 크기만(0 이상)
    double axialShift = 0.0;      /// 축 방향 이동 성분(미터), 부호 있음
    double lateralShift = 0.0;    /// 축에 수직인 이동 성분의 크기(미터), 0 이상
};

/// 부재의 정합 결과 transform(회전 R + 이동 T)을, 그 부재 고유의 중심축(axis, 단위벡터가
/// 아니어도 됨 -- 내부에서 정규화한다) 기준으로 "축 둘레 비틀림(twist)"과 "축이 기울어진
/// 정도(swing)", "축 방향 밀림(axial shift)"과 "축에서 벗어난 옆방향 밀림(lateral shift)"
/// 으로 분해한다. 시공 QA 관점에서는 전체 정합 결과보다 이 값들이 훨씬 직관적이다 --
/// 예를 들어 twistAngleDeg가 크면 배관이 제 위치에서 축을 기준으로 돌아간 것이고,
/// lateralShift가 크면 배관 자체가 설계 위치에서 옆으로 밀려나 시공된 것이다.
///
/// [수학적 근거] 회전 R을 쿼터니언 q로 변환한 뒤, q의 벡터부(v)를 axis 방향으로 투영해
/// twist 쿼터니언을 뽑아내는 표준적인 swing-twist 분해(swing-twist decomposition)를
/// 사용한다: q = swing * twist, twist는 axis를 회전축으로 하는 순수 회전, swing은 그
/// 나머지(축 자체의 기울임)를 담당한다. 이동 T는 벡터 투영으로 축 방향/수직 방향
/// 성분으로 직접 분해한다(회전과 달리 선형이라 별도 분해 기법이 필요 없다).
static SwingTwistDecomposition decomposeSwingTwist(const icp::Transform4x4& transform,
    const std::array<double, 3>& axis) {
    SwingTwistDecomposition result;

    Eigen::Vector3d axisVec(axis[0], axis[1], axis[2]);
    double axisNorm = axisVec.norm();
    if (axisNorm < 1e-9) {
        /// 축이 정의되지 않음(예: PCA가 점 부족으로 실패한 경우) -- 분해할 수 없으므로
        /// 기본값(전부 0)을 반환한다.
        return result;
    }
    axisVec /= axisNorm;

    double R[9];
    transform.getRotation(R);
    Eigen::Matrix3d Rm;
    Rm << R[0], R[1], R[2],
          R[3], R[4], R[5],
          R[6], R[7], R[8];

    Eigen::Quaterniond q(Rm);
    q.normalize();

    /// twist 성분 추출: q의 벡터부를 axis 방향으로 투영
    Eigen::Vector3d vParallel = q.vec().dot(axisVec) * axisVec;
    Eigen::Quaterniond twist(q.w(), vParallel.x(), vParallel.y(), vParallel.z());
    double twistNorm = twist.norm();

    const double radToDeg = 180.0 / std::acos(-1.0);

    if (twistNorm < 1e-9) {
        /// 축과 정확히 수직인 방향으로 180도 회전한 극히 드문 퇴화 케이스 -- twist를
        /// 정의할 수 없으므로 전체 회전을 swing으로 취급한다.
        double w = (std::min)(1.0, (std::max)(-1.0, q.w()));
        result.swingAngleDeg = 2.0 * std::acos(std::fabs(w)) * radToDeg;
    }
    else {
        twist.normalize();

        /// twist.w() = cos(theta/2), twist.vec()·axis = sin(theta/2) (부호 포함) --
        /// atan2로 안전하게 각도를 복원한다.
        double sinHalf = twist.vec().dot(axisVec);
        result.twistAngleDeg = 2.0 * std::atan2(sinHalf, twist.w()) * radToDeg;

        /// swing = q * twist^-1: twist를 먼저 적용한 뒤 나머지(swing)가 축의 기울임을 담당.
        Eigen::Quaterniond swing = q * twist.conjugate();
        double swingW = (std::min)(1.0, (std::max)(-1.0, swing.w()));
        /// 쿼터니언의 이중 덮개(double cover, q와 -q가 같은 회전) 때문에 부호가 뒤집혀도
        /// swing 각도는 항상 0~180도 범위의 "얼마나 기울었는가"만 의미가 있으므로 절대값을 쓴다.
        result.swingAngleDeg = 2.0 * std::acos(std::fabs(swingW)) * radToDeg;
    }

    /// 이동 성분을 axis 기준으로 분해 (선형 투영이라 회전처럼 별도 분해 기법이 필요 없음)
    double tx, ty, tz;
    transform.getTranslation(tx, ty, tz);
    Eigen::Vector3d t(tx, ty, tz);
    double axial = t.dot(axisVec);
    Eigen::Vector3d lateralVec = t - axial * axisVec;

    result.axialShift = axial;
    result.lateralShift = lateralVec.norm();

    return result;
}

/// Transform4x4을 16개 숫자로 이루어진 JSON 배열로 덧붙여 쓰는 작은 헬퍼.
static void writeTransformJson(std::ostringstream& oss, const icp::Transform4x4& t) {
    oss << "[";
    for (int i = 0; i < 16; ++i) {
        oss << t.m[i];
        if (i < 15) oss << ", ";
    }
    oss << "]";
}

/// JSON 문자열 리터럴에 안전하게 삽입할 수 있도록 문자열을 이스케이프한다
/// (큰따옴표와 백슬래시 -- BIM 부재 이름은 그 외에는 평범한 텍스트, 예를 들어
/// 한글/ASCII 식별자다).
static std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out;
}

/// BIM 부재별 정합 결과와 작업 전체의 통합 정합 결과를 정합된 LAS 출력 파일 옆에
/// JSON 파일로 저장한다 (job->outputPath의 확장자를 "_alignment.json"으로 교체한
/// 경로). 부재별 결과를 전체 정합으로 통합하는 수학적 방법론(SVD 재직교화를 통한
/// 가중 회전 평균 + 가중 이동 평균, 위의 aggregateWeightedTransforms())은
/// doc/handover 참고.
static std::string saveAlignmentResults(const std::shared_ptr<icp::IcpJob>& job) {
    std::string path;
    size_t dotPos = job->outputPath.rfind('.');
    path = (dotPos != std::string::npos) ? job->outputPath.substr(0, dotPos) : job->outputPath;
    path += "_alignment.json";

    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"job_id\": \"" << job->jobId << "\",\n";
    oss << "  \"rebase_offset\": [" << job->rebaseOffsetX << ", " << job->rebaseOffsetY << ", " << job->rebaseOffsetZ << "],\n";

    oss << "  \"elements\": [\n";
    for (size_t i = 0; i < job->elementResults.size(); ++i) {
        const auto& e = job->elementResults[i];
        oss << "    {\n";
        oss << "      \"chunk_id\": " << e.chunk_id << ",\n";
        oss << "      \"element_name\": \"" << jsonEscape(e.elementName) << "\",\n";
        oss << "      \"element_revit_id\": " << e.elementRevitId << ",\n";
        oss << "      \"source_point_count\": " << e.sourcePointCount << ",\n";
        oss << "      \"target_point_count\": " << e.targetPointCount << ",\n";
        oss << "      \"rmse\": " << e.rmse << ",\n";
        oss << "      \"converged\": " << (e.converged ? "true" : "false") << ",\n";
        oss << "      \"success\": " << (e.success ? "true" : "false") << ",\n";
        oss << "      \"error_message\": \"" << jsonEscape(e.errorMessage) << "\",\n";
        oss << "      \"transform\": ";
        writeTransformJson(oss, e.transform);
        oss << ",\n";
        oss << "      \"axis_direction\": [" << e.axisDirection[0] << ", " << e.axisDirection[1] << ", " << e.axisDirection[2] << "],\n";
        oss << "      \"axis_source\": \"" << jsonEscape(e.axisSource) << "\",\n";
        oss << "      \"axis_confidence\": " << e.axisConfidence << ",\n";
        oss << "      \"twist_angle_deg\": " << e.twistAngleDeg << ",\n";
        oss << "      \"swing_angle_deg\": " << e.swingAngleDeg << ",\n";
        oss << "      \"axial_shift\": " << e.axialShift << ",\n";
        oss << "      \"lateral_shift\": " << e.lateralShift << "\n";
        oss << "    }";
        if (i + 1 < job->elementResults.size()) oss << ",";
        oss << "\n";
    }
    oss << "  ],\n";

    oss << "  \"coarse_transform\": ";
    writeTransformJson(oss, job->coarseTransform);
    oss << ",\n";
    oss << "  \"coarse_rmse\": " << job->coarseRMSE << ",\n";

    oss << "  \"combined_transform\": ";
    writeTransformJson(oss, job->finalTransform);
    oss << ",\n";
    oss << "  \"combined_rmse\": " << job->finalRMSE << ",\n";
    oss << "  \"combination_method\": \"weighted rotation average (SVD re-orthogonalized, chordal L2 mean) + weighted translation average, weighted by each element's source point count; see aggregateWeightedTransforms() in MasterApplication_ICP.cpp and doc/handover for derivation\",\n";
    oss << "  \"axis_decomposition_method\": \"each element's transform is decomposed relative to its own central axis (axis_direction, from BIM centerline if axis_source=='centerline', else PCA on the element's own mesh vertices, axis_source=='pca'); twist_angle_deg is rotation about that axis (signed), swing_angle_deg is the remaining tilt of the axis itself (unsigned), axial_shift is translation along the axis (signed, meters), lateral_shift is translation perpendicular to the axis (unsigned, meters); see decomposeSwingTwist() in MasterApplication_ICP.cpp\"\n";
    oss << "}\n";

    std::ofstream file(path);
    if (!file.is_open()) {
        WLOG << "[ICP] Failed to open alignment output file for writing: " << path;
        return "";
    }
    file << oss.str();
    file.close();
    return path;
}

void MasterApplication::processIcpJob(std::shared_ptr<icp::IcpJob> job) {
    ILOG << "[ICP] Starting job processing: " << job->jobId;

    try {
        /// =============================================
        /// 1단계: BIM/GLTF 데이터 로딩 (포인트클라우드를 로딩/이동시키기 전에 재배치(rebase)
        /// offset -- BIM 자신의 바운딩 박스 중심 -- 을 계산할 수 있도록 이것부터 먼저 로딩한다)
        /// =============================================
        job->status = icp::IcpJobStatus::LOADING_DATA;
        ILOG << "[ICP] Loading GLTF from: " << job->bimFolderPath;

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            ILOG << "[ICP] Job cancelled: " << job->jobId;
            return;
        }

        /// GLTF 메시 로딩
        std::vector<chunkbim::MeshChunk> meshes;
        if (!loadGltf(job->bimFolderPath, meshes)) {
            throw std::runtime_error("Failed to load GLTF from: " + job->bimFolderPath);
        }

        job->totalMeshTriangles = icp::countTotalTriangles(meshes);
        ILOG << "[ICP] GLTF loaded: " << meshes.size() << " meshes, "
            << job->totalMeshTriangles << " triangles";

        for (auto& mesh : meshes) {
            mesh.calculateBounds();
        }
        std::array<double, 3> rebaseOffset = icp::computeBimRebaseOffset(meshes);
        job->rebaseOffsetX = rebaseOffset[0];
        job->rebaseOffsetY = rebaseOffset[1];
        job->rebaseOffsetZ = rebaseOffset[2];
        ILOG << "[ICP] Rebase offset (BIM bbox center): ("
            << job->rebaseOffsetX << ", " << job->rebaseOffsetY << ", " << job->rebaseOffsetZ << ")";

        /// =============================================
        /// 2단계: LAS 데이터 로딩
        /// =============================================
        ILOG << "[ICP] Loading LAS file: " << job->lasFilePath;

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            return;
        }

        /// LAS 파일 로딩
        std::vector<chunkpc::PointCloudChunk> lasChunks;
        const uint32_t maxPointsPerChunk = 100000;

        if (!loadLasFile(job->lasFilePath, lasChunks, maxPointsPerChunk)) {
            throw std::runtime_error("Failed to load LAS file: " + job->lasFilePath);
        }

        /// ICP 포맷으로 변환 (절대좌표, double -- 일시적으로만 사용)
        std::vector<std::array<double, 3>> sourcePointsAbsolute = icp::convertPointCloudToIcp(lasChunks);
        job->totalPointCount = sourcePointsAbsolute.size();

        ILOG << "[ICP] LAS loaded: " << job->totalPointCount << " points";

        /// 수동 offset 적용 (기존부터 있던, 사용자가 직접 지정하는 기능 -- rebaseOffset과는 별개)
        if (job->offsetX != 0.0 || job->offsetY != 0.0 || job->offsetZ != 0.0) {
            ILOG << "[ICP] Applying offset: dx=" << job->offsetX
                << ", dy=" << job->offsetY
                << ", dz=" << job->offsetZ;

            for (auto& pt : sourcePointsAbsolute) {
                pt[0] += job->offsetX;
                pt[1] += job->offsetY;
                pt[2] += job->offsetZ;
            }
        }

        /// BIM 중심점 기준으로 재배치(rebase)하고 Master의 대량 상주 사본을 위해 float로
        /// 축소한다 -- 실제로 메모리를 절약하는 핵심 부분이다(대형 현장에서는 이 배열이
        /// 수억 개의 점이 될 수 있다). 아래의 실제 ICP 연산은 모두 이 대량 배열로부터
        /// 만들어지는 작고 절대좌표인 청크들에 대해 여전히 double 정밀도로 이루어진다;
        /// 이동(shift)은 이미 계산된(절대좌표 기준) 변환을 이 대량 배열에 직접 적용하는
        /// 아래쪽 두 지점에서만 대수적으로(icp::toShiftedFrame) 보정해 주면 된다.
        for (auto& pt : sourcePointsAbsolute) {
            pt[0] -= job->rebaseOffsetX;
            pt[1] -= job->rebaseOffsetY;
            pt[2] -= job->rebaseOffsetZ;
        }
        std::vector<std::array<float, 3>> sourcePoints = icp::narrowToFloat(sourcePointsAbsolute);
        sourcePointsAbsolute.clear();
        sourcePointsAbsolute.shrink_to_fit();

        /// =============================================
        /// 3단계: 메시로부터 가상점(pseudo point) 생성
        /// =============================================
        job->status = icp::IcpJobStatus::GENERATING_PSEUDO;
        ILOG << "[ICP] Generating pseudo points (grid size: " << job->config.pseudoPointGridSize << "m)...";

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            return;
        }

        /// `sourcePoints`와 동일한 이동된 좌표계로 생성된다(offset을 그대로 전달했으므로),
        /// 따라서 아래에서 별도 변환 없이 바로 float로 축소해 IcpChunk 필드에 저장할 수 있다.
        std::vector<std::array<double, 3>> pseudoFacePoints;
        std::vector<size_t> pseudoFaceIdx;
        std::vector<std::array<double, 3>> facePts;
        std::vector<std::array<double, 3>> targetNormals;
        /// [ToDo] 추출하는 대신 meshes를 직접 사용할 것
        icp::generatePseudoPoints(meshes, job->config.pseudoPointGridSize, pseudoFacePoints, pseudoFaceIdx, facePts, targetNormals,
            job->rebaseOffsetX, job->rebaseOffsetY, job->rebaseOffsetZ);

        job->totalPseudoPoints = pseudoFacePoints.size();
        ILOG << "[ICP] Pseudo points generated: " << job->totalPseudoPoints;

        if (pseudoFacePoints.empty()) {
            throw std::runtime_error("No pseudo points generated from mesh");
        }

        /// =============================================
        /// 4단계: Coarse(전역) 정합 (Master에서 실행)
        /// =============================================
        job->status = icp::IcpJobStatus::COARSE_ALIGNMENT;
        ILOG << "[ICP] Running coarse alignment...";

        auto coarseStartTime = std::chrono::high_resolution_clock::now();

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            return;
        }

        /// coarse 정합을 위해 포인트클라우드를 다운샘플링. `sourcePoints`는 이미 job의
        /// 재배치 offset만큼 이동된 float 상태이며 -- IcpChunk 필드도 동일한 표현이므로,
        /// 다운샘플링 결과를 승격/offset 복원 없이 바로 대입할 수 있다.
        int coarseDownsampleRatio = job->config.downsampleRatio * 2;  /// coarse는 더 적극적으로
        auto coarseSource = icp::downsampleUniform(sourcePoints, coarseDownsampleRatio);

        ILOG << "[ICP] Coarse alignment(pc vs pseudo bim pts): " << coarseSource.size() << " vs " << pseudoFacePoints.size() << " points";

        /// coarse 청크 생성
        icp::IcpChunk coarseChunk;
        coarseChunk.chunk_id = 0;
        coarseChunk.sourcePoints = std::move(coarseSource);
        coarseChunk.targetPoints = icp::narrowToFloat(pseudoFacePoints);
        coarseChunk.faceIndices = pseudoFaceIdx;
        coarseChunk.faceNormals = icp::narrowToFloat(targetNormals);
        coarseChunk.facePts = icp::narrowToFloat(facePts);
        coarseChunk.config = job->config;
        coarseChunk.config.maxCorrespondenceDistance *= 2.0;  /// coarse는 더 큰 거리 임계값 사용
        coarseChunk.offsetX = job->rebaseOffsetX;
        coarseChunk.offsetY = job->rebaseOffsetY;
        coarseChunk.offsetZ = job->rebaseOffsetZ;

        /// coarse ICP 실행
        icp::IcpResult coarseResult = icp::runIcp(coarseChunk, job->cancelRequested);

        job->coarseTransform = coarseResult.transform;
        job->coarseRMSE = coarseResult.finalRMSE;
        job->coarseConverged = coarseResult.converged;

        auto coarseEndTime = std::chrono::high_resolution_clock::now();
        job->coarseAlignmentTimeMs = std::chrono::duration<double, std::milli>(
            coarseEndTime - coarseStartTime).count();

        ILOG << "[ICP] Coarse alignment: RMSE=" << job->coarseRMSE
            << ", converged=" << (job->coarseConverged ? "Yes" : "No")
            << ", iterations=" << coarseResult.actualIterations
            << ", time=" << (job->coarseAlignmentTimeMs / 1000.0) << "s";

        /// =============================================
        /// 5단계: Fine(정밀) 정합 (가능하면 BIM 부재당 청크 하나씩 Slave들에 분산 처리;
        /// 그렇지 않으면 Master 단일 패스로 대체(fallback))
        /// =============================================
        job->status = icp::IcpJobStatus::FINE_ALIGNMENT;

        auto fineStartTime = std::chrono::high_resolution_clock::now();

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            return;
        }

        /// coarse 변환을 source 점들에 적용 -- (분산이든 대체든) 모든 청크는 이 coarse
        /// 정합된 좌표계를 시작점으로 하여, initialTransform을 항등행렬로 두고 미세조정한다.
        /// job->coarseTransform은 절대좌표 청크 데이터로 계산되었으므로(위 4단계 참고),
        /// (float, 이동된) 대량 배열에 직접 적용하기 전에 이동된 좌표계용 등가 변환으로
        /// 변환해 둔다 -- 이렇게 하면 행렬 하나 적용하자고 대량 배열 전체를 double로
        /// 승격시키지 않아도 된다.
        icp::Transform4x4 coarseTransformShifted = icp::toShiftedFrame(
            job->coarseTransform, job->rebaseOffsetX, job->rebaseOffsetY, job->rebaseOffsetZ);
        auto alignedSource = icp::transformPointCloudCopy(sourcePoints, coarseTransformShifted);

        /// 연결된 Slave들에 fine 정합을 BIM 부재(Revit Element ID)당 청크 하나씩 분산시켜
        /// 본다. icp_dispatch_mutex_는 task_manager_가 단일 공유 자원이므로(MasterApplication.h의
        /// icp_dispatch_mutex_ 주석 참고) 동시에 실행되는 ICP 작업들 사이에서 그 사용을
        /// 직렬화한다 -- 다른 작업이 이미 쓰고 있거나 연결된 Slave가 없으면, 이 락을
        /// 기다리며 블로킹하는 대신 Master 단독 처리로 대체(fallback)한다.
        std::unique_lock<std::mutex> dispatchLock(icp_dispatch_mutex_, std::try_to_lock);
        bool distributed = false;
        /// elementInfoByChunkId[i]는 elementChunks[i] 뒤에 있는 BIM 부재를 설명한다
        /// (loadIcpElementChunks가 chunk_id == 인덱스로 배정하므로, 이는 동시에 chunk_id ->
        /// 부재 조회 역할도 한다) -- 6단계 이후 job->elementResults에 이름을 붙이는 데 쓰인다.
        std::vector<IcpElementInfo> elementInfoByChunkId;

        if (dispatchLock.owns_lock() && !job->cancelRequested.load()) {
            size_t slaveCount = 0;
            if (initializeTaskManager(TaskType::ICP_FINE_ALIGNMENT)) {
                slaveCount = task_manager_->getAllSlaves().size();
            }

            if (slaveCount > 0) {
                job->status = icp::IcpJobStatus::DISTRIBUTING;
                ILOG << "[ICP] Distributing fine alignment across " << slaveCount << " slave(s), "
                    << "one chunk per BIM element...";

                auto elementChunks = PointCloudLoader::loadIcpElementChunks(
                    meshes, alignedSource,
                    job->rebaseOffsetX, job->rebaseOffsetY, job->rebaseOffsetZ,
                    job->config, &elementInfoByChunkId);

                if (!elementChunks.empty()) {
                    job->totalChunks = static_cast<int>(elementChunks.size());
                    job->completedChunks = 0;
                    job->failedChunks = 0;

                    {
                        std::lock_guard<std::mutex> lock(icp_jobs_mutex_);
                        for (const auto& chunk : elementChunks) {
                            uint32_t task_id = task_manager_->addTask(chunk);
                            icp_task_to_job_[task_id] = job->jobId;
                        }
                    }

                    ILOG << "[ICP] " << elementChunks.size() << " element chunks queued for job "
                        << job->jobId;

                    /// 모든 청크 결과를 기다린다 (또는 취소/타임아웃). Slave들은 각 청크를
                    /// 로컬 수렴까지 완전히 실행한 뒤 청크당 IcpResult 하나를 보고하므로,
                    /// 반복(iteration)마다 네트워크 왕복이 필요하지 않다 -- 그저 큐가
                    /// 비워지기를 폴링(polling)하면 된다.
                    auto waitStart = std::chrono::steady_clock::now();
                    const auto maxWait = std::chrono::seconds(
                        (std::max)(cfg_.task_timeout_seconds, 300));

                    while (true) {
                        int done;
                        {
                            std::lock_guard<std::mutex> lock(icp_jobs_mutex_);
                            done = job->completedChunks + job->failedChunks;
                        }
                        if (done >= job->totalChunks) break;
                        if (job->cancelRequested.load()) break;
                        if (std::chrono::steady_clock::now() - waitStart > maxWait) {
                            WLOG << "[ICP] Distributed fine alignment timed out for job "
                                << job->jobId << " (" << done << "/" << job->totalChunks
                                << " chunks finished)";
                            break;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    }

                    distributed = !job->chunkResults.empty();
                }
                else {
                    WLOG << "[ICP] loadIcpElementChunks produced no usable chunks for job "
                        << job->jobId << "; falling back to Master-only fine alignment";
                }
            }
        }

        if (!distributed) {
            /// 대체(Fallback): 이 기능이 있기 전과 완전히 동일하게, 전체 (다운샘플링된)
            /// 포인트클라우드에 대해 Master 단일 패스를 실행한다.
            ILOG << "[ICP] Running fine alignment on Master (no slaves distributed to)...";

#ifdef _DEBUG
            ///[ToDo] 적절한 값으로 교체할 것
            coarseChunk.config.downsampleRatio = 1;  /// coarse는 더 큰 거리 임계값 사용
#endif

            /// `alignedSource`는 이미 job의 재배치 offset만큼 이동된 float 상태다 -- IcpChunk
            /// 필드도 동일한 표현이므로 여기서 승격/offset 복원이 필요 없다.
            auto fineSource = icp::downsampleUniform(alignedSource, job->config.downsampleRatio);

            ILOG << "[ICP] Fine alignment(pc vs pseudo bim pts): " << fineSource.size() << " vs " << pseudoFacePoints.size() << " points";

            icp::IcpChunk fineChunk;
            fineChunk.chunk_id = 0;
            fineChunk.sourcePoints = std::move(fineSource);
            fineChunk.targetPoints = icp::narrowToFloat(pseudoFacePoints);
            fineChunk.faceIndices = pseudoFaceIdx;
            fineChunk.faceNormals = icp::narrowToFloat(targetNormals);
            fineChunk.facePts = icp::narrowToFloat(facePts);
            fineChunk.config = job->config;
            fineChunk.offsetX = job->rebaseOffsetX;
            fineChunk.offsetY = job->rebaseOffsetY;
            fineChunk.offsetZ = job->rebaseOffsetZ;

            icp::IcpResult fineResult = icp::runIcp(fineChunk, job->cancelRequested);

            job->chunkResults.push_back(fineResult);
            job->completedChunks = 1;
            job->totalChunks = 1;

            ILOG << "[ICP] Fine alignment: RMSE=" << fineResult.finalRMSE
                << ", converged=" << (fineResult.converged ? "Yes" : "No")
                << ", iterations=" << fineResult.actualIterations;
        }

        auto fineEndTime = std::chrono::high_resolution_clock::now();
        job->fineAlignmentTimeMs = std::chrono::duration<double, std::milli>(
            fineEndTime - fineStartTime).count();

        /// =============================================
        /// 6단계: 결과 통합
        /// =============================================
        job->status = icp::IcpJobStatus::AGGREGATING;
        ILOG << "[ICP] Aggregating results from " << job->chunkResults.size() << " chunk(s)...";

        if (job->chunkResults.empty()) {
            throw std::runtime_error("Fine alignment produced no successful chunk results");
        }

        /// 청크별 변환들의 가중 평균(각 청크의 source 점 개수로 가중치를 줌)을 구하되,
        /// 결과가 단순 원소별 평균이 아니라 올바른 회전 행렬이 되도록 회전 성분은 SVD로
        /// 재직교화한다. 청크 결과가 정확히 1개(대체 경로)인 경우에는 그 청크 자신의
        /// 변환으로 그대로 축소된다 -- 분산 fine 정합이 있기 전과 동일한 동작이다.
        icp::Transform4x4 combinedFineTransform = aggregateWeightedTransforms(job->chunkResults);

        double weightedRmseSum = 0.0;
        double totalWeight = 0.0;
        int convergedCount = 0;
        for (const auto& r : job->chunkResults) {
            double w = static_cast<double>((std::max)(r.sourcePointCount, 1));
            weightedRmseSum += w * r.finalRMSE;
            totalWeight += w;
            if (r.converged) ++convergedCount;
        }

        /// coarse와 fine 변환을 결합
        job->finalTransform = combinedFineTransform * job->coarseTransform;
        job->finalRMSE = (totalWeight > 0.0) ? (weightedRmseSum / totalWeight) : 0.0;

        ILOG << "[ICP] Final transform computed from " << job->chunkResults.size()
            << " chunk(s) (" << convergedCount << " converged), RMSE: " << job->finalRMSE;

        /// 각 부재 자신의 전체 변환(coarse + 그 부재만의 로컬 미세조정)을 BIM 부재 이름/id로
        /// 이름 붙여 기록해 둔다, 아래에서 통합 결과와 함께 저장하기 위해. 분산(부재당 청크
        /// 하나) 경로에서만 의미가 있다 -- Master 단독 대체 경로는 부재별 세부 내역이 없으므로
        /// 여기서는 비어 있는 채로 남는다.
        job->elementResults.clear();
        for (const auto& r : job->chunkResults) {
            icp::IcpElementAlignment elem;
            elem.chunk_id = r.chunk_id;
            if (r.chunk_id < elementInfoByChunkId.size()) {
                const auto& info = elementInfoByChunkId[r.chunk_id];
                elem.elementName = info.name;
                elem.elementRevitId = info.revitId;
                elem.axisDirection = info.axisDirection;
                elem.axisSource = info.axisSource;
                elem.axisConfidence = info.axisConfidence;
            }
            elem.sourcePointCount = r.sourcePointCount;
            elem.targetPointCount = r.targetPointCount;
            elem.rmse = r.finalRMSE;
            elem.converged = r.converged;
            elem.success = r.success;
            elem.errorMessage = r.errorMessage;
            elem.transform = r.transform * job->coarseTransform;

            /// 이 부재의 축을 기준으로 twist/swing/axial/lateral 분해 -- 시공 QA에서는
            /// 통합된 4x4 행렬 자체보다 이 값들이 훨씬 직관적이다.
            SwingTwistDecomposition decomposed = decomposeSwingTwist(elem.transform, elem.axisDirection);
            elem.twistAngleDeg = decomposed.twistAngleDeg;
            elem.swingAngleDeg = decomposed.swingAngleDeg;
            elem.axialShift = decomposed.axialShift;
            elem.lateralShift = decomposed.lateralShift;

            job->elementResults.push_back(elem);
        }

        /// =============================================
        /// 7단계: 정합된 포인트클라우드 저장
        /// =============================================
        job->status = icp::IcpJobStatus::SAVING_RESULT;
        ILOG << "[ICP] Applying final transform to " << sourcePoints.size()
            << " points and saving to: " << job->outputPath;

        /// job->finalTransform은 절대좌표 기준이다(전적으로 절대좌표 청크 결과들로부터
        /// 만들어짐, 6단계 참고) -- (float, 이동된) 대량 배열에 직접 적용하기 전에 이동된
        /// 좌표계용 등가 변환으로 변환해 둔다, 5단계와 동일한 이유다.
        icp::Transform4x4 finalTransformShifted = icp::toShiftedFrame(
            job->finalTransform, job->rebaseOffsetX, job->rebaseOffsetY, job->rebaseOffsetZ);
        auto alignedFullResSource = icp::transformPointCloudCopy(sourcePoints, finalTransformShifted);

        std::vector<las::PointData> outPoints;
        outPoints.reserve(alignedFullResSource.size());
        for (const auto& p : alignedFullResSource) {
            las::PointData d;
            d.x = static_cast<double>(p[0]) + job->rebaseOffsetX;
            d.y = static_cast<double>(p[1]) + job->rebaseOffsetY;
            d.z = static_cast<double>(p[2]) + job->rebaseOffsetZ;
            outPoints.push_back(d);
        }

        if (!las::LASToolsWriter::save(job->outputPath, outPoints)) {
            throw std::runtime_error("Failed to write aligned LAS file: " + job->outputPath);
        }

        ILOG << "[ICP] Aligned point cloud saved: " << job->outputPath
            << " (" << outPoints.size() << " points)";

        job->alignmentOutputPath = saveAlignmentResults(job);
        if (!job->alignmentOutputPath.empty()) {
            ILOG << "[ICP] Per-element + combined alignment saved: " << job->alignmentOutputPath
                << " (" << job->elementResults.size() << " element(s))";
        }

        job->status = icp::IcpJobStatus::COMPLETED;
        job->endTimeMs = static_cast<double>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());

        ILOG << "[ICP] Job completed: " << job->jobId
            << " (Coarse RMSE: " << job->coarseRMSE
            << ", Final RMSE: " << job->finalRMSE
            << ", Time: " << job->getElapsedTimeSec() << "s)";
    }
    catch (const std::exception& e) {
        job->status = icp::IcpJobStatus::FAILED;
        job->errorMessage = e.what();
        job->endTimeMs = static_cast<double>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        ELOG << "[ICP] Job failed: " << job->jobId << " - " << e.what();
    }
}

/// =========================================
/// 합성 테스트 데이터 생성기
/// =========================================

/// BIM 메시로부터 합성 레이저스캔 포인트클라우드를 생성한다: 메시 표면 위에 조밀한
/// 점들을 샘플링하고(스캐너가 실측한 것처럼), 알려진 "ground truth" 강체 변환(Z축
/// 회전 + 이동)을 적용해 실제 시공 오차를 시뮬레이션하고, 선택적으로 가우시안
/// 노이즈를 더해 스캔 측정 오차를 시뮬레이션한 뒤, 결과를 .las 파일로 저장한다.
/// 주입한 변환값을 출력하여 ICP가 복원한 `final_transform`과 비교해 정확성을
/// 검증할 수 있게 한다 -- 소수의 BIM 부재(빠름)로 테스트를 시작해서 전체 모델
/// 규모까지 늘려가며 검증한 뒤, 마지막으로 실제 계측 데이터로 검증할 수 있다.
void MasterApplication::runGenerateSyntheticPointCloud(
    const std::string& bim_folder, const std::string& output_las,
    int num_elements, double grid_size, double noise_sigma,
    double shift_x, double shift_y, double shift_z, double rot_deg) {

    try {
        std::vector<chunkbim::MeshChunk> meshes;
        if (!loadGltf(bim_folder, meshes)) {
            ELOG << "[gensynth] Failed to load GLTF from: " << bim_folder;
            return;
        }
        ILOG << "[gensynth] Loaded " << meshes.size() << " BIM elements from " << bim_folder;

        if (num_elements > 0 && static_cast<size_t>(num_elements) < meshes.size()) {
            meshes.resize(num_elements);
            ILOG << "[gensynth] Using first " << num_elements << " element(s) for this run";
        }

        /// 메시 표면 위에 조밀한 점들을 샘플링 (절대좌표, offset 없음 -- 이 도구는 분산
        /// ICP 파이프라인의 일부가 아니라 독립적인 테스트 데이터 도구다).
        std::vector<std::array<double, 3>> points, facePts, faceNormals;
        std::vector<size_t> faceIdx;
        icp::generatePseudoPoints(meshes, grid_size, points, faceIdx, facePts, faceNormals);

        if (points.empty()) {
            ELOG << "[gensynth] No points generated -- check bim_folder/grid_size";
            return;
        }
        ILOG << "[gensynth] Generated " << points.size() << " synthetic scan points "
            << "(grid_size=" << grid_size << "m)";

        /// Ground-truth 변환: Z축 회전(yaw) + 이동. 이 회전은 절대좌표 원점이 아니라
        /// *선택된 부재들 자신의 중심점*을 기준으로 적용한다 -- 실제 현장 좌표에서 이
        /// BIM 부재들은 원점으로부터 수백 미터 떨어져 있을 수 있으므로, 원점 기준으로
        /// 회전시키면 현실적인 작은 오차가 아니라 수 미터 단위로 부재가 밀려나 버린다
        /// (지렛대 효과, lever-arm effect).
        for (auto& mesh : meshes) {
            mesh.calculateBounds();
        }
        std::array<double, 3> rotationCenter = icp::computeBimRebaseOffset(meshes);

        double rad = rot_deg * 3.14159265358979323846 / 180.0;
        double R[9] = {
            std::cos(rad), -std::sin(rad), 0.0,
            std::sin(rad),  std::cos(rad), 0.0,
            0.0,             0.0,           1.0
        };
        icp::Transform4x4 rotation = icp::Transform4x4::fromRotationTranslation(R, 0.0, 0.0, 0.0);

        std::mt19937 rng(12345); /// 재현 가능성을 위한 고정 시드
        std::normal_distribution<double> noise(0.0, noise_sigma);

        std::vector<las::PointData> outPoints;
        outPoints.reserve(points.size());
        for (const auto& p : points) {
            /// 회전 중심으로 이동시킨 뒤 회전, 다시 원위치로 이동, 그 후 이동(translation) 적용.
            double cx = p[0] - rotationCenter[0];
            double cy = p[1] - rotationCenter[1];
            double cz = p[2] - rotationCenter[2];
            double rx, ry, rz;
            rotation.transformPoint(cx, cy, cz, rx, ry, rz);

            double x = rx + rotationCenter[0] + shift_x;
            double y = ry + rotationCenter[1] + shift_y;
            double z = rz + rotationCenter[2] + shift_z;
            if (noise_sigma > 0.0) {
                x += noise(rng); y += noise(rng); z += noise(rng);
            }
            las::PointData d;
            d.x = x; d.y = y; d.z = z;
            outPoints.push_back(d);
        }

        if (!las::LASToolsWriter::save(output_las, outPoints)) {
            ELOG << "[gensynth] Failed to write: " << output_las;
            return;
        }

        ILOG << "[gensynth] Saved " << outPoints.size() << " points to " << output_las;

        /// 이 합성 스캔은 `groundTruth(mesh)`다 -- 즉 메시를 회전/이동시킨 것. 나중에 ICP가
        /// 이 스캔을(source로) 다시 메시로(target으로) 정합할 때, 복원되는 final_transform은
        /// 여기서 적용한 변환 자체가 아니라 그 *역변환*에 근사해야 한다. 비교할 때 머릿속으로
        /// 역변환을 계산할 필요가 없도록, 적용한 (rotation_z, shift) 매개변수와 final_transform이
        /// 일치해야 할 절대좌표 기준 등가 역변환(R, t)을 함께 출력한다.
        double tEquivX = rotationCenter[0] - (R[0] * rotationCenter[0] + R[1] * rotationCenter[1] + R[2] * rotationCenter[2]) + shift_x;
        double tEquivY = rotationCenter[1] - (R[3] * rotationCenter[0] + R[4] * rotationCenter[1] + R[5] * rotationCenter[2]) + shift_y;
        double tEquivZ = rotationCenter[2] - (R[6] * rotationCenter[0] + R[7] * rotationCenter[1] + R[8] * rotationCenter[2]) + shift_z;
        icp::Transform4x4 groundTruthAbsolute = icp::Transform4x4::fromRotationTranslation(R, tEquivX, tEquivY, tEquivZ);
        icp::Transform4x4 expectedIcpTransform = groundTruthAbsolute.inverse();

        ILOG << "[gensynth] Applied: rotation_z=" << rot_deg << "deg about centroid ("
            << rotationCenter[0] << ", " << rotationCenter[1] << ", " << rotationCenter[2]
            << "), then shift=(" << shift_x << ", " << shift_y << ", " << shift_z
            << "), noise_sigma=" << noise_sigma;
        ILOG << "[gensynth] ICP's final_transform should approximate this inverse (16 values, "
            << "column-major): [" << expectedIcpTransform.m[0] << ", " << expectedIcpTransform.m[1] << ", "
            << expectedIcpTransform.m[2] << ", " << expectedIcpTransform.m[3] << ", "
            << expectedIcpTransform.m[4] << ", " << expectedIcpTransform.m[5] << ", "
            << expectedIcpTransform.m[6] << ", " << expectedIcpTransform.m[7] << ", "
            << expectedIcpTransform.m[8] << ", " << expectedIcpTransform.m[9] << ", "
            << expectedIcpTransform.m[10] << ", " << expectedIcpTransform.m[11] << ", "
            << expectedIcpTransform.m[12] << ", " << expectedIcpTransform.m[13] << ", "
            << expectedIcpTransform.m[14] << ", " << expectedIcpTransform.m[15] << "]";
    }
    catch (const std::exception& e) {
        ELOG << "[gensynth] Error: " << e.what();
    }
}

/// =========================================
/// ICP 결과 처리 (Slave로부터 수신)
/// =========================================

void MasterApplication::handleIcpResult(const icp::IcpResult& result, uint32_t task_id) {
    ILOG << "[ICP] Received result for task " << task_id
        << " (chunk: " << result.chunk_id
        << ", success: " << (result.success ? "Yes" : "No")
        << ", RMSE: " << result.finalRMSE << ")";

    /// [수정 이력] 예전에는 미구현 TODO였다. 지금은 (MasterApplication.h의
    /// icp_task_to_job_를 통한 task_id -> job_id 매핑으로) 제대로 연결되어 있어서, 분산
    /// fine 정합 청크 결과가 올바른 작업(job)에 도달한다. processIcpJob()의 5단계가
    /// ICP_FINE_ALIGNMENT 태스크를 실제로 task_manager_->addTask()를 통해 Slave들에
    /// 분배하고 icp_task_to_job_에 등록하므로, 이 함수는 더 이상 도달 불가능한
    /// 스캐폴딩이 아니라 그 분산 fine 정합 결과가 실제로 들어오는 경로다(연결된
    /// Slave가 없으면 processIcpJob()이 Master 단독 처리로 대체하므로 이 함수는
    /// 호출되지 않는다).
    std::shared_ptr<icp::IcpJob> job;
    {
        std::lock_guard<std::mutex> lock(icp_jobs_mutex_);
        auto task_it = icp_task_to_job_.find(task_id);
        if (task_it == icp_task_to_job_.end()) {
            WLOG << "[ICP] No job mapping found for task " << task_id << ", dropping result";
            return;
        }

        auto job_it = icp_jobs_.find(task_it->second);
        if (job_it == icp_jobs_.end()) {
            WLOG << "[ICP] Job " << task_it->second << " for task " << task_id
                << " no longer exists, dropping result";
            icp_task_to_job_.erase(task_it);
            return;
        }

        job = job_it->second;
        icp_task_to_job_.erase(task_it);
    }

    if (result.success) {
        job->chunkResults.push_back(result);
        job->completedChunks++;
    }
    else {
        job->failedChunks++;
        WLOG << "[ICP] Chunk failed for job " << job->jobId << ": " << result.errorMessage;
    }
}