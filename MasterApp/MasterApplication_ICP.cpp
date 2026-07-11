/**
 * @file MasterApplication_ICP.cpp
 * @brief ICP REST API implementation
 *
 * This file contains:
 * - ICP REST API route setup
 * - ICP Job management (start, status, cancel)
 * - ICP Result handling from Slaves
 */

#include "MasterApplication.h"
#include "../include/MiniJson.h"
#include "../include/bimtree/IcpCore.hpp"
#include "../include/bimtree/IcpSerialization.hpp"
#include "../include/bimtree/PseudoPointGenerator.hpp"
#include "../include/LaslibWriter.hpp"

 /// Data loading from DLLs
#include "../LasImport/LasImport.h"
#include "../BimImport/BimImport.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <iomanip>

/// =========================================
/// ICP REST API Route Setup
/// =========================================

void MasterApplication::setupIcpApiRoutes() {
    /// POST /api/icp/start - Start new ICP job
    api_server_->POST("/api/icp/start", [this](const HttpRequest& req) {
        return handleIcpStart(req);
        });

    /// GET /api/icp/jobs - List all ICP jobs
    api_server_->GET("/api/icp/jobs", [this](const HttpRequest& req) {
        return handleIcpJobs(req);
        });

    /// GET /api/icp/jobs/{id} - Get job status
    api_server_->GET("/api/icp/jobs/{id}", [this](const HttpRequest& req) {
        return handleIcpJobStatus(req);
        });

    /// GET /api/icp/jobs/{id}/result - Get job result
    api_server_->GET("/api/icp/jobs/{id}/result", [this](const HttpRequest& req) {
        return handleIcpJobResult(req);
        });

    /// POST /api/icp/jobs/{id}/cancel - Cancel job
    api_server_->POST("/api/icp/jobs/{id}/cancel", [this](const HttpRequest& req) {
        return handleIcpJobCancel(req);
        });

    /// GET /api/icp/jobs/{id}/stats - Get job statistics
    api_server_->GET("/api/icp/jobs/{id}/stats", [this](const HttpRequest& req) {
        return handleIcpJobStats(req);
        });

    ILOG << "[ICP] REST API routes registered";
}

/// =========================================
/// Job ID Generation
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
/// REST API Handlers
/// =========================================

HttpResponse MasterApplication::handleIcpStart(const HttpRequest& req) {
    ILOG << "[ICP] POST /api/icp/start";

    try {
        /// Parse JSON body
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

        /// Create new ICP job
        auto job = std::make_shared<icp::IcpJob>();
        job->jobId = generateIcpJobId();
        job->lasFilePath = las_file;
        job->bimFolderPath = bim_folder;
        job->status = icp::IcpJobStatus::PENDING;

        /// Optional output path (not used for now, but kept for future)
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

        /// Optional config overrides
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

        /// Optional LAS offset (dx, dy, dz)
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

        /// Record start time
        job->startTimeMs = static_cast<double>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());

        /// Store job
        {
            std::lock_guard<std::mutex> lock(icp_jobs_mutex_);
            icp_jobs_[job->jobId] = job;
        }

        ILOG << "[ICP] Job created: " << job->jobId
            << " (LAS: " << las_file << ", BIM: " << bim_folder << ")";

        /// Start processing in background thread
        std::thread([this, job]() {
            processIcpJob(job);
            }).detach();

        /// Return response
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

        /// Request cancellation
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
/// ICP Job Processing (Background Thread)
/// =========================================

/// Combine multiple per-chunk ICP results (e.g. one per BIM element) into a single rigid
/// transform, weighted by each chunk's source point count. Rotations don't average linearly,
/// so the weighted-average 3x3 rotation block is re-orthogonalized via SVD onto the nearest
/// proper rotation matrix (the standard "chordal L2 mean" approach for averaging rotations
/// that are all close to each other, which they should be here since every chunk is
/// estimating the same global rigid transform from a different, spatially local, subset of
/// correspondences). With exactly one result this reduces to that result's own transform.
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

    /// Project the weighted-average rotation back onto SO(3) (nearest proper rotation matrix).
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(Rsum, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d Ravg = U * V.transpose();

    /// Guard against a reflection (det = -1) instead of a proper rotation (det = +1).
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

/// Write a small helper to append a Transform4x4 as a 16-number JSON array.
static void writeTransformJson(std::ostringstream& oss, const icp::Transform4x4& t) {
    oss << "[";
    for (int i = 0; i < 16; ++i) {
        oss << t.m[i];
        if (i < 15) oss << ", ";
    }
    oss << "]";
}

/// Escape a string for embedding in a JSON string literal (double quotes and backslashes --
/// BIM element names are otherwise plain text, e.g. Korean/ASCII identifiers).
static std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out;
}

/// Save per-BIM-element (부재) alignment results plus the combined job-wide alignment to a
/// JSON file next to the aligned LAS output (job->outputPath with its extension replaced by
/// "_alignment.json"). See doc/handover for the math methodology combining per-element results
/// into the overall alignment (weighted rotation averaging via SVD re-orthogonalization +
/// weighted translation averaging, aggregateWeightedTransforms() above).
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
        oss << "\n    }";
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
    oss << "  \"combination_method\": \"weighted rotation average (SVD re-orthogonalized, chordal L2 mean) + weighted translation average, weighted by each element's source point count; see aggregateWeightedTransforms() in MasterApplication_ICP.cpp and doc/handover for derivation\"\n";
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
        /// Phase 1: Loading BIM/GLTF Data (loaded first so the rebase offset -- the BIM's own
        /// bounding-box center -- can be computed before the point cloud is loaded/shifted)
        /// =============================================
        job->status = icp::IcpJobStatus::LOADING_DATA;
        ILOG << "[ICP] Loading GLTF from: " << job->bimFolderPath;

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            ILOG << "[ICP] Job cancelled: " << job->jobId;
            return;
        }

        /// Load GLTF meshes
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
        /// Phase 2: Loading LAS Data
        /// =============================================
        ILOG << "[ICP] Loading LAS file: " << job->lasFilePath;

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            return;
        }

        /// Load LAS file
        std::vector<chunkpc::PointCloudChunk> lasChunks;
        const uint32_t maxPointsPerChunk = 100000;

        if (!loadLasFile(job->lasFilePath, lasChunks, maxPointsPerChunk)) {
            throw std::runtime_error("Failed to load LAS file: " + job->lasFilePath);
        }

        /// Convert to ICP format (absolute frame, double -- transient)
        std::vector<std::array<double, 3>> sourcePointsAbsolute = icp::convertPointCloudToIcp(lasChunks);
        job->totalPointCount = sourcePointsAbsolute.size();

        ILOG << "[ICP] LAS loaded: " << job->totalPointCount << " points";

        /// Apply manual offset (existing, user-supplied feature -- distinct from rebaseOffset)
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

        /// Rebase to the BIM centroid and narrow to float for the Master's bulk resident copy
        /// -- the real memory-saving point (this array can be hundreds of millions of points
        /// for a large site). All actual ICP computation below still happens in double
        /// precision on small, absolute-frame chunks built from this bulk array; the shift only
        /// needs to be reconciled algebraically (icp::toShiftedFrame) at the two points further
        /// down where an already-computed absolute-frame transform is applied directly to this
        /// bulk array.
        for (auto& pt : sourcePointsAbsolute) {
            pt[0] -= job->rebaseOffsetX;
            pt[1] -= job->rebaseOffsetY;
            pt[2] -= job->rebaseOffsetZ;
        }
        std::vector<std::array<float, 3>> sourcePoints = icp::narrowToFloat(sourcePointsAbsolute);
        sourcePointsAbsolute.clear();
        sourcePointsAbsolute.shrink_to_fit();

        /// =============================================
        /// Phase 3: Generate Pseudo Points from Mesh
        /// =============================================
        job->status = icp::IcpJobStatus::GENERATING_PSEUDO;
        ILOG << "[ICP] Generating pseudo points (grid size: " << job->config.pseudoPointGridSize << "m)...";

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            return;
        }

        /// Generated in the same shifted frame as `sourcePoints` (offset passed through), so
        /// they can be narrowed to float and stored directly into IcpChunk fields below with no
        /// further conversion.
        std::vector<std::array<double, 3>> pseudoFacePoints;
        std::vector<size_t> pseudoFaceIdx;
        std::vector<std::array<double, 3>> facePts;
        std::vector<std::array<double, 3>> targetNormals;
        /// [ToDo] use directly meshes, instead of extraction
        icp::generatePseudoPoints(meshes, job->config.pseudoPointGridSize, pseudoFacePoints, pseudoFaceIdx, facePts, targetNormals,
            job->rebaseOffsetX, job->rebaseOffsetY, job->rebaseOffsetZ);

        job->totalPseudoPoints = pseudoFacePoints.size();
        ILOG << "[ICP] Pseudo points generated: " << job->totalPseudoPoints;

        if (pseudoFacePoints.empty()) {
            throw std::runtime_error("No pseudo points generated from mesh");
        }

        /// =============================================
        /// Phase 4: Coarse Alignment (on Master)
        /// =============================================
        job->status = icp::IcpJobStatus::COARSE_ALIGNMENT;
        ILOG << "[ICP] Running coarse alignment...";

        auto coarseStartTime = std::chrono::high_resolution_clock::now();

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            return;
        }

        /// Downsample for coarse alignment: point cloud. `sourcePoints` is already float,
        /// shifted by the job's rebase offset -- IcpChunk fields are the same representation,
        /// so the downsampled result can be assigned directly with no widen/unshift needed.
        int coarseDownsampleRatio = job->config.downsampleRatio * 2;  /// More aggressive for coarse
        auto coarseSource = icp::downsampleUniform(sourcePoints, coarseDownsampleRatio);

        ILOG << "[ICP] Coarse alignment(pc vs pseudo bim pts): " << coarseSource.size() << " vs " << pseudoFacePoints.size() << " points";

        /// Create coarse chunk
        icp::IcpChunk coarseChunk;
        coarseChunk.chunk_id = 0;
        coarseChunk.sourcePoints = std::move(coarseSource);
        coarseChunk.targetPoints = icp::narrowToFloat(pseudoFacePoints);
        coarseChunk.faceIndices = pseudoFaceIdx;
        coarseChunk.faceNormals = icp::narrowToFloat(targetNormals);
        coarseChunk.facePts = icp::narrowToFloat(facePts);
        coarseChunk.config = job->config;
        coarseChunk.config.maxCorrespondenceDistance *= 2.0;  /// Larger distance for coarse
        coarseChunk.offsetX = job->rebaseOffsetX;
        coarseChunk.offsetY = job->rebaseOffsetY;
        coarseChunk.offsetZ = job->rebaseOffsetZ;

        /// Run coarse ICP
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
        /// Phase 5: Fine Alignment (distributed across Slaves, one chunk per BIM element,
        /// when possible; falls back to a single Master-side pass otherwise)
        /// =============================================
        job->status = icp::IcpJobStatus::FINE_ALIGNMENT;

        auto fineStartTime = std::chrono::high_resolution_clock::now();

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            return;
        }

        /// Apply coarse transform to source points -- every chunk (distributed or fallback)
        /// starts from this coarse-aligned frame, refining from an identity initialTransform.
        /// job->coarseTransform was computed on absolute-frame chunk data (see Phase 4 above),
        /// so convert it to the equivalent shifted-frame transform before applying it directly
        /// to the (float, shifted) bulk array -- this avoids widening the whole bulk array to
        /// double just to apply one matrix.
        icp::Transform4x4 coarseTransformShifted = icp::toShiftedFrame(
            job->coarseTransform, job->rebaseOffsetX, job->rebaseOffsetY, job->rebaseOffsetZ);
        auto alignedSource = icp::transformPointCloudCopy(sourcePoints, coarseTransformShifted);

        /// Try to distribute fine alignment across connected Slaves, one chunk per BIM
        /// element (Revit Element ID). icp_dispatch_mutex_ serializes use of task_manager_
        /// across concurrent ICP jobs, since it is a single shared resource (see the comment
        /// on icp_dispatch_mutex_ in MasterApplication.h) -- if it's busy with another job,
        /// or if no slaves are connected, this job falls back to Master-only processing
        /// instead of blocking.
        std::unique_lock<std::mutex> dispatchLock(icp_dispatch_mutex_, std::try_to_lock);
        bool distributed = false;
        /// elementInfoByChunkId[i] describes the BIM element behind elementChunks[i]
        /// (loadIcpElementChunks assigns chunk_id == index, so this doubles as a chunk_id ->
        /// element lookup) -- used after Phase 6 to label job->elementResults.
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

                    /// Wait for all chunk results (or cancellation / timeout). Slaves run each
                    /// chunk to full local convergence and report one IcpResult per chunk, so
                    /// no per-iteration network round trip is needed here -- just polling for
                    /// the queue to drain.
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
            /// Fallback: single Master-side pass over the whole (downsampled) point cloud,
            /// exactly as before this feature existed.
            ILOG << "[ICP] Running fine alignment on Master (no slaves distributed to)...";

#ifdef _DEBUG
            ///[ToDo] replace it with a proper value
            coarseChunk.config.downsampleRatio = 1;  /// Larger distance for coarse
#endif

            /// `alignedSource` is already float, shifted by the job's rebase offset -- IcpChunk
            /// fields are the same representation, so no widen/unshift is needed here.
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
        /// Phase 6: Aggregate Results
        /// =============================================
        job->status = icp::IcpJobStatus::AGGREGATING;
        ILOG << "[ICP] Aggregating results from " << job->chunkResults.size() << " chunk(s)...";

        if (job->chunkResults.empty()) {
            throw std::runtime_error("Fine alignment produced no successful chunk results");
        }

        /// Weighted average of the per-chunk transforms (weighted by each chunk's source
        /// point count), with the rotation re-orthogonalized via SVD so the result is a
        /// proper rotation matrix rather than just an element-wise average. When there is
        /// exactly one chunk result (the fallback path), this reduces to that chunk's own
        /// transform -- same behavior as before distributed fine alignment existed.
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

        /// Combine coarse and fine transforms
        job->finalTransform = combinedFineTransform * job->coarseTransform;
        job->finalRMSE = (totalWeight > 0.0) ? (weightedRmseSum / totalWeight) : 0.0;

        ILOG << "[ICP] Final transform computed from " << job->chunkResults.size()
            << " chunk(s) (" << convergedCount << " converged), RMSE: " << job->finalRMSE;

        /// Record each element's own full (coarse + its own local refinement) transform,
        /// labeled by BIM element name/id, for saving alongside the combined result below.
        /// Only meaningful for the distributed (one-chunk-per-element) path -- the Master-only
        /// fallback has no per-element breakdown, so this stays empty there.
        job->elementResults.clear();
        for (const auto& r : job->chunkResults) {
            icp::IcpElementAlignment elem;
            elem.chunk_id = r.chunk_id;
            if (r.chunk_id < elementInfoByChunkId.size()) {
                elem.elementName = elementInfoByChunkId[r.chunk_id].name;
                elem.elementRevitId = elementInfoByChunkId[r.chunk_id].revitId;
            }
            elem.sourcePointCount = r.sourcePointCount;
            elem.targetPointCount = r.targetPointCount;
            elem.rmse = r.finalRMSE;
            elem.converged = r.converged;
            elem.success = r.success;
            elem.errorMessage = r.errorMessage;
            elem.transform = r.transform * job->coarseTransform;
            job->elementResults.push_back(elem);
        }

        /// =============================================
        /// Phase 7: Save Aligned Point Cloud
        /// =============================================
        job->status = icp::IcpJobStatus::SAVING_RESULT;
        ILOG << "[ICP] Applying final transform to " << sourcePoints.size()
            << " points and saving to: " << job->outputPath;

        /// job->finalTransform is absolute-frame (built entirely from absolute-frame chunk
        /// results, see Phase 6) -- convert to the equivalent shifted-frame transform before
        /// applying it directly to the (float, shifted) bulk array, same reasoning as Phase 5.
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
/// Synthetic Test Data Generator
/// =========================================

/// Generate a synthetic laser-scan point cloud from a BIM mesh: sample dense points on the
/// mesh surface (as if a scanner had measured it), apply a known "ground truth" rigid
/// transform (rotation about Z + translation) to simulate a real misalignment, optionally add
/// Gaussian noise to simulate scan measurement error, and save the result as a .las file. The
/// injected transform is printed so it can be compared against ICP's recovered `final_transform`
/// to validate correctness -- lets tests start with a handful of BIM elements (fast) and scale
/// up toward the full model before finally validating against real measured data.
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

        /// Sample dense points on the mesh surface (absolute frame, no offset -- this is a
        /// standalone test-data tool, not part of the distributed ICP pipeline).
        std::vector<std::array<double, 3>> points, facePts, faceNormals;
        std::vector<size_t> faceIdx;
        icp::generatePseudoPoints(meshes, grid_size, points, faceIdx, facePts, faceNormals);

        if (points.empty()) {
            ELOG << "[gensynth] No points generated -- check bim_folder/grid_size";
            return;
        }
        ILOG << "[gensynth] Generated " << points.size() << " synthetic scan points "
            << "(grid_size=" << grid_size << "m)";

        /// Ground-truth transform: rotation about Z (yaw) + translation. The rotation is
        /// applied about the *selected elements' own centroid*, not the absolute coordinate
        /// origin -- these BIM elements can sit hundreds of meters from the origin in real site
        /// coordinates, so rotating about the origin would displace them by meters (lever-arm
        /// effect) instead of producing a realistic small misalignment.
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

        std::mt19937 rng(12345); /// fixed seed for reproducibility
        std::normal_distribution<double> noise(0.0, noise_sigma);

        std::vector<las::PointData> outPoints;
        outPoints.reserve(points.size());
        for (const auto& p : points) {
            /// Shift to rotation center, rotate, shift back, then translate.
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

        /// The synthetic scan is `groundTruth(mesh)` -- i.e. the mesh, rotated/shifted. When ICP
        /// later aligns this scan (as source) back to the mesh (as target), its recovered
        /// final_transform should approximate the *inverse* of the transform applied here, not
        /// the transform itself. Print both the applied (rotation_z, shift) parameters and the
        /// equivalent absolute-frame inverse (R, t) that final_transform should match, so no
        /// mental inversion is needed when comparing.
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
/// ICP Result Handling (from Slaves)
/// =========================================

void MasterApplication::handleIcpResult(const icp::IcpResult& result, uint32_t task_id) {
    ILOG << "[ICP] Received result for task " << task_id
        << " (chunk: " << result.chunk_id
        << ", success: " << (result.success ? "Yes" : "No")
        << ", RMSE: " << result.finalRMSE << ")";

    /// [Fix] This used to be an unimplemented TODO. It is now wired up (task_id -> job_id via
    /// icp_task_to_job_, see MasterApplication.h) so a distributed fine-alignment chunk result
    /// lands on the correct job. As of 2026-07-11, nothing in this codebase actually dispatches
    /// ICP_FINE_ALIGNMENT tasks to Slaves -- POST /api/icp/start (processIcpJob()) computes
    /// coarse and fine alignment synchronously on the Master itself. That remains the only
    /// supported ICP path; this function is scaffolding for a future slave-distributed
    /// fine-alignment feature and is currently unreachable in production use.
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