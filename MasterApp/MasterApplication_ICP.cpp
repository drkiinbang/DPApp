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

/// Data loading from DLLs
#include "../LasImport/LasImport.h"
#include "../BimImport/BimImport.h"

#include <chrono>
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

    std::stringstream ss;
    ss << "icp_" << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S")
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
            if (conv_thresh) job->config.convergenceThreshold = static_cast<float>(*conv_thresh);

            auto max_dist = MiniJson::get_ex<double>(cfg, "max_correspondence_distance");
            if (max_dist) job->config.maxCorrespondenceDistance = static_cast<float>(*max_dist);

            auto ds_ratio = MiniJson::get_ex<double>(cfg, "downsample_ratio");
            if (ds_ratio) job->config.downsampleRatio = static_cast<int>(*ds_ratio);

            auto grid_size = MiniJson::get_ex<double>(cfg, "pseudo_point_grid_size");
            if (grid_size) job->config.pseudoPointGridSize = static_cast<float>(*grid_size);
        }

        /// Optional LAS offset (dx, dy, dz)
        auto offset_opt = MiniJson::get_object(json, "offset");
        if (offset_opt) {
            auto& off = *offset_opt;
            auto dx = MiniJson::get_ex<double>(off, "dx");
            if (dx) job->offsetX = static_cast<float>(*dx);

            auto dy = MiniJson::get_ex<double>(off, "dy");
            if (dy) job->offsetY = static_cast<float>(*dy);

            auto dz = MiniJson::get_ex<double>(off, "dz");
            if (dz) job->offsetZ = static_cast<float>(*dz);
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
            return createErrorResponse(202, "Job not yet completed. Status: " + 
                                       std::string(icp::statusToString(job->status)));
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

void MasterApplication::processIcpJob(std::shared_ptr<icp::IcpJob> job) {
    ILOG << "[ICP] Starting job processing: " << job->jobId;

    try {
        /// =============================================
        /// Phase 1: Loading LAS Data
        /// =============================================
        job->status = icp::IcpJobStatus::LOADING_DATA;
        ILOG << "[ICP] Loading LAS file: " << job->lasFilePath;

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            ILOG << "[ICP] Job cancelled: " << job->jobId;
            return;
        }

        /// Load LAS file
        std::vector<chunkpc::PointCloudChunk> lasChunks;
        const uint32_t maxPointsPerChunk = 100000;

        if (!loadLasFile(job->lasFilePath, lasChunks, maxPointsPerChunk)) {
            throw std::runtime_error("Failed to load LAS file: " + job->lasFilePath);
        }

        /// Convert to ICP format
        std::vector<std::array<float, 3>> sourcePoints = icp::convertPointCloudToIcp(lasChunks);
        job->totalPointCount = sourcePoints.size();

        ILOG << "[ICP] LAS loaded: " << job->totalPointCount << " points";

        /// Apply offset to source points
        if (job->offsetX != 0.0f || job->offsetY != 0.0f || job->offsetZ != 0.0f) {
            ILOG << "[ICP] Applying offset: dx=" << job->offsetX
                << ", dy=" << job->offsetY
                << ", dz=" << job->offsetZ;

            for (auto& pt : sourcePoints) {
                pt[0] += job->offsetX;
                pt[1] += job->offsetY;
                pt[2] += job->offsetZ;
            }
        }

        /// =============================================
        /// Phase 2: Loading BIM/GLTF Data
        /// =============================================
        ILOG << "[ICP] Loading GLTF from: " << job->bimFolderPath;

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
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

        /// =============================================
        /// Phase 3: Generate Pseudo Points from Mesh
        /// =============================================
        job->status = icp::IcpJobStatus::GENERATING_PSEUDO;
        ILOG << "[ICP] Generating pseudo points (grid size: " << job->config.pseudoPointGridSize << "m)...";

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            return;
        }

        std::vector<std::array<float, 3>> pseudoFacePoints;
        std::vector<size_t> pseudoFaceIdx;
        std::vector<std::array<float, 3>> facePts;
        std::vector<std::array<float, 3>> targetNormals;
        /// [ToDo] use directly meshes, instead of extraction
        icp::generatePseudoPoints(meshes, job->config.pseudoPointGridSize, pseudoFacePoints, pseudoFaceIdx, facePts, targetNormals);

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

        /// Downsample for coarse alignment: point cluod
        int coarseDownsampleRatio = job->config.downsampleRatio * 2;  /// More aggressive for coarse
        auto coarseSource = icp::downsampleUniform(sourcePoints, coarseDownsampleRatio);

        /// Downsample for coarse alignment: BIM points
        std::vector<size_t> coarseIdx;
        std::vector<std::array<float, 3>> coarseTarget;
        if (!icp::downsampleUniform(pseudoFacePoints, pseudoFaceIdx, coarseTarget, coarseIdx, coarseDownsampleRatio)) {
            throw std::runtime_error("Error from downsampleUniform");
        }

        ILOG << "[ICP] Coarse alignment: " << coarseSource.size() << " vs " << coarseTarget.size() << " points";

        /// Create coarse chunk
        icp::IcpChunk coarseChunk;
        coarseChunk.chunk_id = 0;
        coarseChunk.sourcePoints = std::move(coarseSource);
        coarseChunk.targetPoints = std::move(coarseTarget);
        coarseChunk.faceIndices = std::move(coarseIdx);
        coarseChunk.faceNormals = targetNormals;
        coarseChunk.facePts = facePts;
        coarseChunk.config = job->config;
        coarseChunk.config.maxCorrespondenceDistance *= 2.0f;  /// Larger distance for coarse

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
        /// Phase 5: Fine Alignment (Single Pass on Master)
        /// =============================================
        job->status = icp::IcpJobStatus::FINE_ALIGNMENT;
        ILOG << "[ICP] Running fine alignment...";

        auto fineStartTime = std::chrono::high_resolution_clock::now();

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            return;
        }

        /// Apply coarse transform to source points
        auto alignedSource = icp::transformPointCloudCopy(sourcePoints, job->coarseTransform);

        /// Downsample for fine alignment (less aggressive): point cloud
        auto fineSource = icp::downsampleUniform(alignedSource, job->config.downsampleRatio);

        /// Downsample for fine alignment (less aggressive): BIM points
        std::vector<size_t> fineIdx;
        std::vector<std::array<float, 3>> fineTarget;
        if (!icp::downsampleUniform(pseudoFacePoints, pseudoFaceIdx, fineTarget, fineIdx, job->config.downsampleRatio)) {
            throw std::runtime_error("Error from downsampleUniform: fine");
        }

        ILOG << "[ICP] Fine alignment: " << fineSource.size() << " vs " << fineTarget.size() << " points";

        /// Create fine chunk
        icp::IcpChunk fineChunk;
        fineChunk.chunk_id = 0;
        fineChunk.sourcePoints = std::move(fineSource);
        fineChunk.targetPoints = std::move(fineTarget);
        fineChunk.faceIndices = std::move(fineIdx);
        fineChunk.faceNormals = targetNormals;
        fineChunk.facePts = facePts;
        fineChunk.config = job->config;

        /// Run fine ICP
        icp::IcpResult fineResult = icp::runIcp(fineChunk, job->cancelRequested);

        auto fineEndTime = std::chrono::high_resolution_clock::now();
        job->fineAlignmentTimeMs = std::chrono::duration<double, std::milli>(
            fineEndTime - fineStartTime).count();

        /// Store result
        job->chunkResults.push_back(fineResult);
        job->completedChunks = 1;
        job->totalChunks = 1;

        ILOG << "[ICP] Fine alignment: RMSE=" << fineResult.finalRMSE
             << ", converged=" << (fineResult.converged ? "Yes" : "No")
             << ", iterations=" << fineResult.actualIterations
             << ", time=" << (job->fineAlignmentTimeMs / 1000.0) << "s";

        /// =============================================
        /// Phase 6: Aggregate Results
        /// =============================================
        job->status = icp::IcpJobStatus::AGGREGATING;
        ILOG << "[ICP] Aggregating results...";

        /// Combine coarse and fine transforms
        job->finalTransform = fineResult.transform * job->coarseTransform;
        job->finalRMSE = fineResult.finalRMSE;

        ILOG << "[ICP] Final transform computed, RMSE: " << job->finalRMSE;

        /// =============================================
        /// Phase 7: Complete (No file saving)
        /// =============================================
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
/// ICP Result Handling (from Slaves)
/// =========================================

void MasterApplication::handleIcpResult(const icp::IcpResult& result, uint32_t task_id) {
    ILOG << "[ICP] Received result for task " << task_id
         << " (chunk: " << result.chunk_id
         << ", success: " << (result.success ? "Yes" : "No")
         << ", RMSE: " << result.finalRMSE << ")";

    /// TODO: For distributed processing, map task_id -> job_id
    /// and update the corresponding job's chunk results
}
