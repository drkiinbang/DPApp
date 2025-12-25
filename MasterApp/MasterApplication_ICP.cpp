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

        /// Optional output path
        auto output_opt = MiniJson::get_ex<std::string>(json, "output_path");
        if (output_opt) {
            job->outputPath = *output_opt;
        }
        else {
            /// Default output: input_icp_aligned.las
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
        /// Phase 1: Loading Data
        job->status = icp::IcpJobStatus::LOADING_DATA;
        ILOG << "[ICP] Loading data...";

        /// Check cancellation
        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            ILOG << "[ICP] Job cancelled: " << job->jobId;
            return;
        }

        /// TODO: Load LAS file using existing LaslibReader
        /// std::vector<std::array<float, 3>> pointCloud = loadLasFile(job->lasFilePath);
        /// job->totalPointCount = pointCloud.size();

        /// TODO: Load GLTF/BIM mesh using existing BimInfo
        /// BimInfo bimInfo;
        /// bimInfo.load(job->bimFolderPath);
        /// job->totalMeshTriangles = bimInfo.getTotalFaceCount();

        /// Placeholder: Simulate loading
        job->totalPointCount = 100000;
        job->totalMeshTriangles = 50000;
        ILOG << "[ICP] Data loaded: " << job->totalPointCount << " points, "
            << job->totalMeshTriangles << " triangles";

        /// Phase 2: Generate Pseudo Points
        job->status = icp::IcpJobStatus::GENERATING_PSEUDO;
        ILOG << "[ICP] Generating pseudo points from mesh...";

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            return;
        }

        /// TODO: Use PseudoPtsInMesh.hpp to generate pseudo points
        /// auto pseudoPoints = generateMeshPseudoPoints(bimInfo, job->config.pseudoPointGridSize);
        /// job->totalPseudoPoints = pseudoPoints.size();

        job->totalPseudoPoints = 200000;
        ILOG << "[ICP] Pseudo points generated: " << job->totalPseudoPoints;

        /// Phase 3: Coarse Alignment (on Master)
        job->status = icp::IcpJobStatus::COARSE_ALIGNMENT;
        ILOG << "[ICP] Running coarse alignment...";

        auto coarseStartTime = std::chrono::high_resolution_clock::now();

        if (job->cancelRequested.load()) {
            job->status = icp::IcpJobStatus::CANCELLED;
            return;
        }

        /// TODO: Run coarse ICP on downsampled data
        /// auto downsampledSource = icp::downsampleUniform(pointCloud, job->config.downsampleRatio);
        /// auto downsampledTarget = icp::downsampleUniform(pseudoPoints, job->config.downsampleRatio);
        /// icp::IcpChunk coarseChunk;
        /// coarseChunk.sourcePoints = downsampledSource;
        /// coarseChunk.targetPoints = downsampledTarget;
        /// std::atomic<bool> cancel{false};
        /// auto coarseResult = icp::runIcp(coarseChunk, cancel);
        /// job->coarseTransform = coarseResult.transform;
        /// job->coarseRMSE = coarseResult.finalRMSE;

        /// Placeholder: Identity transform
        job->coarseTransform = icp::Transform4x4::identity();
        job->coarseRMSE = 0.5f;
        job->coarseConverged = true;

        auto coarseEndTime = std::chrono::high_resolution_clock::now();
        job->coarseAlignmentTimeMs = std::chrono::duration<double, std::milli>(
            coarseEndTime - coarseStartTime).count();

        ILOG << "[ICP] Coarse alignment completed: RMSE=" << job->coarseRMSE
            << " in " << (job->coarseAlignmentTimeMs / 1000.0) << "s";

        /// Phase 4: Distribute to Slaves (Fine Alignment)
        job->status = icp::IcpJobStatus::DISTRIBUTING;
        ILOG << "[ICP] Distributing chunks to slaves...";

        /// TODO: Create IcpChunks from spatial partitions
        /// std::vector<icp::IcpChunk> chunks = createIcpChunks(pointCloud, pseudoPoints, job->coarseTransform);
        /// job->totalChunks = chunks.size();

        /// Placeholder: Simulate chunks
        job->totalChunks = 10;
        ILOG << "[ICP] Created " << job->totalChunks << " chunks";

        /// Phase 5: Fine Alignment (on Slaves)
        job->status = icp::IcpJobStatus::FINE_ALIGNMENT;
        auto fineStartTime = std::chrono::high_resolution_clock::now();

        /// TODO: Send IcpChunks to slaves via TaskManager
        /// for (auto& chunk : chunks) {
        ///     sendIcpTaskToSlave(chunk, job->jobId);
        /// }
        /// 
        /// Wait for all results...

        /// Placeholder: Simulate completion
        for (uint32_t i = 0; i < job->totalChunks; ++i) {
            if (job->cancelRequested.load()) {
                job->status = icp::IcpJobStatus::CANCELLED;
                return;
            }

            /// Simulate chunk result
            icp::IcpResult result;
            result.chunk_id = i;
            result.success = true;
            result.converged = true;
            result.finalRMSE = 0.01f + (i * 0.001f);
            result.actualIterations = 10 + i;
            result.transform = job->coarseTransform;

            job->chunkResults.push_back(result);
            job->completedChunks++;

            std::this_thread::sleep_for(std::chrono::milliseconds(100)); /// Simulate work
        }

        auto fineEndTime = std::chrono::high_resolution_clock::now();
        job->fineAlignmentTimeMs = std::chrono::duration<double, std::milli>(
            fineEndTime - fineStartTime).count();

        ILOG << "[ICP] Fine alignment completed in " << (job->fineAlignmentTimeMs / 1000.0) << "s";

        /// Phase 6: Aggregate Results
        job->status = icp::IcpJobStatus::AGGREGATING;
        ILOG << "[ICP] Aggregating results...";

        job->finalTransform = icp::aggregateResultsWeighted(job->chunkResults);

        /// Calculate final RMSE
        float totalRMSE = 0.0f;
        int validCount = 0;
        for (const auto& r : job->chunkResults) {
            if (r.success) {
                totalRMSE += r.finalRMSE;
                validCount++;
            }
        }
        job->finalRMSE = (validCount > 0) ? (totalRMSE / validCount) : 0.0f;

        ILOG << "[ICP] Final RMSE: " << job->finalRMSE;

        /// Phase 7: Apply Transform and Save
        job->status = icp::IcpJobStatus::SAVING_RESULT;
        ILOG << "[ICP] Saving aligned point cloud...";

        /// TODO: Apply transform to point cloud and save
        /// auto alignedPoints = icp::transformPointCloudCopy(pointCloud, job->finalTransform);
        /// saveLasFile(job->outputPath, alignedPoints);

        ILOG << "[ICP] Output saved to: " << job->outputPath;

        /// Complete
        job->status = icp::IcpJobStatus::COMPLETED;
        job->endTimeMs = static_cast<double>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());

        ILOG << "[ICP] Job completed: " << job->jobId
            << " (RMSE: " << job->finalRMSE
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

    /// TODO: Find job by task_id and update chunk results
    /// This requires mapping task_id -> job_id
    /// 
    /// For now, this is a placeholder
}