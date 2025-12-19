#pragma once

#ifdef LASIMPORT_EXPORT
#define LASIMPORT_API __declspec(dllexport)
#else
#define LASIMPORT_API __declspec(dllimport)
#endif

#define API LASIMPORT_API

#include <string>

#include "../include/bim/MeshChunk.h"

bool API loadLasFile(const std::string& file_path, std::vector<chunkpc::PointCloudChunk>& chunks, const uint32_t max_points_per_chunk);