#include "LasImport.h"

#include <iostream>
#include <memory>
#include <vector>

#include "../include/LaslibReader.hpp"
//#include "./../include/bimtree/nanoflann.hpp"

// LAStools headers
#include "lasreader.hpp"
#include "laswriter.hpp"
#include "lasdefinitions.hpp"


bool loadLasFileStreaming(const std::string& file_path, std::vector<chunkpc::PointCloudChunk>& chunks, const uint32_t max_points_per_chunk)
{
    try {
        las::LASToolsReader lasReader;

        if (!lasReader.open(file_path)) {
            std::cerr << "Failed to open input file: " << file_path << std::endl;
            return false;
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

            chunks.push_back(chunkpc::PointCloudChunk());
            chunkpc::PointCloudChunk& current_chunk = chunks.back();
            current_chunk.chunk_id = chunk_counter;
            current_chunk.points.reserve(lasReader.getLoadedPointCount());

            for (const auto& p : lasReader.getLoadedPoints()) {
                current_chunk.points.emplace_back(p.x, p.y, p.z);
            }

            current_chunk.calculateBounds();
            total_points_loaded += current_chunk.points.size();
            chunk_counter++;

            // Progress reporting (every chunk)
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

    return true;
}