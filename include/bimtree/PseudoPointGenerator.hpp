#pragma once

/// ============================================================================
/// PseudoPointGenerator.hpp - Generate pseudo points from mesh surfaces
/// ============================================================================
/// 
/// Generates uniformly distributed points on triangle mesh surfaces.
/// Used as target points for ICP alignment.
/// 
/// Location: F:\repository\DPApp\include\bimtree\PseudoPointGenerator.hpp
/// ============================================================================

#include "../bim/MeshChunk.h"
#include "../IcpTypes.h"

#include <vector>
#include <array>
#include <cmath>
#include <random>

namespace icp {

    /// Calculate triangle area using cross product
    inline float triangleArea(
        const pctree::XYZPoint& v0,
        const pctree::XYZPoint& v1,
        const pctree::XYZPoint& v2)
    {
        /// Edge vectors
        float e1x = v1[0] - v0[0];
        float e1y = v1[1] - v0[1];
        float e1z = v1[2] - v0[2];

        float e2x = v2[0] - v0[0];
        float e2y = v2[1] - v0[1];
        float e2z = v2[2] - v0[2];

        /// Cross product
        float cx = e1y * e2z - e1z * e2y;
        float cy = e1z * e2x - e1x * e2z;
        float cz = e1x * e2y - e1y * e2x;

        /// Area = 0.5 * |cross product|
        return 0.5f * std::sqrt(cx * cx + cy * cy + cz * cz);
    }

    /// Generate pseudo points from a single MeshChunk
    /// @param mesh Input mesh chunk
    /// @param gridSize Approximate spacing between generated points
    /// @param outPoints Output points
    /// @param outNormals Output normals (one per point)
    inline void generatePseudoPointsFromMesh(
        const chunkbim::MeshChunk& mesh,
        float gridSize,
        std::vector<std::array<float, 3>>& outPoints,
        std::vector<std::array<float, 3>>& outNormals)
    {
        if (mesh.vertices.empty() || mesh.faces.empty()) {
            return;
        }

        float gridSizeSquared = gridSize * gridSize;

        for (const auto& face : mesh.faces) {
            const auto& v0 = mesh.vertices[face.idxVtx[0]];
            const auto& v1 = mesh.vertices[face.idxVtx[1]];
            const auto& v2 = mesh.vertices[face.idxVtx[2]];

            /// Calculate triangle area
            float area = triangleArea(v0, v1, v2);
            if (area < 1e-10f) continue;  /// Skip degenerate triangles

            /// Number of points to generate based on area and grid size
            int numPoints = static_cast<int>(std::ceil(area / gridSizeSquared));
            numPoints = (std::max)(1, (std::min)(numPoints, 1000));  /// Clamp

            /// Generate points using barycentric coordinates
            /// Use deterministic pattern for reproducibility
            int gridDim = static_cast<int>(std::ceil(std::sqrt(static_cast<float>(numPoints))));
            
            for (int i = 0; i < gridDim; ++i) {
                for (int j = 0; j < gridDim - i; ++j) {
                    /// Barycentric coordinates
                    float u = (i + 0.5f) / gridDim;
                    float v = (j + 0.5f) / gridDim;
                    float w = 1.0f - u - v;

                    if (w < 0.0f) continue;  /// Outside triangle

                    /// Interpolate position
                    std::array<float, 3> point;
                    point[0] = u * v0[0] + v * v1[0] + w * v2[0];
                    point[1] = u * v0[1] + v * v1[1] + w * v2[1];
                    point[2] = u * v0[2] + v * v1[2] + w * v2[2];

                    /// Normal from face
                    std::array<float, 3> normal;
                    normal[0] = face.normal[0];
                    normal[1] = face.normal[1];
                    normal[2] = face.normal[2];

                    outPoints.push_back(point);
                    outNormals.push_back(normal);
                }
            }
        }
    }

    /// Generate pseudo points from multiple MeshChunks
    /// @param meshes Input mesh chunks (BIM data)
    /// @param gridSize Approximate spacing between generated points (default 0.1m)
    /// @param outPoints Output points
    /// @param outNormals Output normals
    inline void generatePseudoPoints(
        const std::vector<chunkbim::MeshChunk>& meshes,
        float gridSize,
        std::vector<std::array<float, 3>>& outPoints,
        std::vector<std::array<float, 3>>& outNormals)
    {
        outPoints.clear();
        outNormals.clear();

        /// Estimate total points for reservation
        size_t totalFaces = 0;
        for (const auto& mesh : meshes) {
            totalFaces += mesh.faces.size();
        }
        outPoints.reserve(totalFaces * 4);  /// Rough estimate
        outNormals.reserve(totalFaces * 4);

        /// Generate from each mesh
        for (const auto& mesh : meshes) {
            generatePseudoPointsFromMesh(mesh, gridSize, outPoints, outNormals);
        }
    }

    /// Convert PointCloudChunk to ICP format
    inline std::vector<std::array<float, 3>> convertPointCloudToIcp(
        const std::vector<chunkpc::PointCloudChunk>& chunks)
    {
        std::vector<std::array<float, 3>> result;

        /// Count total points
        size_t totalPoints = 0;
        for (const auto& chunk : chunks) {
            totalPoints += chunk.points.size();
        }
        result.reserve(totalPoints);

        /// Convert
        for (const auto& chunk : chunks) {
            for (const auto& pt : chunk.points) {
                result.push_back({ pt[0], pt[1], pt[2] });
            }
        }

        return result;
    }

    /// Count total triangles in meshes
    inline size_t countTotalTriangles(const std::vector<chunkbim::MeshChunk>& meshes) {
        size_t total = 0;
        for (const auto& mesh : meshes) {
            total += mesh.faces.size();
        }
        return total;
    }

} // namespace icp
