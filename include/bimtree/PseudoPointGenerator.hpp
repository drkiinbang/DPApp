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
    inline double triangleArea(
        const pctree::XYZPoint& v0,
        const pctree::XYZPoint& v1,
        const pctree::XYZPoint& v2)
    {
        /// Edge vectors
        double e1x = v1[0] - v0[0];
        double e1y = v1[1] - v0[1];
        double e1z = v1[2] - v0[2];

        double e2x = v2[0] - v0[0];
        double e2y = v2[1] - v0[1];
        double e2z = v2[2] - v0[2];

        /// Cross product
        double cx = e1y * e2z - e1z * e2y;
        double cy = e1z * e2x - e1x * e2z;
        double cz = e1x * e2y - e1y * e2x;

        /// Area = 0.5 * |cross product|
        return 0.5 * std::sqrt(cx * cx + cy * cy + cz * cz);
    }

    /// Exact squared distance from point p to the closest point on triangle (a, b, c).
    /// Standard closest-point-on-triangle algorithm (Ericson, "Real-Time Collision
    /// Detection"), used so BIM-pointcloud QC distances reflect true point-to-surface
    /// distance rather than distance to the nearest sampled pseudo point (see
    /// pointToNearestPseudoPointCandidates() below for how candidate triangles are chosen).
    inline double pointToTriangleDistanceSquared(
        const std::array<double, 3>& p,
        const std::array<double, 3>& a,
        const std::array<double, 3>& b,
        const std::array<double, 3>& c)
    {
        auto sub = [](const std::array<double, 3>& x, const std::array<double, 3>& y) {
            return std::array<double, 3>{ x[0] - y[0], x[1] - y[1], x[2] - y[2] };
            };
        auto dot = [](const std::array<double, 3>& x, const std::array<double, 3>& y) {
            return x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
            };
        auto distSq = [&](const std::array<double, 3>& x) { return dot(x, x); };

        std::array<double, 3> ab = sub(b, a);
        std::array<double, 3> ac = sub(c, a);
        std::array<double, 3> ap = sub(p, a);

        double d1 = dot(ab, ap);
        double d2 = dot(ac, ap);
        if (d1 <= 0.0 && d2 <= 0.0) {
            return distSq(sub(p, a)); /// barycentric (1,0,0): closest to vertex a
        }

        std::array<double, 3> bp = sub(p, b);
        double d3 = dot(ab, bp);
        double d4 = dot(ac, bp);
        if (d3 >= 0.0 && d4 <= d3) {
            return distSq(sub(p, b)); /// closest to vertex b
        }

        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
            double v = d1 / (d1 - d3);
            std::array<double, 3> proj = { a[0] + v * ab[0], a[1] + v * ab[1], a[2] + v * ab[2] };
            return distSq(sub(p, proj)); /// closest to edge ab
        }

        std::array<double, 3> cp = sub(p, c);
        double d5 = dot(ab, cp);
        double d6 = dot(ac, cp);
        if (d6 >= 0.0 && d5 <= d6) {
            return distSq(sub(p, c)); /// closest to vertex c
        }

        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            double w = d2 / (d2 - d6);
            std::array<double, 3> proj = { a[0] + w * ac[0], a[1] + w * ac[1], a[2] + w * ac[2] };
            return distSq(sub(p, proj)); /// closest to edge ac
        }

        double va = d3 * d6 - d5 * d4;
        if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
            double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            std::array<double, 3> proj = { b[0] + w * (c[0] - b[0]), b[1] + w * (c[1] - b[1]), b[2] + w * (c[2] - b[2]) };
            return distSq(sub(p, proj)); /// closest to edge bc
        }

        /// Closest point is inside the face itself
        double denom = 1.0 / (va + vb + vc);
        double v = vb * denom;
        double w = vc * denom;
        std::array<double, 3> proj = {
            a[0] + ab[0] * v + ac[0] * w,
            a[1] + ab[1] * v + ac[1] * w,
            a[2] + ab[2] * v + ac[2] * w
        };
        return distSq(sub(p, proj));
    }

    /// Generate pseudo points from a single MeshChunk
    inline void generatePseudoPointsFromMesh(
        const chunkbim::MeshChunk& mesh,
        double gridSize,
        std::vector<std::array<double, 3>>& outPoints,
        std::vector<size_t>& faceIdx,
        std::vector<std::array<double, 3>>& facePts,
        std::vector<std::array<double, 3>>& outNormals)
    {
        if (mesh.faces.empty())
            return;

        double gridSizeSquared = gridSize * gridSize;

        for (size_t f = 0; f < mesh.faces.size(); ++f) {
            const auto& face = mesh.faces[f];

            std::array<double, 3> fPt;
            fPt[0] = face.vertices[0][0];
            fPt[1] = face.vertices[0][1];
            fPt[2] = face.vertices[0][2];

            facePts.push_back(fPt);

            /// Normal from face
            std::array<double, 3> fNormal;
            fNormal[0] = face.normal[0];
            fNormal[1] = face.normal[1];
            fNormal[2] = face.normal[2];

            outNormals.push_back(fNormal);

            const auto& v0 = face.vertices[0];
            const auto& v1 = face.vertices[1];
            const auto& v2 = face.vertices[2];

            /// Calculate triangle area
            double area = triangleArea(v0, v1, v2);
            if (area < 1e-10) continue;  /// Skip degenerate triangles

            /// Number of points to generate based on area and grid size
            int numPoints = static_cast<int>(std::ceil(area / gridSizeSquared));
            numPoints = (std::max)(1, (std::min)(numPoints, 1000));  /// Clamp

            /// Generate points using barycentric coordinates
            /// Use deterministic pattern for reproducibility
            int gridDim = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(numPoints))));

            for (int i = 0; i < gridDim; ++i) {
                for (int j = 0; j < gridDim - i; ++j) {
                    /// Barycentric coordinates
                    double u = (i + 0.5) / gridDim;
                    double v = (j + 0.5) / gridDim;
                    double w = 1.0 - u - v;

                    if (w < 0.0) continue;  /// Outside triangle

                    /// Interpolate position
                    std::array<double, 3> point;
                    point[0] = u * v0[0] + v * v1[0] + w * v2[0];
                    point[1] = u * v0[1] + v * v1[1] + w * v2[1];
                    point[2] = u * v0[2] + v * v1[2] + w * v2[2];

                    outPoints.push_back(point);
                    faceIdx.push_back(f);
                }
            }
        }
    }

    /// Generate pseudo points from multiple MeshChunks
    inline void generatePseudoPoints(
        const std::vector<chunkbim::MeshChunk>& chunks,
        double gridSize,
        std::vector<std::array<double, 3>>& outPoints,
        std::vector<size_t>& faceIdx,
        std::vector<std::array<double, 3>>& facePts,
        std::vector<std::array<double, 3>>& outNormals)
    {
        outPoints.clear();
        faceIdx.clear();
        facePts.clear();
        outNormals.clear();

        /// Estimate total points for reservation
        size_t totalFaces = 0;
        for (const auto& chunk : chunks) {
            totalFaces += chunk.faces.size();
        }

        outPoints.reserve(totalFaces * 4);  /// Rough estimate
        faceIdx.reserve(totalFaces * 4);  /// Rough estimate
        facePts.reserve(totalFaces);
        outNormals.reserve(totalFaces);

        /// Generate from each mesh
        for (const auto& chunk : chunks) {
            generatePseudoPointsFromMesh(chunk, gridSize, outPoints, faceIdx, facePts, outNormals);
        }

        outPoints.shrink_to_fit();
        faceIdx.shrink_to_fit();
    }

    /// Convert PointCloudChunk to ICP format
    inline std::vector<std::array<double, 3>> convertPointCloudToIcp(
        const std::vector<chunkpc::PointCloudChunk>& chunks)
    {
        std::vector<std::array<double, 3>> result;

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