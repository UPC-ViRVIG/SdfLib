#ifndef TRIANGLES_INFLUENCE_H
#define TRIANGLES_INFLUENCE_H

#ifdef ENOKI_AVAILABLE
#include <fcpw/fcpw.h>
#endif

#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "OctreeSdfUtils.h"
#include "InterpolationMethods.h"
#include "utils/Timer.h"
#include "utils/GJK.h"
#include <InteractiveComputerGraphics/TriangleMeshDistance.h>

#include <vector>
#include <array>
#include <glm/glm.hpp>

#define SDFLIB_PRINT_GJK_STATS

namespace sdflib
{

const std::array<glm::vec3, 8> childrens = 
{
    glm::vec3(-1.0f, -1.0f, -1.0f),
    glm::vec3(1.0f, -1.0f, -1.0f),
    glm::vec3(-1.0f, 1.0f, -1.0f),
    glm::vec3(1.0f, 1.0f, -1.0f),

    glm::vec3(-1.0f, -1.0f, 1.0f),
    glm::vec3(1.0f, -1.0f, 1.0f),
    glm::vec3(-1.0f, 1.0f, 1.0f),
    glm::vec3(1.0f, 1.0f, 1.0f)
};

namespace
{
template<size_t N, typename InterpolationMethod>
inline void standardCalculateVerticesInfo(  const glm::vec3 offset, const float size,
                                            const std::vector<uint32_t>& triangles,
                                            const std::array<glm::vec3, N>& pointsRelPos,
                                            const uint32_t pointsToInterpolateMask,
                                            const std::array<float, InterpolationMethod::NUM_COEFFICIENTS>& interpolationCoeff,
                                            std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, N>& outPointsValues,
                                            std::array<float, N>& outPointsInfo,
                                            const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
{
    outPointsInfo.fill(INFINITY);
    std::array<uint32_t, N> minIndex;

    std::array<glm::vec3, N> inPoints;
    for(uint32_t i=0; i < N; i++)
    {
        inPoints[i] = offset + pointsRelPos[i] * size;
    }

    for(uint32_t t : triangles)
    {
        for(uint32_t i=0; i < N; i++)
        {
            if(pointsToInterpolateMask & (1 << (N-i-1))) continue;
            const float dist = TriangleUtils::getSqDistPointAndTriangle(inPoints[i], trianglesData[t]);
            if(dist < outPointsInfo[i])
            {
                minIndex[i] = t;
                outPointsInfo[i] = dist;
            }
        }
    }

    for(uint32_t i=0; i < N; i++)
    {
        outPointsInfo[i] = glm::sqrt(outPointsInfo[i]);
        if(pointsToInterpolateMask & (1 << (N-i-1)))
        {
            InterpolationMethod::interpolateVertexValues(interpolationCoeff, 0.5f * pointsRelPos[i] + 0.5f, 2.0f * size, outPointsValues[i]);
        }
        else
        {
            InterpolationMethod::calculatePointValues(inPoints[i], minIndex[i], mesh, trianglesData, outPointsValues[i]);
        }
    }
}
}


template<typename T>
struct BasicTrianglesInfluence
{
    typedef T InterpolationMethod;

    typedef float VertexInfo;
    struct NodeInfo {};

    uint32_t gjkIter = 0;
    uint32_t gjkCallsInside = 0;
    std::array<uint32_t, 10> gjkIterHistogramInside = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t gjkCallsOutside = 0;
    std::array<uint32_t, 10> gjkIterHistogramOutside = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    template<size_t N>
    inline void calculateVerticesInfo(  const glm::vec3 nodeCenter, const float nodeHalfSize,
                                        const std::vector<uint32_t>& triangles,
                                        const std::array<glm::vec3, N>& pointsRelPos,
                                        const uint32_t pointsToInterpolateMask,
                                        const std::array<float, InterpolationMethod::NUM_COEFFICIENTS>& interpolationCoeff,
                                        std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, N>& outPointsValues,
                                        std::array<VertexInfo, N>& outPointsInfo,
                                        const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        standardCalculateVerticesInfo<N, InterpolationMethod>(
                                    nodeCenter, nodeHalfSize, triangles, pointsRelPos, 
                                    pointsToInterpolateMask, interpolationCoeff,
                                    outPointsValues, outPointsInfo, mesh, trianglesData);
    }

    inline void filterTriangles(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                const std::vector<uint32_t>& inTriangles, std::vector<uint32_t>& outTriangles,
                                const std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, 8>& verticesValues,
                                const std::array<VertexInfo, 8>& verticesInfo,
                                const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        outTriangles.clear();
        
        // Serach the maximum distance of all node verices minimum distance
        float maxMinDist = 0.0f;
        for(uint32_t i=0; i < 8; i++)
        {
            maxMinDist = glm::max(maxMinDist, verticesInfo[i]);
        }

        const std::vector<glm::vec3>& vertices = mesh.getVertices();
        const std::vector<uint32_t>& indices = mesh.getIndices();

        std::array<glm::vec3, 3> triangle;

        for(const uint32_t& idx : inTriangles)
        {
            triangle[0] = vertices[indices[3 * idx]] - nodeCenter;
            triangle[1] = vertices[indices[3 * idx + 1]] - nodeCenter;
            triangle[2] = vertices[indices[3 * idx + 2]] - nodeCenter;

            uint32_t iter = 0;
            // const float minDist = GJK::getMinDistance(glm::vec3(nodeHalfSize), triangle, &iter);
			// const bool isInside = minDist < maxMinDist;
            // const bool isInside = GJK::IsNear(glm::vec3(nodeHalfSize), triangle, maxMinDist, &iter);
            const bool isInside = GJK::IsNearMinimize(glm::vec3(nodeHalfSize), triangle, maxMinDist, &iter);

            if(isInside)
            {
                gjkIterHistogramInside[glm::min(iter, 9u)]++;
                gjkCallsInside++;
            }
            else
            {
                gjkIterHistogramOutside[glm::min(iter, 9u)]++;
                gjkCallsOutside++;
            }
            gjkIter += iter;

            if(isInside)
            {
                outTriangles.push_back(idx);
            }
        }
    }

    void printStatistics() 
    {
        SPDLOG_INFO("Mean of GJK iterations: {}", static_cast<float>(gjkIter) / static_cast<float>(gjkCallsInside + gjkCallsOutside));
        for(uint32_t p=0; p < 10; p++)
        {
            SPDLOG_INFO("Inter Outside {}: {}%", p, 100.0f * static_cast<float>(gjkIterHistogramOutside[p]) / static_cast<float>(gjkCallsOutside));
        }

        for(uint32_t p=0; p < 10; p++)
        {
            SPDLOG_INFO("Inter Inside {}: {}%", p, 100.0f * static_cast<float>(gjkIterHistogramInside[p]) / static_cast<float>(gjkCallsInside));
        }

        for(uint32_t p=0; p < 10; p++)
        {
            SPDLOG_INFO("Inter {}: {}%", p, 100.0f * static_cast<float>(gjkIterHistogramInside[p] + gjkIterHistogramOutside[p]) / static_cast<float>(gjkCallsInside + gjkCallsOutside));
        }
    }
};

template<typename T>
struct PreciseTrianglesInfluence
{
    typedef T InterpolationMethod;

    typedef float VertexInfo;
    struct NodeInfo {};

    template<size_t N>
    inline void calculateVerticesInfo(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                      const std::vector<uint32_t>& triangles,
                                      const std::array<glm::vec3, N>& pointsRelPos,
                                      const uint32_t pointsToInterpolateMask,
                                      const std::array<float, InterpolationMethod::NUM_COEFFICIENTS>& interpolationCoeff,
                                      std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, N>& outPointsValues,
                                      std::array<VertexInfo, N>& outPointsInfo,
                                      const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    { 
        standardCalculateVerticesInfo<N, InterpolationMethod>(
                                    nodeCenter, nodeHalfSize, triangles, pointsRelPos, 
                                    pointsToInterpolateMask, interpolationCoeff,
                                    outPointsValues, outPointsInfo, mesh, trianglesData);
    }

    inline void filterTriangles(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                const std::vector<uint32_t>& inTriangles, std::vector<uint32_t>& outTriangles,
                                const std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, 8>& verticesValues,
                                const std::array<VertexInfo, 8>& verticesInfo,
                                const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        outTriangles.clear();
        
        // Serach the maximum distance of all node verices minimum distance
        float maxMinDist = 0.0f;
        for(uint32_t i=0; i < 8; i++)
        {
            maxMinDist = glm::max(maxMinDist, verticesInfo[i]);
        }

        const std::vector<glm::vec3>& vertices = mesh.getVertices();
        const std::vector<uint32_t>& indices = mesh.getIndices();

        std::array<glm::vec3, 3> triangle;

        std::vector<std::pair<uint32_t,std::array<float, 8>>> triangleRegions(1);
        triangleRegions.reserve(inTriangles.size()) ;

        for(const uint32_t& idx : inTriangles)
        {
            const uint32_t trIndex = triangleRegions.size() - 1;
            triangleRegions[trIndex].first = idx;
            std::array<float, 8>& dist = triangleRegions[trIndex].second;
            
            bool valid = false;
            for(uint32_t c=0; c < 8; c++)
            {
                dist[c] = glm::sqrt(TriangleUtils::getSqDistPointAndTriangle(nodeCenter + childrens[c] * nodeHalfSize, trianglesData[idx]));
                valid = valid || dist[c] < maxMinDist;
            }

            if(valid) triangleRegions.resize(triangleRegions.size() + 1);
        }

		triangleRegions.resize(triangleRegions.size() - 1);

        for(const uint32_t& idx : inTriangles)
        {
            triangle[0] = vertices[indices[3 * idx]] - nodeCenter;
            triangle[1] = vertices[indices[3 * idx + 1]] - nodeCenter;
            triangle[2] = vertices[indices[3 * idx + 2]] - nodeCenter;

            bool isInside = true;
            for(const auto& region : triangleRegions)
            {
                if(region.first != idx && !GJK::isInsideConvexHull(nodeHalfSize, region.second, triangle))
                {
                    isInside = false;
                    break;
                }
            }

            if(isInside)
            {
                outTriangles.push_back(idx);
            }
        }
    }

    void printStatistics() {}
};

template<int NumVertexTests, typename T>
struct PerVertexTrianglesInfluence
{
    typedef T InterpolationMethod;

    typedef uint32_t VertexInfo;
    struct {} NodeInfo;

    uint32_t gjkIter = 0;
    uint32_t gjkCallsInside = 0;
    std::array<uint32_t, 20> gjkIterHistogramInside = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint32_t gjkCallsOutside = 0;
    std::array<uint32_t, 20> gjkIterHistogramOutside = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    float filterTime = 0.0f;

    const uint32_t CACHE_AXIS_POWER = 5;
    const uint32_t CACHE_AXIS_MASK = (1 << CACHE_AXIS_POWER) - 1;
    const uint32_t CACHE_AXIS_SIZE = 1 << CACHE_AXIS_POWER;
    std::vector<std::pair<glm::uvec3, VertexInfo>> vertexInfoCache;
    glm::vec3 coordToId;
    glm::vec3 minPoint;

    void initCaches(BoundingBox box, uint32_t maxDepth)
    {
        vertexInfoCache.resize(CACHE_AXIS_SIZE * CACHE_AXIS_SIZE * CACHE_AXIS_SIZE,
                               std::make_pair(glm::uvec3((1 << maxDepth) + 1), 0));
        coordToId = glm::vec3(static_cast<float>((1 << maxDepth)) / box.getSize());
        minPoint = box.min;
    }

    template<size_t N>
    inline void calculateVerticesInfo(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                      const std::vector<uint32_t>& triangles,
                                      const std::array<glm::vec3, N>& pointsRelPos,
                                      const uint32_t pointToInterpolateMask,
                                      const std::array<float, InterpolationMethod::NUM_COEFFICIENTS>& interpolationCoeff,
                                      std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, N>& outPointsValues,
                                      std::array<VertexInfo, N>& outPointsInfo,
                                      const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        std::array<float, N> minDistanceToPoint;
        minDistanceToPoint.fill(INFINITY); // It will locally used to store the minimum distance to vertex

        std::array<glm::vec3, N> inPoints;
        // for(uint32_t i=0; i < N; i++)
        // {
        //     inPoints[i] = nodeCenter + pointsRelPos[i] * nodeHalfSize;
        // }
        for(uint32_t i=0; i < N; i++)
        {
            inPoints[i] = nodeCenter + pointsRelPos[i] * nodeHalfSize;
            const glm::uvec3 pointId = glm::uvec3(glm::round((inPoints[i] - minPoint) * coordToId));

            const uint32_t cacheId = ((pointId.z & CACHE_AXIS_MASK) << (2*CACHE_AXIS_POWER)) | 
                                     ((pointId.y & CACHE_AXIS_MASK) << CACHE_AXIS_POWER) | 
                                     (pointId.x & CACHE_AXIS_MASK);

            if(vertexInfoCache[cacheId].first == pointId)
            {
                outPointsInfo[i] = vertexInfoCache[cacheId].second;
            }
            else
            {
                for(const uint32_t& t : triangles)
                {
                    const float dist = TriangleUtils::getSqDistPointAndTriangle(inPoints[i], trianglesData[t]);

                    if(dist < minDistanceToPoint[i])
                    {
                        outPointsInfo[i] = t;
                        minDistanceToPoint[i] = dist;
                    }
                }

                vertexInfoCache[cacheId] = std::make_pair(pointId, outPointsInfo[i]);
            }
        }

        for(uint32_t i=0; i < N; i++)
        {
            if(pointToInterpolateMask & (1 << (N-i-1)))
            {
                InterpolationMethod::interpolateVertexValues(interpolationCoeff, 0.5f * pointsRelPos[i] + 0.5f, 2.0f * nodeHalfSize, outPointsValues[i]);
            }
            else
            {
                InterpolationMethod::calculatePointValues(inPoints[i], outPointsInfo[i], mesh, trianglesData, outPointsValues[i]);
            }
        }
    }

    inline void filterTriangles(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                const std::vector<uint32_t>& inTriangles, std::vector<uint32_t>& outTriangles,
                                const std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, 8>& verticesValues,
                                const std::array<VertexInfo, 8>& verticesInfo,
                                const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        Timer timer;
        timer.start();
        outTriangles.clear();
        
        const std::vector<glm::vec3>& vertices = mesh.getVertices();
        const std::vector<uint32_t>& indices = mesh.getIndices();

        std::array<glm::vec3, 3> triangle;

        std::array<std::array<float, 8>, 8> triangleRegions;
        std::array<float, 8> minDistToVertices;
        minDistToVertices.fill(INFINITY);

        for(uint32_t i=0; i < 8; i++)
        {
            const uint32_t& idx = verticesInfo[i];

            for(uint32_t c=0; c < 8; c++)
            {
                triangleRegions[i][c] = 
                    glm::sqrt(TriangleUtils::getSqDistPointAndTriangle(nodeCenter + childrens[c] * nodeHalfSize, trianglesData[idx]));

                minDistToVertices[i] = glm::min(minDistToVertices[i], triangleRegions[i][c]);
            }

            for(uint32_t c=0; c < 8; c++)
            {
                triangleRegions[i][c] -= minDistToVertices[i];
            }
        }

        std::array<std::pair<uint32_t, float>, NumVertexTests> verticesToTest;
        for(const uint32_t& idx : inTriangles)
        {
            triangle[0] = vertices[indices[3 * idx]] - nodeCenter;
            triangle[1] = vertices[indices[3 * idx + 1]] - nodeCenter;
            triangle[2] = vertices[indices[3 * idx + 2]] - nodeCenter;

            verticesToTest.fill(std::make_pair(0, INFINITY));
            for(uint32_t r=0; r < 8; r++)
            {
                const float dist = 
                    TriangleUtils::getSqDistPointAndTriangle(nodeCenter + childrens[r] * nodeHalfSize, trianglesData[idx]);
                
                if(dist < verticesToTest[0].second)
                    verticesToTest[0] = std::make_pair(r, dist);

                for(uint32_t i=1; i < NumVertexTests; i++)
                {
                    if(verticesToTest[i-1].second < verticesToTest[i].second)
                        std::swap(verticesToTest[i-1], verticesToTest[i]);
                }
            }

            bool isInside = true;
            for(uint32_t r=0; r < NumVertexTests; r++)
            {
                const uint32_t vId = verticesToTest[r].first;
                uint32_t iter = 0;
                // if(verticesInfo[vId] != idx && !GJK::isInsideConvexHull(nodeHalfSize, triangleRegions[vId], triangle, glm::vec3(0.0f, 0.0f, (vId < 4) ? -1.0f : 1.0f), &iter))
                if(verticesInfo[vId] != idx && !GJK::IsNearMinimize(nodeHalfSize, triangleRegions[vId], triangle, minDistToVertices[vId], &iter))
                {
                    isInside = false;
                }

                #ifdef SDFLIB_PRINT_GJK_STATS
                if(verticesInfo[vId] != idx)
                {
                    if(isInside)
                    {
                        gjkIterHistogramInside[glm::min(iter, 19u)]++;
                        gjkCallsInside++;
                    }
                    else
                    {
                        gjkIterHistogramOutside[glm::min(iter, 19u)]++;
                        gjkCallsOutside++;
                    }
                    gjkIter += iter;
                }
                #endif
                
                if(!isInside)
                {
                    break;
                }
            }

            if(isInside)
            {
                outTriangles.push_back(idx);
            }
        }

        filterTime += timer.getElapsedSeconds();
    }

    void printStatistics() 
    {
        #ifdef SDFLIB_PRINT_GJK_STATS
        SPDLOG_INFO("Mean of GJK iterations: {}", static_cast<float>(gjkIter) / static_cast<float>(gjkCallsInside + gjkCallsOutside));
        for(uint32_t p=0; p < 20; p++)
        {
            SPDLOG_INFO("Inter Outside {}: {}%", p, 100.0f * static_cast<float>(gjkIterHistogramOutside[p]) / static_cast<float>(gjkCallsOutside));
        }

        for(uint32_t p=0; p < 20; p++)
        {
            SPDLOG_INFO("Inter Inside {}: {}%", p, 100.0f * static_cast<float>(gjkIterHistogramInside[p]) / static_cast<float>(gjkCallsInside));
        }

        for(uint32_t p=0; p < 20; p++)
        {
            SPDLOG_INFO("Inter {}: {}%", p, 100.0f * static_cast<float>(gjkIterHistogramInside[p] + gjkIterHistogramOutside[p]) / static_cast<float>(gjkCallsInside + gjkCallsOutside));
        }

        SPDLOG_INFO("Filter Time: {}", filterTime);
        #endif
    }
};

template<typename T>
struct PerVertexTrianglesInfluence<8, T>
{
    typedef T InterpolationMethod;

    typedef uint32_t VertexInfo;
    struct {} NodeInfo;

    uint32_t gjkIter = 0;
    uint32_t gjkCalls = 0;

    const uint32_t CACHE_AXIS_POWER = 5;
    const uint32_t CACHE_AXIS_MASK = (1 << CACHE_AXIS_POWER) - 1;
    const uint32_t CACHE_AXIS_SIZE = 1 << CACHE_AXIS_POWER;
    std::vector<std::pair<glm::uvec3, VertexInfo>> vertexInfoCache;
    glm::vec3 coordToId;
    glm::vec3 minPoint;

    void initCaches(BoundingBox box, uint32_t maxDepth)
    {
        vertexInfoCache.resize(CACHE_AXIS_SIZE * CACHE_AXIS_SIZE * CACHE_AXIS_SIZE,
                               std::make_pair(glm::uvec3((1 << maxDepth) + 1), 0));
        coordToId = glm::vec3(static_cast<float>((1 << maxDepth)) / box.getSize());
        minPoint = box.min;
    }

    template<size_t N>
    inline void calculateVerticesInfo(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                      const std::vector<uint32_t>& triangles,
                                      const std::array<glm::vec3, N>& pointsRelPos,
                                      const uint32_t pointToInterpolateMask,
                                      const std::array<float, InterpolationMethod::NUM_COEFFICIENTS>& interpolationCoeff,
                                      std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, N>& outPointsValues,
                                      std::array<VertexInfo, N>& outPointsInfo,
                                      const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        std::array<float, N> minDistanceToPoint;
        minDistanceToPoint.fill(INFINITY); // It will locally used to store the minimum distance to vertex

        std::array<glm::vec3, N> inPoints;
        // for(uint32_t i=0; i < N; i++)
        // {
        //     inPoints[i] = nodeCenter + pointsRelPos[i] * nodeHalfSize;
        // }
        for(uint32_t i=0; i < N; i++)
        {
            inPoints[i] = nodeCenter + pointsRelPos[i] * nodeHalfSize;
            const glm::uvec3 pointId = glm::uvec3(glm::round((inPoints[i] - minPoint) * coordToId));

            const uint32_t cacheId = ((pointId.z & CACHE_AXIS_MASK) << (2*CACHE_AXIS_POWER)) | 
                                     ((pointId.y & CACHE_AXIS_MASK) << CACHE_AXIS_POWER) | 
                                     (pointId.x & CACHE_AXIS_MASK);

            if(vertexInfoCache[cacheId].first == pointId)
            {
                outPointsInfo[i] = vertexInfoCache[cacheId].second;
            }
            else
            {
                for(const uint32_t& t : triangles)
                {
                    const float dist = TriangleUtils::getSqDistPointAndTriangle(inPoints[i], trianglesData[t]);

                    if(dist < minDistanceToPoint[i])
                    {
                        outPointsInfo[i] = t;
                        minDistanceToPoint[i] = dist;
                    }
                }

                vertexInfoCache[cacheId] = std::make_pair(pointId, outPointsInfo[i]);
            }
        }

        // for(const uint32_t& t : triangles)
        // {
        //     for(uint32_t i=0; i < N; i++)
        //     {
        //         const float dist = TriangleUtils::getSqDistPointAndTriangle(inPoints[i], trianglesData[t]);

        //         if(dist < minDistanceToPoint[i])
        //         {
        //             outPointsInfo[i] = t;
        //             minDistanceToPoint[i] = dist;
        //         }
        //     }
        // }

        for(uint32_t i=0; i < N; i++)
        {
            if(pointToInterpolateMask & (1 << (N-i-1)))
            {
                InterpolationMethod::interpolateVertexValues(interpolationCoeff, 0.5f * pointsRelPos[i] + 0.5f, 2.0f * nodeHalfSize, outPointsValues[i]);
            }
            else
            {
                InterpolationMethod::calculatePointValues(inPoints[i], outPointsInfo[i], mesh, trianglesData, outPointsValues[i]);
            }
        }
    }

    inline void filterTriangles(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                const std::vector<uint32_t>& inTriangles, std::vector<uint32_t>& outTriangles,
                                const std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, 8>& verticesValues,
                                const std::array<VertexInfo, 8>& verticesInfo,
                                const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        outTriangles.clear();
        
        const std::vector<glm::vec3>& vertices = mesh.getVertices();
        const std::vector<uint32_t>& indices = mesh.getIndices();

        std::array<glm::vec3, 3> triangle;

        std::array<std::array<float, 8>, 8> triangleRegions;
        std::array<float, 8> minDistToVertices;
        minDistToVertices.fill(INFINITY);

        for(uint32_t i=0; i < 8; i++)
        {
            const uint32_t& idx = verticesInfo[i];

            for(uint32_t c=0; c < 8; c++)
            {
                triangleRegions[i][c] = 
                    glm::sqrt(TriangleUtils::getSqDistPointAndTriangle(nodeCenter + childrens[c] * nodeHalfSize, trianglesData[idx]));
                
                minDistToVertices[i] = glm::min(minDistToVertices[i], triangleRegions[i][c]);
            }

            for(uint32_t c=0; c < 8; c++)
            {
                triangleRegions[i][c] -= minDistToVertices[i];
            }
        }

        for(const uint32_t& idx : inTriangles)
        {
            triangle[0] = vertices[indices[3 * idx]] - nodeCenter;
            triangle[1] = vertices[indices[3 * idx + 1]] - nodeCenter;
            triangle[2] = vertices[indices[3 * idx + 2]] - nodeCenter;

            bool isInside = true;
            for(uint32_t r=0; r < 8; r++)
            {
                //if(verticesInfo[r] != idx && !GJK::isInsideConvexHull(nodeHalfSize, triangleRegions[r], triangle, glm::vec3(0.0f, 0.0f, (r < 4) ? -1.0f : 1.0f)))
                if(verticesInfo[r] != idx && !GJK::IsNearMinimize(nodeHalfSize, triangleRegions[r], triangle, minDistToVertices[r]))
                {
                    isInside = false;
                    break;
                }
            }

            if(isInside)
            {
                outTriangles.push_back(idx);
            }
        }
    }

    void printStatistics() {}
};

template<typename T>
struct PerNodeRegionTrianglesInfluence
{
    typedef T InterpolationMethod;

    typedef uint32_t VertexInfo;
    struct {} NodeInfo;

    uint64_t gjkIter = 0;
    uint64_t gjkCallsInside = 0;
    std::array<uint64_t, 20> gjkIterHistogramInside = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint64_t gjkCallsOutside = 0;
    std::array<uint64_t, 20> gjkIterHistogramOutside = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    float filterTime = 0.0f;

    const uint32_t CACHE_AXIS_POWER = 5;
    const uint32_t CACHE_AXIS_MASK = (1 << CACHE_AXIS_POWER) - 1;
    const uint32_t CACHE_AXIS_SIZE = 1 << CACHE_AXIS_POWER;
    std::vector<std::pair<glm::uvec3, VertexInfo>> vertexInfoCache;
    glm::vec3 coordToId;
    glm::vec3 minPoint;

    void initCaches(BoundingBox box, uint32_t maxDepth)
    {
        vertexInfoCache.resize(CACHE_AXIS_SIZE * CACHE_AXIS_SIZE * CACHE_AXIS_SIZE,
                               std::make_pair(glm::uvec3((1 << maxDepth) + 1), 0));
        coordToId = glm::vec3(static_cast<float>((1 << maxDepth)) / box.getSize());
        minPoint = box.min;
    }
    
    template<size_t N>
    inline void calculateVerticesInfo(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                      const std::vector<uint32_t>& triangles,
                                      const std::array<glm::vec3, N>& pointsRelPos,
                                      const uint32_t pointToInterpolateMask,
                                      const std::array<float, InterpolationMethod::NUM_COEFFICIENTS>& interpolationCoeff,
                                      std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, N>& outPointsValues,
                                      std::array<VertexInfo, N>& outPointsInfo,
                                      const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        std::array<float, N> minDistanceToPoint;
        minDistanceToPoint.fill(INFINITY); // It will locally used to store the minimum distance to vertex

        std::array<glm::vec3, N> inPoints;
        // for(uint32_t i=0; i < N; i++)
        // {
        //     inPoints[i] = nodeCenter + pointsRelPos[i] * nodeHalfSize;
        // }
        for(uint32_t i=0; i < N; i++)
        {
            inPoints[i] = nodeCenter + pointsRelPos[i] * nodeHalfSize;
            const glm::uvec3 pointId = glm::uvec3(glm::round((inPoints[i] - minPoint) * coordToId));

            const uint32_t cacheId = ((pointId.z & CACHE_AXIS_MASK) << (2*CACHE_AXIS_POWER)) | 
                                     ((pointId.y & CACHE_AXIS_MASK) << CACHE_AXIS_POWER) | 
                                     (pointId.x & CACHE_AXIS_MASK);

            if(vertexInfoCache[cacheId].first == pointId)
            {
                outPointsInfo[i] = vertexInfoCache[cacheId].second;
            }
            else
            {
                for(const uint32_t& t : triangles)
                {
                    const float dist = TriangleUtils::getSqDistPointAndTriangle(inPoints[i], trianglesData[t]);

                    if(dist < minDistanceToPoint[i])
                    {
                        outPointsInfo[i] = t;
                        minDistanceToPoint[i] = dist;
                    }
                }

                vertexInfoCache[cacheId] = std::make_pair(pointId, outPointsInfo[i]);
            }
        }

        // for(const uint32_t& t : triangles)
        // {
        //     for(uint32_t i=0; i < N; i++)
        //     {
        //         const float dist = TriangleUtils::getSqDistPointAndTriangle(inPoints[i], trianglesData[t]);

        //         if(dist < minDistanceToPoint[i])
        //         {
        //             outPointsInfo[i] = t;
        //             minDistanceToPoint[i] = dist;
        //         }
        //     }
        // }

        for(uint32_t i=0; i < N; i++)
        {
            if(pointToInterpolateMask & (1 << (N-i-1)))
            {
                InterpolationMethod::interpolateVertexValues(interpolationCoeff, 0.5f * pointsRelPos[i] + 0.5f, 2.0f * nodeHalfSize, outPointsValues[i]);
            }
            else
            {
                InterpolationMethod::calculatePointValues(inPoints[i], outPointsInfo[i], mesh, trianglesData, outPointsValues[i]);
            }
        }
    }

    inline void filterTriangles(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                const std::vector<uint32_t>& inTriangles, std::vector<uint32_t>& outTriangles,
                                const std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, 8>& verticesValues,
                                const std::array<VertexInfo, 8>& verticesInfo,
                                const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        outTriangles.clear();
        
        const std::vector<glm::vec3>& vertices = mesh.getVertices();
        const std::vector<uint32_t>& indices = mesh.getIndices();

        std::array<glm::vec3, 3> triangle;

        std::array<std::array<float, 8>, 8> triangleRegions;
        std::array<float, 8> minDistToVertices;
        minDistToVertices.fill(INFINITY);

        for(uint32_t i=0; i < 8; i++)
        {
            const uint32_t& idx = verticesInfo[i];

            for(uint32_t c=0; c < 8; c++)
            {
                triangleRegions[i][c] = 
                    glm::sqrt(TriangleUtils::getSqDistPointAndTriangle(nodeCenter + childrens[c] * nodeHalfSize, trianglesData[idx]));
                
                minDistToVertices[i] = glm::min(minDistToVertices[i], triangleRegions[i][c]);
            }

            for(uint32_t c=0; c < 8; c++)
            {
                triangleRegions[i][c] -= minDistToVertices[i];
            }
        }

        auto sign = [](float a, uint32_t val) 
        {
            return (a > 0.0f) ? val : -val;
        };

        for(const uint32_t& idx : inTriangles)
        {
            triangle[0] = vertices[indices[3 * idx]] - nodeCenter;
            triangle[1] = vertices[indices[3 * idx + 1]] - nodeCenter;
            triangle[2] = vertices[indices[3 * idx + 2]] - nodeCenter;

            bool isInside = true;
            
            const glm::vec3 point = 0.3333333f * (triangle[0] + triangle[1] + triangle[2]);
            uint32_t vId = ((point.z > 0) ? 4 : 0) + 
                            ((point.y > 0) ? 2 : 0) + 
                            ((point.x > 0) ? 1 : 0);

            uint32_t iter = 0;
            //if(verticesInfo[vId] != idx && !GJK::isInsideConvexHull(nodeHalfSize, triangleRegions[vId], triangle, glm::vec3(0.0f, 0.0f, (vId < 4) ? -1.0f : 1.0f), &iter))
            if(verticesInfo[vId] != idx && !GJK::IsNearMinimize(nodeHalfSize, triangleRegions[vId], triangle, minDistToVertices[vId], &iter))
            {
                isInside = false;
            }

            //if(isInside)
            //{
            //    if(glm::abs(point.x) < glm::abs(point.y)) vId -= (glm::abs(point.x) < glm::abs(point.z)) ? sign(point.x, 1) : sign(point.z, 4);
            //    else vId -= (glm::abs(point.y) < glm::abs(point.z)) ? sign(point.y, 2) : sign(point.z, 4);

            //    if(verticesInfo[vId] != idx && !GJK::IsNearMinimize(nodeHalfSize, triangleRegions[vId], triangle, minDistToVertices[vId], &iter))
            //    {
            //        isInside = false;
            //    }
            //}

#ifdef SDFLIB_PRINT_GJK_STATS
            if(verticesInfo[vId] != idx)
            {
                if(isInside)
                {
                    gjkIterHistogramInside[glm::min(iter, 19u)]++;
                    gjkCallsInside++;
                }
                else
                {
                    gjkIterHistogramOutside[glm::min(iter, 19u)]++;
                    gjkCallsOutside++;
                }
                gjkIter += iter;
            }
#endif

            if(isInside)
            {
                outTriangles.push_back(idx);
            }
        }
    }

    void printStatistics() 
    {
#ifdef SDFLIB_PRINT_GJK_STATS
        SPDLOG_INFO("Mean of GJK iterations: {}", static_cast<float>(gjkIter) / static_cast<float>(gjkCallsInside + gjkCallsOutside));
        for(uint32_t p=0; p < 20; p++)
        {
            SPDLOG_INFO("Inter Outside {}: {}%", p, 100.0f * static_cast<float>(gjkIterHistogramOutside[p]) / static_cast<float>(gjkCallsOutside));
        }

        for(uint32_t p=0; p < 20; p++)
        {
            SPDLOG_INFO("Inter Inside {}: {}%", p, 100.0f * static_cast<float>(gjkIterHistogramInside[p]) / static_cast<float>(gjkCallsInside));
        }

        for(uint32_t p=0; p < 20; p++)
        {
            SPDLOG_INFO("Inter {}: {}%", p, 100.0f * static_cast<float>(gjkIterHistogramInside[p] + gjkIterHistogramOutside[p]) / static_cast<float>(gjkCallsInside + gjkCallsOutside));
        }
#endif
    }
};

class ICG
{
public:
    ICG(const Mesh& mesh)
    : mesh_distance(toDoubleVector(mesh.getVertices()),
                    *reinterpret_cast<const std::vector<std::array<int, 3>>*>(&mesh.getIndices()))
    {}

    inline float getDistance(glm::vec3 samplePoint)
    {
        tmd::Result result = mesh_distance.signed_distance({ samplePoint.x, samplePoint.y, samplePoint.z });
        return result.distance;
    }

    inline uint32_t getNearestTriangle(glm::vec3 samplePoint)
    {
        tmd::Result result = mesh_distance.signed_distance({ samplePoint.x, samplePoint.y, samplePoint.z });
        //numEvaluatedTriangles += static_cast<uint32_t>(result.numEvalTriangles);
        numEvaluatedTriangles = 0;
        numQueries++;
        return static_cast<uint32_t>(result.triangle_id);
    }

    inline uint32_t getNumQueries() const { return numQueries; }
    inline uint32_t getNumEvaluatedTriangles() { return numEvaluatedTriangles; }
private:
    tmd::TriangleMeshDistance mesh_distance;
    uint64_t numQueries = 0;
    uint32_t numEvaluatedTriangles = 0;

    std::vector<std::array<double, 3>> toDoubleVector(const std::vector<glm::vec3>& vec)
    {
        std::vector<std::array<double, 3>> res(vec.size());
        for(uint32_t i=0; i < vec.size(); i++)
        {
            res[i] = { static_cast<double>(vec[i].x), static_cast<double>(vec[i].y), static_cast<double>(vec[i].z) };
        }

        return res;
    }
};

template<typename T>
struct VHQueries
{
    typedef T InterpolationMethod;

    typedef uint32_t VertexInfo;
    struct {} NodeInfo;

    const uint32_t CACHE_AXIS_POWER = 5;
    const uint32_t CACHE_AXIS_MASK = (1 << CACHE_AXIS_POWER) - 1;
    const uint32_t CACHE_AXIS_SIZE = 1 << CACHE_AXIS_POWER;
    std::vector<std::pair<glm::uvec3, VertexInfo>> vertexInfoCache;
    glm::vec3 coordToId;
    glm::vec3 minPoint;

    std::shared_ptr<ICG> icg = nullptr;

    void initCaches(BoundingBox box, uint32_t maxDepth)
    {
        vertexInfoCache.resize(CACHE_AXIS_SIZE * CACHE_AXIS_SIZE * CACHE_AXIS_SIZE,
                               std::make_pair(glm::uvec3((1 << maxDepth) + 1), 0));
        coordToId = glm::vec3(static_cast<float>((1 << maxDepth)) / box.getSize());
        minPoint = box.min;
    }
    
    template<size_t N>
    inline void calculateVerticesInfo(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                      const std::vector<uint32_t>& triangles,
                                      const std::array<glm::vec3, N>& pointsRelPos,
                                      const uint32_t pointToInterpolateMask,
                                      const std::array<float, InterpolationMethod::NUM_COEFFICIENTS>& interpolationCoeff,
                                      std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, N>& outPointsValues,
                                      std::array<VertexInfo, N>& outPointsInfo,
                                      const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        if(icg == nullptr)
        {
            icg = std::make_shared<ICG>(mesh);
        }

        std::array<glm::vec3, N> inPoints;
        for(uint32_t i=0; i < N; i++)
        {
            if(pointToInterpolateMask & (1 << (N-i-1)))
            {
                InterpolationMethod::interpolateVertexValues(interpolationCoeff, 0.5f * pointsRelPos[i] + 0.5f, 2.0f * nodeHalfSize, outPointsValues[i]);
            }
            else
            {
                inPoints[i] = nodeCenter + pointsRelPos[i] * nodeHalfSize;
                const glm::uvec3 pointId = glm::uvec3(glm::round((inPoints[i] - minPoint) * coordToId));

                const uint32_t cacheId = ((pointId.z & CACHE_AXIS_MASK) << (2*CACHE_AXIS_POWER)) | 
                                        ((pointId.y & CACHE_AXIS_MASK) << CACHE_AXIS_POWER) | 
                                        (pointId.x & CACHE_AXIS_MASK);

                if(vertexInfoCache[cacheId].first == pointId)
                {
                    outPointsInfo[i] = vertexInfoCache[cacheId].second;
                }
                else
                {
                    outPointsInfo[i] = icg->getNearestTriangle(inPoints[i]);

                    vertexInfoCache[cacheId] = std::make_pair(pointId, outPointsInfo[i]);
                }
                
                InterpolationMethod::calculatePointValues(inPoints[i], outPointsInfo[i], mesh, trianglesData, outPointsValues[i]);
            }
        }
    }

    inline void filterTriangles(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                const std::vector<uint32_t>& inTriangles, std::vector<uint32_t>& outTriangles,
                                const std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, 8>& verticesValues,
                                const std::array<VertexInfo, 8>& verticesInfo,
                                const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        outTriangles.clear();
    }

    void printStatistics() 
    {
        SPDLOG_INFO("Mean triangles evaulated per query {}", static_cast<float>(icg->getNumEvaluatedTriangles()) / static_cast<float>(icg->getNumQueries()));
        SPDLOG_INFO("Num queries made {}", icg->getNumQueries());
    }
};

#ifdef ENOKI_AVAILABLE
template<typename T>
struct FCPWQueries
{
    typedef T InterpolationMethod;

    typedef uint32_t VertexInfo;
    struct {} NodeInfo;

    const uint32_t CACHE_AXIS_POWER = 5;
    const uint32_t CACHE_AXIS_MASK = (1 << CACHE_AXIS_POWER) - 1;
    const uint32_t CACHE_AXIS_SIZE = 1 << CACHE_AXIS_POWER;
    std::vector<std::pair<glm::uvec3, VertexInfo>> vertexInfoCache;
    glm::vec3 coordToId;
    glm::vec3 minPoint;

    std::shared_ptr<fcpw::Scene<3>> scene = nullptr;
    bool initialized = false;

    void initCaches(BoundingBox box, uint32_t maxDepth)
    {
        vertexInfoCache.resize(CACHE_AXIS_SIZE * CACHE_AXIS_SIZE * CACHE_AXIS_SIZE,
                               std::make_pair(glm::uvec3((1 << maxDepth) + 1), 0));
        coordToId = glm::vec3(static_cast<float>((1 << maxDepth)) / box.getSize());
        minPoint = box.min;
    }
    
    template<size_t N>
    inline void calculateVerticesInfo(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                      const std::vector<uint32_t>& triangles,
                                      const std::array<glm::vec3, N>& pointsRelPos,
                                      const uint32_t pointToInterpolateMask,
                                      const std::array<float, InterpolationMethod::NUM_COEFFICIENTS>& interpolationCoeff,
                                      std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, N>& outPointsValues,
                                      std::array<VertexInfo, N>& outPointsInfo,
                                      const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        if(!initialized)
        {
            scene = std::make_shared<fcpw::Scene<3>>();
            scene->setObjectTypes({{fcpw::PrimitiveType::Triangle}});
            const uint32_t numVertices = mesh.getVertices().size();
            const uint32_t numIndices = mesh.getIndices().size() / 3;
            scene->setObjectVertexCount(numVertices, 0);
            scene->setObjectTriangleCount(numIndices, 0);

            // specify the vertex positions
            for (int i = 0; i < numVertices; i++) {
                scene->setObjectVertex(*reinterpret_cast<const fcpw::Vector3*>(&mesh.getVertices()[i]), i, 0);
            }

            // specify the triangle indices
            for (int i = 0; i < numIndices; i++) {
                scene->setObjectTriangle(reinterpret_cast<const int*>(&mesh.getIndices()[3*i]), i, 0);
            }

            scene->build(fcpw::AggregateType::Bvh_Volume, true);

            initialized = true;
        }

        std::array<glm::vec3, N> inPoints;
        for(uint32_t i=0; i < N; i++)
        {
            inPoints[i] = nodeCenter + pointsRelPos[i] * nodeHalfSize;
            const glm::uvec3 pointId = glm::uvec3(glm::round((inPoints[i] - minPoint) * coordToId));

            const uint32_t cacheId = ((pointId.z & CACHE_AXIS_MASK) << (2*CACHE_AXIS_POWER)) | 
                                     ((pointId.y & CACHE_AXIS_MASK) << CACHE_AXIS_POWER) | 
                                     (pointId.x & CACHE_AXIS_MASK);

            if(vertexInfoCache[cacheId].first == pointId)
            {
                outPointsInfo[i] = vertexInfoCache[cacheId].second;
            }
            else
            {
                fcpw::Interaction<3> interaction;
                scene->findClosestPoint(*reinterpret_cast<fcpw::Vector3*>(&inPoints[i]), interaction);
                outPointsInfo[i] = interaction.primitiveIndex;

                vertexInfoCache[cacheId] = std::make_pair(pointId, outPointsInfo[i]);
            }
        }

        for(uint32_t i=0; i < N; i++)
        {
            if(pointToInterpolateMask & (1 << (N-i-1)))
            {
                InterpolationMethod::interpolateVertexValues(interpolationCoeff, 0.5f * pointsRelPos[i] + 0.5f, 2.0f * nodeHalfSize, outPointsValues[i]);
            }
            else
            {
                InterpolationMethod::calculatePointValues(inPoints[i], outPointsInfo[i], mesh, trianglesData, outPointsValues[i]);
            }
        }
    }

    inline void filterTriangles(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                const std::vector<uint32_t>& inTriangles, std::vector<uint32_t>& outTriangles,
                                const std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, 8>& verticesValues,
                                const std::array<VertexInfo, 8>& verticesInfo,
                                const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        outTriangles.clear();
    }

    void printStatistics() 
    {
    }
};
#endif

}

#endif
