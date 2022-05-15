#ifndef TRIANGLES_INFLUENCE_H
#define TRIANGLES_INFLUENCE_H

#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "OctreeSdfUtils.h"
#include "InterpolationMethods.h"
#include "utils/Timer.h"
#include "utils/GJK.h"

#include <vector>
#include <array>
#include <glm/glm.hpp>

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
template<int N, typename InterpolationMethod>
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

    template<int N>
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

            const float minDist = GJK::getMinDistance(glm::vec3(nodeHalfSize), triangle);

            if(minDist <= maxMinDist)
            {
                outTriangles.push_back(idx);
            }
        }
    }

    void printStatistics() {}
};

template<typename T>
struct PreciseTrianglesInfluence
{
    typedef T InterpolationMethod;

    typedef float VertexInfo;
    struct NodeInfo {};

    template<int N>
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
    uint32_t gjkCalls = 0;

    template<int N>
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
        for(uint32_t i=0; i < N; i++)
        {
            inPoints[i] = nodeCenter + pointsRelPos[i] * nodeHalfSize;
        }

        for(const uint32_t& t : triangles)
        {
            for(uint32_t i=0; i < N; i++)
            {
                const float dist = TriangleUtils::getSqDistPointAndTriangle(inPoints[i], trianglesData[t]);

                if(dist < minDistanceToPoint[i])
                {
                    outPointsInfo[i] = t;
                    minDistanceToPoint[i] = dist;
                }
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
        
        const std::vector<glm::vec3>& vertices = mesh.getVertices();
        const std::vector<uint32_t>& indices = mesh.getIndices();

        std::array<glm::vec3, 3> triangle;

        std::array<std::array<float, 8>, 8> triangleRegions;

        for(uint32_t i=0; i < 8; i++)
        {
            const uint32_t& idx = verticesInfo[i];

            for(uint32_t c=0; c < 8; c++)
            {
                triangleRegions[i][c] = 
                    glm::sqrt(TriangleUtils::getSqDistPointAndTriangle(nodeCenter + childrens[c] * nodeHalfSize, trianglesData[idx]));
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
                if(verticesInfo[vId] != idx && !GJK::isInsideConvexHull(nodeHalfSize, triangleRegions[vId], triangle, glm::vec3(0.0f, 0.0f, (r < 4) ? -1.0f : 1.0f)))
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

// template<typename InterpolationMethod>
// inline void PerVertexTrianglesInfluence<8, InterpolationMethod>::filterTriangles(
//                                         const glm::vec3 nodeCenter, const float nodeHalfSize,
//                                         const std::vector<uint32_t>& inTriangles, std::vector<uint32_t>& outTriangles,
//                                         const std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, 8>& distanceToVertices,
//                                         const std::array<VertexInfo, 8>& verticesInfo,
//                                         const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
// {
//     outTriangles.clear();
        
//     const std::vector<glm::vec3>& vertices = mesh.getVertices();
//     const std::vector<uint32_t>& indices = mesh.getIndices();

//     std::array<glm::vec3, 3> triangle;

//     std::array<std::array<float, 8>, 8> triangleRegions;

//     for(uint32_t i=0; i < 8; i++)
//     {
//         const uint32_t& idx = verticesInfo[i];

//         for(uint32_t c=0; c < 8; c++)
//         {
//             triangleRegions[i][c] = 
//                 glm::sqrt(TriangleUtils::getSqDistPointAndTriangle(nodeCenter + childrens[c] * nodeHalfSize, trianglesData[idx]));
//         }
//     }

//     for(const uint32_t& idx : inTriangles)
//     {
//         triangle[0] = vertices[indices[3 * idx]] - nodeCenter;
//         triangle[1] = vertices[indices[3 * idx + 1]] - nodeCenter;
//         triangle[2] = vertices[indices[3 * idx + 2]] - nodeCenter;

//         bool isInside = true;
//         for(uint32_t r=0; r < 8; r++)
//         {
//             if(verticesInfo[r] != idx && !GJK::isInsideConvexHull(nodeHalfSize, triangleRegions[r], triangle, glm::vec3(0.0f, 0.0f, (r < 4) ? -1.0f : 1.0f)))
//             {
//                 isInside = false;
//                 break;
//             }
//         }

//         if(isInside)
//         {
//             outTriangles.push_back(idx);
//         }
//     }
// }

#endif