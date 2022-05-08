#ifndef TRIANGLES_INFLUENCE_H
#define TRIANGLES_INFLUENCE_H

#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "OctreeSdfUtils.h"
#include "utils/Timer.h"

#include <vector>
#include <array>
#include <glm/glm.hpp>

const std::array<std::pair<uint32_t, uint32_t>, 12> nodeEdgesIndices
{
    std::make_pair(0, 1),
    std::make_pair(1, 3),
    std::make_pair(3, 2),
    std::make_pair(2, 1),

    std::make_pair(0, 4),
    std::make_pair(1, 5),
    std::make_pair(2, 6),
    std::make_pair(3, 7),

    std::make_pair(4, 5),
    std::make_pair(5, 7),
    std::make_pair(7, 6),
    std::make_pair(6, 5),
};

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

class TrianglesInfluence
{
public:
    TrianglesInfluence(float halfNodeSize)
    {
        mNodeHalfSize = halfNodeSize;
        mSpheresShape.resize(8);
        mSpheresShape[0] = std::make_pair(glm::vec3(-halfNodeSize), INFINITY);
        mSpheresShape[1] = std::make_pair(glm::vec3(halfNodeSize, -halfNodeSize, -halfNodeSize), INFINITY);
        mSpheresShape[2] = std::make_pair(glm::vec3(-halfNodeSize, halfNodeSize, -halfNodeSize), INFINITY);
        mSpheresShape[3] = std::make_pair(glm::vec3(halfNodeSize, halfNodeSize, -halfNodeSize), INFINITY);
        
        mSpheresShape[4] = std::make_pair(glm::vec3(-halfNodeSize, -halfNodeSize, halfNodeSize), INFINITY);
        mSpheresShape[5] = std::make_pair(glm::vec3(halfNodeSize, -halfNodeSize, halfNodeSize), INFINITY);
        mSpheresShape[6] = std::make_pair(glm::vec3(-halfNodeSize, halfNodeSize, halfNodeSize), INFINITY);
        mSpheresShape[7] = std::make_pair(glm::vec3(halfNodeSize), INFINITY);

        mEdgesIntervals.fill(std::vector<IntervalNode>());
    }

    void addTriangle(const std::array<float, 8>& distanceToVertices)
    {
        std::array<bool, 8> needsUpdate;

        // Mark which vertices need update
        for(uint32_t i=0; i < 8; i++)
        {
            needsUpdate[i] = mSpheresShape[i].second > distanceToVertices[i];
        }

        // Put mid points if needed
        for(uint32_t edgeId=0; edgeId < 12; edgeId++)
        {
            const std::pair<uint32_t, uint32_t>& edgeVertices = nodeEdgesIndices[edgeId];
            std::vector<IntervalNode>& intervalArray = mEdgesIntervals[edgeId];
            if(needsUpdate[edgeVertices.first] && needsUpdate[edgeVertices.second])
            {
                intervalArray.clear();
            }
            else
            {
                float newMinTRadius, newDiffTRadius;
                calculateOuterTangent(0.0f, distanceToVertices[nodeEdgesIndices[edgeId].first], 
                                      1.0f, distanceToVertices[nodeEdgesIndices[edgeId].second],
                                      newMinTRadius, newDiffTRadius);

                // Intersect the convex interval array and the new line
				const float lastMinTRadius = (intervalArray.empty()) ? mSpheresShape[edgeVertices.first].second : intervalArray.back().radius;
                bool removing = needsUpdate[edgeVertices.first];
                float minT = 0.0f, maxT = 0.0f;
                uint32_t nextInterval = 0;
                uint32_t i;
                float newT;
                for(i=0; i < intervalArray.size(); i++)
                {
                    IntervalNode& inter = intervalArray[i];
                    minT = maxT;
                    maxT = intervalArray[i].maxT;

                    newT = (newMinTRadius - inter.minTRadius) / 
                           (inter.diffTRadius - newDiffTRadius);

                    if(newT > minT && newT < maxT) // If the lines intersect
                    {
                        if(removing) break;
                        else {
                            // Create the new interval
                            inter.position = mSpheresShape[edgeVertices.first].first * (1.0f - newT) +
                                            mSpheresShape[edgeVertices.second].first * newT;
                            inter.radius = newMinTRadius + newDiffTRadius * newT;
                            inter.maxT = newT;
                            nextInterval = i + 1;
                            removing = true;
                        }
                    }
                }

                if(removing ^ needsUpdate[edgeVertices.second])
                {
                    if(i >= intervalArray.size())
                    {
                        float minTRadius, diffTRadius;
                        calculateOuterTangent(maxT, lastMinTRadius, 
                                              1.0f, mSpheresShape[edgeVertices.second].second,
                                              minTRadius, diffTRadius);
                        newT = (newMinTRadius - minTRadius) /
							   (diffTRadius - newDiffTRadius);
                    }
                    if(i == nextInterval) intervalArray.emplace(intervalArray.begin() + nextInterval, IntervalNode());
                    else if(i > nextInterval + 1)
                    {
                        // Move the previous intervals
                        for(uint32_t j=nextInterval + 1; i < intervalArray.size(); j++, i++)
                            intervalArray[j] = intervalArray[i];
                    }

                    // Insert the new interval
                    const float maxTRadius = newMinTRadius + newDiffTRadius * newT;
                    const float minT = (nextInterval > 0) ? intervalArray[nextInterval-1].maxT : 0.0f;
                    const float minTRadius = (nextInterval > 0) 
								? intervalArray[nextInterval-1].radius 
								: glm::min(distanceToVertices[edgeVertices.first], mSpheresShape[edgeVertices.first].second);
                    float resMinTRadius, resDiffTRadius;
                    calculateOuterTangent(minT, minTRadius, newT, maxTRadius,
                                          resMinTRadius, resDiffTRadius);
                    intervalArray[nextInterval] = IntervalNode
                    {
                        mSpheresShape[edgeVertices.first].first * (1.0f - newT) +
                        mSpheresShape[edgeVertices.second].first * newT,
                        newMinTRadius + newDiffTRadius * newT,

						resMinTRadius,
                        resDiffTRadius,
                        newT
                    };
                }
            }
        }

        // Rewrite the vertices
        for(uint32_t i=0; i < 8; i++)
        {
            if(needsUpdate[i]) mSpheresShape[i].second = distanceToVertices[i];
        }
    }

    void getSpheresShape(std::vector<std::pair<glm::vec3, float>>& outShape) const
    {
        outShape.clear();

        for(const auto& p : mSpheresShape) outShape.push_back(p);

        for(const std::vector<IntervalNode>& interval : mEdgesIntervals)
        {
            for(const IntervalNode& node : interval)
                outShape.push_back(std::make_pair(node.position, node.radius));
        }
    }
private:
    struct IntervalNode
    {
        glm::vec3 position;
        float radius;

        float minTRadius;
        float diffTRadius;
        float maxT;
    };

    float mNodeHalfSize;
    std::vector<std::pair<glm::vec3, float>> mSpheresShape;
    std::array<std::vector<IntervalNode>, 12> mEdgesIntervals;

    inline void calculateOuterTangent(float t1, float radius1, float t2, float radius2,
                                      float& outMinTRadius, float& outDiffTRadius)
    {
        const float length = 2.0f * glm::abs(t2 - t1) * mNodeHalfSize;
        const float radiusDiff = glm::abs(radius1 - radius2);
        
        const float sin = radiusDiff / length;
        const float cos = glm::sqrt(1.0f - sin * sin);

        const float x1 = glm::sign(radius1 - radius2) * radius1 * sin / length;
        const float y1 = radius1 * cos;
		const float x2 = 1.0f + glm::sign(radius2 - radius1) * radius2 * sin / length;
        const float y2 = radius2 * cos;

        const float slope = (y2 - y1) / (x2 - x1);

        outMinTRadius = y1 - slope * (t1 + x1);
        outDiffTRadius = slope;
    }
};

namespace
{
template<int N, typename VertexInfo>
inline void standardCalculateVerticesInfo(  const glm::vec3 offset, const float size,
                                            const std::vector<uint32_t>& triangles,
                                            const std::array<glm::vec3, N>& pointsRelPos,
                                            const uint32_t pointsToInterpolateMask,
                                            const std::array<float, 8>& interpolationPoints,
                                            std::array<float, N>& outDistanceToPoint,
                                            std::array<VertexInfo, N>& outPointInfo,
                                            const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
{
    outDistanceToPoint.fill(INFINITY);
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
            if(dist < outDistanceToPoint[i])
            {
                minIndex[i] = t;
                outDistanceToPoint[i] = dist;
            }
        }
    }

    for(uint32_t i=0; i < N; i++)
    {
        if(pointsToInterpolateMask & (1 << (N-i-1)))
        {
            outDistanceToPoint[i] = interpolateValue(reinterpret_cast<const float*>(&interpolationPoints), 0.5f * pointsRelPos[i] + 0.5f);
        }
        else
        {
            outDistanceToPoint[i] = TriangleUtils::getSignedDistPointAndTriangle(inPoints[i], trianglesData[minIndex[i]]);
        }
    }
}
}


struct BasicTrianglesInfluence
{
    struct VertexInfo {};
    struct NodeInfo {};

    template<int N>
    inline void calculateVerticesInfo(  const glm::vec3 nodeCenter, const float nodeHalfSize,
                                        const std::vector<uint32_t>& triangles,
                                        const std::array<glm::vec3, N>& pointsRelPos,
                                        const uint32_t pointsToInterpolateMask,
                                        const std::array<float, 8>& interpolationPoints,
                                        std::array<float, N>& outDistanceToPoint,
                                        std::array<VertexInfo, N>& outPointInfo,
                                        const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        standardCalculateVerticesInfo(nodeCenter, nodeHalfSize, triangles, pointsRelPos, 
                                      pointsToInterpolateMask, interpolationPoints,
                                      outDistanceToPoint, outPointInfo, mesh, trianglesData);
    }

    inline void filterTriangles(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                const std::vector<uint32_t>& inTriangles, std::vector<uint32_t>& outTriangles,
                                const std::array<float, 8>& distanceToVertices,
                                const std::array<VertexInfo, 8>& verticesInfo,
                                const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        outTriangles.clear();
        
        // Serach the maximum distance of all node verices minimum distance
        float maxMinDist = 0.0f;
        for(uint32_t i=0; i < 8; i++)
        {
            maxMinDist = glm::max(maxMinDist, glm::abs(distanceToVertices[i]));
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
};


struct PreciseTrianglesInfluence
{
    struct VertexInfo {};
    struct NodeInfo {};

    template<int N>
    inline void calculateVerticesInfo(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                      const std::vector<uint32_t>& triangles,
                                      const std::array<glm::vec3, N>& pointsRelPos,
                                      const uint32_t pointsToInterpolateMask,
                                      const std::array<float, 8>& interpolationPoints,
                                      std::array<float, N>& outDistanceToPoint,
                                      std::array<VertexInfo, N>& outPointInfo,
                                      const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    { 
        standardCalculateVerticesInfo(nodeCenter, nodeHalfSize, triangles, pointsRelPos, 
                                      pointsToInterpolateMask, interpolationPoints,
                                      outDistanceToPoint, outPointInfo, mesh, trianglesData);
    }

    inline void filterTriangles(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                const std::vector<uint32_t>& inTriangles, std::vector<uint32_t>& outTriangles,
                                const std::array<float, 8>& distanceToVertices,
                                const std::array<VertexInfo, 8>& verticesInfo,
                                const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        outTriangles.clear();
        
        // Serach the maximum distance of all node verices minimum distance
        float maxMinDist = 0.0f;
        for(uint32_t i=0; i < 8; i++)
        {
            maxMinDist = glm::max(maxMinDist, glm::abs(distanceToVertices[i]));
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
};

struct PerVertexTrianglesInfluence
{
    struct {} NodeInfo;
    typedef uint32_t VertexInfo;

    template<int N>
    inline void calculateVerticesInfo(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                      const std::vector<uint32_t>& triangles,
                                      const std::array<glm::vec3, N>& pointsRelPos,
                                      const uint32_t pointToInterpolateMask,
                                      const std::array<float, 8>& interpolationPoints,
                                      std::array<float, N>& outDistanceToPoint,
                                      std::array<VertexInfo, N>& outPointInfo,
                                      const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        outDistanceToPoint.fill(INFINITY); // It will locally used to store the minimum distance to vertex

        for(const uint32_t& t : triangles)
        {
            for(uint32_t i=0; i < N; i++)
            {
                const float dist = TriangleUtils::getSqDistPointAndTriangle(nodeCenter + pointsRelPos[i] * nodeHalfSize, trianglesData[t]);

                if(dist < outDistanceToPoint[i])
                {
                    outPointInfo[i] = t;
                    outDistanceToPoint[i] = dist;
                }
            }
        }

        for(uint32_t i=0; i < N; i++)
        {
            if(pointToInterpolateMask & (1 << (N-i-1)))
            {
                outDistanceToPoint[i] = interpolateValue(reinterpret_cast<const float*>(&interpolationPoints), 0.5f * pointsRelPos[i] + 0.5f);
            }
            else
            {
                outDistanceToPoint[i] = TriangleUtils::getSignedDistPointAndTriangle(nodeCenter + pointsRelPos[i] * nodeHalfSize, trianglesData[outPointInfo[i]]);
            }
        }
    }

    inline void filterTriangles(const glm::vec3 nodeCenter, const float nodeHalfSize,
                                const std::vector<uint32_t>& inTriangles, std::vector<uint32_t>& outTriangles,
                                const std::array<float, 8>& distanceToVertices,
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

        for(const uint32_t& idx : inTriangles)
        {
            triangle[0] = vertices[indices[3 * idx]] - nodeCenter;
            triangle[1] = vertices[indices[3 * idx + 1]] - nodeCenter;
            triangle[2] = vertices[indices[3 * idx + 2]] - nodeCenter;

            bool isInside = true;
            for(uint32_t r=0; r < 8; r++)
            {
                if(verticesInfo[r] != idx && !GJK::isInsideConvexHull(nodeHalfSize, triangleRegions[r], triangle))
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
};

#endif