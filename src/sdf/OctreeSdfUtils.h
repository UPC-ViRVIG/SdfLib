#ifndef OCTREE_SDF_UTILS_H
#define OCTREE_SDF_UTILS_H

#include "utils/TriangleUtils.h"
#include "glm/glm.hpp"
#include <vector>
#include <array>
#include <stack>

template<int N>
inline void calculateMinDistances(const std::array<glm::vec3, N>& inPos, std::array<float, N>& outDistance, 
                                   const std::vector<uint32_t>& triangles, const std::vector<TriangleUtils::TriangleData>& trianglesData)
{
    outDistance.fill(INFINITY);
    std::array<uint32_t, N> minIndex;

    for(uint32_t t : triangles)
    {
        for(uint32_t i=0; i < N; i++)
        {
            const float dist = TriangleUtils::getSqDistPointAndTriangle(inPos[i], trianglesData[t]);
            if(dist < outDistance[i])
            {
                minIndex[i] = t;
                outDistance[i] = dist;
            }
        }
    }

    for(uint32_t i=0; i < N; i++)
    {
        outDistance[i] = TriangleUtils::getSignedDistPointAndTriangle(inPos[i], trianglesData[minIndex[i]]);
    }
}

inline float interpolateValue(const float* values, glm::vec3 fracPart)
{
    float d00 = values[0] * (1.0f - fracPart.x) +
                values[1] * fracPart.x;
    float d01 = values[2] * (1.0f - fracPart.x) +
                values[3] * fracPart.x;
    float d10 = values[4] * (1.0f - fracPart.x) +
                values[5] * fracPart.x;
    float d11 = values[6] * (1.0f - fracPart.x) +
                values[7] * fracPart.x;

    float d0 = d00 * (1.0f - fracPart.y) + d01 * fracPart.y;
    float d1 = d10 * (1.0f - fracPart.y) + d11 * fracPart.y;

    return d0 * (1.0f - fracPart.z) + d1 * fracPart.z;
}

inline float pow2(float a)
{
    return a * a;
}

inline float estimateErrorFunctionIntegralByTrapezoidRule(const std::array<float, 8> vertexPoints, std::array<float, 19> middlePoints)
{
    return 2.0f / 64.0f * pow2(middlePoints[0] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 0.0f, 0.0f))) +
           2.0f / 64.0f * pow2(middlePoints[1] - interpolateValue(vertexPoints.data(), glm::vec3(0.0f, 0.5f, 0.0f))) +
           4.0f / 64.0f * pow2(middlePoints[2] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 0.5f, 0.0f))) +
           2.0f / 64.0f * pow2(middlePoints[3] - interpolateValue(vertexPoints.data(), glm::vec3(1.0f, 0.5f, 0.0f))) +
           2.0f / 64.0f * pow2(middlePoints[4] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 1.0f, 0.0f))) +

           2.0f / 64.0f * pow2(middlePoints[5] - interpolateValue(vertexPoints.data(), glm::vec3(0.0f, 0.0f, 0.5f))) +
           4.0f / 64.0f * pow2(middlePoints[6] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 0.0f, 0.5f))) +
           2.0f / 64.0f * pow2(middlePoints[7] - interpolateValue(vertexPoints.data(), glm::vec3(1.0f, 0.0f, 0.5f))) +
           4.0f / 64.0f * pow2(middlePoints[8] - interpolateValue(vertexPoints.data(), glm::vec3(0.0f, 0.5f, 0.5f))) +
           8.0f / 64.0f * pow2(middlePoints[9] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 0.5f, 0.5f))) +
           4.0f / 64.0f * pow2(middlePoints[10] - interpolateValue(vertexPoints.data(), glm::vec3(1.0f, 0.5f, 0.5f))) +
           2.0f / 64.0f * pow2(middlePoints[11] - interpolateValue(vertexPoints.data(), glm::vec3(0.0f, 1.0f, 0.5f))) +
           4.0f / 64.0f * pow2(middlePoints[12] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 1.0f, 0.5f))) +
           2.0f / 64.0f * pow2(middlePoints[13] - interpolateValue(vertexPoints.data(), glm::vec3(1.0f, 1.0f, 0.5f))) +

           2.0f / 64.0f * pow2(middlePoints[14] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 0.0f, 1.0f))) +
           2.0f / 64.0f * pow2(middlePoints[15] - interpolateValue(vertexPoints.data(), glm::vec3(0.0f, 0.5f, 1.0f))) +
           4.0f / 64.0f * pow2(middlePoints[16] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 0.5f, 1.0f))) +
           2.0f / 64.0f * pow2(middlePoints[17] - interpolateValue(vertexPoints.data(), glm::vec3(1.0f, 0.5f, 1.0f))) +
           2.0f / 64.0f * pow2(middlePoints[18] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 1.0f, 1.0f)));
}

inline float estimateErrorFunctionIntegralBySimpsonsRule(const std::array<float, 8> vertexPoints, std::array<float, 19> middlePoints)
{
    return 4.0f / 216.0f * pow2(middlePoints[0] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 0.0f, 0.0f))) +
           4.0f / 216.0f * pow2(middlePoints[1] - interpolateValue(vertexPoints.data(), glm::vec3(0.0f, 0.5f, 0.0f))) +
           16.0f / 216.0f * pow2(middlePoints[2] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 0.5f, 0.0f))) +
           4.0f / 216.0f * pow2(middlePoints[3] - interpolateValue(vertexPoints.data(), glm::vec3(1.0f, 0.5f, 0.0f))) +
           4.0f / 216.0f * pow2(middlePoints[4] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 1.0f, 0.0f))) +

           4.0f / 216.0f * pow2(middlePoints[5] - interpolateValue(vertexPoints.data(), glm::vec3(0.0f, 0.0f, 0.5f))) +
           16.0f / 216.0f * pow2(middlePoints[6] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 0.0f, 0.5f))) +
           4.0f / 216.0f * pow2(middlePoints[7] - interpolateValue(vertexPoints.data(), glm::vec3(1.0f, 0.0f, 0.5f))) +
           16.0f / 216.0f * pow2(middlePoints[8] - interpolateValue(vertexPoints.data(), glm::vec3(0.0f, 0.5f, 0.5f))) +
           64.0f / 216.0f * pow2(middlePoints[9] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 0.5f, 0.5f))) +
           16.0f / 216.0f * pow2(middlePoints[10] - interpolateValue(vertexPoints.data(), glm::vec3(1.0f, 0.5f, 0.5f))) +
           4.0f / 216.0f * pow2(middlePoints[11] - interpolateValue(vertexPoints.data(), glm::vec3(0.0f, 1.0f, 0.5f))) +
           16.0f / 216.0f * pow2(middlePoints[12] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 1.0f, 0.5f))) +
           4.0f / 216.0f * pow2(middlePoints[13] - interpolateValue(vertexPoints.data(), glm::vec3(1.0f, 1.0f, 0.5f))) +

           4.0f / 216.0f * pow2(middlePoints[14] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 0.0f, 1.0f))) +
           4.0f / 216.0f * pow2(middlePoints[15] - interpolateValue(vertexPoints.data(), glm::vec3(0.0f, 0.5f, 1.0f))) +
           16.0f / 216.0f * pow2(middlePoints[16] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 0.5f, 1.0f))) +
           4.0f / 216.0f * pow2(middlePoints[17] - interpolateValue(vertexPoints.data(), glm::vec3(1.0f, 0.5f, 1.0f))) +
           4.0f / 216.0f * pow2(middlePoints[18] - interpolateValue(vertexPoints.data(), glm::vec3(0.5f, 1.0f, 1.0f)));
}


#endif