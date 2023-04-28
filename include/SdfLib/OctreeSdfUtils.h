#ifndef OCTREE_SDF_UTILS_H
#define OCTREE_SDF_UTILS_H

#include "utils/TriangleUtils.h"
#include "glm/glm.hpp"
#include <vector>
#include <array>
#include <stack>

namespace sdflib
{
template<size_t N>
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

template<typename Inter>
inline float estimateErrorFunctionIntegralByTrapezoidRule(const std::array<float, Inter::NUM_COEFFICIENTS>& interpolationCoeff,
                                                          const std::array<std::array<float, Inter::VALUES_PER_VERTEX>, 19>& middlePoints)
{
    return 2.0f / 64.0f * pow2(middlePoints[0][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.0f, 0.0f))) +
           2.0f / 64.0f * pow2(middlePoints[1][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.5f, 0.0f))) +
           4.0f / 64.0f * pow2(middlePoints[2][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.5f, 0.0f))) +
           2.0f / 64.0f * pow2(middlePoints[3][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.5f, 0.0f))) +
           2.0f / 64.0f * pow2(middlePoints[4][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 1.0f, 0.0f))) +

           2.0f / 64.0f * pow2(middlePoints[5][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.0f, 0.5f))) +
           4.0f / 64.0f * pow2(middlePoints[6][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.0f, 0.5f))) +
           2.0f / 64.0f * pow2(middlePoints[7][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.0f, 0.5f))) +
           4.0f / 64.0f * pow2(middlePoints[8][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.5f, 0.5f))) +
           8.0f / 64.0f * pow2(middlePoints[9][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.5f, 0.5f))) +
           4.0f / 64.0f * pow2(middlePoints[10][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.5f, 0.5f))) +
           2.0f / 64.0f * pow2(middlePoints[11][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 1.0f, 0.5f))) +
           4.0f / 64.0f * pow2(middlePoints[12][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 1.0f, 0.5f))) +
           2.0f / 64.0f * pow2(middlePoints[13][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 1.0f, 0.5f))) +

           2.0f / 64.0f * pow2(middlePoints[14][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.0f, 1.0f))) +
           2.0f / 64.0f * pow2(middlePoints[15][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.5f, 1.0f))) +
           4.0f / 64.0f * pow2(middlePoints[16][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.5f, 1.0f))) +
           2.0f / 64.0f * pow2(middlePoints[17][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.5f, 1.0f))) +
           2.0f / 64.0f * pow2(middlePoints[18][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 1.0f, 1.0f)));
}

template<typename Inter>
inline float estimateFaceErrorFunctionIntegralByTrapezoidRule(const std::array<float, Inter::NUM_COEFFICIENTS>& interpolationCoeff,
                                                          const std::array<std::array<float, Inter::VALUES_PER_VERTEX>, 19>& middlePoints)
{
    float f0 =  2.0f / 16.0f * pow2(middlePoints[0][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.0f, 0.0f))) +
                2.0f / 16.0f * pow2(middlePoints[1][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.5f, 0.0f))) +
                4.0f / 16.0f * pow2(middlePoints[2][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.5f, 0.0f))) +
                2.0f / 16.0f * pow2(middlePoints[3][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.5f, 0.0f))) +
                2.0f / 16.0f * pow2(middlePoints[4][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 1.0f, 0.0f)));

    float f1 =  2.0f / 16.0f * pow2(middlePoints[14][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.0f, 1.0f))) +
                2.0f / 16.0f * pow2(middlePoints[15][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.5f, 1.0f))) +
                4.0f / 16.0f * pow2(middlePoints[16][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.5f, 1.0f))) +
                2.0f / 16.0f * pow2(middlePoints[17][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.5f, 1.0f))) +
                2.0f / 16.0f * pow2(middlePoints[18][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 1.0f, 1.0f)));


    float f2 = 2.0f / 16.0f * pow2(middlePoints[1][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.5f, 0.0f))) +
               2.0f / 16.0f * pow2(middlePoints[5][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.0f, 0.5f))) +
               4.0f / 16.0f * pow2(middlePoints[8][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.5f, 0.5f))) +
               2.0f / 16.0f * pow2(middlePoints[11][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 1.0f, 0.5f))) +
               2.0f / 16.0f * pow2(middlePoints[15][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.5f, 1.0f)));

    float f3 =  2.0f / 16.0f * pow2(middlePoints[3][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.5f, 0.0f))) +
                2.0f / 16.0f * pow2(middlePoints[7][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.0f, 0.5f))) +
                4.0f / 16.0f * pow2(middlePoints[10][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.5f, 0.5f))) +
                2.0f / 16.0f * pow2(middlePoints[13][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 1.0f, 0.5f))) +
                2.0f / 16.0f * pow2(middlePoints[17][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.5f, 1.0f)));

    float f4 =  2.0f / 16.0f * pow2(middlePoints[0][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.0f, 0.0f))) +
                2.0f / 16.0f * pow2(middlePoints[5][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.0f, 0.5f))) +
                4.0f / 16.0f * pow2(middlePoints[6][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.0f, 0.5f))) +
                2.0f / 16.0f * pow2(middlePoints[7][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.0f, 0.5f))) +
                2.0f / 16.0f * pow2(middlePoints[14][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.0f, 1.0f)));

    float f5 =  2.0f / 16.0f * pow2(middlePoints[4][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 1.0f, 0.0f))) +
                2.0f / 16.0f * pow2(middlePoints[11][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 1.0f, 0.5f))) +
                4.0f / 16.0f * pow2(middlePoints[12][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 1.0f, 0.5f))) +
                2.0f / 16.0f * pow2(middlePoints[13][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 1.0f, 0.5f))) +
                2.0f / 16.0f * pow2(middlePoints[18][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 1.0f, 1.0f)));
    
    return glm::max(glm::max(glm::max(f0, f1), glm::max(f2, f3)), glm::max(f4, f5));
}

template<typename Inter>
inline float estimateMaxError(const std::array<float, Inter::NUM_COEFFICIENTS>& interpolationCoeff,
                                                          const std::array<std::array<float, Inter::VALUES_PER_VERTEX>, 19>& middlePoints)
{ 
    float max = 0.0f;

    max = glm::max(max, pow2(middlePoints[0][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.0f, 0.0f))));
    max = glm::max(max, pow2(middlePoints[1][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.5f, 0.0f))));
    max = glm::max(max, pow2(middlePoints[2][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.5f, 0.0f))));
    max = glm::max(max, pow2(middlePoints[3][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.5f, 0.0f))));
    max = glm::max(max, pow2(middlePoints[4][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 1.0f, 0.0f))));
    max = glm::max(max, pow2(middlePoints[5][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.0f, 0.5f))));
    max = glm::max(max, pow2(middlePoints[6][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.0f, 0.5f))));
    max = glm::max(max, pow2(middlePoints[7][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.0f, 0.5f))));
    max = glm::max(max, pow2(middlePoints[8][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.5f, 0.5f))));
    max = glm::max(max, pow2(middlePoints[9][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.5f, 0.5f))));
    max = glm::max(max, pow2(middlePoints[10][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.5f, 0.5f))));
    max = glm::max(max, pow2(middlePoints[11][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 1.0f, 0.5f))));
    max = glm::max(max, pow2(middlePoints[12][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 1.0f, 0.5f))));
    max = glm::max(max, pow2(middlePoints[13][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 1.0f, 0.5f))));
    max = glm::max(max, pow2(middlePoints[14][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.0f, 1.0f))));
    max = glm::max(max, pow2(middlePoints[15][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.5f, 1.0f))));
    max = glm::max(max, pow2(middlePoints[16][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.5f, 1.0f))));
    max = glm::max(max, pow2(middlePoints[17][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.5f, 1.0f))));
    max = glm::max(max, pow2(middlePoints[18][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 1.0f, 1.0f))));

    return max;
}

template<typename Inter>
inline float estimateErrorFunctionIntegralBySimpsonsRule(const std::array<float, Inter::NUM_COEFFICIENTS>& interpolationCoeff,
                                                         const std::array<std::array<float, Inter::VALUES_PER_VERTEX>, 19>& middlePoints)
{
    return 4.0f / 216.0f * pow2(middlePoints[0][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.0f, 0.0f))) +
           4.0f / 216.0f * pow2(middlePoints[1][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.5f, 0.0f))) +
           16.0f / 216.0f * pow2(middlePoints[2][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.5f, 0.0f))) +
           4.0f / 216.0f * pow2(middlePoints[3][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.5f, 0.0f))) +
           4.0f / 216.0f * pow2(middlePoints[4][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 1.0f, 0.0f))) +

           4.0f / 216.0f * pow2(middlePoints[5][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.0f, 0.5f))) +
           16.0f / 216.0f * pow2(middlePoints[6][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.0f, 0.5f))) +
           4.0f / 216.0f * pow2(middlePoints[7][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.0f, 0.5f))) +
           16.0f / 216.0f * pow2(middlePoints[8][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.5f, 0.5f))) +
           64.0f / 216.0f * pow2(middlePoints[9][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.5f, 0.5f))) +
           16.0f / 216.0f * pow2(middlePoints[10][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.5f, 0.5f))) +
           4.0f / 216.0f * pow2(middlePoints[11][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 1.0f, 0.5f))) +
           16.0f / 216.0f * pow2(middlePoints[12][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 1.0f, 0.5f))) +
           4.0f / 216.0f * pow2(middlePoints[13][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 1.0f, 0.5f))) +

           4.0f / 216.0f * pow2(middlePoints[14][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.0f, 1.0f))) +
           4.0f / 216.0f * pow2(middlePoints[15][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.0f, 0.5f, 1.0f))) +
           16.0f / 216.0f * pow2(middlePoints[16][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 0.5f, 1.0f))) +
           4.0f / 216.0f * pow2(middlePoints[17][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(1.0f, 0.5f, 1.0f))) +
           4.0f / 216.0f * pow2(middlePoints[18][0] - Inter::interpolateValue(interpolationCoeff, glm::vec3(0.5f, 1.0f, 1.0f)));
}
}

#endif