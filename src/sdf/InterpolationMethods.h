#ifndef INTERPOLATION_METHODS_H
#define INTERPOLATION_METHODS_H

#include <array>

#include "utils/TriangleUtils.h"

struct NoneInterpolation
{
    static constexpr uint32_t VALUES_PER_VERTEX = 0;
    static constexpr uint32_t EXTRA_VALUES = 0;
    static constexpr uint32_t NUM_COEFFICIENTS = 0;

    inline void calculateCoefficients(const std::array<std::array<float, VALUES_PER_VERTEX>, 8>& valuesPerVertex,
                                      const std::vector<uint32_t>& triangles,
                                      const std::vector<TriangleUtils::TriangleData>& trianglesData,
                                      std::array<float, NUM_COEFFICIENTS>& outCoefficients) {}

    inline void calculatePointsValues(const TriangleUtils::TriangleData& nearestTriangle,
                                      glm::vec3 point, 
                                      std::array<float, VALUES_PER_VERTEX>& outValues)
    { }

    inline float interpolateValue(const std::array<float, NUM_COEFFICIENTS>& coefficients, glm::vec3 fracPart) 
    {
        return 0.0f;
    }
};

struct TriLinearInterpolation
{
    static constexpr uint32_t VALUES_PER_VERTEX = 1;
    static constexpr uint32_t EXTRA_VALUES = 0;
    static constexpr uint32_t NUM_COEFFICIENTS = 8;

    inline static void calculateCoefficients(const std::array<std::array<float, VALUES_PER_VERTEX>, 8>& valuesPerVertex,
                                      const std::vector<uint32_t>& triangles,
                                      const Mesh& mesh,
                                      const std::vector<TriangleUtils::TriangleData>& trianglesData,
                                      std::array<float, NUM_COEFFICIENTS>& outCoefficients) 
    {
        outCoefficients = *reinterpret_cast<const std::array<float, NUM_COEFFICIENTS>*>(&valuesPerVertex);
    }

    inline static void calculatePointsValues(glm::vec3 point,
                                      uint32_t nearestTriangleIndex,
                                      const Mesh& mesh,
                                      const std::vector<TriangleUtils::TriangleData>& trianglesData, 
                                      std::array<float, VALUES_PER_VERTEX>& outValues)
    { 
        outValues[0] = TriangleUtils::getSignedDistPointAndTriangle(point, trianglesData[nearestTriangleIndex]);
    }

    inline static float interpolateValue(const std::array<float, NUM_COEFFICIENTS>& values, glm::vec3 fracPart) 
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
};

struct TriCubicInterpolation
{
    static constexpr uint32_t VALUES_PER_VERTEX = 4;
    static constexpr uint32_t EXTRA_VALUES = 0;
    static constexpr uint32_t NUM_COEFFICIENTS = 64;

    inline static void calculatePointsValues(glm::vec3 point,
                                      uint32_t nearestTriangleIndex,
                                      const Mesh& mesh,
                                      const std::vector<TriangleUtils::TriangleData>& trianglesData, 
                                      std::array<float, VALUES_PER_VERTEX>& outValues)
    {
        const std::vector<uint32_t>& indices = mesh.getIndices();
        const std::vector<glm::vec3>& vertices = mesh.getVertices();
        glm::vec3 gradient;
        outValues[0] = TriangleUtils::getSignedDistPointAndTriangle(point, trianglesData[nearestTriangleIndex], 
                                                                    vertices[indices[3 * nearestTriangleIndex]],
                                                                    vertices[indices[3 * nearestTriangleIndex + 1]],
                                                                    vertices[indices[3 * nearestTriangleIndex + 2]],
                                                                    gradient);
        outValues[1] = gradient.x; outValues[2] = gradient.y; outValues[3] = gradient.z;
    }

    inline static void calculateCoefficients(const std::array<std::array<float, VALUES_PER_VERTEX>, 8>& inValues,
                                      const std::vector<uint32_t>& triangles,
                                      const Mesh& mesh,
                                      const std::vector<TriangleUtils::TriangleData>& trianglesData,
                                      std::array<float, NUM_COEFFICIENTS>& outCoeff) 
    {
        outCoeff[0] = 1 * inValues[0][0] + 0.0f;
        outCoeff[1] = 1 * inValues[0][1] + 0.0f;
        outCoeff[2] = -3 * inValues[0][0] + -2 * inValues[0][1] + 3 * inValues[1][0] + -1 * inValues[1][1] + 0.0f;
        outCoeff[3] = 2 * inValues[0][0] + 1 * inValues[0][1] + -2 * inValues[1][0] + 1 * inValues[1][1] + 0.0f;
        outCoeff[4] = 1 * inValues[0][2] + 0.0f;
        outCoeff[5] = 0.0f;
        outCoeff[6] = -3 * inValues[0][2] + 3 * inValues[1][2] + 0.0f;
        outCoeff[7] = 2 * inValues[0][2] + -2 * inValues[1][2] + 0.0f;
        outCoeff[8] = -3 * inValues[0][0] + -2 * inValues[0][2] + 3 * inValues[2][0] + -1 * inValues[2][2] + 0.0f;
        outCoeff[9] = -3 * inValues[0][1] + 3 * inValues[2][1] + 0.0f;
        outCoeff[10] = 9 * inValues[0][0] + 6 * inValues[0][1] + 6 * inValues[0][2] + -9 * inValues[1][0] + 3 * inValues[1][1] + -6 * inValues[1][2] + -9 * inValues[2][0] + -6 * inValues[2][1] + 3 * inValues[2][2] + 9 * inValues[3][0] + -3 * inValues[3][1] + -3 * inValues[3][2] + 0.0f;
        outCoeff[11] = -6 * inValues[0][0] + -3 * inValues[0][1] + -4 * inValues[0][2] + 6 * inValues[1][0] + -3 * inValues[1][1] + 4 * inValues[1][2] + 6 * inValues[2][0] + 3 * inValues[2][1] + -2 * inValues[2][2] + -6 * inValues[3][0] + 3 * inValues[3][1] + 2 * inValues[3][2] + 0.0f;
        outCoeff[12] = 2 * inValues[0][0] + 1 * inValues[0][2] + -2 * inValues[2][0] + 1 * inValues[2][2] + 0.0f;
        outCoeff[13] = 2 * inValues[0][1] + -2 * inValues[2][1] + 0.0f;
        outCoeff[14] = -6 * inValues[0][0] + -4 * inValues[0][1] + -3 * inValues[0][2] + 6 * inValues[1][0] + -2 * inValues[1][1] + 3 * inValues[1][2] + 6 * inValues[2][0] + 4 * inValues[2][1] + -3 * inValues[2][2] + -6 * inValues[3][0] + 2 * inValues[3][1] + 3 * inValues[3][2] + 0.0f;
        outCoeff[15] = 4 * inValues[0][0] + 2 * inValues[0][1] + 2 * inValues[0][2] + -4 * inValues[1][0] + 2 * inValues[1][1] + -2 * inValues[1][2] + -4 * inValues[2][0] + -2 * inValues[2][1] + 2 * inValues[2][2] + 4 * inValues[3][0] + -2 * inValues[3][1] + -2 * inValues[3][2] + 0.0f;
        outCoeff[16] = 1 * inValues[0][3] + 0.0f;
        outCoeff[17] = 0.0f;
        outCoeff[18] = -3 * inValues[0][3] + 3 * inValues[1][3] + 0.0f;
        outCoeff[19] = 2 * inValues[0][3] + -2 * inValues[1][3] + 0.0f;
        outCoeff[20] = 0.0f;
        outCoeff[21] = 0.0f;
        outCoeff[22] = 0.0f;
        outCoeff[23] = 0.0f;
        outCoeff[24] = -3 * inValues[0][3] + 3 * inValues[2][3] + 0.0f;
        outCoeff[25] = 0.0f;
        outCoeff[26] = 9 * inValues[0][3] + -9 * inValues[1][3] + -9 * inValues[2][3] + 9 * inValues[3][3] + 0.0f;
        outCoeff[27] = -6 * inValues[0][3] + 6 * inValues[1][3] + 6 * inValues[2][3] + -6 * inValues[3][3] + 0.0f;
        outCoeff[28] = 2 * inValues[0][3] + -2 * inValues[2][3] + 0.0f;
        outCoeff[29] = 0.0f;
        outCoeff[30] = -6 * inValues[0][3] + 6 * inValues[1][3] + 6 * inValues[2][3] + -6 * inValues[3][3] + 0.0f;
        outCoeff[31] = 4 * inValues[0][3] + -4 * inValues[1][3] + -4 * inValues[2][3] + 4 * inValues[3][3] + 0.0f;
        outCoeff[32] = -3 * inValues[0][0] + -2 * inValues[0][3] + 3 * inValues[4][0] + -1 * inValues[4][3] + 0.0f;
        outCoeff[33] = -3 * inValues[0][1] + 3 * inValues[4][1] + 0.0f;
        outCoeff[34] = 9 * inValues[0][0] + 6 * inValues[0][1] + 6 * inValues[0][3] + -9 * inValues[1][0] + 3 * inValues[1][1] + -6 * inValues[1][3] + -9 * inValues[4][0] + -6 * inValues[4][1] + 3 * inValues[4][3] + 9 * inValues[5][0] + -3 * inValues[5][1] + -3 * inValues[5][3] + 0.0f;
        outCoeff[35] = -6 * inValues[0][0] + -3 * inValues[0][1] + -4 * inValues[0][3] + 6 * inValues[1][0] + -3 * inValues[1][1] + 4 * inValues[1][3] + 6 * inValues[4][0] + 3 * inValues[4][1] + -2 * inValues[4][3] + -6 * inValues[5][0] + 3 * inValues[5][1] + 2 * inValues[5][3] + 0.0f;
        outCoeff[36] = -3 * inValues[0][2] + 3 * inValues[4][2] + 0.0f;
        outCoeff[37] = 0.0f;
        outCoeff[38] = 9 * inValues[0][2] + -9 * inValues[1][2] + -9 * inValues[4][2] + 9 * inValues[5][2] + 0.0f;
        outCoeff[39] = -6 * inValues[0][2] + 6 * inValues[1][2] + 6 * inValues[4][2] + -6 * inValues[5][2] + 0.0f;
        outCoeff[40] = 9 * inValues[0][0] + 6 * inValues[0][2] + 6 * inValues[0][3] + -9 * inValues[2][0] + 3 * inValues[2][2] + -6 * inValues[2][3] + -9 * inValues[4][0] + -6 * inValues[4][2] + 3 * inValues[4][3] + 9 * inValues[6][0] + -3 * inValues[6][2] + -3 * inValues[6][3] + 0.0f;
        outCoeff[41] = 9 * inValues[0][1] + -9 * inValues[2][1] + -9 * inValues[4][1] + 9 * inValues[6][1] + 0.0f;
        outCoeff[42] = -27 * inValues[0][0] + -18 * inValues[0][1] + -18 * inValues[0][2] + -18 * inValues[0][3] + 27 * inValues[1][0] + -9 * inValues[1][1] + 18 * inValues[1][2] + 18 * inValues[1][3] + 27 * inValues[2][0] + 18 * inValues[2][1] + -9 * inValues[2][2] + 18 * inValues[2][3] + -27 * inValues[3][0] + 9 * inValues[3][1] + 9 * inValues[3][2] + -18 * inValues[3][3] + 27 * inValues[4][0] + 18 * inValues[4][1] + 18 * inValues[4][2] + -9 * inValues[4][3] + -27 * inValues[5][0] + 9 * inValues[5][1] + -18 * inValues[5][2] + 9 * inValues[5][3] + -27 * inValues[6][0] + -18 * inValues[6][1] + 9 * inValues[6][2] + 9 * inValues[6][3] + 27 * inValues[7][0] + -9 * inValues[7][1] + -9 * inValues[7][2] + -9 * inValues[7][3] + 0.0f;
        outCoeff[43] = 18 * inValues[0][0] + 9 * inValues[0][1] + 12 * inValues[0][2] + 12 * inValues[0][3] + -18 * inValues[1][0] + 9 * inValues[1][1] + -12 * inValues[1][2] + -12 * inValues[1][3] + -18 * inValues[2][0] + -9 * inValues[2][1] + 6 * inValues[2][2] + -12 * inValues[2][3] + 18 * inValues[3][0] + -9 * inValues[3][1] + -6 * inValues[3][2] + 12 * inValues[3][3] + -18 * inValues[4][0] + -9 * inValues[4][1] + -12 * inValues[4][2] + 6 * inValues[4][3] + 18 * inValues[5][0] + -9 * inValues[5][1] + 12 * inValues[5][2] + -6 * inValues[5][3] + 18 * inValues[6][0] + 9 * inValues[6][1] + -6 * inValues[6][2] + -6 * inValues[6][3] + -18 * inValues[7][0] + 9 * inValues[7][1] + 6 * inValues[7][2] + 6 * inValues[7][3] + 0.0f;
        outCoeff[44] = -6 * inValues[0][0] + -3 * inValues[0][2] + -4 * inValues[0][3] + 6 * inValues[2][0] + -3 * inValues[2][2] + 4 * inValues[2][3] + 6 * inValues[4][0] + 3 * inValues[4][2] + -2 * inValues[4][3] + -6 * inValues[6][0] + 3 * inValues[6][2] + 2 * inValues[6][3] + 0.0f;
        outCoeff[45] = -6 * inValues[0][1] + 6 * inValues[2][1] + 6 * inValues[4][1] + -6 * inValues[6][1] + 0.0f;
        outCoeff[46] = 18 * inValues[0][0] + 12 * inValues[0][1] + 9 * inValues[0][2] + 12 * inValues[0][3] + -18 * inValues[1][0] + 6 * inValues[1][1] + -9 * inValues[1][2] + -12 * inValues[1][3] + -18 * inValues[2][0] + -12 * inValues[2][1] + 9 * inValues[2][2] + -12 * inValues[2][3] + 18 * inValues[3][0] + -6 * inValues[3][1] + -9 * inValues[3][2] + 12 * inValues[3][3] + -18 * inValues[4][0] + -12 * inValues[4][1] + -9 * inValues[4][2] + 6 * inValues[4][3] + 18 * inValues[5][0] + -6 * inValues[5][1] + 9 * inValues[5][2] + -6 * inValues[5][3] + 18 * inValues[6][0] + 12 * inValues[6][1] + -9 * inValues[6][2] + -6 * inValues[6][3] + -18 * inValues[7][0] + 6 * inValues[7][1] + 9 * inValues[7][2] + 6 * inValues[7][3] + 0.0f;
        outCoeff[47] = -12 * inValues[0][0] + -6 * inValues[0][1] + -6 * inValues[0][2] + -8 * inValues[0][3] + 12 * inValues[1][0] + -6 * inValues[1][1] + 6 * inValues[1][2] + 8 * inValues[1][3] + 12 * inValues[2][0] + 6 * inValues[2][1] + -6 * inValues[2][2] + 8 * inValues[2][3] + -12 * inValues[3][0] + 6 * inValues[3][1] + 6 * inValues[3][2] + -8 * inValues[3][3] + 12 * inValues[4][0] + 6 * inValues[4][1] + 6 * inValues[4][2] + -4 * inValues[4][3] + -12 * inValues[5][0] + 6 * inValues[5][1] + -6 * inValues[5][2] + 4 * inValues[5][3] + -12 * inValues[6][0] + -6 * inValues[6][1] + 6 * inValues[6][2] + 4 * inValues[6][3] + 12 * inValues[7][0] + -6 * inValues[7][1] + -6 * inValues[7][2] + -4 * inValues[7][3] + 0.0f;
        outCoeff[48] = 2 * inValues[0][0] + 1 * inValues[0][3] + -2 * inValues[4][0] + 1 * inValues[4][3] + 0.0f;
        outCoeff[49] = 2 * inValues[0][1] + -2 * inValues[4][1] + 0.0f;
        outCoeff[50] = -6 * inValues[0][0] + -4 * inValues[0][1] + -3 * inValues[0][3] + 6 * inValues[1][0] + -2 * inValues[1][1] + 3 * inValues[1][3] + 6 * inValues[4][0] + 4 * inValues[4][1] + -3 * inValues[4][3] + -6 * inValues[5][0] + 2 * inValues[5][1] + 3 * inValues[5][3] + 0.0f;
        outCoeff[51] = 4 * inValues[0][0] + 2 * inValues[0][1] + 2 * inValues[0][3] + -4 * inValues[1][0] + 2 * inValues[1][1] + -2 * inValues[1][3] + -4 * inValues[4][0] + -2 * inValues[4][1] + 2 * inValues[4][3] + 4 * inValues[5][0] + -2 * inValues[5][1] + -2 * inValues[5][3] + 0.0f;
        outCoeff[52] = 2 * inValues[0][2] + -2 * inValues[4][2] + 0.0f;
        outCoeff[53] = 0.0f;
        outCoeff[54] = -6 * inValues[0][2] + 6 * inValues[1][2] + 6 * inValues[4][2] + -6 * inValues[5][2] + 0.0f;
        outCoeff[55] = 4 * inValues[0][2] + -4 * inValues[1][2] + -4 * inValues[4][2] + 4 * inValues[5][2] + 0.0f;
        outCoeff[56] = -6 * inValues[0][0] + -4 * inValues[0][2] + -3 * inValues[0][3] + 6 * inValues[2][0] + -2 * inValues[2][2] + 3 * inValues[2][3] + 6 * inValues[4][0] + 4 * inValues[4][2] + -3 * inValues[4][3] + -6 * inValues[6][0] + 2 * inValues[6][2] + 3 * inValues[6][3] + 0.0f;
        outCoeff[57] = -6 * inValues[0][1] + 6 * inValues[2][1] + 6 * inValues[4][1] + -6 * inValues[6][1] + 0.0f;
        outCoeff[58] = 18 * inValues[0][0] + 12 * inValues[0][1] + 12 * inValues[0][2] + 9 * inValues[0][3] + -18 * inValues[1][0] + 6 * inValues[1][1] + -12 * inValues[1][2] + -9 * inValues[1][3] + -18 * inValues[2][0] + -12 * inValues[2][1] + 6 * inValues[2][2] + -9 * inValues[2][3] + 18 * inValues[3][0] + -6 * inValues[3][1] + -6 * inValues[3][2] + 9 * inValues[3][3] + -18 * inValues[4][0] + -12 * inValues[4][1] + -12 * inValues[4][2] + 9 * inValues[4][3] + 18 * inValues[5][0] + -6 * inValues[5][1] + 12 * inValues[5][2] + -9 * inValues[5][3] + 18 * inValues[6][0] + 12 * inValues[6][1] + -6 * inValues[6][2] + -9 * inValues[6][3] + -18 * inValues[7][0] + 6 * inValues[7][1] + 6 * inValues[7][2] + 9 * inValues[7][3] + 0.0f;
        outCoeff[59] = -12 * inValues[0][0] + -6 * inValues[0][1] + -8 * inValues[0][2] + -6 * inValues[0][3] + 12 * inValues[1][0] + -6 * inValues[1][1] + 8 * inValues[1][2] + 6 * inValues[1][3] + 12 * inValues[2][0] + 6 * inValues[2][1] + -4 * inValues[2][2] + 6 * inValues[2][3] + -12 * inValues[3][0] + 6 * inValues[3][1] + 4 * inValues[3][2] + -6 * inValues[3][3] + 12 * inValues[4][0] + 6 * inValues[4][1] + 8 * inValues[4][2] + -6 * inValues[4][3] + -12 * inValues[5][0] + 6 * inValues[5][1] + -8 * inValues[5][2] + 6 * inValues[5][3] + -12 * inValues[6][0] + -6 * inValues[6][1] + 4 * inValues[6][2] + 6 * inValues[6][3] + 12 * inValues[7][0] + -6 * inValues[7][1] + -4 * inValues[7][2] + -6 * inValues[7][3] + 0.0f;
        outCoeff[60] = 4 * inValues[0][0] + 2 * inValues[0][2] + 2 * inValues[0][3] + -4 * inValues[2][0] + 2 * inValues[2][2] + -2 * inValues[2][3] + -4 * inValues[4][0] + -2 * inValues[4][2] + 2 * inValues[4][3] + 4 * inValues[6][0] + -2 * inValues[6][2] + -2 * inValues[6][3] + 0.0f;
        outCoeff[61] = 4 * inValues[0][1] + -4 * inValues[2][1] + -4 * inValues[4][1] + 4 * inValues[6][1] + 0.0f;
        outCoeff[62] = -12 * inValues[0][0] + -8 * inValues[0][1] + -6 * inValues[0][2] + -6 * inValues[0][3] + 12 * inValues[1][0] + -4 * inValues[1][1] + 6 * inValues[1][2] + 6 * inValues[1][3] + 12 * inValues[2][0] + 8 * inValues[2][1] + -6 * inValues[2][2] + 6 * inValues[2][3] + -12 * inValues[3][0] + 4 * inValues[3][1] + 6 * inValues[3][2] + -6 * inValues[3][3] + 12 * inValues[4][0] + 8 * inValues[4][1] + 6 * inValues[4][2] + -6 * inValues[4][3] + -12 * inValues[5][0] + 4 * inValues[5][1] + -6 * inValues[5][2] + 6 * inValues[5][3] + -12 * inValues[6][0] + -8 * inValues[6][1] + 6 * inValues[6][2] + 6 * inValues[6][3] + 12 * inValues[7][0] + -4 * inValues[7][1] + -6 * inValues[7][2] + -6 * inValues[7][3] + 0.0f;
        outCoeff[63] = 8 * inValues[0][0] + 4 * inValues[0][1] + 4 * inValues[0][2] + 4 * inValues[0][3] + -8 * inValues[1][0] + 4 * inValues[1][1] + -4 * inValues[1][2] + -4 * inValues[1][3] + -8 * inValues[2][0] + -4 * inValues[2][1] + 4 * inValues[2][2] + -4 * inValues[2][3] + 8 * inValues[3][0] + -4 * inValues[3][1] + -4 * inValues[3][2] + 4 * inValues[3][3] + -8 * inValues[4][0] + -4 * inValues[4][1] + -4 * inValues[4][2] + 4 * inValues[4][3] + 8 * inValues[5][0] + -4 * inValues[5][1] + 4 * inValues[5][2] + -4 * inValues[5][3] + 8 * inValues[6][0] + 4 * inValues[6][1] + -4 * inValues[6][2] + -4 * inValues[6][3] + -8 * inValues[7][0] + 4 * inValues[7][1] + 4 * inValues[7][2] + 4 * inValues[7][3] + 0.0f;
    }

    inline static float interpolateValue(const std::array<float, NUM_COEFFICIENTS>& values, glm::vec3 fracPart) 
    {
        return 0.0f
         + values[0] + values[1] * fracPart[0] + values[2] * fracPart[0] * fracPart[0] + values[3] * fracPart[0] * fracPart[0] * fracPart[0] + values[4] * fracPart[1] + values[5] * fracPart[0] * fracPart[1] + values[6] * fracPart[0] * fracPart[0] * fracPart[1] + values[7] * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] + values[8] * fracPart[1] * fracPart[1] + values[9] * fracPart[0] * fracPart[1] * fracPart[1] + values[10] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] + values[11] * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] + values[12] * fracPart[1] * fracPart[1] * fracPart[1] + values[13] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] + values[14] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] + values[15] * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1]
         + values[16] * fracPart[2] + values[17] * fracPart[0] * fracPart[2] + values[18] * fracPart[0] * fracPart[0] * fracPart[2] + values[19] * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[2] + values[20] * fracPart[1] * fracPart[2] + values[21] * fracPart[0] * fracPart[1] * fracPart[2] + values[22] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] + values[23] * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] + values[24] * fracPart[1] * fracPart[1] * fracPart[2] + values[25] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + values[26] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + values[27] * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + values[28] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] + values[29] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] + values[30] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] + values[31] * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2]
         + values[32] * fracPart[2] * fracPart[2] + values[33] * fracPart[0] * fracPart[2] * fracPart[2] + values[34] * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] + values[35] * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] + values[36] * fracPart[1] * fracPart[2] * fracPart[2] + values[37] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + values[38] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + values[39] * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + values[40] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + values[41] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + values[42] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + values[43] * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + values[44] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + values[45] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + values[46] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + values[47] * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2]
         + values[48] * fracPart[2] * fracPart[2] * fracPart[2] + values[49] * fracPart[0] * fracPart[2] * fracPart[2] * fracPart[2] + values[50] * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] * fracPart[2] + values[51] * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] * fracPart[2] + values[52] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + values[53] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + values[54] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + values[55] * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + values[56] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + values[57] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + values[58] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + values[59] * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + values[60] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + values[61] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + values[62] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + values[63] * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2];
    }
};

#endif