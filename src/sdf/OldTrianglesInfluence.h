#ifndef OLD_TRIANGLES_INFLUENCE_H
#define OLD_TRIANGLES_INFLUENCE_H

#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "OctreeSdfUtils.h"
#include "utils/Timer.h"
#include "utils/GJK.h"

#include <vector>
#include <array>
#include <glm/glm.hpp>

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
}

#endif