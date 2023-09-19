#include "SdfLib/OctreeSdf.h"
#include "SdfLib/utils/Timer.h"
#include "SdfLib/utils/GJK.h"
#include "SdfLib/OctreeSdfUtils.h"
#include "SdfLib/TrianglesInfluence.h"
#include "SdfLib/InterpolationMethods.h"
#include "sdf/OctreeSdfDepthFirst.h"
#include "sdf/OctreeSdfBreadthFirst.h"
#include "sdf/OctreeSdfBreadthFirstNoDelay.h"
#include <array>
#include <stack>

namespace sdflib
{
// typedef TriLinearInterpolation InterpolationMethod;
typedef TriCubicInterpolation InterpolationMethod;

OctreeSdf::OctreeSdf(const Mesh& mesh, BoundingBox box, 
                     uint32_t depth, uint32_t startDepth,
                     float terminationThreshold,
                     OctreeSdf::InitAlgorithm initAlgorithm,
                     uint32_t numThreads)
{
    const OctreeSdf::TerminationRule terminationRule = TerminationRule::TRAPEZOIDAL_RULE;
    mMaxDepth = depth;

    const glm::vec3 bbSize = box.getSize();
    const float maxSize = glm::max(glm::max(bbSize.x, bbSize.y), bbSize.z);
    mBox.min = box.getCenter() - 0.5f * maxSize;
    mBox.max = box.getCenter() + 0.5f * maxSize;

    mStartGridSize = 1 << startDepth;
    mStartGridXY = mStartGridSize * mStartGridSize;

    mStartGridCellSize = maxSize / static_cast<float>(mStartGridSize);

    switch(initAlgorithm)
    {
        case OctreeSdf::InitAlgorithm::UNIFORM:
            initUniformOctree(mesh, startDepth, depth);
            break;
        case OctreeSdf::InitAlgorithm::NO_CONTINUITY:
            // initOctree<PerNodeRegionTrianglesInfluence<InterpolationMethod>>(mesh, startDepth, depth, terminationThreshold, terminationRule, numThreads);
            initOctree<VHQueries<InterpolationMethod>>(mesh, startDepth, depth, terminationThreshold, terminationRule, numThreads);
            //initOctree<FCPWQueries<InterpolationMethod>>(mesh, startDepth, depth, terminationThreshold, terminationRule, numThreads);
            break;
        case OctreeSdf::InitAlgorithm::CONTINUITY:
            //initOctreeWithContinuity<PerNodeRegionTrianglesInfluence<InterpolationMethod>>(mesh, startDepth, depth, terminationThreshold, terminationRule);
            if(DELAY_NODE_TERMINATION)
            {
                initOctreeWithContinuity<VHQueries<InterpolationMethod>>(mesh, startDepth, depth, terminationThreshold, terminationRule);
            }
            else
            {
                initOctreeWithContinuityNoDelay<VHQueries<InterpolationMethod>>(mesh, startDepth, depth, terminationThreshold, terminationRule, numThreads);
            }
            break;
        // case OctreeSdf::InitAlgorithm::GPU_IMPLEMENTATION:
        //     Timer time;
        //     time.start();
        //     //initOctree<PerNodeRegionTrianglesInfluence<InterpolationMethod>>(mesh, startDepth, depth, terminationThreshold, terminationRule, 1);
        //     SPDLOG_INFO("CPU version takes: {}s", time.getElapsedSeconds());
        //     time.start();
        //     initOctreeInGPU(mesh, startDepth, depth, terminationThreshold, terminationRule);
        //     SPDLOG_INFO("GPU version takes: {}s", time.getElapsedSeconds());
        //     break;
    }

    computeMinBorderValue();
}

inline uint32_t roundFloat(float a)
{
    return (a >= 0.5f) ? 1 : 0;
}

float OctreeSdf::getDistance(glm::vec3 sample) const
{
    glm::vec3 fracPart = (sample - mBox.min) / mStartGridCellSize;
    glm::ivec3 startArrayPos = glm::floor(fracPart);
    fracPart = glm::fract(fracPart);

    if(startArrayPos.x < 0 || startArrayPos.x >= mStartGridSize ||
       startArrayPos.y < 0 || startArrayPos.y >= mStartGridSize ||
       startArrayPos.z < 0 || startArrayPos.z >= mStartGridSize)
    {
        return mBox.getDistance(sample) + mMinBorderValue;
    }

    const OctreeNode* currentNode = &mOctreeData[startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x];

    while(!currentNode->isLeaf())
    {
        const uint32_t childIdx = (roundFloat(fracPart.z) << 2) + 
                                  (roundFloat(fracPart.y) << 1) + 
                                   roundFloat(fracPart.x);

        currentNode = &mOctreeData[currentNode->getChildrenIndex() + childIdx];
        fracPart = glm::fract(2.0f * fracPart);
    }

    auto& values = *reinterpret_cast<const std::array<float, InterpolationMethod::NUM_COEFFICIENTS>*>(&mOctreeData[currentNode->getChildrenIndex()]);

    return InterpolationMethod::interpolateValue(values, fracPart);
}

float OctreeSdf::getDistance(glm::vec3 sample, glm::vec3& outGradient) const
{
    glm::vec3 fracPart = (sample - mBox.min) / mStartGridCellSize;
    glm::ivec3 startArrayPos = glm::floor(fracPart);
    fracPart = glm::fract(fracPart);

    if(startArrayPos.x < 0 || startArrayPos.x >= mStartGridSize ||
       startArrayPos.y < 0 || startArrayPos.y >= mStartGridSize ||
       startArrayPos.z < 0 || startArrayPos.z >= mStartGridSize)
    {
        return mBox.getDistance(sample, outGradient) + mMinBorderValue;
    }

    const OctreeNode* currentNode = &mOctreeData[startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x];

    while(!currentNode->isLeaf())
    {
        const uint32_t childIdx = (roundFloat(fracPart.z) << 2) + 
                                  (roundFloat(fracPart.y) << 1) + 
                                   roundFloat(fracPart.x);

        currentNode = &mOctreeData[currentNode->getChildrenIndex() + childIdx];
        fracPart = glm::fract(2.0f * fracPart);
    }

    auto& values = *reinterpret_cast<const std::array<float, InterpolationMethod::NUM_COEFFICIENTS>*>(&mOctreeData[currentNode->getChildrenIndex()]);

    outGradient = glm::normalize(InterpolationMethod::interpolateGradient(values, fracPart));
    return InterpolationMethod::interpolateValue(values, fracPart);
}


void OctreeSdf::computeMinBorderValue()
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

    std::function<float(uint32_t nIdx, glm::vec3 pos, float halfSize)> processNode;
    processNode = [&](uint32_t nIdx, glm::vec3 pos, float halfSize) -> float
    {
        if(!mOctreeData[nIdx].isLeaf())
        {
            float minValue = INFINITY;
            for(uint32_t i=0; i < 8; i++)
            {
                const glm::vec3 cp = pos + 0.5f * halfSize * childrens[i];
                if(cp.x < halfSize || cp.y < halfSize || cp.z < halfSize ||
                   cp.x > (1.0f-halfSize) || cp.y > (1.0f-halfSize) || cp.z > (1.0f-halfSize))
                {
                    minValue = glm::min(minValue, processNode(mOctreeData[nIdx].getChildrenIndex() + i, cp, 0.5f * halfSize));
                }
            }

            return minValue;
        }
        else
        {
            float minValue = INFINITY;
            for(uint32_t i=0; i < 8; i++)
            {
                const glm::vec3 sp = pos + halfSize * childrens[i];
                if(sp.x < 1e-4 || sp.y < 1e-4 || sp.z < 1e-4 ||
                   sp.x > (1.0f-1e-4) || sp.y > (1.0f-1e-4) || sp.z > (1.0f-1e-4))
                {
                    std::array<float, InterpolationMethod::NUM_COEFFICIENTS>* coeff = reinterpret_cast<std::array<float, InterpolationMethod::NUM_COEFFICIENTS>*>(
                                                                                            &mOctreeData[mOctreeData[nIdx].getChildrenIndex()]);
                    minValue = glm::min(minValue, InterpolationMethod::interpolateValue(*coeff, 0.5f * childrens[i] + glm::vec3(0.5f)));

                    float val = InterpolationMethod::interpolateValue(*coeff, 0.5f * childrens[i] + glm::vec3(0.5f));
                }
            }

            return minValue;
        }

        return INFINITY;
    };

    const float gridCellSize = 1.0f / static_cast<float>(mStartGridSize);

    float minValue = INFINITY;
    for(uint32_t k=0; k < mStartGridSize; k++)
    {
        for(uint32_t j=0; j < mStartGridSize; j++)
        {
            for(uint32_t i=0; i < mStartGridSize; i++)
            {
                const uint32_t idx = k * mStartGridXY + j * mStartGridSize + i;
                const glm::vec3 pos((static_cast<float>(i) + 0.5f) * gridCellSize,
                                    (static_cast<float>(j) + 0.5f) * gridCellSize,
                                    (static_cast<float>(k) + 0.5f) * gridCellSize);
                minValue = glm::min(minValue, processNode(idx, pos, 0.5f * gridCellSize));
            }
        }
    }

    mMinBorderValue = minValue;
}

void OctreeSdf::getDepthDensity(std::vector<float>& depthsDensity)
{
    depthsDensity.resize(mMaxDepth + 1);
    std::vector<uint32_t> nodesPerDepth(depthsDensity.size(), 0);
    uint32_t numLeaves = 0;
    std::function<void(OctreeNode&, uint32_t)> vistNode;
    vistNode = [&](OctreeNode& node, uint32_t depth)
    {
        node.removeMark();

        // Iterate children
        if(!node.isLeaf())
        {
            for(uint32_t i = 0; i < 8; i++)
            {
                vistNode(mOctreeData[node.getChildrenIndex() + i], depth+1);
            }
        }
        else
        {
            nodesPerDepth[depth]++;
            numLeaves++;
        }
    };

    const uint32_t startDepth = glm::round(glm::log2(static_cast<float>(mStartGridSize)));

    for(uint32_t k=0; k < mStartGridSize; k++)
    {
        for(uint32_t j=0; j < mStartGridSize; j++)
        {
            for(uint32_t i=0; i < mStartGridSize; i++)
            {
                const uint32_t nodeStartIndex = k * mStartGridSize * mStartGridSize + j * mStartGridSize + i;
                vistNode(mOctreeData[nodeStartIndex], startDepth);
            }
        }
    }

    float size = 1.0f;
    for(uint32_t d=0; d < depthsDensity.size(); d++)
    {
        depthsDensity[d] = size * static_cast<float>(nodesPerDepth[d]);
        size *= 0.125f;
    }
}
}
