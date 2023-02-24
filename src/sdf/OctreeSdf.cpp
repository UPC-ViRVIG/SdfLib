#include "OctreeSdf.h"
#include "utils/Timer.h"
#include "utils/GJK.h"
#include "OctreeSdfUtils.h"
#include "TrianglesInfluence.h"
#include "InterpolationMethods.h"
#include <array>
#include <stack>

// typedef TriLinearInterpolation InterpolationMethod;
typedef TriCubicInterpolation InterpolationMethod;

OctreeSdf::OctreeSdf(const Mesh& mesh, BoundingBox box, 
                     uint32_t depth, uint32_t startDepth,
                     float terminationThreshold,
					 OctreeSdf::TerminationRule terminationRule,
                     OctreeSdf::InitAlgorithm initAlgorithm,
                     uint32_t numThreads)
{
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
        case OctreeSdf::InitAlgorithm::DF_UNIFORM:
            initUniformOctree(mesh, startDepth, depth);
            break;
        case OctreeSdf::InitAlgorithm::DF_ADAPTATIVE:
            // initOctree<PerNodeRegionTrianglesInfluence<InterpolationMethod>>(mesh, startDepth, depth, terminationThreshold, terminationRule, numThreads);
            initOctree<VHQueries<InterpolationMethod>>(mesh, startDepth, depth, terminationThreshold, terminationRule, numThreads);
            //initOctree<FCPWQueries<InterpolationMethod>>(mesh, startDepth, depth, terminationThreshold, terminationRule, numThreads);
            break;
        case OctreeSdf::InitAlgorithm::BF_ADAPTATIVE:
            //initOctreeWithContinuity<PerNodeRegionTrianglesInfluence<InterpolationMethod>>(mesh, startDepth, depth, terminationThreshold, terminationRule);
            // initOctreeWithContinuity<VHQueries<InterpolationMethod>>(mesh, startDepth, depth, terminationThreshold, terminationRule);
            initOctreeWithContinuityNoDelay<VHQueries<InterpolationMethod>>(mesh, startDepth, depth, terminationThreshold, terminationRule, numThreads);
            break;
        case OctreeSdf::InitAlgorithm::GPU_IMPLEMENTATION:
            Timer time;
            time.start();
            //initOctree<PerNodeRegionTrianglesInfluence<InterpolationMethod>>(mesh, startDepth, depth, terminationThreshold, terminationRule, 1);
            SPDLOG_INFO("CPU version takes: {}s", time.getElapsedSeconds());
            time.start();
            initOctreeInGPU(mesh, startDepth, depth, terminationThreshold, terminationRule);
            SPDLOG_INFO("GPU version takes: {}s", time.getElapsedSeconds());
            break;
    }
}

inline uint32_t roundFloat(float a)
{
    return (a > 0.5f) ? 1 : 0;
}

float OctreeSdf::getDistance(glm::vec3 sample) const
{
    glm::vec3 fracPart = (sample - mBox.min) / mStartGridCellSize;
    glm::ivec3 startArrayPos = glm::floor(fracPart);
    fracPart = glm::fract(fracPart);

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
    //TODO
    return 0.0f;
}