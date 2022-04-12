#include "OctreeSdf.h"
#include "utils/Timer.h"
#include "utils/GJK.h"
#include "OctreeSdfUtils.h"
#include <array>
#include <stack>

OctreeSdf::OctreeSdf(const Mesh& mesh, BoundingBox box, 
                     uint32_t depth, uint32_t startDepth,
                     float terminationThreshold,
					 OctreeSdf::TerminationRule terminationRule,
                     OctreeSdf::InitAlgorithm initAlgorithm)
{
    mMaxDepth = depth;

    const glm::vec3 bbSize = box.getSize();
    const float maxSize = glm::max(glm::max(bbSize.x, bbSize.y), bbSize.z);
    mBox.min = box.min;
    mBox.max = box.min + maxSize;

    mStartGridSize = 1 << startDepth;
    mStartGridXY = mStartGridSize * mStartGridSize;

    mStartGridCellSize = maxSize / static_cast<float>(mStartGridSize);

    switch(initAlgorithm)
    {
        case OctreeSdf::InitAlgorithm::DF_UNIFORM:
            initUniformOctree(mesh, startDepth, depth);
            break;
        case OctreeSdf::InitAlgorithm::DF_ADAPTATIVE:
            initOctree(mesh, startDepth, depth, terminationThreshold, terminationRule);
            break;
        case OctreeSdf::InitAlgorithm::BF_ADAPTATIVE:
            initOctreeWithContinuity(mesh, startDepth, depth, terminationThreshold, terminationRule);
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

    const float* values = reinterpret_cast<const float*>(&mOctreeData[currentNode->getChildrenIndex()]);

    return interpolateValue(values, fracPart);
}