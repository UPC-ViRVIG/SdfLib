#include "ExactOctreeSdf.h"
#include "TrianglesInfluence.h"

ExactOctreeSdf::ExactOctreeSdf(const Mesh& mesh, BoundingBox box, uint32_t maxDepth,
                               uint32_t startDepth, uint32_t minTrianglesPerNode)
{
    mMaxDepth = maxDepth;

    const glm::vec3 bbSize = box.getSize();
    const float maxSize = glm::max(glm::max(bbSize.x, bbSize.y), bbSize.z);
    mBox.min = box.min;
    mBox.max = box.min + maxSize;

    mStartGridSize = 1 << startDepth;
    mStartGridXY = mStartGridSize * mStartGridSize;

    mStartGridCellSize = maxSize / static_cast<float>(mStartGridSize);

    mTrianglesData = TriangleUtils::calculateMeshTriangleData(mesh);

    initOctree<PerVertexTrianglesInfluence<1>>(mesh, startDepth, maxDepth, minTrianglesPerNode);
}

inline uint32_t roundFloat(float a)
{
    return (a > 0.5f) ? 1 : 0;
}

float ExactOctreeSdf::getDistance(glm::vec3 sample) const
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

    uint32_t leafIndex = currentNode->getChildrenIndex();
    const uint32_t numTriangles = mOctreeData[leafIndex++].size;

    float minDist = INFINITY;
    uint32_t minIndex = 0;

    for(uint32_t t=0; t < numTriangles; t++)
    {
        const uint32_t tIndex = mOctreeData[leafIndex + t].triangleIndex;
        const float dist = TriangleUtils::getSqDistPointAndTriangle(sample, mTrianglesData[tIndex]);
        if(dist < minDist)
        {
            minIndex = tIndex;
            minDist = dist;
        }
    }

    return TriangleUtils::getSignedDistPointAndTriangle(sample, mTrianglesData[minIndex]);
}