#include "SdfLib/ExactOctreeSdf.h"
#include "SdfLib/TrianglesInfluence.h"
#include "SdfLib/InterpolationMethods.h"

namespace sdflib
{
ExactOctreeSdf::ExactOctreeSdf(const Mesh& mesh, BoundingBox box, uint32_t maxDepth,
                               uint32_t startDepth, uint32_t minTrianglesPerNode,
                               uint32_t numThreads)
{
    mMaxDepth = maxDepth;

    const glm::vec3 bbSize = box.getSize();
    const float maxSize = glm::max(glm::max(bbSize.x, bbSize.y), bbSize.z);
    mBox.min = box.getCenter() - 0.5f * maxSize;
    mBox.max = box.getCenter() + 0.5f * maxSize;

    mStartGridSize = 1 << startDepth;
    mStartGridXY = mStartGridSize * mStartGridSize;
    mStartDepth = startDepth;

    mStartGridCellSize = maxSize / static_cast<float>(mStartGridSize);

    mTrianglesData = TriangleUtils::calculateMeshTriangleData(mesh);

    initOctree<PerNodeRegionTrianglesInfluence<NoneInterpolation>>(mesh, startDepth, maxDepth, minTrianglesPerNode, numThreads);
    //initOctree<PerVertexTrianglesInfluence<1, NoneInterpolation>>(mesh, startDepth, maxDepth, minTrianglesPerNode);
    // calculateStatistics();
    mTrianglesCache[0].resize(mMaxTrianglesEncodedInLeafs);
    mTrianglesCache[1].resize(mMaxTrianglesEncodedInLeafs);
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

    if(startArrayPos.x < 0 || startArrayPos.x >= mStartGridSize ||
       startArrayPos.y < 0 || startArrayPos.y >= mStartGridSize ||
       startArrayPos.z < 0 || startArrayPos.z >= mStartGridSize)
    {
        return mBox.getDistance(sample) + glm::sqrt(3.0f) * mBox.getSize().x;
    }

    const OctreeNode* currentNode = &mOctreeData[startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x];

    float minDist = INFINITY;
    uint32_t minIndex = 0;
    uint32_t depth = mStartDepth;

    while(!currentNode->isLeaf() && depth < mBitEncodingStartDepth)
    {
        const uint32_t childIdx = (roundFloat(fracPart.z) << 2) + 
                                  (roundFloat(fracPart.y) << 1) + 
                                   roundFloat(fracPart.x);

        currentNode = &mOctreeData[currentNode->getChildrenIndex() + childIdx];
        fracPart = glm::fract(2.0f * fracPart);
        depth++;
    }

    if(currentNode->isLeaf())
    {
        uint32_t leafIndex = currentNode->trianglesArrayIndex;
        const uint32_t numTriangles = mTrianglesSets[leafIndex++];

        uint32_t bIdx = 0;
        for(uint32_t t=0; t < numTriangles; t++, bIdx += mBitsPerIndex)
        {
            uint32_t idx = bIdx >> 5;
            uint32_t bit = bIdx & 0b0011111;
            const uint32_t tIndex = ((mTrianglesSets[leafIndex + idx] << bit) >> (32-mBitsPerIndex)) |
                                static_cast<uint32_t>(static_cast<uint64_t>(mTrianglesSets[leafIndex + idx + 1]) >> (64 - (bit + mBitsPerIndex)));

            const float dist = TriangleUtils::getSqDistPointAndTriangle(sample, mTrianglesData[tIndex]);
            if(dist < minDist)
            {
                minIndex = tIndex;
                minDist = dist;
            }
        }

        return TriangleUtils::getSignedDistPointAndTriangle(sample, mTrianglesData[minIndex]);
    }


    uint32_t setIndex = currentNode->trianglesArrayIndex;

    // Pass to next child
    {
    const uint32_t childIdx = (roundFloat(fracPart.z) << 2) + 
                              (roundFloat(fracPart.y) << 1) + 
                               roundFloat(fracPart.x);

    currentNode = &mOctreeData[currentNode->getChildrenIndex() + childIdx];
    fracPart = glm::fract(2.0f * fracPart);
    }

    uint32_t numTriangles = mTrianglesSets[setIndex++];
    uint32_t* inputTriangles = mTrianglesCache[0].data();
    {
        const uint8_t* mask = mTrianglesMasks.data() + currentNode->trianglesArrayIndex;

        uint32_t newTriangles = 0;
        uint32_t t = 0;
        uint32_t bIdx = 0;
        for(uint32_t b=0; t < numTriangles; b++)
        {
            uint32_t i=0;
            uint8_t code = mask[b];
            for(; i < 8; i++, t++, bIdx += mBitsPerIndex)
            {
                if(code & 0b10000000) 
                {
                    uint32_t idx = bIdx >> 5;
                    uint32_t bit = bIdx & 0b0011111;
                    inputTriangles[newTriangles++] = ((mTrianglesSets[setIndex + idx] << bit) >> (32-mBitsPerIndex)) |
                                                     static_cast<uint32_t>(static_cast<uint64_t>(mTrianglesSets[setIndex + idx + 1]) >> (64 - (bit + mBitsPerIndex)));;
                }
                code = code << 1;
            }
        }

        numTriangles = newTriangles;
    }

    uint32_t* outputTriangles = mTrianglesCache[1].data();
    while(!currentNode->isLeaf())
    {
        const uint32_t childIdx = (roundFloat(fracPart.z) << 2) + 
                                  (roundFloat(fracPart.y) << 1) + 
                                   roundFloat(fracPart.x);

        currentNode = &mOctreeData[currentNode->getChildrenIndex() + childIdx];
        fracPart = glm::fract(2.0f * fracPart);

        const uint8_t* mask = mTrianglesMasks.data() + currentNode->trianglesArrayIndex;

        uint32_t newTriangles = 0;
        uint32_t idx = 0;
        for(uint32_t b=0; idx < numTriangles; b++)
        {
            uint32_t i=0;
            uint8_t code = mask[b];
            for(; i < 8; i++, idx++)
            {
                if(code & 0b10000000) 
                {
                    outputTriangles[newTriangles++] = inputTriangles[idx];
                }
                code = code << 1;
            }
        }

        numTriangles = newTriangles;

        std::swap(outputTriangles, inputTriangles);
    }

    for(uint32_t t=0; t < numTriangles; t++)
    {
        const uint32_t tIndex = inputTriangles[t];
        const float dist = TriangleUtils::getSqDistPointAndTriangle(sample, mTrianglesData[tIndex]);
        if(dist < minDist)
        {
            minIndex = tIndex;
            minDist = dist;
        }
    }

    return TriangleUtils::getSignedDistPointAndTriangle(sample, mTrianglesData[minIndex]);
}

float ExactOctreeSdf::getDistance(glm::vec3 sample, glm::vec3& outGradient) const
{
    glm::vec3 fracPart = (sample - mBox.min) / mStartGridCellSize;
    glm::ivec3 startArrayPos = glm::floor(fracPart);
    fracPart = glm::fract(fracPart);

    if(startArrayPos.x < 0 || startArrayPos.x >= mStartGridSize ||
       startArrayPos.y < 0 || startArrayPos.y >= mStartGridSize ||
       startArrayPos.z < 0 || startArrayPos.z >= mStartGridSize)
    {
        return mBox.getDistance(sample) + glm::sqrt(3.0f) * mBox.getSize().x;
    }

    const OctreeNode* currentNode = &mOctreeData[startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x];

    float minDist = INFINITY;
    uint32_t minIndex = 0;
    uint32_t depth = mStartDepth;

    while(!currentNode->isLeaf() && depth < mBitEncodingStartDepth)
    {
        const uint32_t childIdx = (roundFloat(fracPart.z) << 2) + 
                                  (roundFloat(fracPart.y) << 1) + 
                                   roundFloat(fracPart.x);

        currentNode = &mOctreeData[currentNode->getChildrenIndex() + childIdx];
        fracPart = glm::fract(2.0f * fracPart);
        depth++;
    }

    if(currentNode->isLeaf())
    {
        uint32_t leafIndex = currentNode->trianglesArrayIndex;
        const uint32_t numTriangles = mTrianglesSets[leafIndex++];

        uint32_t bIdx = 0;
        for(uint32_t t=0; t < numTriangles; t++, bIdx += mBitsPerIndex)
        {
            uint32_t idx = bIdx >> 5;
            uint32_t bit = bIdx & 0b0011111;
            const uint32_t tIndex = ((mTrianglesSets[leafIndex + idx] << bit) >> (32-mBitsPerIndex)) |
                                static_cast<uint32_t>(static_cast<uint64_t>(mTrianglesSets[leafIndex + idx + 1]) >> (64 - (bit + mBitsPerIndex)));

            const float dist = TriangleUtils::getSqDistPointAndTriangle(sample, mTrianglesData[tIndex]);
            if(dist < minDist)
            {
                minIndex = tIndex;
                minDist = dist;
            }
        }

        return TriangleUtils::getSignedDistPointAndTriangle(sample, mTrianglesData[minIndex], outGradient);
    }


    uint32_t setIndex = currentNode->trianglesArrayIndex;

    // Pass to next child
    {
    const uint32_t childIdx = (roundFloat(fracPart.z) << 2) + 
                                  (roundFloat(fracPart.y) << 1) + 
                                   roundFloat(fracPart.x);

    currentNode = &mOctreeData[currentNode->getChildrenIndex() + childIdx];
    fracPart = glm::fract(2.0f * fracPart);
    }

    uint32_t numTriangles = mTrianglesSets[setIndex++];
    uint32_t* inputTriangles = mTrianglesCache[0].data();
    {
        const uint8_t* mask = mTrianglesMasks.data() + currentNode->trianglesArrayIndex;

        uint32_t newTriangles = 0;
        uint32_t t = 0;
        uint32_t bIdx = 0;
        for(uint32_t b=0; t < numTriangles; b++)
        {
            uint32_t i=0;
            uint8_t code = mask[b];
            for(; i < 8; i++, t++, bIdx += mBitsPerIndex)
            {
                if(code & 0b10000000) 
                {
                    uint32_t idx = bIdx >> 5;
                    uint32_t bit = bIdx & 0b0011111;
                    inputTriangles[newTriangles++] = ((mTrianglesSets[setIndex + idx] << bit) >> (32-mBitsPerIndex)) |
                                                     static_cast<uint32_t>(static_cast<uint64_t>(mTrianglesSets[setIndex + idx + 1]) >> (64 - (bit + mBitsPerIndex)));;
                }
                code = code << 1;
            }
        }

        numTriangles = newTriangles;
    }

    uint32_t* outputTriangles = mTrianglesCache[1].data();
    while(!currentNode->isLeaf())
    {
        const uint32_t childIdx = (roundFloat(fracPart.z) << 2) + 
                                  (roundFloat(fracPart.y) << 1) + 
                                   roundFloat(fracPart.x);

        currentNode = &mOctreeData[currentNode->getChildrenIndex() + childIdx];
        fracPart = glm::fract(2.0f * fracPart);

        const uint8_t* mask = mTrianglesMasks.data() + currentNode->trianglesArrayIndex;

        uint32_t newTriangles = 0;
        uint32_t idx = 0;
        for(uint32_t b=0; idx < numTriangles; b++)
        {
            uint32_t i=0;
            uint8_t code = mask[b];
            for(; i < 8; i++, idx++)
            {
                if(code & 0b10000000) 
                {
                    outputTriangles[newTriangles++] = inputTriangles[idx];
                }
                code = code << 1;
            }
        }

        numTriangles = newTriangles;

        std::swap(outputTriangles, inputTriangles);
    }

    for(uint32_t t=0; t < numTriangles; t++)
    {
        const uint32_t tIndex = inputTriangles[t];
        const float dist = TriangleUtils::getSqDistPointAndTriangle(sample, mTrianglesData[tIndex]);
        if(dist < minDist)
        {
            minIndex = tIndex;
            minDist = dist;
        }
    }

    return TriangleUtils::getSignedDistPointAndTriangle(sample, mTrianglesData[minIndex], outGradient);
}

std::vector<uint32_t> ExactOctreeSdf::evalNode(uint32_t nodeIndex, uint32_t depth, 
                                               std::vector<uint32_t>& mergedTriangles, 
                                               std::vector<uint32_t>& mergedNodes,
                                               std::vector<uint32_t>& differentTriangles)
{
    // std::vector<uint32_t> triangles;
    // if(!mOctreeData[nodeIndex].isLeaf())
    // {
    //     const uint32_t idx = mOctreeData[nodeIndex].getChildrenIndex();
    //     triangles = evalNode(idx + 0, depth+1, mergedTriangles, mergedNodes, differentTriangles);
    //     uint32_t sumTriangles = 0;
    //     for(uint32_t c=1; c < 8; c++)
    //     {
    //         std::vector<uint32_t> tri = evalNode(idx + c, depth+1, mergedTriangles, mergedNodes, differentTriangles);
    //         sumTriangles += tri.size();
    //         std::vector<uint32_t> newT;
    //         std::set_intersection(triangles.begin(), triangles.end(), tri.begin(), tri.end(), std::back_inserter(newT));
    //         triangles = newT;
    //     }

    //     mergedTriangles[depth] += triangles.size();
    //     mergedNodes[depth] += 1;
    //     differentTriangles[depth] += (sumTriangles - 8 * triangles.size()) / 8;
    // }
    // else
    // {
    //     uint32_t leafIndex = mOctreeData[nodeIndex].getChildrenIndex();
    //     const uint32_t numTriangles = mOctreeData[leafIndex++].size;

    //     for(uint32_t t=0; t < numTriangles; t++)
    //     {
    //         triangles.push_back(mOctreeData[leafIndex + t].triangleIndex);
    //     }
    // }

    // return triangles;
    return std::vector<uint32_t>();
}

void ExactOctreeSdf::calculateStatistics()
{
    // uint32_t startDepth = std::log2(mStartGridSize);
    // std::vector<uint32_t> mergedTriangles(mMaxDepth);
    // std::vector<uint32_t> mergedNodes(mMaxDepth);
    // std::vector<uint32_t> differentTriangles(mMaxDepth);
    // for(uint32_t k=0; k < mStartGridSize; k++)
    // {
    //     for(uint32_t j=0; j < mStartGridSize; j++)
    //     {
    //         for(uint32_t i=0; i < mStartGridSize; i++)
    //         {
    //             evalNode(k * mStartGridSize * mStartGridSize + j * mStartGridSize + i, startDepth,
    //                     mergedTriangles,
    //                     mergedNodes,
    //                     differentTriangles);
    //         }
    //     }
    // }

    // for(uint32_t d=0; d < mMaxDepth; d++)
    // {
    //     const float mergedMean = static_cast<float>(mergedTriangles[d]) / static_cast<float>(mergedNodes[d]);
    //     const float differentMean = static_cast<float>(differentTriangles[d]) / static_cast<float>(mergedNodes[d]);
    //     SPDLOG_INFO("Depth {}, mean of merged nodes: {} // mean of different nodes: {}", d, mergedMean, differentMean);
    // }
}
}