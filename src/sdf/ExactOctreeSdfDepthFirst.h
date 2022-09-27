#ifndef EXACT_OCTREE_SDF_DEPTH_FIRST_H
#define EXACT_OCTREE_SDF_DEPTH_FIRST_H

#include <string>

template<typename VertexInfo, int VALUES_PER_VERTEX>
struct DepthFirstNodeInfoExactOctree
{
	DepthFirstNodeInfoExactOctree(uint8_t childIndex, uint32_t nodeIndex, uint16_t depth, glm::vec3 center, float size)
                        : childIndex(childIndex), nodeIndex(nodeIndex), trianglesProcessed(false), depth(depth), center(center), size(size) {}
    uint32_t nodeIndex;
    uint8_t childIndex;
    bool trianglesProcessed;
    uint16_t depth;
    glm::vec3 center;
    float size;
    std::array<std::array<float, VALUES_PER_VERTEX>, 8> verticesValues;
    std::array<VertexInfo, 8> verticesInfo;
};

template<typename TrianglesInfluenceStrategy>
void ExactOctreeSdf::initOctree(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                                uint32_t minTrianglesPerNode)
{
    typedef TrianglesInfluenceStrategy::InterpolationMethod InterpolationMethod;
    typedef DepthFirstNodeInfoExactOctree<TrianglesInfluenceStrategy::VertexInfo, InterpolationMethod::VALUES_PER_VERTEX> NodeInfo;

    mMinTrianglesInLeafs = minTrianglesPerNode;

    std::vector<TriangleUtils::TriangleData>& trianglesData = mTrianglesData;
    
    const uint32_t startOctreeDepth = glm::min(startDepth, START_OCTREE_DEPTH);

    const uint32_t numTriangles = trianglesData.size();
    std::vector<std::array<std::vector<uint32_t>, 8>> triangles(maxDepth - startOctreeDepth + 2);
    std::vector<std::vector<uint32_t>*> trianglesCache(maxDepth - startOctreeDepth + 2);
    triangles[0].fill(std::vector<uint32_t>());
	triangles[0][0].resize(numTriangles);
    for(uint32_t i=0; i < numTriangles; i++)
    {
        triangles[0][0][i] = i;
    }
    trianglesCache[0] = &triangles[0][0];

    uint32_t bitsPerIndex = static_cast<int32_t>(glm::ceil(glm::log2(static_cast<float>(numTriangles))));
    uint32_t invBitsPerIndex = 32 - bitsPerIndex;
    mBitsPerIndex = bitsPerIndex;


    TrianglesInfluenceStrategy trianglesInfluence;
    trianglesInfluence.initCaches(mBox, maxDepth);

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

    const std::array<glm::vec3, 19> nodeSamplePoints =
    {
        glm::vec3(0.0f, -1.0f, -1.0f),
        glm::vec3(-1.0f, 0.0f, -1.0f),
        glm::vec3(0.0f, 0.0f, -1.0f),
        glm::vec3(1.0f, 0.0f, -1.0f),
        glm::vec3(0.0f, 1.0f, -1.0f),

        glm::vec3(-1.0f, -1.0f, 0.0f),
        glm::vec3(0.0f, -1.0f, 0.0f),
        glm::vec3(1.0f, -1.0f, 0.0f),
        glm::vec3(-1.0f, 0.0f, 0.0f),
        glm::vec3(0.0f),
        glm::vec3(1.0f, 0.0f, 0.0f),
        glm::vec3(-1.0f, 1.0f, 0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        glm::vec3(1.0f, 1.0f, 0.0f),

        glm::vec3(0.0f, -1.0f, 1.0f),
        glm::vec3(-1.0f, 0.0f, 1.0f),
        glm::vec3(0.0f, 0.0f, 1.0f),
        glm::vec3(1.0f, 0.0f, 1.0f),
        glm::vec3(0.0f, 1.0f, 1.0f),
    };

    // Create the grid
    {
        const uint32_t voxlesPerAxis = 1 << startDepth;
        mOctreeData.resize(voxlesPerAxis * voxlesPerAxis * voxlesPerAxis);
    }
    
    std::stack<NodeInfo> nodes;
    {
        float newSize = 0.5f * mBox.getSize().x * glm::pow(0.5f, startOctreeDepth);
        const glm::vec3 startCenter = mBox.min + newSize;
        const uint32_t voxlesPerAxis = 1 << startOctreeDepth;

        for(uint32_t k=0; k < voxlesPerAxis; k++)
        {
            for(uint32_t j=0; j < voxlesPerAxis; j++)
            {
                for(uint32_t i=0; i < voxlesPerAxis; i++)
                {
                    nodes.push(NodeInfo(0, std::numeric_limits<uint32_t>::max(), startOctreeDepth, startCenter + glm::vec3(i, j, k) * 2.0f * newSize, newSize));
                    NodeInfo& n = nodes.top();
                    std::array<float, InterpolationMethod::NUM_COEFFICIENTS> nullArray;
                    trianglesInfluence.calculateVerticesInfo(n.center, n.size, triangles[0][0], childrens,
                                                             0u, nullArray,
                                                             n.verticesValues, n.verticesInfo,
                                                             mesh, trianglesData);
                }
            }
        }
    }

    std::array<glm::vec3, 3> triangle;

    std::array<std::vector<uint32_t>, 8> outputTrianglesCache;
    std::array<std::vector<uint8_t>, 8> outputTrianglesMaskCache;
    outputTrianglesCache.fill(std::vector<uint32_t>());

    const std::vector<glm::vec3>& vertices = mesh.getVertices();
    const std::vector<uint32_t>& indices = mesh.getIndices();

#ifdef PRINT_STATISTICS
    std::vector<std::pair<uint32_t, uint32_t>> verticesStatistics(maxDepth + 1, std::make_pair(0, 0));
    verticesStatistics[0] = std::make_pair(trianglesData.size(), 1);
    std::vector<uint32_t> endedNodes(maxDepth + 1, 0);
    std::vector<float> elapsedTime(maxDepth + 1, 0.0f);
    std::vector<uint32_t> numTrianglesEvaluated(maxDepth + 1, 0);
    uint64_t numTrianglesInLeafs = 0;
    uint64_t numLeafs = 0;
    Timer timer;
#endif

    mMaxTrianglesInLeafs = 0;
    mMaxTrianglesEncodedInLeafs = 0;

    uint32_t bitEncodingStartDepth = maxDepth - BIT_ENCODING_DEPTH;
    mBitEncodingStartDepth = bitEncodingStartDepth;

    while(!nodes.empty())
    {
        #ifdef PRINT_STATISTICS
        timer.start();
        #endif
        const NodeInfo node = nodes.top();
        const uint32_t rDepth = node.depth - startOctreeDepth + 1;

        OctreeNode* octreeNode = (node.nodeIndex < std::numeric_limits<uint32_t>::max()) 
                                    ? &mOctreeData[node.nodeIndex]
                                    : nullptr;

        if(node.depth == startDepth)
        {
            assert(octreeNode == nullptr);
            glm::ivec3 startArrayPos = glm::floor((node.center - mBox.min) / mStartGridCellSize);
            octreeNode = &mOctreeData[startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x];
        }

        if(node.trianglesProcessed)
        {
            nodes.pop();

            OctreeNode* octreeNodesChildren = &mOctreeData[octreeNode->getChildrenIndex()];

            std::vector<uint32_t>& nodeTriangles = triangles[rDepth][node.childIndex];
            std::vector<uint32_t> oldTriangles = nodeTriangles;

            for(uint32_t c=0; c < 8; c++)
            {
                outputTrianglesMaskCache[c].resize((nodeTriangles.size() + 7) / 8, 0);
            }

            nodeTriangles.clear();
            const std::array<std::vector<uint32_t>, 8> chTriangles = triangles[rDepth + 1];
            std::array<uint32_t, 8> chIndices;
            std::array<uint32_t, 8> childrenCache;
            chIndices.fill(0);

            uint32_t numCollisions;
            do {
                numCollisions = 0;
                uint32_t minIndex = std::numeric_limits<uint32_t>::max();
                for(uint32_t c=0; c < 8; c++)
                {
                    if(chIndices[c] < chTriangles[c].size())
                    {
                        const uint32_t index = chTriangles[c][chIndices[c]];
                        if(index < minIndex)
                        {
                            minIndex = index;
                            numCollisions = 0;
                            childrenCache[numCollisions++] = c;
                        }
                        else if(index == minIndex) childrenCache[numCollisions++] = c;
                    }
                }

                if(numCollisions > 0)
                {
                    uint32_t maskIdx = nodeTriangles.size()/8;
                    uint8_t maskBit = 1 << (7 - (nodeTriangles.size() & 0b0111));
                    nodeTriangles.push_back(minIndex);
                    for(uint32_t c=0; c < numCollisions; c++)
                    {
                        chIndices[childrenCache[c]]++;
                        outputTrianglesMaskCache[childrenCache[c]][maskIdx] |= maskBit;
                    }
                }
            } while(numCollisions > 0);

            for(uint32_t c=0; c < 8; c++)
            {
                uint32_t arrayStartIndex = mTrianglesMasks.size();
                const uint32_t numTriangles = nodeTriangles.size();
                const uint32_t numBytes = (numTriangles + 7) / 8;
                mTrianglesMasks.resize(mTrianglesMasks.size() + numBytes);

                octreeNodesChildren[c].trianglesArrayIndex = arrayStartIndex;

                std::memcpy(mTrianglesMasks.data() + arrayStartIndex, outputTrianglesMaskCache[c].data(), numBytes);

                outputTrianglesMaskCache[c].clear();
            }

            if(node.depth == bitEncodingStartDepth)
            {
                // uint32_t arrayStartIndex = mTrianglesSets.size();
                // const uint32_t numTriangles = nodeTriangles.size();
                // mTrianglesSets.resize(mTrianglesSets.size() + numTriangles + 1);

                // octreeNode->trianglesArrayIndex = arrayStartIndex;

                // mTrianglesSets[arrayStartIndex++] = numTriangles;
                // std::memcpy(mTrianglesSets.data() + arrayStartIndex, nodeTriangles.data(), sizeof(uint32_t) * numTriangles);

                uint32_t arrayStartIndex = mTrianglesSets.size();
                const uint32_t numTriangles = nodeTriangles.size();
                const uint32_t arraySize = (numTriangles * bitsPerIndex + 31)/32;
                mTrianglesSets.resize(mTrianglesSets.size() + arraySize + 1);

                octreeNode->trianglesArrayIndex = arrayStartIndex;

                mTrianglesSets[arrayStartIndex++] = numTriangles;
                uint32_t bIdx = 0;
                for(uint32_t t=0; t < numTriangles; t++, bIdx += bitsPerIndex)
                {
                    const uint32_t index = nodeTriangles[t];
                    uint32_t idx = bIdx >> 5;
                    uint32_t bit = bIdx & 0b0011111;
                    mTrianglesSets[arrayStartIndex + idx] |= (index << invBitsPerIndex) >> bit;
                    mTrianglesSets[arrayStartIndex + idx + 1] |= (static_cast<uint64_t>(index) << (64 - (bit + bitsPerIndex)));
                }

                mMaxTrianglesEncodedInLeafs = glm::max(mMaxTrianglesEncodedInLeafs, numTriangles);
            }

            continue;
        }

        std::array<float, InterpolationMethod::NUM_COEFFICIENTS> interpolationCoeff;

        const std::vector<uint32_t>& parentTriangles = *trianglesCache[rDepth - 1];
        std::vector<uint32_t>& nodeTriangles = triangles[rDepth][node.childIndex];
        trianglesInfluence.filterTriangles(node.center, node.size, parentTriangles, 
                                            nodeTriangles, node.verticesValues, node.verticesInfo,
                                            mesh, trianglesData);

        nodes.top().trianglesProcessed = true;

        bool isTerminalNode = false;
        if(node.depth >= startDepth)
        {
            isTerminalNode = nodeTriangles.size() <= minTrianglesPerNode;
            // const float newNumTriangles = static_cast<float>(triangles[rDepth].size());
            // const float oldNumTriangles = static_cast<float>(triangles[rDepth-1].size());
            // const float oldoldNumTriangles = (rDepth > 1) ? static_cast<float>(triangles[rDepth-2].size()) : oldNumTriangles;
            // isTerminalNode = (0.7f * (1.0f - newNumTriangles / oldNumTriangles) + 0.3f * (1.0f - oldNumTriangles / oldoldNumTriangles)) * (newNumTriangles-32) < static_cast<float>(minTrianglesPerNode);
        }
    

        if(!isTerminalNode && node.depth < maxDepth)
        {
            if(node.depth < bitEncodingStartDepth) nodes.pop();

            std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, 19> midPointsValues;
            std::array<TrianglesInfluenceStrategy::VertexInfo, 19> pointsInfo;

            trianglesInfluence.calculateVerticesInfo(node.center, node.size, nodeTriangles, nodeSamplePoints,
                                                     0u, interpolationCoeff,
                                                     midPointsValues, pointsInfo,
                                                     mesh, trianglesData);

            trianglesCache[rDepth] = &nodeTriangles;

            // Generate new childrens
            const float newSize = 0.5f * node.size;

            uint32_t childIndex = (node.depth >= startDepth) ? mOctreeData.size() : std::numeric_limits<uint32_t>::max();
            uint32_t childOffsetMask = (node.depth >= startDepth) ? ~0 : 0;
            if(octreeNode != nullptr) octreeNode->setValues(false, childIndex);

            if(node.depth >= startDepth) mOctreeData.resize(mOctreeData.size() + 8);

            // Low Z children
            nodes.push(NodeInfo(0, childIndex, node.depth + 1, node.center + glm::vec3(-newSize, -newSize, -newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.verticesValues[0] = node.verticesValues[0]; child.verticesValues[1] = midPointsValues[0]; 
                child.verticesValues[2] = midPointsValues[1]; child.verticesValues[3] = midPointsValues[2];
                child.verticesValues[4] = midPointsValues[5]; child.verticesValues[5] = midPointsValues[6];
                child.verticesValues[6] = midPointsValues[8]; child.verticesValues[7] = midPointsValues[9];

                child.verticesInfo[0] = node.verticesInfo[0]; child.verticesInfo[1] = pointsInfo[0]; 
                child.verticesInfo[2] = pointsInfo[1]; child.verticesInfo[3] = pointsInfo[2];
                child.verticesInfo[4] = pointsInfo[5]; child.verticesInfo[5] = pointsInfo[6];
                child.verticesInfo[6] = pointsInfo[8]; child.verticesInfo[7] = pointsInfo[9];
            }

            nodes.push(NodeInfo(1, childIndex + (childOffsetMask & 1), node.depth + 1, node.center + glm::vec3(newSize, -newSize, -newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.verticesValues[0] = midPointsValues[0]; child.verticesValues[1] = node.verticesValues[1];
                child.verticesValues[2] = midPointsValues[2]; child.verticesValues[3] = midPointsValues[3];
                child.verticesValues[4] = midPointsValues[6]; child.verticesValues[5] = midPointsValues[7];
                child.verticesValues[6] = midPointsValues[9]; child.verticesValues[7] = midPointsValues[10];

                child.verticesInfo[0] = pointsInfo[0]; child.verticesInfo[1] = node.verticesInfo[1];
                child.verticesInfo[2] = pointsInfo[2]; child.verticesInfo[3] = pointsInfo[3];
                child.verticesInfo[4] = pointsInfo[6]; child.verticesInfo[5] = pointsInfo[7];
                child.verticesInfo[6] = pointsInfo[9]; child.verticesInfo[7] = pointsInfo[10];
            }

            nodes.push(NodeInfo(2, childIndex + (childOffsetMask & 2), node.depth + 1, node.center + glm::vec3(-newSize, newSize, -newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.verticesValues[0] = midPointsValues[1]; child.verticesValues[1] = midPointsValues[2];
                child.verticesValues[2] = node.verticesValues[2]; child.verticesValues[3] = midPointsValues[4];
                child.verticesValues[4] = midPointsValues[8]; child.verticesValues[5] = midPointsValues[9];
                child.verticesValues[6] = midPointsValues[11]; child.verticesValues[7] = midPointsValues[12];

                child.verticesInfo[0] = pointsInfo[1]; child.verticesInfo[1] = pointsInfo[2];
                child.verticesInfo[2] = node.verticesInfo[2]; child.verticesInfo[3] = pointsInfo[4];
                child.verticesInfo[4] = pointsInfo[8]; child.verticesInfo[5] = pointsInfo[9];
                child.verticesInfo[6] = pointsInfo[11]; child.verticesInfo[7] = pointsInfo[12];
            }

            nodes.push(NodeInfo(3, childIndex + (childOffsetMask & 3), node.depth + 1, node.center + glm::vec3(newSize, newSize, -newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.verticesValues[0] = midPointsValues[2]; child.verticesValues[1] = midPointsValues[3];
                child.verticesValues[2] = midPointsValues[4]; child.verticesValues[3] = node.verticesValues[3];
                child.verticesValues[4] = midPointsValues[9]; child.verticesValues[5] = midPointsValues[10];
                child.verticesValues[6] = midPointsValues[12]; child.verticesValues[7] = midPointsValues[13];

                child.verticesInfo[0] = pointsInfo[2]; child.verticesInfo[1] = pointsInfo[3];
                child.verticesInfo[2] = pointsInfo[4]; child.verticesInfo[3] = node.verticesInfo[3];
                child.verticesInfo[4] = pointsInfo[9]; child.verticesInfo[5] = pointsInfo[10];
                child.verticesInfo[6] = pointsInfo[12]; child.verticesInfo[7] = pointsInfo[13];
            }

            // High Z children
            nodes.push(NodeInfo(4, childIndex + (childOffsetMask & 4), node.depth + 1, node.center + glm::vec3(-newSize, -newSize, newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.verticesValues[0] = midPointsValues[5]; child.verticesValues[1] = midPointsValues[6];
                child.verticesValues[2] = midPointsValues[8]; child.verticesValues[3] = midPointsValues[9];
                child.verticesValues[4] = node.verticesValues[4]; child.verticesValues[5] = midPointsValues[14];
                child.verticesValues[6] = midPointsValues[15]; child.verticesValues[7] = midPointsValues[16];

                child.verticesInfo[0] = pointsInfo[5]; child.verticesInfo[1] = pointsInfo[6];
                child.verticesInfo[2] = pointsInfo[8]; child.verticesInfo[3] = pointsInfo[9];
                child.verticesInfo[4] = node.verticesInfo[4]; child.verticesInfo[5] = pointsInfo[14];
                child.verticesInfo[6] = pointsInfo[15]; child.verticesInfo[7] = pointsInfo[16];
            }

            nodes.push(NodeInfo(5, childIndex + (childOffsetMask & 5), node.depth + 1, node.center + glm::vec3(newSize, -newSize, newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.verticesValues[0] = midPointsValues[6]; child.verticesValues[1] = midPointsValues[7];
                child.verticesValues[2] = midPointsValues[9]; child.verticesValues[3] = midPointsValues[10];
                child.verticesValues[4] = midPointsValues[14]; child.verticesValues[5] = node.verticesValues[5];
                child.verticesValues[6] = midPointsValues[16]; child.verticesValues[7] = midPointsValues[17];

                child.verticesInfo[0] = pointsInfo[6]; child.verticesInfo[1] = pointsInfo[7];
                child.verticesInfo[2] = pointsInfo[9]; child.verticesInfo[3] = pointsInfo[10];
                child.verticesInfo[4] = pointsInfo[14]; child.verticesInfo[5] = node.verticesInfo[5];
                child.verticesInfo[6] = pointsInfo[16]; child.verticesInfo[7] = pointsInfo[17];
            }

            nodes.push(NodeInfo(6, childIndex + (childOffsetMask & 6), node.depth + 1, node.center + glm::vec3(-newSize, newSize, newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.verticesValues[0] = midPointsValues[8]; child.verticesValues[1] = midPointsValues[9];
                child.verticesValues[2] = midPointsValues[11]; child.verticesValues[3] = midPointsValues[12];
                child.verticesValues[4] = midPointsValues[15]; child.verticesValues[5] = midPointsValues[16];
                child.verticesValues[6] = node.verticesValues[6]; child.verticesValues[7] = midPointsValues[18];

                child.verticesInfo[0] = pointsInfo[8]; child.verticesInfo[1] = pointsInfo[9];
                child.verticesInfo[2] = pointsInfo[11]; child.verticesInfo[3] = pointsInfo[12];
                child.verticesInfo[4] = pointsInfo[15]; child.verticesInfo[5] = pointsInfo[16];
                child.verticesInfo[6] = node.verticesInfo[6]; child.verticesInfo[7] = pointsInfo[18];
            }

            nodes.push(NodeInfo(7, childIndex + (childOffsetMask & 7), node.depth + 1, node.center + glm::vec3(newSize, newSize, newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.verticesValues[0] = midPointsValues[9]; child.verticesValues[1] = midPointsValues[10];
                child.verticesValues[2] = midPointsValues[12]; child.verticesValues[3] = midPointsValues[13];
                child.verticesValues[4] = midPointsValues[16]; child.verticesValues[5] = midPointsValues[17];
                child.verticesValues[6] = midPointsValues[18]; child.verticesValues[7] = node.verticesValues[7];

                child.verticesInfo[0] = pointsInfo[9]; child.verticesInfo[1] = pointsInfo[10];
                child.verticesInfo[2] = pointsInfo[12]; child.verticesInfo[3] = pointsInfo[13];
                child.verticesInfo[4] = pointsInfo[16]; child.verticesInfo[5] = pointsInfo[17];
                child.verticesInfo[6] = pointsInfo[18]; child.verticesInfo[7] = node.verticesInfo[7];
            }
        }
        else
        {
            assert(node.depth >= startDepth);
            octreeNode->setValues(true, std::numeric_limits<uint32_t>::max());
            nodes.pop();

            if(node.depth <= bitEncodingStartDepth)
            {
                // uint32_t arrayStartIndex = mTrianglesSets.size();
                // const uint32_t numTriangles = nodeTriangles.size();
                // mTrianglesSets.resize(mTrianglesSets.size() + numTriangles + 1);

                // octreeNode->trianglesArrayIndex = arrayStartIndex;

                // mTrianglesSets[arrayStartIndex++] = numTriangles;
                // std::memcpy(mTrianglesSets.data() + arrayStartIndex, nodeTriangles.data(), sizeof(uint32_t) * numTriangles);

                uint32_t arrayStartIndex = mTrianglesSets.size();
                const uint32_t numTriangles = nodeTriangles.size();
                const uint32_t arraySize = (numTriangles * bitsPerIndex + 31)/32;
                mTrianglesSets.resize(mTrianglesSets.size() + arraySize + 1);

                octreeNode->trianglesArrayIndex = arrayStartIndex;

                mTrianglesSets[arrayStartIndex++] = numTriangles;
                uint32_t bIdx = 0;
                for(uint32_t t=0; t < numTriangles; t++, bIdx += bitsPerIndex)
                {
                    const uint32_t index = nodeTriangles[t];
                    uint32_t idx = bIdx >> 5;
                    uint32_t bit = bIdx & 0b0011111;
                    mTrianglesSets[arrayStartIndex + idx] |= (index << invBitsPerIndex) >> bit;
                    mTrianglesSets[arrayStartIndex + idx + 1] |= (static_cast<uint64_t>(index) << (64 - (bit + bitsPerIndex)));
                }
            }

            #ifdef PRINT_STATISTICS
                const uint64_t numNodes = 1 << (3 * (maxDepth - node.depth));
                numTrianglesInLeafs += nodeTriangles.size() * numNodes;
                numLeafs += numNodes;
                // numTrianglesInLeafs += nodeTriangles.size();
                // numLeafs += 1;
                endedNodes[node.depth]++;
            #endif
            mMaxTrianglesInLeafs = glm::max(mMaxTrianglesInLeafs, static_cast<uint32_t>(nodeTriangles.size()));
        }

#ifdef PRINT_STATISTICS
        {
            verticesStatistics[node.depth].first += nodeTriangles.size();
            verticesStatistics[node.depth].second += 1;
            elapsedTime[node.depth] += timer.getElapsedSeconds();
            numTrianglesEvaluated[node.depth] += parentTriangles.size();
        }
#endif
    }
	
#ifdef PRINT_STATISTICS
    SPDLOG_INFO("Used an octree of max depth {}", maxDepth);
    for(uint32_t d=0; d < maxDepth+1; d++)
    {
        const float mean = static_cast<float>(verticesStatistics[d].first) / 
                           static_cast<float>(glm::max(1u, verticesStatistics[d].second));
        SPDLOG_INFO("Depth {}, mean of triangles per node: {} [{}s]", d, mean, elapsedTime[d]);
        if(numTrianglesEvaluated[d] < 1000)
        {
            SPDLOG_INFO("Depth {}, number of evaluations: {}", d, numTrianglesEvaluated[d]);
        }
        else if(numTrianglesEvaluated[d] < 1000000)
        {
            SPDLOG_INFO("Depth {}, number of evaluations: {:.3f}K", d, static_cast<float>(numTrianglesEvaluated[d]) * 1e-3);
        }
        else
        {
            SPDLOG_INFO("Depth {}, number of evaluations: {:.3f}M", d, static_cast<float>(numTrianglesEvaluated[d]) * 1e-6);
        }
        //SPDLOG_INFO("Depth {}, ended nodes: {}%", d, 100.0f * static_cast<float>(endedNodes[d]) / static_cast<float>(1 << (3 * (d-1))));
        SPDLOG_INFO("Depth {}, ended nodes: {}", d, endedNodes[d]);
    }

    SPDLOG_INFO("Mean triangles in leaves: {}", static_cast<float>(numTrianglesInLeafs) / static_cast<float>(numLeafs));
    SPDLOG_INFO("Maximum triangles in a leaf: {}", mMaxTrianglesInLeafs);

    trianglesInfluence.printStatistics();
#endif
}

#endif