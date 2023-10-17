#ifndef EXACT_OCTREE_SDF_DEPTH_FIRST_H
#define EXACT_OCTREE_SDF_DEPTH_FIRST_H

#include <string>
#include <stack>
#ifdef OPENMP_AVAILABLE
#include <omp.h>
#endif

namespace sdflib
{
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
                                uint32_t minTrianglesPerNode, uint32_t numThreads)
{
    typedef typename TrianglesInfluenceStrategy::InterpolationMethod InterpolationMethod;
    typedef DepthFirstNodeInfoExactOctree<typename TrianglesInfluenceStrategy::VertexInfo, InterpolationMethod::VALUES_PER_VERTEX> NodeInfo;

    struct ThreadContext
    {
        std::vector<std::array<std::vector<uint32_t>, 8>> triangles;
        std::vector<std::vector<uint32_t>*> trianglesCache;
        std::array<std::vector<uint32_t>, 8> outputTrianglesCache;
        std::array<std::vector<uint8_t>, 8> outputTrianglesMaskCache;
        TrianglesInfluenceStrategy trianglesInfluence;
        std::stack<NodeInfo> nodesStack;
        uint32_t startDepth;
        uint32_t startOctreeDepth;
        uint32_t maxDepth;
        uint32_t bitEncodingStartDepth;
        uint32_t bitsPerIndex;
        uint32_t minTrianglesPerNode;
        uint32_t maxTrianglesInLeafs;
        uint32_t maxTrianglesEncodedInLeafs;
        uint32_t padding[16];

#ifdef SDFLIB_PRINT_STATISTICS
        std::vector<std::pair<uint32_t, uint32_t>> verticesStatistics;
        std::vector<uint32_t> endedNodes;
        std::vector<float> elapsedTime;
        std::vector<uint32_t> numTrianglesEvaluated;
        uint64_t numTrianglesInLeafs;
        uint64_t numLeafs;
        Timer timer;
#endif
    };

    mMinTrianglesInLeafs = minTrianglesPerNode;

    std::vector<TriangleUtils::TriangleData>& trianglesData = mTrianglesData;
    const uint32_t startOctreeDepth = glm::min(startDepth, START_OCTREE_DEPTH);

    uint32_t bitEncodingStartDepth = maxDepth - BIT_ENCODING_DEPTH;
    mBitEncodingStartDepth = bitEncodingStartDepth;

    uint32_t bitsPerIndex = static_cast<int32_t>(glm::ceil(glm::log2(static_cast<float>(trianglesData.size()))));
    mBitsPerIndex = bitsPerIndex;

    ThreadContext mainThread;
    mainThread.triangles.resize(maxDepth - startOctreeDepth + 2);
    mainThread.trianglesCache.resize(maxDepth - startOctreeDepth + 2);
    mainThread.outputTrianglesCache.fill(std::vector<uint32_t>());
    mainThread.outputTrianglesMaskCache.fill(std::vector<uint8_t>());
    mainThread.trianglesInfluence.initCaches(mBox, maxDepth);
    mainThread.startDepth = startDepth;
    mainThread.startOctreeDepth = startOctreeDepth;
    mainThread.maxDepth = maxDepth;
    mainThread.bitEncodingStartDepth = bitEncodingStartDepth;
    mainThread.bitsPerIndex = bitsPerIndex;
    mainThread.minTrianglesPerNode = minTrianglesPerNode;
    mainThread.maxTrianglesInLeafs = 0;
    mainThread.maxTrianglesEncodedInLeafs = 0;

#ifdef SDFLIB_PRINT_STATISTICS
    mainThread.verticesStatistics.resize(maxDepth + 1, std::make_pair(0, 0));
    mainThread.verticesStatistics[0] = std::make_pair(trianglesData.size(), 1);
    mainThread.endedNodes.resize(maxDepth + 1, 0);
    mainThread.elapsedTime.resize(maxDepth + 1, 0.0f);
    mainThread.numTrianglesEvaluated.resize(maxDepth + 1, 0);
    mainThread.numTrianglesInLeafs = 0;
    mainThread.numLeafs = 0;
#endif

    const uint32_t numTriangles = trianglesData.size();
    mainThread.triangles[0].fill(std::vector<uint32_t>());
	mainThread.triangles[0][0].resize(numTriangles);
    {
        uint32_t triIndex = 0;
        for(uint32_t t=0; t < numTriangles; t++)
        {
            if(glm::dot(trianglesData[t].getTriangleNormal(), trianglesData[t].getTriangleNormal()) > 1e-3f)
                mainThread.triangles[0][0][triIndex++] = t;
        }
        
        mainThread.triangles[0][0].resize(triIndex);
    }

    mainThread.trianglesCache[0] = &mainThread.triangles[0][0];

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
    
    {
        std::stack<NodeInfo>& nodes = mainThread.nodesStack;
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
                    mainThread.trianglesInfluence.calculateVerticesInfo(n.center, n.size, mainThread.triangles[0][0], childrens,
                                                                        0u, nullArray,
                                                                        n.verticesValues, n.verticesInfo,
                                                                        mesh, trianglesData);
                }
            }
        }
    }

    mMaxTrianglesInLeafs = 0;
    mMaxTrianglesEncodedInLeafs = 0;

    auto processNode = [&mesh, &trianglesData] (const NodeInfo& node, ThreadContext& tContext, 
                                                std::vector<OctreeNode>& outputOctree,
                                                std::vector<uint32_t>& outputTrianglesSets,
                                                std::vector<uint8_t>& outputTrianglesMasks)
    {
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

        #ifdef SDFLIB_PRINT_STATISTICS
            tContext.timer.start();
        #endif

        OctreeNode* octreeNode = (node.nodeIndex < std::numeric_limits<uint32_t>::max()) 
                                    ? &outputOctree[node.nodeIndex]
                                    : nullptr;

        const uint32_t rDepth = node.depth - tContext.startOctreeDepth + 1;

        if(node.trianglesProcessed)
        {
            tContext.nodesStack.pop();

            OctreeNode* octreeNodesChildren = &outputOctree[octreeNode->getChildrenIndex()];

            std::vector<uint32_t>& nodeTriangles = tContext.triangles[rDepth][node.childIndex];
            std::vector<uint32_t> oldTriangles = nodeTriangles;

            for(uint32_t c=0; c < 8; c++)
            {
                tContext.outputTrianglesMaskCache[c].resize((nodeTriangles.size() + 7) / 8, 0);
            }

            nodeTriangles.clear();
            const std::array<std::vector<uint32_t>, 8> chTriangles = tContext.triangles[rDepth + 1];
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
                        tContext.outputTrianglesMaskCache[childrenCache[c]][maskIdx] |= maskBit;
                    }
                }
            } while(numCollisions > 0);

            for(uint32_t c=0; c < 8; c++)
            {
                uint32_t arrayStartIndex = outputTrianglesMasks.size();
                const uint32_t numTriangles = nodeTriangles.size();
                const uint32_t numBytes = (numTriangles + 7) / 8;
                outputTrianglesMasks.resize(outputTrianglesMasks.size() + numBytes);

                octreeNodesChildren[c].trianglesArrayIndex = arrayStartIndex;

                std::memcpy(outputTrianglesMasks.data() + arrayStartIndex, tContext.outputTrianglesMaskCache[c].data(), numBytes);

                tContext.outputTrianglesMaskCache[c].clear();
            }

            if(node.depth == tContext.bitEncodingStartDepth)
            {
                uint32_t arrayStartIndex = outputTrianglesSets.size();
                const uint32_t numTriangles = nodeTriangles.size();
                const uint32_t arraySize = (numTriangles * tContext.bitsPerIndex + 31)/32;
                outputTrianglesSets.resize(outputTrianglesSets.size() + arraySize + 2);

                octreeNode->trianglesArrayIndex = arrayStartIndex;

                const uint32_t invBitsPerIndex = 32 - tContext.bitsPerIndex;
                outputTrianglesSets[arrayStartIndex++] = numTriangles;
                uint32_t bIdx = 0;
                for(uint32_t t=0; t < numTriangles; t++, bIdx += tContext.bitsPerIndex)
                {
                    const uint32_t index = nodeTriangles[t];
                    uint32_t idx = bIdx >> 5;
                    uint32_t bit = bIdx & 0b0011111;
                    outputTrianglesSets[arrayStartIndex + idx] |= (index << invBitsPerIndex) >> bit;
                    outputTrianglesSets[arrayStartIndex + idx + 1] |= (static_cast<uint64_t>(index) << (64 - (bit + tContext.bitsPerIndex)));
                }

                tContext.maxTrianglesEncodedInLeafs = glm::max(tContext.maxTrianglesEncodedInLeafs, numTriangles);
            }

            return;
        }

        std::array<float, InterpolationMethod::NUM_COEFFICIENTS> interpolationCoeff;

        const std::vector<uint32_t>& parentTriangles = *tContext.trianglesCache[rDepth - 1];
        std::vector<uint32_t>& nodeTriangles = tContext.triangles[rDepth][node.childIndex];
        tContext.trianglesInfluence.filterTriangles(node.center, node.size, parentTriangles, 
                                            nodeTriangles, node.verticesValues, node.verticesInfo,
                                            mesh, trianglesData);

        tContext.nodesStack.top().trianglesProcessed = true;

        bool isTerminalNode = false;
        if(node.depth >= tContext.startDepth)
        {
            isTerminalNode = nodeTriangles.size() <= tContext.minTrianglesPerNode;
        }
    

        if(!isTerminalNode && node.depth < tContext.maxDepth)
        {
            if(node.depth < tContext.bitEncodingStartDepth) tContext.nodesStack.pop();

            std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, 19> midPointsValues;
            std::array<typename TrianglesInfluenceStrategy::VertexInfo, 19> pointsInfo;

            tContext.trianglesInfluence.calculateVerticesInfo(node.center, node.size, nodeTriangles, nodeSamplePoints,
                                                     0u, interpolationCoeff,
                                                     midPointsValues, pointsInfo,
                                                     mesh, trianglesData);

            tContext.trianglesCache[rDepth] = &nodeTriangles;

            // Generate new childrens
            const float newSize = 0.5f * node.size;

            uint32_t childIndex = (node.depth >= tContext.startDepth) ? outputOctree.size() : std::numeric_limits<uint32_t>::max();
            uint32_t childOffsetMask = (node.depth >= tContext.startDepth) ? ~0 : 0;
            if(octreeNode != nullptr) octreeNode->setValues(false, childIndex);

            if(node.depth >= tContext.startDepth) outputOctree.resize(outputOctree.size() + 8);

            // Low Z children
            tContext.nodesStack.push(NodeInfo(0, childIndex, node.depth + 1, node.center + glm::vec3(-newSize, -newSize, -newSize), newSize));
            {
                NodeInfo& child = tContext.nodesStack.top();
                child.verticesValues[0] = node.verticesValues[0]; child.verticesValues[1] = midPointsValues[0]; 
                child.verticesValues[2] = midPointsValues[1]; child.verticesValues[3] = midPointsValues[2];
                child.verticesValues[4] = midPointsValues[5]; child.verticesValues[5] = midPointsValues[6];
                child.verticesValues[6] = midPointsValues[8]; child.verticesValues[7] = midPointsValues[9];

                child.verticesInfo[0] = node.verticesInfo[0]; child.verticesInfo[1] = pointsInfo[0]; 
                child.verticesInfo[2] = pointsInfo[1]; child.verticesInfo[3] = pointsInfo[2];
                child.verticesInfo[4] = pointsInfo[5]; child.verticesInfo[5] = pointsInfo[6];
                child.verticesInfo[6] = pointsInfo[8]; child.verticesInfo[7] = pointsInfo[9];
            }

            tContext.nodesStack.push(NodeInfo(1, childIndex + (childOffsetMask & 1), node.depth + 1, node.center + glm::vec3(newSize, -newSize, -newSize), newSize));
            {
                NodeInfo& child = tContext.nodesStack.top();
                child.verticesValues[0] = midPointsValues[0]; child.verticesValues[1] = node.verticesValues[1];
                child.verticesValues[2] = midPointsValues[2]; child.verticesValues[3] = midPointsValues[3];
                child.verticesValues[4] = midPointsValues[6]; child.verticesValues[5] = midPointsValues[7];
                child.verticesValues[6] = midPointsValues[9]; child.verticesValues[7] = midPointsValues[10];

                child.verticesInfo[0] = pointsInfo[0]; child.verticesInfo[1] = node.verticesInfo[1];
                child.verticesInfo[2] = pointsInfo[2]; child.verticesInfo[3] = pointsInfo[3];
                child.verticesInfo[4] = pointsInfo[6]; child.verticesInfo[5] = pointsInfo[7];
                child.verticesInfo[6] = pointsInfo[9]; child.verticesInfo[7] = pointsInfo[10];
            }

            tContext.nodesStack.push(NodeInfo(2, childIndex + (childOffsetMask & 2), node.depth + 1, node.center + glm::vec3(-newSize, newSize, -newSize), newSize));
            {
                NodeInfo& child = tContext.nodesStack.top();
                child.verticesValues[0] = midPointsValues[1]; child.verticesValues[1] = midPointsValues[2];
                child.verticesValues[2] = node.verticesValues[2]; child.verticesValues[3] = midPointsValues[4];
                child.verticesValues[4] = midPointsValues[8]; child.verticesValues[5] = midPointsValues[9];
                child.verticesValues[6] = midPointsValues[11]; child.verticesValues[7] = midPointsValues[12];

                child.verticesInfo[0] = pointsInfo[1]; child.verticesInfo[1] = pointsInfo[2];
                child.verticesInfo[2] = node.verticesInfo[2]; child.verticesInfo[3] = pointsInfo[4];
                child.verticesInfo[4] = pointsInfo[8]; child.verticesInfo[5] = pointsInfo[9];
                child.verticesInfo[6] = pointsInfo[11]; child.verticesInfo[7] = pointsInfo[12];
            }

            tContext.nodesStack.push(NodeInfo(3, childIndex + (childOffsetMask & 3), node.depth + 1, node.center + glm::vec3(newSize, newSize, -newSize), newSize));
            {
                NodeInfo& child = tContext.nodesStack.top();
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
            tContext.nodesStack.push(NodeInfo(4, childIndex + (childOffsetMask & 4), node.depth + 1, node.center + glm::vec3(-newSize, -newSize, newSize), newSize));
            {
                NodeInfo& child = tContext.nodesStack.top();
                child.verticesValues[0] = midPointsValues[5]; child.verticesValues[1] = midPointsValues[6];
                child.verticesValues[2] = midPointsValues[8]; child.verticesValues[3] = midPointsValues[9];
                child.verticesValues[4] = node.verticesValues[4]; child.verticesValues[5] = midPointsValues[14];
                child.verticesValues[6] = midPointsValues[15]; child.verticesValues[7] = midPointsValues[16];

                child.verticesInfo[0] = pointsInfo[5]; child.verticesInfo[1] = pointsInfo[6];
                child.verticesInfo[2] = pointsInfo[8]; child.verticesInfo[3] = pointsInfo[9];
                child.verticesInfo[4] = node.verticesInfo[4]; child.verticesInfo[5] = pointsInfo[14];
                child.verticesInfo[6] = pointsInfo[15]; child.verticesInfo[7] = pointsInfo[16];
            }

            tContext.nodesStack.push(NodeInfo(5, childIndex + (childOffsetMask & 5), node.depth + 1, node.center + glm::vec3(newSize, -newSize, newSize), newSize));
            {
                NodeInfo& child = tContext.nodesStack.top();
                child.verticesValues[0] = midPointsValues[6]; child.verticesValues[1] = midPointsValues[7];
                child.verticesValues[2] = midPointsValues[9]; child.verticesValues[3] = midPointsValues[10];
                child.verticesValues[4] = midPointsValues[14]; child.verticesValues[5] = node.verticesValues[5];
                child.verticesValues[6] = midPointsValues[16]; child.verticesValues[7] = midPointsValues[17];

                child.verticesInfo[0] = pointsInfo[6]; child.verticesInfo[1] = pointsInfo[7];
                child.verticesInfo[2] = pointsInfo[9]; child.verticesInfo[3] = pointsInfo[10];
                child.verticesInfo[4] = pointsInfo[14]; child.verticesInfo[5] = node.verticesInfo[5];
                child.verticesInfo[6] = pointsInfo[16]; child.verticesInfo[7] = pointsInfo[17];
            }

            tContext.nodesStack.push(NodeInfo(6, childIndex + (childOffsetMask & 6), node.depth + 1, node.center + glm::vec3(-newSize, newSize, newSize), newSize));
            {
                NodeInfo& child = tContext.nodesStack.top();
                child.verticesValues[0] = midPointsValues[8]; child.verticesValues[1] = midPointsValues[9];
                child.verticesValues[2] = midPointsValues[11]; child.verticesValues[3] = midPointsValues[12];
                child.verticesValues[4] = midPointsValues[15]; child.verticesValues[5] = midPointsValues[16];
                child.verticesValues[6] = node.verticesValues[6]; child.verticesValues[7] = midPointsValues[18];

                child.verticesInfo[0] = pointsInfo[8]; child.verticesInfo[1] = pointsInfo[9];
                child.verticesInfo[2] = pointsInfo[11]; child.verticesInfo[3] = pointsInfo[12];
                child.verticesInfo[4] = pointsInfo[15]; child.verticesInfo[5] = pointsInfo[16];
                child.verticesInfo[6] = node.verticesInfo[6]; child.verticesInfo[7] = pointsInfo[18];
            }

            tContext.nodesStack.push(NodeInfo(7, childIndex + (childOffsetMask & 7), node.depth + 1, node.center + glm::vec3(newSize, newSize, newSize), newSize));
            {
                NodeInfo& child = tContext.nodesStack.top();
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
            assert(node.depth >= tContext.startDepth);
            octreeNode->setValues(true, std::numeric_limits<uint32_t>::max());
            tContext.nodesStack.pop();

            if(node.depth <= tContext.bitEncodingStartDepth)
            {
                uint32_t arrayStartIndex = outputTrianglesSets.size();
                const uint32_t numTriangles = nodeTriangles.size();
                const uint32_t arraySize = (numTriangles * tContext.bitsPerIndex + 31)/32;
                outputTrianglesSets.resize(outputTrianglesSets.size() + arraySize + 2);

                octreeNode->trianglesArrayIndex = arrayStartIndex;

                const uint32_t invBitsPerIndex = 32 - tContext.bitsPerIndex;
                outputTrianglesSets[arrayStartIndex++] = numTriangles;
                uint32_t bIdx = 0;
                for(uint32_t t=0; t < numTriangles; t++, bIdx += tContext.bitsPerIndex)
                {
                    const uint32_t index = nodeTriangles[t];
                    uint32_t idx = bIdx >> 5;
                    uint32_t bit = bIdx & 0b0011111;
                    outputTrianglesSets[arrayStartIndex + idx] |= (index << invBitsPerIndex) >> bit;
                    outputTrianglesSets[arrayStartIndex + idx + 1] |= (static_cast<uint64_t>(index) << (64 - (bit + tContext.bitsPerIndex)));
                }
            }

            #ifdef SDFLIB_PRINT_STATISTICS
                const uint64_t numNodes = 1 << (3 * (tContext.maxDepth - node.depth));
                tContext.numTrianglesInLeafs += nodeTriangles.size() * numNodes;
                tContext.numLeafs += numNodes;
                // numTrianglesInLeafs += nodeTriangles.size();
                // numLeafs += 1;
                tContext.endedNodes[node.depth]++;
            #endif
            tContext.maxTrianglesInLeafs = glm::max(tContext.maxTrianglesInLeafs, static_cast<uint32_t>(nodeTriangles.size()));
        }

#ifdef SDFLIB_PRINT_STATISTICS
        {
            tContext.verticesStatistics[node.depth].first += nodeTriangles.size();
            tContext.verticesStatistics[node.depth].second += 1;
            tContext.elapsedTime[node.depth] += tContext.timer.getElapsedSeconds();
            tContext.numTrianglesEvaluated[node.depth] += parentTriangles.size();
        }
#endif

    };

    const uint32_t voxlesPerAxis = 1 << startDepth;
    #ifdef OPENMP_AVAILABLE
    if(numThreads < 2)
    #endif
    {
        // Create the grid
        mOctreeData.resize(voxlesPerAxis * voxlesPerAxis * voxlesPerAxis);

        while(!mainThread.nodesStack.empty())
        {
            NodeInfo node = mainThread.nodesStack.top();
            
            if(node.depth == startDepth)
            {
                glm::ivec3 startArrayPos = glm::floor((node.center - mBox.min) / mStartGridCellSize);
                node.nodeIndex = startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x;
            }

            processNode(node, mainThread, mOctreeData, mTrianglesSets, mTrianglesMasks);
        }

        mMaxTrianglesInLeafs = mainThread.maxTrianglesInLeafs;
        mMaxTrianglesEncodedInLeafs = mainThread.maxTrianglesEncodedInLeafs;
    }
    #ifdef OPENMP_AVAILABLE
    else
    {
        std::vector<ThreadContext> threadsContext(numThreads, mainThread);

        struct OctreeDataWithPadding
        {
            OctreeDataWithPadding() {}
            std::vector<OctreeNode> octreeData;
            std::vector<uint32_t> trianglesSets;
            std::vector<uint8_t> trianglesMasks;
            uint32_t padding[16];
        };
        std::vector<OctreeDataWithPadding> subOctrees(voxlesPerAxis * voxlesPerAxis * voxlesPerAxis);

        omp_set_dynamic(0);
        omp_set_num_threads(numThreads);

        #pragma omp parallel default(shared)
        #pragma omp single
        while(!mainThread.nodesStack.empty())
        {
            NodeInfo node = mainThread.nodesStack.top();
            
            if(node.depth == startDepth)
            {
                mainThread.nodesStack.pop();
                glm::ivec3 startArrayPos = glm::floor((node.center - mBox.min) / mStartGridCellSize);
                node.nodeIndex = 0;
                std::vector<OctreeNode>* subOctreePtr = &subOctrees[startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x].octreeData;
                subOctreePtr->resize(1);
                std::vector<uint32_t>* subTrianglesSetsPtr = &subOctrees[startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x].trianglesSets;
                std::vector<uint8_t>* subTrianglesMasksPtr = &subOctrees[startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x].trianglesMasks;
                const uint32_t rDepth = startDepth - mainThread.startOctreeDepth;
                std::vector<uint32_t> startTriangles = *mainThread.trianglesCache[rDepth];
                #pragma omp task shared(threadsContext) firstprivate(node, subOctreePtr, subTrianglesSetsPtr, subTrianglesMasksPtr, rDepth, startTriangles)
                {
                    std::vector<OctreeNode>& subOctree = *subOctreePtr;
                    std::vector<uint32_t>& subTrianglesSets = *subTrianglesSetsPtr;
                    std::vector<uint8_t>& subTrianglesMasks = *subTrianglesMasksPtr;
                    const uint32_t tId = omp_get_thread_num();
                    ThreadContext& threadContext = threadsContext[tId];
                    threadContext.triangles[rDepth][0] = std::move(startTriangles);
                    threadContext.trianglesCache[rDepth] = &threadContext.triangles[rDepth][0];
                    threadContext.nodesStack = std::stack<NodeInfo>(); // Reset stack
                    threadContext.nodesStack.push(node);
                    while(!threadContext.nodesStack.empty())
                    {
                        NodeInfo node1 = threadContext.nodesStack.top();

                        processNode(node1, threadContext, subOctree, subTrianglesSets, subTrianglesMasks);
                    }
                }
            }
            else
            {
                processNode(node, mainThread, mOctreeData, mTrianglesSets, mTrianglesMasks);
            }
        }

        // Merge all the subtrees
        mOctreeData.resize(voxlesPerAxis * voxlesPerAxis * voxlesPerAxis);
        for(uint32_t i=0; i < subOctrees.size(); i++)
        {
            std::vector<OctreeNode>& octreeData = subOctrees[i].octreeData;

            const uint32_t startIndex = mOctreeData.size();
            const uint32_t startTrianglesSetsIndex = mTrianglesSets.size();
            const uint32_t startTrianglesMasksIndex = mTrianglesMasks.size();
            
            // Add start index to the subtree
            std::function<void(OctreeNode&, uint32_t)> vistNode;
            vistNode = [&](OctreeNode& node, uint32_t depth)
            {
                // Iterate children
                if(!node.isLeaf())
                {
                    for(uint32_t i = 0; i < 8; i++)
                    {
                        vistNode(octreeData[node.getChildrenIndex() + i], depth + 1);
                    }

                    // Update node index
                    node.setValues(false, node.getChildrenIndex() + startIndex - 1);
                }

                if(depth > mainThread.bitEncodingStartDepth)
                {
                    node.trianglesArrayIndex += startTrianglesMasksIndex;
                }
                else if(node.isLeaf() || 
                        depth == mainThread.bitEncodingStartDepth)
                {
                    node.trianglesArrayIndex += startTrianglesSetsIndex;
                }                
            };

            vistNode(octreeData[0], mainThread.startOctreeDepth);

            // Move the fist node to the correct start grid position
            mOctreeData[i] = octreeData[0];

            // Copy to final array
            mOctreeData.insert(mOctreeData.end(), octreeData.begin() + 1, octreeData.end());
            mTrianglesSets.insert(mTrianglesSets.end(), subOctrees[i].trianglesSets.begin(), subOctrees[i].trianglesSets.end());
            mTrianglesMasks.insert(mTrianglesMasks.end(), subOctrees[i].trianglesMasks.begin(), subOctrees[i].trianglesMasks.end());
        }

        mMaxTrianglesEncodedInLeafs = 0.0f;
        mMaxTrianglesInLeafs = 0.0f;
        for(ThreadContext& tCtx : threadsContext)
        {
            mMaxTrianglesEncodedInLeafs = glm::max(mMaxTrianglesEncodedInLeafs, tCtx.maxTrianglesEncodedInLeafs);
            mMaxTrianglesInLeafs = glm::max(mMaxTrianglesInLeafs, tCtx.maxTrianglesInLeafs);
        }

        #ifdef SDFLIB_PRINT_STATISTICS
            for(ThreadContext& tCtx : threadsContext)
            {
                for(uint32_t d=0; d < maxDepth+1; d++)
                {
                    mainThread.verticesStatistics[d].first += tCtx.verticesStatistics[d].first;
                    mainThread.verticesStatistics[d].second += tCtx.verticesStatistics[d].second;

                    mainThread.elapsedTime[d] += tCtx.elapsedTime[d];

                    mainThread.numTrianglesEvaluated[d] += tCtx.numTrianglesEvaluated[d];
                    mainThread.endedNodes[d] += tCtx.endedNodes[d];
                }

                mainThread.numTrianglesInLeafs += tCtx.numTrianglesInLeafs;
                mainThread.numLeafs += tCtx.numLeafs;
            }
        #endif
    }
    #endif
	
#ifdef SDFLIB_PRINT_STATISTICS
    SPDLOG_INFO("Used an octree of max depth {}", maxDepth);
    for(uint32_t d=0; d < maxDepth+1; d++)
    {
        const float mean = static_cast<float>(mainThread.verticesStatistics[d].first) / 
                           static_cast<float>(glm::max(1u, mainThread.verticesStatistics[d].second));
        SPDLOG_INFO("Depth {}, mean of triangles per node: {} [{}s]", d, mean, mainThread.elapsedTime[d]);
        if(mainThread.numTrianglesEvaluated[d] < 1000)
        {
            SPDLOG_INFO("Depth {}, number of evaluations: {}", d, mainThread.numTrianglesEvaluated[d]);
        }
        else if(mainThread.numTrianglesEvaluated[d] < 1000000)
        {
            SPDLOG_INFO("Depth {}, number of evaluations: {:.3f}K", d, static_cast<float>(mainThread.numTrianglesEvaluated[d]) * 1e-3);
        }
        else
        {
            SPDLOG_INFO("Depth {}, number of evaluations: {:.3f}M", d, static_cast<float>(mainThread.numTrianglesEvaluated[d]) * 1e-6);
        }
        //SPDLOG_INFO("Depth {}, ended nodes: {}%", d, 100.0f * static_cast<float>(endedNodes[d]) / static_cast<float>(1 << (3 * (d-1))));
        SPDLOG_INFO("Depth {}, ended nodes: {}", d, mainThread.endedNodes[d]);
    }

    SPDLOG_INFO("Mean triangles in leaves: {}", static_cast<float>(mainThread.numTrianglesInLeafs) / static_cast<float>(mainThread.numLeafs));
    SPDLOG_INFO("Maximum triangles in a leaf: {}", mMaxTrianglesInLeafs);

    mainThread.trianglesInfluence.printStatistics();
#endif
}
}

#endif