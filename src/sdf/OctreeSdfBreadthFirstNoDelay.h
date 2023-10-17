#ifndef OCTREE_SDF_BREADTH_FIRST_NO_DELAY_H
#define OCTREE_SDF_BREADTH_FIRST_NO_DELAY_H

#include "SdfLib/OctreeSdf.h"
#include "SdfLib/utils/Timer.h"
#include "SdfLib/utils/GJK.h"
#include "SdfLib/OctreeSdfUtils.h"
#include <array>
#include <stack>
#ifdef OPENMP_AVAILABLE
#include <omp.h>
#endif

namespace sdflib
{
template<typename VertexInfo, int VALUES_PER_VERTEX, int NUM_COEFFICIENTS>
struct BreadthFirstNoDelayNodeInfo
{
		BreadthFirstNoDelayNodeInfo() {}
		BreadthFirstNoDelayNodeInfo(uint32_t parentChildrenIndex, uint8_t childIndices, glm::vec3 center, float size, bool isTerminalNode = false)
			: parentChildrenIndex(parentChildrenIndex), childIndices(childIndices), center(center), size(size), isTerminalNode(isTerminalNode), ignoreNode(false) {}
		
        uint64_t childIndices;
        uint32_t parentChildrenIndex;
		bool isTerminalNode;
        bool ignoreNode;

        std::array<uint8_t, 6> neighbourDepth;
		std::array<uint32_t, 6> neighbourIndices;

        glm::vec3 center;
		float size;        

		std::array<std::array<float, VALUES_PER_VERTEX>, 8> verticesValues;
        std::array<VertexInfo, 8> verticesInfo;

        // Temporal data
        std::array<float, NUM_COEFFICIENTS> interpolationCoeff;
        std::array<std::array<float, VALUES_PER_VERTEX>, 19> midPointsValues;
        std::array<VertexInfo, 19> midPointsInfo;

		std::vector<uint32_t>* parentTriangles;
		std::vector<uint32_t> triangles;
};

// inline void getNeighboursVector(uint32_t outChildId, uint32_t childId, uint32_t parentChildrenIndex, const std::array<uint32_t, 6>& parentNeighbours, std::array<uint32_t, 6>& outNeighbours)
// {
//     for (uint32_t n = 1; n <= 6; n++)
//     {
//         const uint32_t nIdx = (~(outChildId ^ childId)) & n;
//         outNeighbours[n - 1] = ((nIdx != 0)
//             ? parentNeighbours[nIdx - 1]
//             : parentChildrenIndex
//             ) + (n ^ childId);
//     }
// }

// inline void getNeighboursVectorInUniformGrid(uint32_t outChildId, glm::ivec3 currentPos, uint32_t gridSize, std::array<uint32_t, 6>& outNeighbours)
// {
//     for (uint32_t n = 1; n <= 6; n++)
//     {
//         const glm::ivec3 nPos =
//             currentPos +
//             glm::ivec3(
//                 (n & 0b0001) ? ((outChildId & 0b0001) ? 1 : -1) : 0,
//                 (n & 0b0010) ? ((outChildId & 0b0010) ? 1 : -1) : 0,
//                 (n & 0b0100) ? ((outChildId & 0b0100) ? 1 : -1) : 0
//             );

//         if (nPos.x >= 0 && nPos.x < gridSize &&
//             nPos.y >= 0 && nPos.y < gridSize &&
//             nPos.z >= 0 && nPos.z < gridSize)
//         {
//             outNeighbours[n - 1] = nPos.z * gridSize * gridSize + nPos.y * gridSize + nPos.x;
//         }
//         else
//         {
//             outNeighbours[n - 1] = 1 << 30;
//         }
//     }
// }

template<typename TrianglesInfluenceStrategy>
void OctreeSdf::initOctreeWithContinuityNoDelay(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                              float terminationThreshold, OctreeSdf::TerminationRule terminationRule,
                              uint32_t numThreads)
{
    typedef typename TrianglesInfluenceStrategy::InterpolationMethod InterpolationMethod;
    typedef BreadthFirstNoDelayNodeInfo<typename TrianglesInfluenceStrategy::VertexInfo, InterpolationMethod::VALUES_PER_VERTEX, InterpolationMethod::NUM_COEFFICIENTS> NodeInfo;

    // terminationThreshold = terminationThreshold * glm::length(mesh.getBoundingBox().getSize());
    // const float sqTerminationThreshold = terminationThreshold * terminationThreshold * glm::length(mesh.getBoundingBox().getSize());
    terminationThreshold *= glm::length(mesh.getBoundingBox().getSize());
    const float sqTerminationThreshold = terminationThreshold * terminationThreshold;

    std::vector<TriangleUtils::TriangleData> trianglesData(TriangleUtils::calculateMeshTriangleData(mesh));
    
    const uint32_t startOctreeDepth = glm::min(startDepth, START_OCTREE_DEPTH);

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

    const std::array<uint32_t, 24> neigbourMasks =
    {
        // 001
        0b00100010010010001000, // [-,_,_]
        0b00001000100100100010, // [+,_,_]
        0b00000000000000000000, // NULL
        0b00000000000000000000, // NULL
        
        // 010
        0b01000011100000010000, // [_,-,_]
        0b00000100000011100001, // [_,+,_]
        0b00000000000000000000, // NULL
        0b00000000000000000000, // NULL

        // 011
        0b00000010000000000000, // [-,-,_]
        0b00000000100000000000, // [+,-,_]
        0b00000000000010000000, // [-,+,_]
        0b00000000000000100000, // [+,+,_]

        // 100
        0b01111100000000000000, // [_,_,-]
        0b00000000000000011111, // [_,_,+]
        0b00000000000000000000, // NULL
        0b00000000000000000000, // NULL

        // 101
        0b00100000000000000000, // [-,_,-]
        0b00001000000000000000, // [+,_,-]
        0b00000000000000001000, // [-,_,+]
        0b00000000000000000010, // [+,_,+]

        // 110
        0b01000000000000000000, // [_,-,-]
        0b00000100000000000000, // [_,+,-]        
        0b00000000000000010000, // [_,-,+]
        0b00000000000000000001  // [_,+,+]
    };

    uint8_t currentBuffer = 0;
    uint8_t nextBuffer = 1;
    //std::array<std::vector<NodeInfo>, 3> nodesBuffer;
	// nodesBuffer.fill(std::vector<NodeInfo>());
    std::vector<std::vector<NodeInfo>> nodesBuffer(maxDepth+1);

    const uint32_t numTriangles = trianglesData.size();
    std::vector<uint32_t> startTriangles(numTriangles);
    for(uint32_t i=0; i < numTriangles; i++)
    {
        startTriangles[i] = i;
    }

    TrianglesInfluenceStrategy trianglesInfluence;
    trianglesInfluence.initCaches(mBox, maxDepth);

    // Create the grid
    {
        const uint32_t voxlesPerAxis = 1 << startDepth;
        mOctreeData.resize(voxlesPerAxis * voxlesPerAxis * voxlesPerAxis);
    }

    // Create first start nodes
    {
        float newSize = 0.5f * mBox.getSize().x * glm::pow(0.5f, startOctreeDepth);
        const glm::vec3 startCenter = mBox.min + newSize;
        const uint32_t voxelsPerAxis = 1 << startOctreeDepth;

        std::vector<NodeInfo>& nodes = nodesBuffer[startOctreeDepth];

        for(uint32_t k=0; k < voxelsPerAxis; k++)
        {
            for(uint32_t j=0; j < voxelsPerAxis; j++)
            {
                for(uint32_t i=0; i < voxelsPerAxis; i++)
                {
                    nodes.push_back(NodeInfo(std::numeric_limits<uint32_t>::max(), 0, startCenter + glm::vec3(i, j, k) * 2.0f * newSize, newSize));
                    NodeInfo& n = nodes.back();
                    n.parentTriangles = &startTriangles;
                    std::array<float, InterpolationMethod::NUM_COEFFICIENTS> nullArray;
                    trianglesInfluence.calculateVerticesInfo(n.center, n.size, startTriangles, childrens,
                                                              0u, nullArray,
                                                              n.verticesValues, n.verticesInfo,
                                                              mesh, trianglesData);
                }
            }
        }
    }

    const std::vector<glm::vec3>& vertices = mesh.getVertices();
    const std::vector<uint32_t>& indices = mesh.getIndices();
    std::array<glm::vec3, 3> triangle;

    std::vector<std::pair<uint32_t, uint32_t>> verticesStatistics(maxDepth, std::make_pair(0, 0));
    verticesStatistics[0] = std::make_pair(trianglesData.size(), 1);

    std::vector<uint32_t> nodesToSubdivide;
    std::map<uint32_t, std::pair<uint32_t, uint32_t>> leavesData;

    std::vector<float> elapsedTime(maxDepth);
    std::vector<uint32_t> numTrianglesEvaluated(maxDepth, 0);
    std::vector<NodeInfo> nodesCache;
    Timer timer;
    float iter1TotalTime = 0.0f;
    float iter2TotalTime = 0.0f;
    float afterSubdivisionTime = 0.0f;
    uint32_t numNodesSubdividedAfterDecision = 0;

    #ifdef OPENMP_AVAILABLE
    omp_set_dynamic(0);
    omp_set_num_threads(numThreads);
    #else
    numThreads = 1;
    #endif

    std::vector<TrianglesInfluenceStrategy> threadTrianglesInfluence(numThreads, trianglesInfluence);

    for(uint32_t currentDepth=startOctreeDepth; currentDepth <= maxDepth; currentDepth++)
    {
        // Iter 1
        timer.start();
        if(currentDepth < maxDepth)
        {
            const auto nodesBufferSize = nodesBuffer[currentDepth].size();
            #ifdef OPENMP_AVAILABLE
            #pragma omp parallel for default(shared) schedule(dynamic, 16)
            #endif
            for(uint32_t nId=0; nId < nodesBufferSize; nId++)
            {
                #ifdef OPENMP_AVAILABLE
                const uint32_t tId = omp_get_thread_num();
                #else
                const uint32_t tId = 0;
                #endif

                NodeInfo& node = nodesBuffer[currentDepth][nId];
                if(node.ignoreNode) continue;

                OctreeNode* octreeNode = (currentDepth > startDepth) 
                                        ? &mOctreeData[node.parentChildrenIndex + (node.childIndices & 0b0111)]
                                        : nullptr;

                if(currentDepth == startDepth)
                {
                    glm::ivec3 nodeStartGridPos = glm::floor((node.center - mBox.min) / mStartGridCellSize);
                    const uint32_t nodeStartIndex = 
                                        nodeStartGridPos.z * mStartGridSize * mStartGridSize + 
                                        nodeStartGridPos.y * mStartGridSize + 
                                        nodeStartGridPos.x;
                    octreeNode = &mOctreeData[nodeStartIndex];
                }

                threadTrianglesInfluence[tId].filterTriangles(node.center, node.size, *node.parentTriangles, 
                                                   node.triangles, node.verticesValues, node.verticesInfo,
                                                   mesh, trianglesData);

                // Get current neighbours
                if(currentDepth > startDepth)
                {
                    for(uint8_t neighbour = 1; neighbour <= 6; neighbour++)
                    {
                        if((((node.neighbourIndices[neighbour - 1]) >> 30) & 0b01) == 0) // Calculate next neigbour
                        {
                            if (mOctreeData[node.neighbourIndices[neighbour - 1] & (~(1 << 31))].isLeaf())
                            {
                                node.neighbourIndices[neighbour - 1] = (1 << 31) | node.neighbourIndices[neighbour - 1];
                            }
                            else
                            {
                                node.neighbourIndices[neighbour - 1] = mOctreeData[node.neighbourIndices[neighbour - 1] & (~(1 << 31))].getChildrenIndex();
                                node.neighbourDepth[neighbour - 1]++;

                                while (node.neighbourDepth[neighbour - 1] < currentDepth)
                                {
                                    const uint32_t depthDiff = currentDepth - node.neighbourDepth[neighbour - 1];
                                    const uint32_t childId = (node.childIndices >> (3 * depthDiff)) & 0b0111;
                                    node.neighbourIndices[neighbour - 1] += (neighbour ^ childId);

                                    if (mOctreeData[node.neighbourIndices[neighbour - 1] & (~(1 << 31))].isLeaf())
                                    {
                                        node.neighbourIndices[neighbour - 1] = (1 << 31) | node.neighbourIndices[neighbour - 1];
                                        break;
                                    }
                                    else
                                    {
                                        node.neighbourIndices[neighbour - 1] = mOctreeData[node.neighbourIndices[neighbour - 1] & (~(1 << 31))].getChildrenIndex();
                                        node.neighbourDepth[neighbour - 1]++;
                                    }
                                }
                            }
                        }
                    }
                }

                if(currentDepth >= startDepth)
                {
                    InterpolationMethod::calculateCoefficients(node.verticesValues, 2.0f * node.size, node.triangles, mesh, trianglesData, node.interpolationCoeff);
                }
                
                threadTrianglesInfluence[tId].calculateVerticesInfo(node.center, node.size, node.triangles, nodeSamplePoints,
                                                                    0u, node.interpolationCoeff,
                                                                    node.midPointsValues, node.midPointsInfo,
                                                                    mesh, trianglesData);

                bool generateTerminalNodes = false;
                if(currentDepth >= startDepth)
                {
                    float value;
                    switch(terminationRule)
                    {
                        case TerminationRule::TRAPEZOIDAL_RULE:
                            {
                            // float value1 = estimateFaceErrorFunctionIntegralByTrapezoidRule<InterpolationMethod>(node.interpolationCoeff, node.midPointsValues);
                            // float value2 = estimateErrorFunctionIntegralByTrapezoidRule<InterpolationMethod>(node.interpolationCoeff, node.midPointsValues);
                            // generateTerminalNodes = value1 < sqTerminationThreshold && value2 < sqTerminationThreshold;
                            value = estimateErrorFunctionIntegralByTrapezoidRule<InterpolationMethod>(node.interpolationCoeff, node.midPointsValues);
                            generateTerminalNodes = value < sqTerminationThreshold;
                            }
                            break;
                        case TerminationRule::SIMPSONS_RULE:
                            value = estimateErrorFunctionIntegralBySimpsonsRule<InterpolationMethod>(node.interpolationCoeff, node.midPointsValues);
                            generateTerminalNodes = value < sqTerminationThreshold;
                            break;
                        case TerminationRule::NONE:
                            value = INFINITY;
                            break;
                    }
                }

                node.isTerminalNode = generateTerminalNodes;
                if(octreeNode != nullptr) octreeNode->setValues(generateTerminalNodes, std::numeric_limits<uint32_t>::max());
            }
        }
        iter1TotalTime += timer.getElapsedSeconds();

        // Iter 2
        timer.start();
        nodesToSubdivide.clear();
        for(uint32_t nId=0; nId < nodesBuffer[currentDepth].size(); nId++)
        {
            NodeInfo& node = nodesBuffer[currentDepth][nId];
            if(node.ignoreNode) continue;

            uint32_t octreeNodeIndex = (currentDepth > startDepth) 
                                        ? node.parentChildrenIndex + (node.childIndices & 0b0111)
                                        : std::numeric_limits<uint32_t>::max();

            const std::vector<OctreeSdf::OctreeNode>& octreeData = mOctreeData;

            glm::ivec3 nodeStartGridPos;
            if(currentDepth == startDepth)
            {
                nodeStartGridPos = glm::floor((node.center - mBox.min) / mStartGridCellSize);
                const uint32_t nodeStartIndex = 
                                    nodeStartGridPos.z * mStartGridSize * mStartGridSize + 
                                    nodeStartGridPos.y * mStartGridSize + 
                                    nodeStartGridPos.x;
                octreeNodeIndex = nodeStartIndex;
            }

            OctreeNode* octreeNode = (currentDepth >= startDepth) ? &mOctreeData[octreeNodeIndex] : nullptr;

            if(!node.isTerminalNode && currentDepth < maxDepth)
            {
                // Get current neighbours
                uint32_t samplesMask = 0; // Calculate which sample points must be interpolated
                std::array<uint32_t, 24> neighbourIds;
                neighbourIds.fill(std::numeric_limits<uint32_t>::max());
                if(currentDepth > startDepth)
                {
                    auto getNeighbourMask = [&](uint32_t nodeId, uint8_t dir, uint8_t sign) -> uint32_t
                    {
                        if((nodeId >> 31) || (!(nodeId >> 30) && octreeData[nodeId + (dir ^ (node.childIndices & 0b0111))].isLeaf()))
                        {
                            neighbourIds[4 * (dir - 1) + sign] = (nodeId >> 31) 
                                                                    ? nodeId & (~(1 << 31))
                                                                    : nodeId + (dir ^ (node.childIndices & 0b0111));
                            return neigbourMasks[4 * (dir - 1) + sign];
                        }
                        return 0;
                    };

                    samplesMask |= getNeighbourMask(node.neighbourIndices[0], 0b0001, (node.childIndices & 0b0111) & 0b001);
                    samplesMask |= getNeighbourMask(node.neighbourIndices[0], 0b0011, 0b010 ^ ((node.childIndices & 0b0111) & 0b011));
                    samplesMask |= getNeighbourMask(node.neighbourIndices[0], 0b0101, (((~(node.childIndices & 0b0111)) >> 1) & 0b010) + ((node.childIndices & 0b0111) & 0b001));
                    samplesMask |= getNeighbourMask(node.neighbourIndices[1], 0b0010, ((node.childIndices & 0b0111) >> 1) & 0b001);
                    samplesMask |= getNeighbourMask(node.neighbourIndices[1], 0b0011, 0b001 ^ ((node.childIndices & 0b0111) & 0b011));
                    samplesMask |= getNeighbourMask(node.neighbourIndices[1], 0b0110, 0b010 ^ (((node.childIndices & 0b0111) >> 1) & 0b011));
                    samplesMask |= getNeighbourMask(node.neighbourIndices[2], 0b0011, (node.childIndices & 0b0111) & 0b011);
                    samplesMask |= getNeighbourMask(node.neighbourIndices[3], 0b0100, ((node.childIndices & 0b0111) >> 2) & 0b001);
                    samplesMask |= getNeighbourMask(node.neighbourIndices[3], 0b0101, (((node.childIndices & 0b0111) >> 1) & 0b010) + ((~(node.childIndices & 0b0111)) & 0b001));
                    samplesMask |= getNeighbourMask(node.neighbourIndices[3], 0b0110, 0b001 ^ (((node.childIndices & 0b0111) >> 1) & 0b011));
                    samplesMask |= getNeighbourMask(node.neighbourIndices[4], 0b0101, (((node.childIndices & 0b0111) >> 1) & 0b010) + ((node.childIndices & 0b0111) & 0b001));
                    samplesMask |= getNeighbourMask(node.neighbourIndices[5], 0b0110, ((node.childIndices & 0b0111) >> 1) & 0b011);

                    samplesMask |= getNeighbourMask(node.parentChildrenIndex, 0b0001, (~(node.childIndices & 0b0111)) & 0b001);
                    samplesMask |= getNeighbourMask(node.parentChildrenIndex, 0b0010, ((~(node.childIndices & 0b0111)) >> 1) & 0b001);
                    samplesMask |= getNeighbourMask(node.parentChildrenIndex, 0b0100, ((~(node.childIndices & 0b0111)) >> 2) & 0b001);

                    samplesMask |= getNeighbourMask(node.parentChildrenIndex, 0b0011, (~(node.childIndices & 0b0111)) & 0b011);
                    samplesMask |= getNeighbourMask(node.parentChildrenIndex, 0b0101, (((~(node.childIndices & 0b0111)) >> 1) & 0b010) + ((~(node.childIndices & 0b0111)) & 0b001));
                    samplesMask |= getNeighbourMask(node.parentChildrenIndex, 0b0110, ((~(node.childIndices & 0b0111)) >> 1) & 0b011);
                }
                else if(currentDepth == startDepth)
                {
                    const int gridSize = mStartGridSize;
                    auto getNeighbourMask = [&](glm::ivec3 nPos, uint8_t dir, uint8_t sign) -> uint32_t
                    {
                        if (nPos.x >= 0 && nPos.x < gridSize &&
                            nPos.y >= 0 && nPos.y < gridSize &&
                            nPos.z >= 0 && nPos.z < gridSize)
                        {
                            if(octreeData[nPos.z * gridSize * gridSize + nPos.y * gridSize + nPos.x].isLeaf())
                            {
                                neighbourIds[4 * (dir - 1) + sign] = nPos.z * gridSize * gridSize + nPos.y * gridSize + nPos.x;
                                return neigbourMasks[4 * (dir - 1) + sign];
                            }
                            return 0;
                        }
                        return 0;
                    };

                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(-1, 0, 0), 0b0001, 0b000);
                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(1, 0, 0), 0b0001, 0b001);

                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(0, -1, 0), 0b0010, 0b000);
                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(0, 1, 0), 0b0010, 0b001);

                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(-1, -1, 0), 0b0011, 0b000);
                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(1, -1, 0), 0b0011, 0b001);
                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(-1, 1, 0), 0b0011, 0b010);
                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(1, 1, 0), 0b0011, 0b011);

                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(0, 0, -1), 0b0100, 0b000);
                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(0, 0, 1), 0b0100, 0b001);

                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(-1, 0, -1), 0b0101, 0b000);
                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(1, 0, -1), 0b0101, 0b001);
                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(-1, 0, 1), 0b0101, 0b010);
                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(1, 0, 1), 0b0101, 0b011);

                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(0, -1, -1), 0b0110, 0b000);
                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(0, 1, -1), 0b0110, 0b001);
                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(0, -1, 1), 0b0110, 0b010);
                    samplesMask |= getNeighbourMask(nodeStartGridPos + glm::ivec3(0, 1, 1), 0b0110, 0b011);
                }

                // Change distance if required
                {
                    uint32_t subdivisionMask = 0; // Calculate which sample points cannot be interpolated
                    for(uint32_t i=0; i < 19; i++)
                    {
                        if(samplesMask & (1 << (18-i)))
                        {
                            // InterpolationMethod::interpolateVertexValues(node.interpolationCoeff, 0.5f * nodeSamplePoints[i] + 0.5f, 2.0f * node.size, node.midPointsValues[i]);
                            const float interValue = InterpolationMethod::interpolateValue(node.interpolationCoeff, 0.5f * nodeSamplePoints[i] + 0.5f);
                            // Test if the node can be interpolated regarding the error
                            if(pow2(node.midPointsValues[i][0] - interValue) > sqTerminationThreshold)
                            {
                                subdivisionMask |= (samplesMask & (1 << (18-i)));
                            }
                            else
                            {
                                InterpolationMethod::interpolateVertexValues(node.interpolationCoeff, 0.5f * nodeSamplePoints[i] + 0.5f, 2.0f * node.size, node.midPointsValues[i]);
                            }
                        }
                    }

                    // Set the leaves that must be further subdivided
                    for(uint32_t i=0; i < 24; i++)
                    {
                        if((subdivisionMask & neigbourMasks[i]) && !(neighbourIds[i] >> 30))
                        {
                            nodesToSubdivide.push_back(neighbourIds[i]);
                        }
                    }
                }

                // Generate new children
				const float newSize = 0.5f * node.size;

                uint32_t childIndex = std::numeric_limits<uint32_t>::max();
                if(currentDepth >= startDepth)
                {
                    childIndex = mOctreeData.size();
                    octreeNode->setValues(false, childIndex);
                    mOctreeData.resize(mOctreeData.size() + 8);
                    for(uint32_t i=0; i < 8; i++)
                        mOctreeData[childIndex + i].childrenIndex = ~(0b0111u << 29);
                }

                std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, 19>& midPointsValues = node.midPointsValues;
                std::array<typename TrianglesInfluenceStrategy::VertexInfo, 19>& midPointsInfo = node.midPointsInfo;

				// Low Z children
				nodesBuffer[currentDepth + 1].push_back(NodeInfo(childIndex, (node.childIndices << 3) | 0, node.center + glm::vec3(-newSize, -newSize, -newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[currentDepth + 1].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = node.verticesValues[0]; child.verticesValues[1] = midPointsValues[0]; 
                    child.verticesValues[2] = midPointsValues[1]; child.verticesValues[3] = midPointsValues[2];
					child.verticesValues[4] = midPointsValues[5]; child.verticesValues[5] = midPointsValues[6];
					child.verticesValues[6] = midPointsValues[8]; child.verticesValues[7] = midPointsValues[9];

                    child.verticesInfo[0] = node.verticesInfo[0]; child.verticesInfo[1] = midPointsInfo[0]; 
                    child.verticesInfo[2] = midPointsInfo[1]; child.verticesInfo[3] = midPointsInfo[2];
					child.verticesInfo[4] = midPointsInfo[5]; child.verticesInfo[5] = midPointsInfo[6];
					child.verticesInfo[6] = midPointsInfo[8]; child.verticesInfo[7] = midPointsInfo[9];

                    if(currentDepth == startDepth) 
                    {
                        getNeighboursVectorInUniformGrid(0, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                        for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = currentDepth;
                    }
                    else getNeighboursVector(0, (node.childIndices & 0b0111), node.parentChildrenIndex, currentDepth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
				}

				nodesBuffer[currentDepth + 1].push_back(NodeInfo(childIndex, (node.childIndices << 3) | 1, node.center + glm::vec3(newSize, -newSize, -newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[currentDepth + 1].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = midPointsValues[0]; child.verticesValues[1] = node.verticesValues[1];
					child.verticesValues[2] = midPointsValues[2]; child.verticesValues[3] = midPointsValues[3];
					child.verticesValues[4] = midPointsValues[6]; child.verticesValues[5] = midPointsValues[7];
					child.verticesValues[6] = midPointsValues[9]; child.verticesValues[7] = midPointsValues[10];

                    child.verticesInfo[0] = midPointsInfo[0]; child.verticesInfo[1] = node.verticesInfo[1];
					child.verticesInfo[2] = midPointsInfo[2]; child.verticesInfo[3] = midPointsInfo[3];
					child.verticesInfo[4] = midPointsInfo[6]; child.verticesInfo[5] = midPointsInfo[7];
					child.verticesInfo[6] = midPointsInfo[9]; child.verticesInfo[7] = midPointsInfo[10];

                    if(currentDepth == startDepth) 
                    {
                        getNeighboursVectorInUniformGrid(1, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                        for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = currentDepth;
                    }
                    else getNeighboursVector(1, (node.childIndices & 0b0111), node.parentChildrenIndex, currentDepth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
				}

                nodesBuffer[currentDepth + 1].push_back(NodeInfo(childIndex, (node.childIndices << 3) | 2, node.center + glm::vec3(-newSize, newSize, -newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[currentDepth + 1].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = midPointsValues[1]; child.verticesValues[1] = midPointsValues[2];
					child.verticesValues[2] = node.verticesValues[2]; child.verticesValues[3] = midPointsValues[4];
					child.verticesValues[4] = midPointsValues[8]; child.verticesValues[5] = midPointsValues[9];
					child.verticesValues[6] = midPointsValues[11]; child.verticesValues[7] = midPointsValues[12];

                    child.verticesInfo[0] = midPointsInfo[1]; child.verticesInfo[1] = midPointsInfo[2];
					child.verticesInfo[2] = node.verticesInfo[2]; child.verticesInfo[3] = midPointsInfo[4];
					child.verticesInfo[4] = midPointsInfo[8]; child.verticesInfo[5] = midPointsInfo[9];
					child.verticesInfo[6] = midPointsInfo[11]; child.verticesInfo[7] = midPointsInfo[12];

                    if(currentDepth == startDepth) 
                    {
                        getNeighboursVectorInUniformGrid(2, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                        for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = currentDepth;
                    }
                    else getNeighboursVector(2, (node.childIndices & 0b0111), node.parentChildrenIndex, currentDepth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
				}

                nodesBuffer[currentDepth + 1].push_back(NodeInfo(childIndex, (node.childIndices << 3) | 3, node.center + glm::vec3(newSize, newSize, -newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[currentDepth + 1].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = midPointsValues[2]; child.verticesValues[1] = midPointsValues[3];
					child.verticesValues[2] = midPointsValues[4]; child.verticesValues[3] = node.verticesValues[3];
					child.verticesValues[4] = midPointsValues[9]; child.verticesValues[5] = midPointsValues[10];
					child.verticesValues[6] = midPointsValues[12]; child.verticesValues[7] = midPointsValues[13];

                    child.verticesInfo[0] = midPointsInfo[2]; child.verticesInfo[1] = midPointsInfo[3];
					child.verticesInfo[2] = midPointsInfo[4]; child.verticesInfo[3] = node.verticesInfo[3];
					child.verticesInfo[4] = midPointsInfo[9]; child.verticesInfo[5] = midPointsInfo[10];
					child.verticesInfo[6] = midPointsInfo[12]; child.verticesInfo[7] = midPointsInfo[13];

                    if(currentDepth == startDepth) 
                    {
                        getNeighboursVectorInUniformGrid(3, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                        for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = currentDepth;
                    }
                    else getNeighboursVector(3, (node.childIndices & 0b0111), node.parentChildrenIndex, currentDepth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
				}

                // High Z children
                nodesBuffer[currentDepth + 1].push_back(NodeInfo(childIndex, (node.childIndices << 3) | 4, node.center + glm::vec3(-newSize, -newSize, newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[currentDepth + 1].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = midPointsValues[5]; child.verticesValues[1] = midPointsValues[6];
					child.verticesValues[2] = midPointsValues[8]; child.verticesValues[3] = midPointsValues[9];
					child.verticesValues[4] = node.verticesValues[4]; child.verticesValues[5] = midPointsValues[14];
					child.verticesValues[6] = midPointsValues[15]; child.verticesValues[7] = midPointsValues[16];

                    child.verticesInfo[0] = midPointsInfo[5]; child.verticesInfo[1] = midPointsInfo[6];
					child.verticesInfo[2] = midPointsInfo[8]; child.verticesInfo[3] = midPointsInfo[9];
					child.verticesInfo[4] = node.verticesInfo[4]; child.verticesInfo[5] = midPointsInfo[14];
					child.verticesInfo[6] = midPointsInfo[15]; child.verticesInfo[7] = midPointsInfo[16];

                    if(currentDepth == startDepth) 
                    {
                        getNeighboursVectorInUniformGrid(4, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                        for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = currentDepth;
                    }
                    else getNeighboursVector(4, (node.childIndices & 0b0111), node.parentChildrenIndex, currentDepth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
				}

                nodesBuffer[currentDepth + 1].push_back(NodeInfo(childIndex, (node.childIndices << 3) | 5, node.center + glm::vec3(newSize, -newSize, newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[currentDepth + 1].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = midPointsValues[6]; child.verticesValues[1] = midPointsValues[7];
					child.verticesValues[2] = midPointsValues[9]; child.verticesValues[3] = midPointsValues[10];
					child.verticesValues[4] = midPointsValues[14]; child.verticesValues[5] = node.verticesValues[5];
					child.verticesValues[6] = midPointsValues[16]; child.verticesValues[7] = midPointsValues[17];

                    child.verticesInfo[0] = midPointsInfo[6]; child.verticesInfo[1] = midPointsInfo[7];
					child.verticesInfo[2] = midPointsInfo[9]; child.verticesInfo[3] = midPointsInfo[10];
					child.verticesInfo[4] = midPointsInfo[14]; child.verticesInfo[5] = node.verticesInfo[5];
					child.verticesInfo[6] = midPointsInfo[16]; child.verticesInfo[7] = midPointsInfo[17];

                    if(currentDepth == startDepth)
                    { 
                        getNeighboursVectorInUniformGrid(5, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                        for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = currentDepth;
                    }
                    else getNeighboursVector(5, (node.childIndices & 0b0111), node.parentChildrenIndex, currentDepth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
				}

                nodesBuffer[currentDepth + 1].push_back(NodeInfo(childIndex, (node.childIndices << 3) | 6, node.center + glm::vec3(-newSize, newSize, newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[currentDepth + 1].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = midPointsValues[8]; child.verticesValues[1] = midPointsValues[9];
					child.verticesValues[2] = midPointsValues[11]; child.verticesValues[3] = midPointsValues[12];
					child.verticesValues[4] = midPointsValues[15]; child.verticesValues[5] = midPointsValues[16];
					child.verticesValues[6] = node.verticesValues[6]; child.verticesValues[7] = midPointsValues[18];

                    child.verticesInfo[0] = midPointsInfo[8]; child.verticesInfo[1] = midPointsInfo[9];
					child.verticesInfo[2] = midPointsInfo[11]; child.verticesInfo[3] = midPointsInfo[12];
					child.verticesInfo[4] = midPointsInfo[15]; child.verticesInfo[5] = midPointsInfo[16];
					child.verticesInfo[6] = node.verticesInfo[6]; child.verticesInfo[7] = midPointsInfo[18];

                    if(currentDepth == startDepth)
                    {
                        getNeighboursVectorInUniformGrid(6, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                        for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = currentDepth;
                    }
                    else getNeighboursVector(6, (node.childIndices & 0b0111), node.parentChildrenIndex, currentDepth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
				}

                nodesBuffer[currentDepth + 1].push_back(NodeInfo(childIndex, (node.childIndices << 3) | 7, node.center + glm::vec3(newSize, newSize, newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[currentDepth + 1].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = midPointsValues[9]; child.verticesValues[1] = midPointsValues[10];
					child.verticesValues[2] = midPointsValues[12]; child.verticesValues[3] = midPointsValues[13];
					child.verticesValues[4] = midPointsValues[16]; child.verticesValues[5] = midPointsValues[17];
					child.verticesValues[6] = midPointsValues[18]; child.verticesValues[7] = node.verticesValues[7];

                    child.verticesInfo[0] = midPointsInfo[9]; child.verticesInfo[1] = midPointsInfo[10];
					child.verticesInfo[2] = midPointsInfo[12]; child.verticesInfo[3] = midPointsInfo[13];
					child.verticesInfo[4] = midPointsInfo[16]; child.verticesInfo[5] = midPointsInfo[17];
					child.verticesInfo[6] = midPointsInfo[18]; child.verticesInfo[7] = node.verticesInfo[7];

                    if(currentDepth == startDepth) 
                    {
                        getNeighboursVectorInUniformGrid(7, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                        for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = currentDepth;
                    }
                    else getNeighboursVector(7, (node.childIndices & 0b0111), node.parentChildrenIndex, currentDepth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
				}
            }
			else
			{
				uint32_t childIndex = mOctreeData.size();
				octreeNode->setValues(true, childIndex);

				mOctreeData.resize(mOctreeData.size() + InterpolationMethod::NUM_COEFFICIENTS);

                if(currentDepth >= maxDepth)
                {
                    InterpolationMethod::calculateCoefficients(node.verticesValues, 2.0f * node.size, node.triangles, mesh, trianglesData, node.interpolationCoeff);
                }

                for(uint32_t i=0; i < InterpolationMethod::NUM_COEFFICIENTS; i++)
                {
                    mOctreeData[childIndex + i].value = node.interpolationCoeff[i];
                }

				for (uint32_t i = 0; i < 8; i++)
				{
					mValueRange = glm::max(mValueRange, glm::abs(node.verticesValues[i][0]));
				}

                //leavesData.insert(std::make_pair(octreeNodeIndex, LeafData{currentDepth, node.neighbourIndices}));
                leavesData.insert(std::make_pair(octreeNodeIndex, std::make_pair(currentDepth, nId)));
            }
        }

        iter2TotalTime += timer.getElapsedSeconds();
        timer.start();

        numNodesSubdividedAfterDecision += nodesToSubdivide.size();

        for(uint32_t i=0; i < nodesToSubdivide.size(); i++)
        {
            const uint32_t nodeId = nodesToSubdivide[i];
            auto it = leavesData.find(nodeId);
            if(it == leavesData.end())
            {
                std::cout << "Leaf data not found" << std::endl;
            }

            nodesCache.clear();
            nodesCache.push_back(nodesBuffer[it->second.first][it->second.second]);
            std::vector<uint32_t> depthCache;
            depthCache.push_back(it->second.first);

            const NodeInfo& parentNode = nodesCache.back();
            OctreeNode* parentOctreeNode;
            if(it->second.first > startDepth) parentOctreeNode = &mOctreeData[parentNode.parentChildrenIndex + (parentNode.childIndices & 0b0111)];
            else
            {
                glm::ivec3 nodeStartGridPos = glm::floor((parentNode.center - mBox.min) / mStartGridCellSize);
                const uint32_t nodeStartIndex = 
                                    nodeStartGridPos.z * mStartGridSize * mStartGridSize + 
                                    nodeStartGridPos.y * mStartGridSize + 
                                    nodeStartGridPos.x;
                parentOctreeNode = &mOctreeData[nodeStartIndex];
            }
            if(!parentOctreeNode->isLeaf()) continue;
            bool recycledOldCoefficients = false;
            uint32_t oldCoefficientsIndex = parentOctreeNode->getChildrenIndex();

            bool firstIteration = true;
            uint32_t nodesCacheIndex = 0;
            while(nodesCacheIndex < nodesCache.size())
            {
                NodeInfo node = nodesCache[nodesCacheIndex];
                const uint32_t depth = depthCache[nodesCacheIndex];
                nodesCacheIndex++;

                OctreeNode* octreeNode = (depth > startDepth) ? &mOctreeData[node.parentChildrenIndex + (node.childIndices & 0b0111)] : nullptr;

                glm::ivec3 nodeStartGridPos;
                if(depth == startDepth)
                {
                    nodeStartGridPos = glm::floor((node.center - mBox.min) / mStartGridCellSize);
                    const uint32_t nodeStartIndex = 
                                        nodeStartGridPos.z * mStartGridSize * mStartGridSize + 
                                        nodeStartGridPos.y * mStartGridSize + 
                                        nodeStartGridPos.x;
                    octreeNode = &mOctreeData[nodeStartIndex];
                }

                uint32_t samplesMask = 0;
                uint32_t subdividedMask = 0;
                if(depth > startDepth)
                {
                    for(uint8_t neighbour = 1; neighbour <= 6; neighbour++)
                    {
                        const uint32_t sign = ((((neighbour & (node.childIndices & 0b0111)) >> 2) & 0b0001) << ((neighbour & 0b0001) | ((neighbour & 0b0010) >> 1))) +
                                            ((((neighbour & (node.childIndices & 0b0111)) >> 1) & 0b0001) << (neighbour & 0b0001)) +
                                            (neighbour & (node.childIndices & 0b0111) & 0b0001);

                        if((((node.neighbourIndices[neighbour - 1]) >> 30) & 0b01) == 0) // Calculate next neigbour
                        {
                            if ((!firstIteration || (node.neighbourIndices[neighbour - 1] >> 31)) &&
                                mOctreeData[node.neighbourIndices[neighbour - 1] & (~(1 << 31))].isLeaf())
                            {
                                node.neighbourIndices[neighbour - 1] = (1 << 31) | node.neighbourIndices[neighbour - 1];
                                samplesMask |= neigbourMasks[4 * (neighbour - 1) + sign];
                            }
                            else
                            {
                                if (!firstIteration || (node.neighbourIndices[neighbour - 1] >> 31))
                                {
                                    node.neighbourIndices[neighbour - 1] = mOctreeData[node.neighbourIndices[neighbour - 1] & (~(1 << 31))].getChildrenIndex();
                                    node.neighbourDepth[neighbour - 1]++;
                                }

                                while (node.neighbourDepth[neighbour - 1] < depth &&
                                    node.neighbourDepth[neighbour - 1] < currentDepth)
                                {
                                    const uint32_t depthDiff = depth - node.neighbourDepth[neighbour - 1];
                                    const uint32_t childId = (node.childIndices >> (3 * depthDiff)) & 0b0111;
                                    node.neighbourIndices[neighbour - 1] += (neighbour ^ childId);

                                    if (mOctreeData[node.neighbourIndices[neighbour - 1] & (~(1 << 31))].isLeaf())
                                    {
                                        node.neighbourIndices[neighbour - 1] = (1 << 31) | node.neighbourIndices[neighbour - 1];
                                        samplesMask |= neigbourMasks[4 * (neighbour - 1) + sign];
                                        break;
                                    }
                                    else
                                    {
                                        node.neighbourIndices[neighbour - 1] = mOctreeData[node.neighbourIndices[neighbour - 1] & (~(1 << 31))].getChildrenIndex();
                                        node.neighbourDepth[neighbour - 1]++;
                                    }
                                }


                                if((currentDepth >= depth) && !(node.neighbourIndices[neighbour - 1] >> 31))
                                {
                                    const uint32_t nextIndex = (node.neighbourIndices[neighbour - 1] & (~(1 << 31))) + (neighbour ^ (node.childIndices & 0b0111));
                                    subdividedMask |= (mOctreeData[nextIndex].isLeaf() || mOctreeData[nextIndex].isMarked()) ? 0 : neigbourMasks[4 * (neighbour - 1) + sign];
                                }
                            }
                        }
                    }
                }

                if(currentDepth >= depth)
                {
                    if(depth > startDepth)
                    {
                        auto updateNeighbourMask = [&](uint32_t nodeId, uint8_t dir, uint8_t sign)
                        {
                            const bool isLeaf = ((nodeId >> 31) || (nodeId >> 30) || mOctreeData[nodeId + (dir ^ (node.childIndices & 0b0111))].isLeaf() ||
                                                mOctreeData[nodeId + (dir ^ (node.childIndices & 0b0111))].isMarked());
                            subdividedMask |= isLeaf ? 0 : neigbourMasks[4 * (dir - 1) + sign];
                        };

                        updateNeighbourMask(node.parentChildrenIndex, 0b0001, (~(node.childIndices & 0b0111)) & 0b001);
                        updateNeighbourMask(node.parentChildrenIndex, 0b0010, ((~(node.childIndices & 0b0111)) >> 1) & 0b001);
                        updateNeighbourMask(node.parentChildrenIndex, 0b0100, ((~(node.childIndices & 0b0111)) >> 2) & 0b001);
                        updateNeighbourMask(node.parentChildrenIndex, 0b0011, (~(node.childIndices & 0b0111)) & 0b011);
                        updateNeighbourMask(node.parentChildrenIndex, 0b0101, (((~(node.childIndices & 0b0111)) >> 1) & 0b010) + ((~(node.childIndices & 0b0111)) & 0b001));
                        updateNeighbourMask(node.parentChildrenIndex, 0b0110, ((~(node.childIndices & 0b0111)) >> 1) & 0b011);
                        updateNeighbourMask(node.neighbourIndices[0], 0b0011, 0b010 ^ ((node.childIndices & 0b0111) & 0b011));
                        updateNeighbourMask(node.neighbourIndices[0], 0b0101, (((~(node.childIndices & 0b0111)) >> 1) & 0b010) + ((node.childIndices & 0b0111) & 0b001));
                        updateNeighbourMask(node.neighbourIndices[1], 0b0011, 0b001 ^ ((node.childIndices & 0b0111) & 0b011));
                        updateNeighbourMask(node.neighbourIndices[1], 0b0110, 0b010 ^ (((node.childIndices & 0b0111) >> 1) & 0b011));
                        updateNeighbourMask(node.neighbourIndices[3], 0b0101, (((node.childIndices & 0b0111) >> 1) & 0b010) + ((~(node.childIndices & 0b0111)) & 0b001));
                        updateNeighbourMask(node.neighbourIndices[3], 0b0110, 0b001 ^ (((node.childIndices & 0b0111) >> 1) & 0b011));
                    }
                    else if(depth == startDepth)
                    {
                        const int gridSize = mStartGridSize;
                        auto updateNeighbourMask = [&](glm::ivec3 nPos, uint8_t dir, uint8_t sign)
                        {
                            if (nPos.x >= 0 && nPos.x < gridSize &&
                                nPos.y >= 0 && nPos.y < gridSize &&
                                nPos.z >= 0 && nPos.z < gridSize)
                            {
                                subdividedMask |= (mOctreeData[nPos.z * gridSize * gridSize + nPos.y * gridSize + nPos.x].isLeaf() ||
                                                   mOctreeData[nPos.z * gridSize * gridSize + nPos.y * gridSize + nPos.x].isMarked())
                                                    ? 0 : neigbourMasks[4 * (dir - 1) + sign];
                            }
                        };

                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(-1, 0, 0), 0b0001, 0b000);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(1, 0, 0), 0b0001, 0b001);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(0, -1, 0), 0b0010, 0b000);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(0, 1, 0), 0b0010, 0b001);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(-1, -1, 0), 0b0011, 0b000);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(1, -1, 0), 0b0011, 0b001);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(-1, 1, 0), 0b0011, 0b010);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(1, 1, 0), 0b0011, 0b011);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(0, 0, -1), 0b0100, 0b000);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(0, 0, 1), 0b0100, 0b001);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(-1, 0, -1), 0b0101, 0b000);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(1, 0, -1), 0b0101, 0b001);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(-1, 0, 1), 0b0101, 0b010);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(1, 0, 1), 0b0101, 0b011);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(0, -1, -1), 0b0110, 0b000);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(0, 1, -1), 0b0110, 0b001);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(0, -1, 1), 0b0110, 0b010);
                        updateNeighbourMask(nodeStartGridPos + glm::ivec3(0, 1, 1), 0b0110, 0b011);    
                    }

                    samplesMask = ~subdividedMask;
                }
                
                if(currentDepth >= depth && samplesMask != (~0))
                {
                    const bool recycleMidPointsValues = firstIteration && !node.ignoreNode;
                    if(!recycleMidPointsValues)
                    {
                        InterpolationMethod::calculateCoefficients(node.verticesValues, 2.0f * node.size, node.triangles, mesh, trianglesData, node.interpolationCoeff);
                        trianglesInfluence.calculateVerticesInfo(node.center, node.size, node.triangles, nodeSamplePoints,
                                                                samplesMask, node.interpolationCoeff,
                                                                node.midPointsValues, node.midPointsInfo,
                                                                mesh, trianglesData);
                    }

                    for(uint32_t i=0; i < 19; i++)
                    {
                        if ((samplesMask & (1 << (18-i))) == 0)
                        {
                            // InterpolationMethod::interpolateVertexValues(node.interpolationCoeff, 0.5f * nodeSamplePoints[i] + 0.5f, 2.0f * node.size, node.midPointsValues[i]);
                            const float interValue = InterpolationMethod::interpolateValue(node.interpolationCoeff, 0.5f * nodeSamplePoints[i] + 0.5f);
                            
                            if(pow2(node.midPointsValues[i][0] - interValue) < sqTerminationThreshold)
                            {
                                InterpolationMethod::interpolateVertexValues(node.interpolationCoeff, 0.5f * nodeSamplePoints[i] + 0.5f, 2.0f * node.size, node.midPointsValues[i]);
                            }
                        }
                        else if(recycleMidPointsValues)
                        {
                            InterpolationMethod::interpolateVertexValues(node.interpolationCoeff, 0.5f * nodeSamplePoints[i] + 0.5f, 2.0f * node.size, node.midPointsValues[i]);
                        }
                    }

                    // Generate new children
                    const float newSize = 0.5f * node.size;

                    uint32_t childIndex = std::numeric_limits<uint32_t>::max();
                    if(depth >= startDepth)
                    {
                        childIndex = mOctreeData.size();
                        octreeNode->setValues(false, childIndex);
                        octreeNode->markNode();
                        mOctreeData.resize(mOctreeData.size() + 8);
                        for(uint32_t i=0; i < 8; i++)
                            mOctreeData[childIndex + i].setValues(true, 0);
                    }

                    std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, 19>& midPointsValues = node.midPointsValues;
                    std::array<typename TrianglesInfluenceStrategy::VertexInfo, 19>& midPointsInfo = node.midPointsInfo;

                    // Low Z children
                    depthCache.push_back(depth+1);
                    nodesCache.push_back(NodeInfo(childIndex, (node.childIndices << 3) | 0, node.center + glm::vec3(-newSize, -newSize, -newSize), newSize, false));
                    {
                        NodeInfo& child = nodesCache.back();
                        child.parentTriangles = &node.triangles;
                        child.verticesValues[0] = node.verticesValues[0]; child.verticesValues[1] = midPointsValues[0]; 
                        child.verticesValues[2] = midPointsValues[1]; child.verticesValues[3] = midPointsValues[2];
                        child.verticesValues[4] = midPointsValues[5]; child.verticesValues[5] = midPointsValues[6];
                        child.verticesValues[6] = midPointsValues[8]; child.verticesValues[7] = midPointsValues[9];

                        child.verticesInfo[0] = node.verticesInfo[0]; child.verticesInfo[1] = midPointsInfo[0]; 
                        child.verticesInfo[2] = midPointsInfo[1]; child.verticesInfo[3] = midPointsInfo[2];
                        child.verticesInfo[4] = midPointsInfo[5]; child.verticesInfo[5] = midPointsInfo[6];
                        child.verticesInfo[6] = midPointsInfo[8]; child.verticesInfo[7] = midPointsInfo[9];

                        if(depth == startDepth)
                        {
                            getNeighboursVectorInUniformGrid(0, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                            for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = depth;
                        }
                        else getNeighboursVector(0, (node.childIndices & 0b0111), node.parentChildrenIndex, depth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
                    }

                    depthCache.push_back(depth+1);
                    nodesCache.push_back(NodeInfo(childIndex, (node.childIndices << 3) | 1, node.center + glm::vec3(newSize, -newSize, -newSize), newSize, false));
                    {
                        NodeInfo& child = nodesCache.back();
                        child.parentTriangles = &node.triangles;
                        child.verticesValues[0] = midPointsValues[0]; child.verticesValues[1] = node.verticesValues[1];
                        child.verticesValues[2] = midPointsValues[2]; child.verticesValues[3] = midPointsValues[3];
                        child.verticesValues[4] = midPointsValues[6]; child.verticesValues[5] = midPointsValues[7];
                        child.verticesValues[6] = midPointsValues[9]; child.verticesValues[7] = midPointsValues[10];

                        child.verticesInfo[0] = midPointsInfo[0]; child.verticesInfo[1] = node.verticesInfo[1];
                        child.verticesInfo[2] = midPointsInfo[2]; child.verticesInfo[3] = midPointsInfo[3];
                        child.verticesInfo[4] = midPointsInfo[6]; child.verticesInfo[5] = midPointsInfo[7];
                        child.verticesInfo[6] = midPointsInfo[9]; child.verticesInfo[7] = midPointsInfo[10];

                        if(depth == startDepth)
                        {
                            getNeighboursVectorInUniformGrid(1, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                            for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = depth;
                        }
                        else getNeighboursVector(1, (node.childIndices & 0b0111), node.parentChildrenIndex, depth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
                    }

                    depthCache.push_back(depth+1);
                    nodesCache.push_back(NodeInfo(childIndex, (node.childIndices << 3) | 2, node.center + glm::vec3(-newSize, newSize, -newSize), newSize, false));
                    {
                        NodeInfo& child = nodesCache.back();
                        child.parentTriangles = &node.triangles;
                        child.verticesValues[0] = midPointsValues[1]; child.verticesValues[1] = midPointsValues[2];
                        child.verticesValues[2] = node.verticesValues[2]; child.verticesValues[3] = midPointsValues[4];
                        child.verticesValues[4] = midPointsValues[8]; child.verticesValues[5] = midPointsValues[9];
                        child.verticesValues[6] = midPointsValues[11]; child.verticesValues[7] = midPointsValues[12];

                        child.verticesInfo[0] = midPointsInfo[1]; child.verticesInfo[1] = midPointsInfo[2];
                        child.verticesInfo[2] = node.verticesInfo[2]; child.verticesInfo[3] = midPointsInfo[4];
                        child.verticesInfo[4] = midPointsInfo[8]; child.verticesInfo[5] = midPointsInfo[9];
                        child.verticesInfo[6] = midPointsInfo[11]; child.verticesInfo[7] = midPointsInfo[12];

                        if(depth == startDepth)
                        {
                            getNeighboursVectorInUniformGrid(2, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                            for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = depth;
                        }
                        else getNeighboursVector(2, (node.childIndices & 0b0111), node.parentChildrenIndex, depth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
                    }

                    depthCache.push_back(depth+1);
                    nodesCache.push_back(NodeInfo(childIndex, (node.childIndices << 3) | 3, node.center + glm::vec3(newSize, newSize, -newSize), newSize, false));
                    {
                        NodeInfo& child = nodesCache.back();
                        child.parentTriangles = &node.triangles;
                        child.verticesValues[0] = midPointsValues[2]; child.verticesValues[1] = midPointsValues[3];
                        child.verticesValues[2] = midPointsValues[4]; child.verticesValues[3] = node.verticesValues[3];
                        child.verticesValues[4] = midPointsValues[9]; child.verticesValues[5] = midPointsValues[10];
                        child.verticesValues[6] = midPointsValues[12]; child.verticesValues[7] = midPointsValues[13];

                        child.verticesInfo[0] = midPointsInfo[2]; child.verticesInfo[1] = midPointsInfo[3];
                        child.verticesInfo[2] = midPointsInfo[4]; child.verticesInfo[3] = node.verticesInfo[3];
                        child.verticesInfo[4] = midPointsInfo[9]; child.verticesInfo[5] = midPointsInfo[10];
                        child.verticesInfo[6] = midPointsInfo[12]; child.verticesInfo[7] = midPointsInfo[13];

                        if(depth == startDepth)
                        {
                            getNeighboursVectorInUniformGrid(3, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                            for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = depth;
                        }
                        else getNeighboursVector(3, (node.childIndices & 0b0111), node.parentChildrenIndex, depth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
                    }

                    // High Z children
                    depthCache.push_back(depth+1);
                    nodesCache.push_back(NodeInfo(childIndex, (node.childIndices << 3) | 4, node.center + glm::vec3(-newSize, -newSize, newSize), newSize, false));
                    {
                        NodeInfo& child = nodesCache.back();
                        child.parentTriangles = &node.triangles;
                        child.verticesValues[0] = midPointsValues[5]; child.verticesValues[1] = midPointsValues[6];
                        child.verticesValues[2] = midPointsValues[8]; child.verticesValues[3] = midPointsValues[9];
                        child.verticesValues[4] = node.verticesValues[4]; child.verticesValues[5] = midPointsValues[14];
                        child.verticesValues[6] = midPointsValues[15]; child.verticesValues[7] = midPointsValues[16];

                        child.verticesInfo[0] = midPointsInfo[5]; child.verticesInfo[1] = midPointsInfo[6];
                        child.verticesInfo[2] = midPointsInfo[8]; child.verticesInfo[3] = midPointsInfo[9];
                        child.verticesInfo[4] = node.verticesInfo[4]; child.verticesInfo[5] = midPointsInfo[14];
                        child.verticesInfo[6] = midPointsInfo[15]; child.verticesInfo[7] = midPointsInfo[16];

                        if(depth == startDepth)
                        {
                            getNeighboursVectorInUniformGrid(4, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                            for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = depth;
                        }
                        else getNeighboursVector(4, (node.childIndices & 0b0111), node.parentChildrenIndex, depth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
                    }

                    depthCache.push_back(depth+1);
                    nodesCache.push_back(NodeInfo(childIndex, (node.childIndices << 3) | 5, node.center + glm::vec3(newSize, -newSize, newSize), newSize, false));
                    {
                        NodeInfo& child = nodesCache.back();
                        child.parentTriangles = &node.triangles;
                        child.verticesValues[0] = midPointsValues[6]; child.verticesValues[1] = midPointsValues[7];
                        child.verticesValues[2] = midPointsValues[9]; child.verticesValues[3] = midPointsValues[10];
                        child.verticesValues[4] = midPointsValues[14]; child.verticesValues[5] = node.verticesValues[5];
                        child.verticesValues[6] = midPointsValues[16]; child.verticesValues[7] = midPointsValues[17];

                        child.verticesInfo[0] = midPointsInfo[6]; child.verticesInfo[1] = midPointsInfo[7];
                        child.verticesInfo[2] = midPointsInfo[9]; child.verticesInfo[3] = midPointsInfo[10];
                        child.verticesInfo[4] = midPointsInfo[14]; child.verticesInfo[5] = node.verticesInfo[5];
                        child.verticesInfo[6] = midPointsInfo[16]; child.verticesInfo[7] = midPointsInfo[17];

                        if(depth == startDepth)
                        {
                            getNeighboursVectorInUniformGrid(5, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                            for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = depth;
                        }
                        else getNeighboursVector(5, (node.childIndices & 0b0111), node.parentChildrenIndex, depth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
                    }

                    depthCache.push_back(depth+1);
                    nodesCache.push_back(NodeInfo(childIndex, (node.childIndices << 3) | 6, node.center + glm::vec3(-newSize, newSize, newSize), newSize, false));
                    {
                        NodeInfo& child = nodesCache.back();
                        child.parentTriangles = &node.triangles;
                        child.verticesValues[0] = midPointsValues[8]; child.verticesValues[1] = midPointsValues[9];
                        child.verticesValues[2] = midPointsValues[11]; child.verticesValues[3] = midPointsValues[12];
                        child.verticesValues[4] = midPointsValues[15]; child.verticesValues[5] = midPointsValues[16];
                        child.verticesValues[6] = node.verticesValues[6]; child.verticesValues[7] = midPointsValues[18];

                        child.verticesInfo[0] = midPointsInfo[8]; child.verticesInfo[1] = midPointsInfo[9];
                        child.verticesInfo[2] = midPointsInfo[11]; child.verticesInfo[3] = midPointsInfo[12];
                        child.verticesInfo[4] = midPointsInfo[15]; child.verticesInfo[5] = midPointsInfo[16];
                        child.verticesInfo[6] = node.verticesInfo[6]; child.verticesInfo[7] = midPointsInfo[18];

                        if(depth == startDepth)
                        {
                            getNeighboursVectorInUniformGrid(6, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                            for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = depth;
                        }
                        else getNeighboursVector(6, (node.childIndices & 0b0111), node.parentChildrenIndex, depth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
                    }

                    depthCache.push_back(depth+1);
                    nodesCache.push_back(NodeInfo(childIndex, (node.childIndices << 3) | 7, node.center + glm::vec3(newSize, newSize, newSize), newSize, false));
                    {
                        NodeInfo& child = nodesCache.back();
                        child.parentTriangles = &node.triangles;
                        child.verticesValues[0] = midPointsValues[9]; child.verticesValues[1] = midPointsValues[10];
                        child.verticesValues[2] = midPointsValues[12]; child.verticesValues[3] = midPointsValues[13];
                        child.verticesValues[4] = midPointsValues[16]; child.verticesValues[5] = midPointsValues[17];
                        child.verticesValues[6] = midPointsValues[18]; child.verticesValues[7] = node.verticesValues[7];

                        child.verticesInfo[0] = midPointsInfo[9]; child.verticesInfo[1] = midPointsInfo[10];
                        child.verticesInfo[2] = midPointsInfo[12]; child.verticesInfo[3] = midPointsInfo[13];
                        child.verticesInfo[4] = midPointsInfo[16]; child.verticesInfo[5] = midPointsInfo[17];
                        child.verticesInfo[6] = midPointsInfo[18]; child.verticesInfo[7] = node.verticesInfo[7];

                        if(depth == startDepth)
                        {
                            getNeighboursVectorInUniformGrid(7, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                            for(uint32_t i=0; i < 6; i++) child.neighbourDepth[i] = depth;
                        }
                        else getNeighboursVector(7, (node.childIndices & 0b0111), node.parentChildrenIndex, depth, node.neighbourIndices, node.neighbourDepth, child.neighbourIndices, child.neighbourDepth);
                    }
                }
                else
                {
                    uint32_t childIndex = mOctreeData.size();
                    if(recycledOldCoefficients)
                    {
                        octreeNode->setValues(true, childIndex);
                        mOctreeData.resize(mOctreeData.size() + InterpolationMethod::NUM_COEFFICIENTS);
                    }
                    else
                    {
                        childIndex = oldCoefficientsIndex;
                        octreeNode->setValues(true, childIndex);
                        recycledOldCoefficients = true;
                    }

                    InterpolationMethod::calculateCoefficients(node.verticesValues, 2.0f * node.size, node.triangles, mesh, trianglesData, node.interpolationCoeff);

                    for(uint32_t i=0; i < InterpolationMethod::NUM_COEFFICIENTS; i++)
                    {
                        mOctreeData[childIndex + i].value = node.interpolationCoeff[i];
                        // mOctreeData[childIndex + i].value = 0.0;
                    }

                    // for (uint32_t i = 0; i < 8; i++)
                    // {
                    //     mValueRange = glm::max(mValueRange, glm::abs(node.verticesValues[i][0]));
                    // }

                    node.isTerminalNode = true;
                    node.ignoreNode = true;
                    nodesBuffer[depth].push_back(node);
                    leavesData.insert(std::make_pair(node.parentChildrenIndex + (node.childIndices & 0b0111), std::make_pair(depth, nodesBuffer[depth].size()-1)));
                }

                firstIteration = false;
            }
        }

        afterSubdivisionTime += timer.getElapsedSeconds();

        // Swap buffers
        // currentBuffer = (currentBuffer + 1) % 3;
        // nextBuffer = (nextBuffer + 1) % 3;
        // nodesBuffer[nextBuffer].clear();
    }

    // Add start index to the subtree
    std::function<void(OctreeNode&)> vistNode;
    vistNode = [&](OctreeNode& node)
    {
        node.removeMark();

        // Iterate children
        if(!node.isLeaf())
        {
            for(uint32_t i = 0; i < 8; i++)
            {
                vistNode(mOctreeData[node.getChildrenIndex() + i]);
            }
        }
    };

    for(uint32_t k=0; k < mStartGridSize; k++)
    {
        for(uint32_t j=0; j < mStartGridSize; j++)
        {
            for(uint32_t i=0; i < mStartGridSize; i++)
            {
                const uint32_t nodeStartIndex = k * mStartGridSize * mStartGridSize + j * mStartGridSize + i;
                vistNode(mOctreeData[nodeStartIndex]);
            }
        }
    }

#ifdef SDFLIB_PRINT_STATISTICS
    SPDLOG_INFO("Iter 1 {}s // Iter 2 {}s // After {}s // {}", iter1TotalTime, iter2TotalTime, afterSubdivisionTime, iter1TotalTime/(iter2TotalTime + afterSubdivisionTime));
    SPDLOG_INFO("Num nodes subdivided after desicion: {}", numNodesSubdividedAfterDecision);
#endif

}
}

#endif