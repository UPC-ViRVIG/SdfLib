#ifndef OCTREE_SDF_BREADTH_FIRST_NO_DELAY_H
#define OCTREE_SDF_BREADTH_FIRST_NO_DELAY_H

#include "OctreeSdf.h"
#include "utils/Timer.h"
#include "utils/GJK.h"
#include "OctreeSdfUtils.h"
#include <array>
#include <stack>
#include <omp.h>

template<typename VertexInfo, int VALUES_PER_VERTEX, int NUM_COEFFICIENTS>
struct BreadthFirstNoDelayNodeInfo
{
		BreadthFirstNoDelayNodeInfo() {}
		BreadthFirstNoDelayNodeInfo(uint32_t parentChildrenIndex, uint8_t childIndex, glm::vec3 center, float size, bool isTerminalNode = false)
			: parentChildrenIndex(parentChildrenIndex), childIndex(childIndex), center(center), size(size), isTerminalNode(isTerminalNode) {}
		
        uint32_t parentChildrenIndex;
		uint8_t childIndex;
		bool isTerminalNode;

		glm::vec3 center;
		float size;

		std::array<uint32_t, 6> neighbourIndices;

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
    typedef TrianglesInfluenceStrategy::InterpolationMethod InterpolationMethod;
    typedef BreadthFirstNoDelayNodeInfo<TrianglesInfluenceStrategy::VertexInfo, InterpolationMethod::VALUES_PER_VERTEX, InterpolationMethod::NUM_COEFFICIENTS> NodeInfo;

    // terminationThreshold = terminationThreshold * glm::length(mesh.getBoundingBox().getSize());
    // const float sqTerminationThreshold = terminationThreshold * terminationThreshold * glm::length(mesh.getBoundingBox().getSize());
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
    std::array<std::vector<NodeInfo>, 3> nodesBuffer;
	nodesBuffer.fill(std::vector<NodeInfo>());

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

        std::vector<NodeInfo>& nodes = nodesBuffer[currentBuffer];

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

    std::vector<float> elapsedTime(maxDepth);
    std::vector<uint32_t> numTrianglesEvaluated(maxDepth, 0);
    Timer timer;
    float iter1TotalTime = 0.0f;
    float iter2TotalTime = 0.0f;

    omp_set_dynamic(0);
    omp_set_num_threads(numThreads);

    for(uint32_t currentDepth=startOctreeDepth; currentDepth <= maxDepth; currentDepth++)
    {
        // Iter 1
        timer.start();
        if(currentDepth < maxDepth)
        {
            //for(NodeInfo& node : nodesBuffer[currentBuffer])
            const auto nodesBufferSize = nodesBuffer[currentBuffer].size();
            #pragma omp parallel for default(shared) schedule(dynamic, 16)
            for(uint32_t nId=0; nId < nodesBufferSize; nId++)
            {
                // std::cout << "enter" << std::endl;
                NodeInfo& node = nodesBuffer[currentBuffer][nId];

                OctreeNode* octreeNode = (currentDepth > startDepth) 
                                        ? &mOctreeData[node.parentChildrenIndex + node.childIndex]
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

                trianglesInfluence.filterTriangles(node.center, node.size, *node.parentTriangles, 
                                                   node.triangles, node.verticesValues, node.verticesInfo,
                                                   mesh, trianglesData);

                // Get current neighbours
                uint32_t samplesMask = 0; // Calculate which sample points must be interpolated
                if(currentDepth > startDepth)
                {
                    for(uint8_t neighbour = 1; neighbour <= 6; neighbour++)
                    {
                        if(((node.neighbourIndices[neighbour - 1]) >> 30) == 0) // Calculate next neigbour
                        {                            
                            if (mOctreeData[node.neighbourIndices[neighbour - 1]].isLeaf())
                            {
                                node.neighbourIndices[neighbour - 1] = 1 << 31;
                            }
                            else
                            {
                                node.neighbourIndices[neighbour - 1] = mOctreeData[node.neighbourIndices[neighbour - 1]].getChildrenIndex();
                            }
                        }
                        
                        const uint32_t sign = ((((neighbour & node.childIndex) >> 2) & 0b0001) << ((neighbour & 0b0001) | ((neighbour & 0b0010) >> 1))) +
											  ((((neighbour & node.childIndex) >> 1) & 0b0001) << (neighbour & 0b0001)) +
											  (neighbour & node.childIndex & 0b0001);
                        assert(sign >= 0 && sign < 4);
                        samplesMask |= (node.neighbourIndices[neighbour - 1] >> 31)
                                        ? neigbourMasks[4 * (neighbour - 1) + sign]
                                        : 0;
                        
                    }
                }

                if(currentDepth >= startDepth)
                {
                    InterpolationMethod::calculateCoefficients(node.verticesValues, 2.0f * node.size, node.triangles, mesh, trianglesData, node.interpolationCoeff);
                }
                
                trianglesInfluence.calculateVerticesInfo(node.center, node.size, node.triangles, nodeSamplePoints,
                                                         samplesMask, node.interpolationCoeff,
                                                         node.midPointsValues, node.midPointsInfo,
                                                         mesh, trianglesData);

                bool generateTerminalNodes = false;
                if(currentDepth >= startDepth)
                {
                    float value;
                    switch(terminationRule)
                    {
                        case TerminationRule::TRAPEZOIDAL_RULE:
                            value = estimateErrorFunctionIntegralByTrapezoidRule<InterpolationMethod>(node.interpolationCoeff, node.midPointsValues);
                            break;
                        case TerminationRule::SIMPSONS_RULE:
                            value = estimateErrorFunctionIntegralBySimpsonsRule<InterpolationMethod>(node.interpolationCoeff, node.midPointsValues);
                            break;
                        case TerminationRule::NONE:
                            value = INFINITY;
                            break;
                    }

                    generateTerminalNodes = value < sqTerminationThreshold;
                }

                node.isTerminalNode = generateTerminalNodes;
                if(octreeNode != nullptr) octreeNode->setValues(generateTerminalNodes, std::numeric_limits<uint32_t>::max());
            }
        }
        iter1TotalTime += timer.getElapsedSeconds();

        // Iter 2
        timer.start();
        for(NodeInfo& node : nodesBuffer[currentBuffer])
        {
            OctreeNode* octreeNode = (currentDepth > startDepth) 
                                        ? &mOctreeData[node.parentChildrenIndex + node.childIndex]
                                        : nullptr;

            const std::vector<OctreeSdf::OctreeNode>& octreeData = mOctreeData;

            glm::ivec3 nodeStartGridPos;
            if(currentDepth == startDepth)
            {
                nodeStartGridPos = glm::floor((node.center - mBox.min) / mStartGridCellSize);
                const uint32_t nodeStartIndex = 
                                    nodeStartGridPos.z * mStartGridSize * mStartGridSize + 
                                    nodeStartGridPos.y * mStartGridSize + 
                                    nodeStartGridPos.x;
                octreeNode = &mOctreeData[nodeStartIndex];
            }

            if(!node.isTerminalNode && currentDepth < maxDepth)
            {
                // Get current neighbours
                uint32_t samplesMask = 0; // Calculate which sample points must be interpolated
                if(currentDepth > startDepth)
                {
                    auto getNeighbourMask = [&](uint32_t nodeId, uint8_t dir, uint8_t sign) -> uint32_t
                    {
                        return ((nodeId >> 31) || (!(nodeId >> 30) && octreeData[nodeId + (dir ^ node.childIndex)].isLeaf()))
                                ? neigbourMasks[4 * (dir - 1) + sign]
                                : 0;
                    };

                    samplesMask |= getNeighbourMask(node.neighbourIndices[0], 0b0001, node.childIndex & 0b001);
                    samplesMask |= getNeighbourMask(node.neighbourIndices[0], 0b0011, 0b010 ^ (node.childIndex & 0b011));
                    samplesMask |= getNeighbourMask(node.neighbourIndices[0], 0b0101, (((~node.childIndex) >> 1) & 0b010) + (node.childIndex & 0b001));
                    samplesMask |= getNeighbourMask(node.neighbourIndices[1], 0b0010, (node.childIndex >> 1) & 0b001);
                    samplesMask |= getNeighbourMask(node.neighbourIndices[1], 0b0011, 0b001 ^ (node.childIndex & 0b011));
                    samplesMask |= getNeighbourMask(node.neighbourIndices[1], 0b0110, 0b010 ^ ((node.childIndex >> 1) & 0b011));
                    samplesMask |= getNeighbourMask(node.neighbourIndices[2], 0b0011, node.childIndex & 0b011);
                    samplesMask |= getNeighbourMask(node.neighbourIndices[3], 0b0100, (node.childIndex >> 2) & 0b001);
                    samplesMask |= getNeighbourMask(node.neighbourIndices[3], 0b0101, ((node.childIndex >> 1) & 0b010) + ((~node.childIndex) & 0b001));
                    samplesMask |= getNeighbourMask(node.neighbourIndices[3], 0b0110, 0b001 ^ ((node.childIndex >> 1) & 0b011));
                    samplesMask |= getNeighbourMask(node.neighbourIndices[4], 0b0101, ((node.childIndex >> 1) & 0b010) + (node.childIndex & 0b001));
                    samplesMask |= getNeighbourMask(node.neighbourIndices[5], 0b0110, (node.childIndex >> 1) & 0b011);

                    samplesMask |= getNeighbourMask(node.parentChildrenIndex, 0b0001, (~node.childIndex) & 0b001);
                    samplesMask |= getNeighbourMask(node.parentChildrenIndex, 0b0010, ((~node.childIndex) >> 1) & 0b001);
                    samplesMask |= getNeighbourMask(node.parentChildrenIndex, 0b0100, ((~node.childIndex) >> 2) & 0b001);

                    samplesMask |= getNeighbourMask(node.parentChildrenIndex, 0b0011, (~node.childIndex) & 0b011);
                    samplesMask |= getNeighbourMask(node.parentChildrenIndex, 0b0101, (((~node.childIndex) >> 1) & 0b010) + ((~node.childIndex) & 0b001));
                    samplesMask |= getNeighbourMask(node.parentChildrenIndex, 0b0110, ((~node.childIndex) >> 1) & 0b011);
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
                            return (octreeData[nPos.z * gridSize * gridSize + nPos.y * gridSize + nPos.x].isLeaf())
                                ? neigbourMasks[4 * (dir - 1) + sign]
                                : 0;
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

                // Change distance if requires
                {
                    for(uint32_t i=0; i < 19; i++)
                    {
                        if(samplesMask & (1 << (18-i)))
                        {
                            InterpolationMethod::interpolateVertexValues(node.interpolationCoeff, 0.5f * nodeSamplePoints[i] + 0.5f, 2.0f * node.size, node.midPointsValues[i]);
                        }
                    }
                }

                // Generate new childrens
				const float newSize = 0.5f * node.size;

                uint32_t childIndex = std::numeric_limits<uint32_t>::max();
                if(currentDepth >= startDepth)
                {
                    childIndex = mOctreeData.size();
                    octreeNode->setValues(false, childIndex);
                    mOctreeData.resize(mOctreeData.size() + 8);
                }

                std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, 19>& midPointsValues = node.midPointsValues;
                std::array<TrianglesInfluenceStrategy::VertexInfo, 19>& midPointsInfo = node.midPointsInfo;

				// Low Z children
				nodesBuffer[nextBuffer].push_back(NodeInfo(childIndex, 0, node.center + glm::vec3(-newSize, -newSize, -newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = node.verticesValues[0]; child.verticesValues[1] = midPointsValues[0]; 
                    child.verticesValues[2] = midPointsValues[1]; child.verticesValues[3] = midPointsValues[2];
					child.verticesValues[4] = midPointsValues[5]; child.verticesValues[5] = midPointsValues[6];
					child.verticesValues[6] = midPointsValues[8]; child.verticesValues[7] = midPointsValues[9];

                    child.verticesInfo[0] = node.verticesInfo[0]; child.verticesInfo[1] = midPointsInfo[0]; 
                    child.verticesInfo[2] = midPointsInfo[1]; child.verticesInfo[3] = midPointsInfo[2];
					child.verticesInfo[4] = midPointsInfo[5]; child.verticesInfo[5] = midPointsInfo[6];
					child.verticesInfo[6] = midPointsInfo[8]; child.verticesInfo[7] = midPointsInfo[9];

                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(0, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(0, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
				}

				nodesBuffer[nextBuffer].push_back(NodeInfo(childIndex, 1, node.center + glm::vec3(newSize, -newSize, -newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = midPointsValues[0]; child.verticesValues[1] = node.verticesValues[1];
					child.verticesValues[2] = midPointsValues[2]; child.verticesValues[3] = midPointsValues[3];
					child.verticesValues[4] = midPointsValues[6]; child.verticesValues[5] = midPointsValues[7];
					child.verticesValues[6] = midPointsValues[9]; child.verticesValues[7] = midPointsValues[10];

                    child.verticesInfo[0] = midPointsInfo[0]; child.verticesInfo[1] = node.verticesInfo[1];
					child.verticesInfo[2] = midPointsInfo[2]; child.verticesInfo[3] = midPointsInfo[3];
					child.verticesInfo[4] = midPointsInfo[6]; child.verticesInfo[5] = midPointsInfo[7];
					child.verticesInfo[6] = midPointsInfo[9]; child.verticesInfo[7] = midPointsInfo[10];

                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(1, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(1, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
				}

                nodesBuffer[nextBuffer].push_back(NodeInfo(childIndex, 2, node.center + glm::vec3(-newSize, newSize, -newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = midPointsValues[1]; child.verticesValues[1] = midPointsValues[2];
					child.verticesValues[2] = node.verticesValues[2]; child.verticesValues[3] = midPointsValues[4];
					child.verticesValues[4] = midPointsValues[8]; child.verticesValues[5] = midPointsValues[9];
					child.verticesValues[6] = midPointsValues[11]; child.verticesValues[7] = midPointsValues[12];

                    child.verticesInfo[0] = midPointsInfo[1]; child.verticesInfo[1] = midPointsInfo[2];
					child.verticesInfo[2] = node.verticesInfo[2]; child.verticesInfo[3] = midPointsInfo[4];
					child.verticesInfo[4] = midPointsInfo[8]; child.verticesInfo[5] = midPointsInfo[9];
					child.verticesInfo[6] = midPointsInfo[11]; child.verticesInfo[7] = midPointsInfo[12];

                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(2, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(2, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
				}

                nodesBuffer[nextBuffer].push_back(NodeInfo(childIndex, 3, node.center + glm::vec3(newSize, newSize, -newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = midPointsValues[2]; child.verticesValues[1] = midPointsValues[3];
					child.verticesValues[2] = midPointsValues[4]; child.verticesValues[3] = node.verticesValues[3];
					child.verticesValues[4] = midPointsValues[9]; child.verticesValues[5] = midPointsValues[10];
					child.verticesValues[6] = midPointsValues[12]; child.verticesValues[7] = midPointsValues[13];

                    child.verticesInfo[0] = midPointsInfo[2]; child.verticesInfo[1] = midPointsInfo[3];
					child.verticesInfo[2] = midPointsInfo[4]; child.verticesInfo[3] = node.verticesInfo[3];
					child.verticesInfo[4] = midPointsInfo[9]; child.verticesInfo[5] = midPointsInfo[10];
					child.verticesInfo[6] = midPointsInfo[12]; child.verticesInfo[7] = midPointsInfo[13];

                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(3, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(3, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
				}

                // High Z children
                nodesBuffer[nextBuffer].push_back(NodeInfo(childIndex, 4, node.center + glm::vec3(-newSize, -newSize, newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = midPointsValues[5]; child.verticesValues[1] = midPointsValues[6];
					child.verticesValues[2] = midPointsValues[8]; child.verticesValues[3] = midPointsValues[9];
					child.verticesValues[4] = node.verticesValues[4]; child.verticesValues[5] = midPointsValues[14];
					child.verticesValues[6] = midPointsValues[15]; child.verticesValues[7] = midPointsValues[16];

                    child.verticesInfo[0] = midPointsInfo[5]; child.verticesInfo[1] = midPointsInfo[6];
					child.verticesInfo[2] = midPointsInfo[8]; child.verticesInfo[3] = midPointsInfo[9];
					child.verticesInfo[4] = node.verticesInfo[4]; child.verticesInfo[5] = midPointsInfo[14];
					child.verticesInfo[6] = midPointsInfo[15]; child.verticesInfo[7] = midPointsInfo[16];

                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(4, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(4, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
				}

                nodesBuffer[nextBuffer].push_back(NodeInfo(childIndex, 5, node.center + glm::vec3(newSize, -newSize, newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = midPointsValues[6]; child.verticesValues[1] = midPointsValues[7];
					child.verticesValues[2] = midPointsValues[9]; child.verticesValues[3] = midPointsValues[10];
					child.verticesValues[4] = midPointsValues[14]; child.verticesValues[5] = node.verticesValues[5];
					child.verticesValues[6] = midPointsValues[16]; child.verticesValues[7] = midPointsValues[17];

                    child.verticesInfo[0] = midPointsInfo[6]; child.verticesInfo[1] = midPointsInfo[7];
					child.verticesInfo[2] = midPointsInfo[9]; child.verticesInfo[3] = midPointsInfo[10];
					child.verticesInfo[4] = midPointsInfo[14]; child.verticesInfo[5] = node.verticesInfo[5];
					child.verticesInfo[6] = midPointsInfo[16]; child.verticesInfo[7] = midPointsInfo[17];

                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(5, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(5, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
				}

                nodesBuffer[nextBuffer].push_back(NodeInfo(childIndex, 6, node.center + glm::vec3(-newSize, newSize, newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = midPointsValues[8]; child.verticesValues[1] = midPointsValues[9];
					child.verticesValues[2] = midPointsValues[11]; child.verticesValues[3] = midPointsValues[12];
					child.verticesValues[4] = midPointsValues[15]; child.verticesValues[5] = midPointsValues[16];
					child.verticesValues[6] = node.verticesValues[6]; child.verticesValues[7] = midPointsValues[18];

                    child.verticesInfo[0] = midPointsInfo[8]; child.verticesInfo[1] = midPointsInfo[9];
					child.verticesInfo[2] = midPointsInfo[11]; child.verticesInfo[3] = midPointsInfo[12];
					child.verticesInfo[4] = midPointsInfo[15]; child.verticesInfo[5] = midPointsInfo[16];
					child.verticesInfo[6] = node.verticesInfo[6]; child.verticesInfo[7] = midPointsInfo[18];

                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(6, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(6, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
				}

                nodesBuffer[nextBuffer].push_back(NodeInfo(childIndex, 7, node.center + glm::vec3(newSize, newSize, newSize), newSize, false));
				{
					NodeInfo& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.verticesValues[0] = midPointsValues[9]; child.verticesValues[1] = midPointsValues[10];
					child.verticesValues[2] = midPointsValues[12]; child.verticesValues[3] = midPointsValues[13];
					child.verticesValues[4] = midPointsValues[16]; child.verticesValues[5] = midPointsValues[17];
					child.verticesValues[6] = midPointsValues[18]; child.verticesValues[7] = node.verticesValues[7];

                    child.verticesInfo[0] = midPointsInfo[9]; child.verticesInfo[1] = midPointsInfo[10];
					child.verticesInfo[2] = midPointsInfo[12]; child.verticesInfo[3] = midPointsInfo[13];
					child.verticesInfo[4] = midPointsInfo[16]; child.verticesInfo[5] = midPointsInfo[17];
					child.verticesInfo[6] = midPointsInfo[18]; child.verticesInfo[7] = node.verticesInfo[7];

                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(7, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(7, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
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
            }
        }
        iter2TotalTime += timer.getElapsedSeconds();

        // Swap buffers
        currentBuffer = (currentBuffer + 1) % 3;
        nextBuffer = (nextBuffer + 1) % 3;
        nodesBuffer[nextBuffer].clear();
    }

    SPDLOG_INFO("Iter 1 {}s // Iter 2 {}s // {}", iter1TotalTime, iter2TotalTime, iter1TotalTime/iter2TotalTime);
}

#endif