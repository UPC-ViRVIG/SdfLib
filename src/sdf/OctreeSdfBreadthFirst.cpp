#include "OctreeSdf.h"
#include "utils/Timer.h"
#include "utils/GJK.h"
#include "OctreeSdfUtils.h"
#include <array>
#include <stack>

namespace
{
	struct NodeInfo1
	{
		NodeInfo1() {}
		NodeInfo1(uint32_t parentChildrenIndex, uint8_t childIndex, glm::vec3 center, float size, bool isTerminalNode = false)
			: parentChildrenIndex(parentChildrenIndex), childIndex(childIndex), center(center), size(size), isTerminalNode(isTerminalNode) {}
		uint32_t parentChildrenIndex;
		uint8_t childIndex;
		bool isTerminalNode;

		glm::vec3 center;
		float size;

		std::array<uint32_t, 6> neighbourIndices;

		std::array<float, 8> distanceToVertices;
		std::array<float, 19> distanceToMidPoints;

		std::vector<uint32_t>* parentTriangles;
		std::vector<uint32_t> triangles;
	};
}

inline void getNeighboursVector(uint32_t outChildId, uint32_t childId, uint32_t parentChildrenIndex, const std::array<uint32_t, 6>& parentNeighbours, std::array<uint32_t, 6>& outNeighbours)
{
    for(uint32_t n=1; n <= 6; n++)
    {
		const uint32_t nIdx = (~(outChildId ^ childId)) & n;
        outNeighbours[n - 1] = ((nIdx != 0) 
                                    ? parentNeighbours[nIdx - 1] 
                                    : parentChildrenIndex
                               ) + (n ^ childId);
    }
}

inline void getNeighboursVectorInUniformGrid(uint32_t outChildId, glm::ivec3 currentPos, uint32_t gridSize, std::array<uint32_t, 6>& outNeighbours)
{
    for(uint32_t n=1; n <= 6; n++)
    {
        const glm::ivec3 nPos = 
                currentPos +
                glm::ivec3(
                    (n & 0b0001) ? ((outChildId & 0b0001) ? 1 : -1) : 0,
                    (n & 0b0010) ? ((outChildId & 0b0010) ? 1 : -1) : 0,
                    (n & 0b0100) ? ((outChildId & 0b0100) ? 1 : -1) : 0
                );

        if(nPos.x >= 0 && nPos.x < gridSize &&
           nPos.y >= 0 && nPos.y < gridSize &&
           nPos.z >= 0 && nPos.z < gridSize)
        {
            outNeighbours[n - 1] = nPos.z * gridSize * gridSize + nPos.y * gridSize + nPos.x;
        }
        else
        {
            outNeighbours[n - 1] = 1 << 30;
        }
    }
}

void OctreeSdf::initOctreeWithContinuity(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                              float terminationThreshold, OctreeSdf::TerminationRule terminationRule)
{
    const float sqTerminationThreshold = terminationThreshold;

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
        0b00000000000000010000, // [_,-,+]        
        0b00000100000000000000, // [_,+,-]
        0b00000000000000000001  // [_,+,+]
    };

    // Create the grid
    {
        const uint32_t voxlesPerAxis = 1 << startDepth;
        mOctreeData.resize(voxlesPerAxis * voxlesPerAxis * voxlesPerAxis);
    }

    uint8_t currentBuffer = 0;
    uint8_t nextBuffer = 1;
    std::array<std::vector<NodeInfo1>, 3> nodesBuffer;
	nodesBuffer.fill(std::vector<NodeInfo1>());

    const uint32_t numTriangles = trianglesData.size();
    std::vector<uint32_t> startTriangles(numTriangles);
    for(uint32_t i=0; i < numTriangles; i++)
    {
        startTriangles[i] = i;
    }

    {
        const uint32_t voxlesPerAxis = 1 << startDepth;
        const uint32_t numVoxels = voxlesPerAxis * voxlesPerAxis * voxlesPerAxis;
        mOctreeData.resize(voxlesPerAxis * voxlesPerAxis * voxlesPerAxis);
    }

    // Create first start nodes
    {        
        float newSize = 0.5f * mBox.getSize().x * glm::pow(0.5f, startDepth);
        const glm::vec3 startCenter = mBox.min + newSize;
        const uint32_t voxelsPerAxis = 1 << startOctreeDepth;

        std::vector<NodeInfo1>& nodes = nodesBuffer[currentBuffer];

        for(uint32_t k=0; k < voxelsPerAxis; k++)
        {
            for(uint32_t j=0; j < voxelsPerAxis; j++)
            {
                for(uint32_t i=0; i < voxelsPerAxis; i++)
                {
                    nodes.push_back(NodeInfo1(k * mStartGridXY + j * mStartGridSize + i, 0, startCenter + glm::vec3(i, j, k) * 2.0f * newSize, newSize));
                    NodeInfo1& n = nodes.back();
                    n.parentTriangles = &startTriangles;

                    n.distanceToVertices.fill(INFINITY);
                    std::array<uint32_t, 8> minIndex;

                    for(uint32_t t=0; t < trianglesData.size(); t++)
                    {
                        for(uint32_t i=0; i < 8; i++)
                        {
                            const float dist = TriangleUtils::getSqDistPointAndTriangle(n.center + childrens[i] * n.size, trianglesData[t]);
                            if(dist < n.distanceToVertices[i])
                            {
                                minIndex[i] = t;
                                n.distanceToVertices[i] = dist;
                            }
                        }
                    }

                    for(uint32_t i=0; i < 8; i++)
                    {
                        n.distanceToVertices[i] = TriangleUtils::getSignedDistPointAndTriangle(n.center + childrens[i] * n.size, trianglesData[minIndex[i]]);
                    }
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

    for(uint32_t currentDepth=startOctreeDepth; currentDepth <= maxDepth; currentDepth++)
    {
        // Iter 1
        for(NodeInfo1& node : nodesBuffer[currentBuffer])
        {
            OctreeNode* octreeNode = &mOctreeData[node.parentChildrenIndex + node.childIndex];

            if(!node.isTerminalNode && currentDepth < maxDepth)
            {
                // Get triangles influencing the node
                float maxMinDist = 0.0f;
                for(uint32_t i=0; i < 8; i++)
                {
                    maxMinDist = glm::max(maxMinDist, glm::abs(node.distanceToVertices[i]));
                }

                for(const uint32_t& idx : *node.parentTriangles)
                {
                    triangle[0] = vertices[indices[3 * idx]] - node.center;
                    triangle[1] = vertices[indices[3 * idx + 1]] - node.center;
                    triangle[2] = vertices[indices[3 * idx + 2]] - node.center;

                    const float minDist = GJK::getMinDistance(glm::vec3(node.size), triangle);

                    if(minDist <= maxMinDist)
                    {
                        node.triangles.push_back(idx);
                    }
                }

                // Get current neighbours
                uint32_t samplesMask = 0;
                if(!node.isTerminalNode && currentDepth != startOctreeDepth)
                {
                    for(uint8_t neighbour = 1; neighbour <= 6; neighbour++)
                    {
                        if(((node.neighbourIndices[neighbour - 1]) >> 30) == 0) // Calculate next neigbour
                        {
							assert(!mOctreeData[node.neighbourIndices[neighbour - 1]].isLeaf());
                            node.neighbourIndices[neighbour - 1] = mOctreeData[node.neighbourIndices[neighbour - 1]].getChildrenIndex();

							if (mOctreeData[node.neighbourIndices[neighbour - 1]].isLeaf())
							{
								// Check if the values are correct
								node.neighbourIndices[neighbour - 1] = 1 << 31;
							}
                        }
                        
                        const uint32_t sign = ((((neighbour & node.childIndex) >> 2) & 0b0001) << ((neighbour & 0b0001) | ((neighbour & 0b0010) >> 1))) +
											  ((((neighbour & node.childIndex) >> 1) & 0b0001) << (neighbour & 0b0001)) +
											  (neighbour & node.childIndex & 0b0001);
                        samplesMask |= (node.neighbourIndices[neighbour - 1] >> 31)
                                        ? neigbourMasks[4 * (neighbour - 1) + sign]
                                        : 0;
                        
                    }
                }

                // Get distance to mid points to estimate integral
                {
                    node.distanceToMidPoints.fill(INFINITY);
                    std::array<uint32_t, 19> minIndex;

                    for(uint32_t i=0; i < 19; i++)
                    {
                        if(samplesMask & (1 << (18-i))) continue;
                        for(uint32_t t : node.triangles)
                        {
                            const float dist = TriangleUtils::getSqDistPointAndTriangle(node.center + nodeSamplePoints[i] * node.size, trianglesData[t]);
                            if(dist < node.distanceToMidPoints[i])
                            {
                                minIndex[i] = t;
                                node.distanceToMidPoints[i] = dist;
                            }
                        }
                    }

                    for(uint32_t i=0; i < 19; i++)
                    {
                        if(samplesMask & (1 << (18-i)))
                        {
                            node.distanceToMidPoints[i] = interpolateValue(reinterpret_cast<float*>(&node.distanceToVertices), 0.5f * nodeSamplePoints[i] + 0.5f);
                        }
                        else
                        {
                            node.distanceToMidPoints[i] = TriangleUtils::getSignedDistPointAndTriangle(node.center + nodeSamplePoints[i] * node.size, trianglesData[minIndex[i]]);
                        }
                    }
                }

                bool generateTerminalNodes = false;
                if(currentDepth >= startDepth)
                {
                    float value;
                    switch(terminationRule)
                    {
                        case TerminationRule::TRAPEZOIDAL_RULE:
                            value = estimateErrorFunctionIntegralByTrapezoidRule(node.distanceToVertices, node.distanceToMidPoints) / (8.0f * node.size * node.size * node.size);
                            break;
                        case TerminationRule::SIMPSONS_RULE:
                            value = estimateErrorFunctionIntegralBySimpsonsRule(node.distanceToVertices, node.distanceToMidPoints) / (8.0f * node.size * node.size * node.size);
                            break;
                        case TerminationRule::NONE:
                            value = INFINITY;
                            break;
                    }

                    generateTerminalNodes = value < sqTerminationThreshold;
                }

                // Generate new childrens
				const float newSize = 0.5f * node.size;

				uint32_t childIndex = mOctreeData.size();
				octreeNode->setValues(false, childIndex);

			    mOctreeData.resize(mOctreeData.size() + 8, 
                                    (generateTerminalNodes) ? OctreeNode::getLeafNode() : OctreeNode::getInnerNode());

                glm::ivec3 nodeStartGridPos;
                if(currentDepth == startDepth)
                    nodeStartGridPos = glm::floor((node.center - mBox.min) / mStartGridCellSize);

				// Low Z children
				nodesBuffer[nextBuffer].push_back(NodeInfo1(childIndex, 0, node.center + glm::vec3(-newSize, -newSize, -newSize), newSize, generateTerminalNodes));
				{
					NodeInfo1& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.distanceToVertices[0] = node.distanceToVertices[0]; child.distanceToVertices[1] = node.distanceToMidPoints[0]; 
                    child.distanceToVertices[2] = node.distanceToMidPoints[1]; child.distanceToVertices[3] = node.distanceToMidPoints[2];
					child.distanceToVertices[4] = node.distanceToMidPoints[5]; child.distanceToVertices[5] = node.distanceToMidPoints[6];
					child.distanceToVertices[6] = node.distanceToMidPoints[8]; child.distanceToVertices[7] = node.distanceToMidPoints[9];
                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(0, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(0, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
				}

				nodesBuffer[nextBuffer].push_back(NodeInfo1(childIndex, 1, node.center + glm::vec3(newSize, -newSize, -newSize), newSize, generateTerminalNodes));
				{
					NodeInfo1& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.distanceToVertices[0] = node.distanceToMidPoints[0]; child.distanceToVertices[1] = node.distanceToVertices[1];
					child.distanceToVertices[2] = node.distanceToMidPoints[2]; child.distanceToVertices[3] = node.distanceToMidPoints[3];
					child.distanceToVertices[4] = node.distanceToMidPoints[6]; child.distanceToVertices[5] = node.distanceToMidPoints[7];
					child.distanceToVertices[6] = node.distanceToMidPoints[9]; child.distanceToVertices[7] = node.distanceToMidPoints[10];
                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(1, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(1, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
				}

                nodesBuffer[nextBuffer].push_back(NodeInfo1(childIndex, 2, node.center + glm::vec3(-newSize, newSize, -newSize), newSize, generateTerminalNodes));
				{
					NodeInfo1& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.distanceToVertices[0] = node.distanceToMidPoints[1]; child.distanceToVertices[1] = node.distanceToMidPoints[2];
					child.distanceToVertices[2] = node.distanceToVertices[2]; child.distanceToVertices[3] = node.distanceToMidPoints[4];
					child.distanceToVertices[4] = node.distanceToMidPoints[8]; child.distanceToVertices[5] = node.distanceToMidPoints[9];
					child.distanceToVertices[6] = node.distanceToMidPoints[11]; child.distanceToVertices[7] = node.distanceToMidPoints[12];
                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(2, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(2, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
				}

                nodesBuffer[nextBuffer].push_back(NodeInfo1(childIndex, 3, node.center + glm::vec3(newSize, newSize, -newSize), newSize, generateTerminalNodes));
				{
					NodeInfo1& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.distanceToVertices[0] = node.distanceToMidPoints[2]; child.distanceToVertices[1] = node.distanceToMidPoints[3];
					child.distanceToVertices[2] = node.distanceToMidPoints[4]; child.distanceToVertices[3] = node.distanceToVertices[3];
					child.distanceToVertices[4] = node.distanceToMidPoints[9]; child.distanceToVertices[5] = node.distanceToMidPoints[10];
					child.distanceToVertices[6] = node.distanceToMidPoints[12]; child.distanceToVertices[7] = node.distanceToMidPoints[13];
                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(3, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(3, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
				}

                // High Z children
                nodesBuffer[nextBuffer].push_back(NodeInfo1(childIndex, 4, node.center + glm::vec3(-newSize, -newSize, newSize), newSize, generateTerminalNodes));
				{
					NodeInfo1& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.distanceToVertices[0] = node.distanceToMidPoints[5]; child.distanceToVertices[1] = node.distanceToMidPoints[6];
					child.distanceToVertices[2] = node.distanceToMidPoints[8]; child.distanceToVertices[3] = node.distanceToMidPoints[9];
					child.distanceToVertices[4] = node.distanceToVertices[4]; child.distanceToVertices[5] = node.distanceToMidPoints[14];
					child.distanceToVertices[6] = node.distanceToMidPoints[15]; child.distanceToVertices[7] = node.distanceToMidPoints[16];
                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(4, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(4, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
				}

                nodesBuffer[nextBuffer].push_back(NodeInfo1(childIndex, 5, node.center + glm::vec3(newSize, -newSize, newSize), newSize, generateTerminalNodes));
				{
					NodeInfo1& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.distanceToVertices[0] = node.distanceToMidPoints[6]; child.distanceToVertices[1] = node.distanceToMidPoints[7];
					child.distanceToVertices[2] = node.distanceToMidPoints[9]; child.distanceToVertices[3] = node.distanceToMidPoints[10];
					child.distanceToVertices[4] = node.distanceToMidPoints[14]; child.distanceToVertices[5] = node.distanceToVertices[5];
					child.distanceToVertices[6] = node.distanceToMidPoints[16]; child.distanceToVertices[7] = node.distanceToMidPoints[17];
                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(5, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(5, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
				}

                nodesBuffer[nextBuffer].push_back(NodeInfo1(childIndex, 6, node.center + glm::vec3(-newSize, newSize, newSize), newSize, generateTerminalNodes));
				{
					NodeInfo1& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.distanceToVertices[0] = node.distanceToMidPoints[8]; child.distanceToVertices[1] = node.distanceToMidPoints[9];
					child.distanceToVertices[2] = node.distanceToMidPoints[11]; child.distanceToVertices[3] = node.distanceToMidPoints[12];
					child.distanceToVertices[4] = node.distanceToMidPoints[15]; child.distanceToVertices[5] = node.distanceToMidPoints[16];
					child.distanceToVertices[6] = node.distanceToVertices[6]; child.distanceToVertices[7] = node.distanceToMidPoints[18];
                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(6, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(6, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
				}

                nodesBuffer[nextBuffer].push_back(NodeInfo1(childIndex, 7, node.center + glm::vec3(newSize, newSize, newSize), newSize, generateTerminalNodes));
				{
					NodeInfo1& child = nodesBuffer[nextBuffer].back();
                    child.parentTriangles = &node.triangles;
					child.distanceToVertices[0] = node.distanceToMidPoints[9]; child.distanceToVertices[1] = node.distanceToMidPoints[10];
					child.distanceToVertices[2] = node.distanceToMidPoints[12]; child.distanceToVertices[3] = node.distanceToMidPoints[13];
					child.distanceToVertices[4] = node.distanceToMidPoints[16]; child.distanceToVertices[5] = node.distanceToMidPoints[17];
					child.distanceToVertices[6] = node.distanceToMidPoints[18]; child.distanceToVertices[7] = node.distanceToVertices[7];
                    if(currentDepth == startDepth) getNeighboursVectorInUniformGrid(7, nodeStartGridPos, mStartGridSize, child.neighbourIndices);
                    else getNeighboursVector(7, node.childIndex, node.parentChildrenIndex, node.neighbourIndices, child.neighbourIndices);
				}
            }
			else
			{
				uint32_t childIndex = mOctreeData.size();
				octreeNode->setValues(true, childIndex);

				mOctreeData.resize(mOctreeData.size() + 8);

				for (uint32_t i = 0; i < 8; i++)
				{
					mOctreeData[childIndex + i].value = node.distanceToVertices[i];
					mValueRange = glm::max(mValueRange, glm::abs(node.distanceToVertices[i]));
				}
            }
        }

        // Swap buffers
        currentBuffer = (currentBuffer + 1) % 3;
        nextBuffer = (nextBuffer + 1) % 3;
        nodesBuffer[nextBuffer].clear();
    }
}