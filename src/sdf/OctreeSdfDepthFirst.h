#ifndef OCTREE_SDF_DEPTH_FIRST_H
#define OCTREE_SDF_DEPTH_FIRST_H

#include "OctreeSdf.h"
#include "utils/Timer.h"
#include "utils/GJK.h"
#include "OctreeSdfUtils.h"
#include <array>
#include <stack>

template<typename VertexInfo, int VALUES_PER_VERTEX>
struct DepthFirstNodeInfo
{
    DepthFirstNodeInfo(uint32_t nodeIndex, uint16_t depth, glm::vec3 center, float size, bool isTerminalNode = false)
                        : nodeIndex(nodeIndex), depth(depth), center(center), size(size), isTerminalNode(isTerminalNode) {}
    uint32_t nodeIndex;
    uint16_t depth;
    glm::vec3 center;
    float size;
    bool isTerminalNode;
    std::array<std::array<float, VALUES_PER_VERTEX>, 8> verticesValues;
    std::array<VertexInfo, 8> verticesInfo;
};


template<typename TrianglesInfluenceStrategy>
void OctreeSdf::initOctree(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                           float terminationThreshold, OctreeSdf::TerminationRule terminationRule)
{
    typedef TrianglesInfluenceStrategy::InterpolationMethod InterpolationMethod;
    typedef DepthFirstNodeInfo<TrianglesInfluenceStrategy::VertexInfo, InterpolationMethod::VALUES_PER_VERTEX> NodeInfo;

    const float sqTerminationThreshold = terminationThreshold * terminationThreshold;

    std::vector<TriangleUtils::TriangleData> trianglesData(TriangleUtils::calculateMeshTriangleData(mesh));
    
    const uint32_t startOctreeDepth = glm::min(startDepth, START_OCTREE_DEPTH);

    const uint32_t numTriangles = trianglesData.size();
    std::vector<std::vector<uint32_t>> triangles(maxDepth - startOctreeDepth + 1);
	triangles[0].resize(numTriangles);
    for(uint32_t i=0; i < numTriangles; i++)
    {
        triangles[0][i] = i;
    }

    TrianglesInfluenceStrategy trianglesInfluence;

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
                    nodes.push(NodeInfo(std::numeric_limits<uint32_t>::max(), startOctreeDepth, startCenter + glm::vec3(i, j, k) * 2.0f * newSize, newSize));
                    NodeInfo& n = nodes.top();
					std::array<float, InterpolationMethod::NUM_COEFFICIENTS> nullArray;
                    trianglesInfluence.calculateVerticesInfo(n.center, n.size, triangles[0], childrens,
                                                             0u, nullArray,
                                                             n.verticesValues, n.verticesInfo,
                                                             mesh, trianglesData);
                }
            }
        }
    }

    mValueRange = 0.0f;

    std::array<glm::vec3, 3> triangle;

    const std::vector<glm::vec3>& vertices = mesh.getVertices();
    const std::vector<uint32_t>& indices = mesh.getIndices();

#ifdef PRINT_STATISTICS
    std::vector<std::pair<uint32_t, uint32_t>> verticesStatistics(maxDepth, std::make_pair(0, 0));
    verticesStatistics[0] = std::make_pair(trianglesData.size(), 1);
    std::vector<float> elapsedTime(maxDepth);
    std::vector<uint32_t> numTrianglesEvaluated(maxDepth, 0);
    Timer timer;
#endif

    while(!nodes.empty())
    {
        #ifdef PRINT_STATISTICS
        timer.start();
        #endif
        const NodeInfo node = nodes.top();
        nodes.pop();

        OctreeNode* octreeNode = (node.nodeIndex < std::numeric_limits<uint32_t>::max()) 
                                    ? &mOctreeData[node.nodeIndex]
                                    : nullptr;

        if(node.depth == startDepth)
        {
            assert(octreeNode == nullptr);
            glm::ivec3 startArrayPos = glm::floor((node.center - mBox.min) / mStartGridCellSize);
            octreeNode = &mOctreeData[startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x];
        }

        const uint32_t rDepth = node.depth - startOctreeDepth + 1;
        std::array<float, InterpolationMethod::NUM_COEFFICIENTS> interpolationCoeff;

        if(!node.isTerminalNode && node.depth < maxDepth)
        {
            trianglesInfluence.filterTriangles(node.center, node.size, triangles[rDepth-1], 
                                               triangles[rDepth], node.verticesValues, node.verticesInfo,
                                               mesh, trianglesData);

            std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, 19> midPointsValues;
            std::array<TrianglesInfluenceStrategy::VertexInfo, 19> pointsInfo;

            trianglesInfluence.calculateVerticesInfo(node.center, node.size, triangles[rDepth], nodeSamplePoints,
                                                     0u, interpolationCoeff,
                                                     midPointsValues, pointsInfo,
                                                     mesh, trianglesData);
            
            bool generateTerminalNodes = false;
            if(node.depth >= startDepth)
            {
                InterpolationMethod::calculateCoefficients(node.verticesValues, 2.0f * node.size, triangles[rDepth], mesh, trianglesData, interpolationCoeff);
                float value;
                switch(terminationRule)
                {
                    case TerminationRule::TRAPEZOIDAL_RULE:
                        value = estimateErrorFunctionIntegralByTrapezoidRule<InterpolationMethod>(interpolationCoeff, midPointsValues);
                        break;
                    case TerminationRule::SIMPSONS_RULE:
                        value = estimateErrorFunctionIntegralBySimpsonsRule<InterpolationMethod>(interpolationCoeff, midPointsValues);
                        break;
                    case TerminationRule::NONE:
                        value = INFINITY;
                        break;
                }

                generateTerminalNodes = value < sqTerminationThreshold;
            }

			if(DELAY_NODE_TERMINATION || !generateTerminalNodes) 
            {
				// Generate new childrens
				const float newSize = 0.5f * node.size;

				uint32_t childIndex = (node.depth >= startDepth) ? mOctreeData.size() : std::numeric_limits<uint32_t>::max();
                uint32_t childOffsetMask = (node.depth >= startDepth) ? ~0 : 0;
				if(octreeNode != nullptr) octreeNode->setValues(false, childIndex);

				if(node.depth >= startDepth) mOctreeData.resize(mOctreeData.size() + 8);

				// Low Z children
				nodes.push(NodeInfo(childIndex, node.depth + 1, node.center + glm::vec3(-newSize, -newSize, -newSize), newSize, generateTerminalNodes));
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

				nodes.push(NodeInfo(childIndex + (childOffsetMask & 1), node.depth + 1, node.center + glm::vec3(newSize, -newSize, -newSize), newSize, generateTerminalNodes));
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

                nodes.push(NodeInfo(childIndex + (childOffsetMask & 2), node.depth + 1, node.center + glm::vec3(-newSize, newSize, -newSize), newSize, generateTerminalNodes));
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

                nodes.push(NodeInfo(childIndex + (childOffsetMask & 3), node.depth + 1, node.center + glm::vec3(newSize, newSize, -newSize), newSize, generateTerminalNodes));
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
                nodes.push(NodeInfo(childIndex + (childOffsetMask & 4), node.depth + 1, node.center + glm::vec3(-newSize, -newSize, newSize), newSize, generateTerminalNodes));
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

                nodes.push(NodeInfo(childIndex + (childOffsetMask & 5), node.depth + 1, node.center + glm::vec3(newSize, -newSize, newSize), newSize, generateTerminalNodes));
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

                nodes.push(NodeInfo(childIndex + (childOffsetMask & 6), node.depth + 1, node.center + glm::vec3(-newSize, newSize, newSize), newSize, generateTerminalNodes));
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

                nodes.push(NodeInfo(childIndex + (childOffsetMask & 7), node.depth + 1, node.center + glm::vec3(newSize, newSize, newSize), newSize, generateTerminalNodes));
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
                uint32_t childIndex = mOctreeData.size();
                assert(octreeNode != nullptr);
                octreeNode->setValues(true, childIndex);

                InterpolationMethod::calculateCoefficients(node.verticesValues, 2.0f * node.size, triangles[rDepth], mesh, trianglesData, interpolationCoeff);
                mOctreeData.resize(mOctreeData.size() + InterpolationMethod::NUM_COEFFICIENTS);

                for(uint32_t i=0; i < InterpolationMethod::NUM_COEFFICIENTS; i++)
                {
                    mOctreeData[childIndex + i].value = interpolationCoeff[i];
                }

                for(uint32_t i=0; i < 8; i++)
                {
                    mValueRange = glm::max(mValueRange, glm::abs(node.verticesValues[i][0]));
                }
            }

#ifdef PRINT_STATISTICS
            {
                verticesStatistics[node.depth].first += triangles[rDepth].size();
                verticesStatistics[node.depth].second += 1;
                elapsedTime[node.depth] += timer.getElapsedSeconds();
                numTrianglesEvaluated[node.depth] += triangles[rDepth-1].size();
            }
#endif
        }
        else
        {
            assert(node.depth >= startDepth);
            uint32_t childIndex = mOctreeData.size();
            assert(octreeNode != nullptr);
            octreeNode->setValues(true, childIndex);

            InterpolationMethod::calculateCoefficients(node.verticesValues, 2.0f * node.size, triangles[rDepth-1], mesh, trianglesData, interpolationCoeff);
            mOctreeData.resize(mOctreeData.size() + InterpolationMethod::NUM_COEFFICIENTS);

            for(uint32_t i=0; i < InterpolationMethod::NUM_COEFFICIENTS; i++)
            {
                mOctreeData[childIndex + i].value = interpolationCoeff[i];
            }

            for(uint32_t i=0; i < 8; i++)
            {
                mValueRange = glm::max(mValueRange, glm::abs(node.verticesValues[i][0]));
            }
        }
    }
	
#ifdef PRINT_STATISTICS
    SPDLOG_INFO("Used an octree of max depth {}", maxDepth);
    for(uint32_t d=0; d < maxDepth; d++)
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
    }

    trianglesInfluence.printStatistics();
#endif
}

#endif