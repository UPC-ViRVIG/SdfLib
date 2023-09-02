#ifndef OCTREE_SDF_DEPTH_FIRST_H
#define OCTREE_SDF_DEPTH_FIRST_H

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
                           float terminationThreshold, OctreeSdf::TerminationRule terminationRule,
                           uint32_t numThreads)
{
    typedef typename TrianglesInfluenceStrategy::InterpolationMethod InterpolationMethod;
    typedef DepthFirstNodeInfo<typename TrianglesInfluenceStrategy::VertexInfo, InterpolationMethod::VALUES_PER_VERTEX> NodeInfo;

    terminationThreshold *= glm::length(mesh.getBoundingBox().getSize());

    struct ThreadContext
    {
        std::vector<std::vector<uint32_t>> triangles;
        TrianglesInfluenceStrategy trianglesInfluence;
        std::stack<NodeInfo> nodesStack;
        uint32_t startDepth;
        uint32_t startOctreeDepth;
        uint32_t maxDepth;
        OctreeSdf::TerminationRule terminationRule;
        float sqTerminationThreshold;
        float valueRange;
        uint32_t padding[16];

#ifdef SDFLIB_PRINT_STATISTICS
        std::vector<std::pair<uint32_t, uint32_t>> verticesStatistics;
        std::vector<uint32_t> notEndedNodes;
        std::vector<float> elapsedTime;
        std::vector<uint32_t> numTrianglesEvaluated;
        Timer timer;
#endif
    };

    std::vector<TriangleUtils::TriangleData> trianglesData(TriangleUtils::calculateMeshTriangleData(mesh));
    const uint32_t startOctreeDepth = glm::min(startDepth, START_OCTREE_DEPTH);

    ThreadContext mainThread;
    mainThread.triangles.resize(maxDepth - startOctreeDepth + 1);
    mainThread.trianglesInfluence.initCaches(mBox, maxDepth);
    mainThread.startDepth = startDepth;
    mainThread.startOctreeDepth = startOctreeDepth;
    mainThread.maxDepth = maxDepth;
    mainThread.terminationRule = terminationRule;
    mainThread.sqTerminationThreshold = terminationThreshold * terminationThreshold;
    mainThread.valueRange = 0.0f;

    #ifdef SDFLIB_PRINT_STATISTICS
        mainThread.verticesStatistics.resize(maxDepth, std::make_pair(0, 0));
        mainThread.verticesStatistics[0] = std::make_pair(trianglesData.size(), 1);
        mainThread.notEndedNodes.resize(maxDepth + 1, 0);
        mainThread.elapsedTime.resize(maxDepth);
        mainThread.numTrianglesEvaluated.resize(maxDepth, 0);
    #endif

    const uint32_t numTriangles = trianglesData.size();
	mainThread.triangles[0].resize(numTriangles);
    {
        uint32_t triIndex = 0;
        for(uint32_t t=0; t < numTriangles; t++)
        {
            if(glm::dot(trianglesData[t].getTriangleNormal(), trianglesData[t].getTriangleNormal()) > 1e-3f)
                mainThread.triangles[0][triIndex++] = t;
        }
        mainThread.triangles[0].resize(triIndex);
    }

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
                    nodes.push(NodeInfo(std::numeric_limits<uint32_t>::max(), startOctreeDepth, startCenter + glm::vec3(i, j, k) * 2.0f * newSize, newSize));
                    NodeInfo& n = nodes.top();
					std::array<float, InterpolationMethod::NUM_COEFFICIENTS> nullArray;
                    mainThread.trianglesInfluence.calculateVerticesInfo(n.center, n.size, mainThread.triangles[0], childrens,
                                                             0u, nullArray,
                                                             n.verticesValues, n.verticesInfo,
                                                             mesh, trianglesData);
                }
            }
        }
    }

    auto processNode = [&mesh, &trianglesData] (const NodeInfo& node, ThreadContext& tContext, std::vector<OctreeNode>& outputOctree)
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
        std::array<float, InterpolationMethod::NUM_COEFFICIENTS> interpolationCoeff;

        if(!node.isTerminalNode && node.depth < tContext.maxDepth)
        {
            tContext.trianglesInfluence.filterTriangles(node.center, node.size, tContext.triangles[rDepth-1], 
                                               tContext.triangles[rDepth], node.verticesValues, node.verticesInfo,
                                               mesh, trianglesData);

            std::array<std::array<float, InterpolationMethod::VALUES_PER_VERTEX>, 19> midPointsValues;
            std::array<typename TrianglesInfluenceStrategy::VertexInfo, 19> pointsInfo;

            tContext.trianglesInfluence.calculateVerticesInfo(node.center, node.size, tContext.triangles[rDepth], nodeSamplePoints,
                                                     0u, interpolationCoeff,
                                                     midPointsValues, pointsInfo,
                                                     mesh, trianglesData);
            
            bool generateTerminalNodes = false;
            if(node.depth >= tContext.startDepth)
            {
                InterpolationMethod::calculateCoefficients(node.verticesValues, 2.0f * node.size, tContext.triangles[rDepth], mesh, trianglesData, interpolationCoeff);
                float value;
                switch(tContext.terminationRule)
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

                generateTerminalNodes = value < tContext.sqTerminationThreshold;
            }

			if(DELAY_NODE_TERMINATION || !generateTerminalNodes) 
            {
				// Generate new childrens
				const float newSize = 0.5f * node.size;

				uint32_t childIndex = (node.depth >= tContext.startDepth) ? outputOctree.size() : std::numeric_limits<uint32_t>::max();
                uint32_t childOffsetMask = (node.depth >= tContext.startDepth) ? ~0 : 0;
				if(octreeNode != nullptr) octreeNode->setValues(false, childIndex);

				if(node.depth >= tContext.startDepth) outputOctree.resize(outputOctree.size() + 8);

				// Low Z children
				tContext.nodesStack.push(NodeInfo(childIndex, node.depth + 1, node.center + glm::vec3(-newSize, -newSize, -newSize), newSize, generateTerminalNodes));
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

				tContext.nodesStack.push(NodeInfo(childIndex + (childOffsetMask & 1), node.depth + 1, node.center + glm::vec3(newSize, -newSize, -newSize), newSize, generateTerminalNodes));
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

                tContext.nodesStack.push(NodeInfo(childIndex + (childOffsetMask & 2), node.depth + 1, node.center + glm::vec3(-newSize, newSize, -newSize), newSize, generateTerminalNodes));
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

                tContext.nodesStack.push(NodeInfo(childIndex + (childOffsetMask & 3), node.depth + 1, node.center + glm::vec3(newSize, newSize, -newSize), newSize, generateTerminalNodes));
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
                tContext.nodesStack.push(NodeInfo(childIndex + (childOffsetMask & 4), node.depth + 1, node.center + glm::vec3(-newSize, -newSize, newSize), newSize, generateTerminalNodes));
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

                tContext.nodesStack.push(NodeInfo(childIndex + (childOffsetMask & 5), node.depth + 1, node.center + glm::vec3(newSize, -newSize, newSize), newSize, generateTerminalNodes));
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

                tContext.nodesStack.push(NodeInfo(childIndex + (childOffsetMask & 6), node.depth + 1, node.center + glm::vec3(-newSize, newSize, newSize), newSize, generateTerminalNodes));
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

                tContext.nodesStack.push(NodeInfo(childIndex + (childOffsetMask & 7), node.depth + 1, node.center + glm::vec3(newSize, newSize, newSize), newSize, generateTerminalNodes));
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
                #ifdef SDFLIB_PRINT_STATISTICS
                    tContext.verticesStatistics[node.depth].first += tContext.triangles[rDepth].size();
                    tContext.verticesStatistics[node.depth].second += 1;
                    tContext.notEndedNodes[node.depth]++;
                #endif
            }
            else
            {
                assert(node.depth >= tContext.startDepth);
                uint32_t childIndex = outputOctree.size();
                assert(octreeNode != nullptr);
                octreeNode->setValues(true, childIndex);

                InterpolationMethod::calculateCoefficients(node.verticesValues, 2.0f * node.size, tContext.triangles[rDepth], mesh, trianglesData, interpolationCoeff);
                outputOctree.resize(outputOctree.size() + InterpolationMethod::NUM_COEFFICIENTS);

                for(uint32_t i=0; i < InterpolationMethod::NUM_COEFFICIENTS; i++)
                {
                    outputOctree[childIndex + i].value = interpolationCoeff[i];
                }

                for(uint32_t i=0; i < 8; i++)
                {
                    tContext.valueRange = glm::max(tContext.valueRange, glm::abs(node.verticesValues[i][0]));
                }
            }

#ifdef SDFLIB_PRINT_STATISTICS
            {
                tContext.elapsedTime[node.depth] += tContext.timer.getElapsedSeconds();
                tContext.numTrianglesEvaluated[node.depth] += tContext.triangles[rDepth-1].size();
            }
#endif
        }
        else
        {
            assert(node.depth >= tContext.startDepth);
            uint32_t childIndex = outputOctree.size();
            assert(octreeNode != nullptr);
            octreeNode->setValues(true, childIndex);

            InterpolationMethod::calculateCoefficients(node.verticesValues, 2.0f * node.size, tContext.triangles[rDepth-1], mesh, trianglesData, interpolationCoeff);
            outputOctree.resize(outputOctree.size() + InterpolationMethod::NUM_COEFFICIENTS);

            for(uint32_t i=0; i < InterpolationMethod::NUM_COEFFICIENTS; i++)
            {
                outputOctree[childIndex + i].value = interpolationCoeff[i];
            }

            for(uint32_t i=0; i < 8; i++)
            {
                tContext.valueRange = glm::max(tContext.valueRange, glm::abs(node.verticesValues[i][0]));
            }
        }
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
            mainThread.nodesStack.pop();
            
            if(node.depth == startDepth)
            {
                glm::ivec3 startArrayPos = glm::floor((node.center - mBox.min) / mStartGridCellSize);
                node.nodeIndex = startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x;
            }

            processNode(node, mainThread, mOctreeData);
        }

        mValueRange = mainThread.valueRange;
    }
#ifdef OPENMP_AVAILABLE
    else 
    {
        std::vector<ThreadContext> threadsContext(numThreads, mainThread);

        struct OctreeDataWithPadding
        {
            OctreeDataWithPadding() {}
            std::vector<OctreeNode> octreeData;
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
            mainThread.nodesStack.pop();
            
            if(node.depth == startDepth)
            {
                glm::ivec3 startArrayPos = glm::floor((node.center - mBox.min) / mStartGridCellSize);
                node.nodeIndex = 0;
                std::vector<OctreeNode>* subOctreePtr = &subOctrees[startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x].octreeData;
                subOctreePtr->resize(1);
                const uint32_t rDepth = startDepth - mainThread.startOctreeDepth;
                std::vector<uint32_t> startTriangles = mainThread.triangles[rDepth];
                #pragma omp task shared(threadsContext) firstprivate(node, subOctreePtr, rDepth, startTriangles)
                {
                    std::vector<OctreeNode>& subOctree = *subOctreePtr;
                    const uint32_t tId = omp_get_thread_num();
                    ThreadContext& threadContext = threadsContext[tId];
                    threadContext.triangles[rDepth] = std::move(startTriangles);
                    threadContext.nodesStack = std::stack<NodeInfo>(); // Reset stack
                    threadContext.nodesStack.push(node);
                    while(!threadContext.nodesStack.empty())
                    {
                        NodeInfo node1 = threadContext.nodesStack.top();
                        threadContext.nodesStack.pop();

                        processNode(node1, threadContext, subOctree);
                    }
                }
            }
            else
            {
                processNode(node, mainThread, mOctreeData);
            }
        }

        // Merge all the subtrees
        mOctreeData.resize(voxlesPerAxis * voxlesPerAxis * voxlesPerAxis);
        for(uint32_t i=0; i < subOctrees.size(); i++)
        {
            std::vector<OctreeNode>& octreeData = subOctrees[i].octreeData;

            const uint32_t startIndex = mOctreeData.size();
            
            // Add start index to the subtree
            std::function<void(OctreeNode&)> vistNode;
            vistNode = [&](OctreeNode& node)
            {
                // Iterate children
                if(!node.isLeaf())
                {
                    for(uint32_t i = 0; i < 8; i++)
                    {
                        vistNode(octreeData[node.getChildrenIndex() + i]);
                    }
                }

                // Update node index
                node.setValues(node.isLeaf(), node.getChildrenIndex() + startIndex - 1);
            };

            vistNode(octreeData[0]);

            // Move the fist node to the correct start grid position
            mOctreeData[i] = octreeData[0];

            // Copy to final array
            mOctreeData.insert(mOctreeData.end(), octreeData.begin()+1, octreeData.end());
        }

        mValueRange = 0.0f;
        for(ThreadContext& tCtx : threadsContext)
        {
            mValueRange = glm::max(mValueRange, tCtx.valueRange);
        }

        #ifdef SDFLIB_PRINT_STATISTICS
            for(ThreadContext& tCtx : threadsContext)
            {
                for(uint32_t d=0; d < maxDepth; d++)
                {
                    mainThread.verticesStatistics[d].first += tCtx.verticesStatistics[d].first;
                    mainThread.verticesStatistics[d].second += tCtx.verticesStatistics[d].second;

                    mainThread.elapsedTime[d] += tCtx.elapsedTime[d];

                    mainThread.numTrianglesEvaluated[d] += tCtx.numTrianglesEvaluated[d];
                    mainThread.notEndedNodes[d] += tCtx.notEndedNodes[d];
                }
            }
        #endif
    }
#endif
	
#ifdef SDFLIB_PRINT_STATISTICS
    SPDLOG_INFO("Used an octree of max depth {}", maxDepth);
    for(uint32_t d=0; d < maxDepth; d++)
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

        SPDLOG_INFO("Depth {}, not ended nodes {}", d, 8 * mainThread.notEndedNodes[d]);
        if(d > 1)
        {
            SPDLOG_INFO("Depth {}, ended nodes {}", d, 8 * mainThread.notEndedNodes[d-1] - mainThread.notEndedNodes[d]);
        }
    }

    mainThread.trianglesInfluence.printStatistics();
#endif
}
}

#endif
