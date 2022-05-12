#ifndef EXACT_OCTREE_SDF_DEPTH_FIRST_H
#define EXACT_OCTREE_SDF_DEPTH_FIRST_H

template<typename VertexInfo>
struct DepthFirstNodeInfoExactOctree
{
	DepthFirstNodeInfoExactOctree(uint32_t nodeIndex, uint16_t depth, glm::vec3 center, float size)
                        : nodeIndex(nodeIndex), depth(depth), center(center), size(size) {}
    uint32_t nodeIndex;
    uint16_t depth;
    glm::vec3 center;
    float size;
    std::array<float, 8> distanceToVertices;
    std::array<VertexInfo, 8> verticesInfo;
};

template<typename TrianglesInfluenceStrategy>
void ExactOctreeSdf::initOctree(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                                uint32_t minTrianglesPerNode)
{
    typedef DepthFirstNodeInfoExactOctree<TrianglesInfluenceStrategy::VertexInfo> NodeInfo;

    std::vector<TriangleUtils::TriangleData>& trianglesData = mTrianglesData;
    
    const uint32_t startOctreeDepth = glm::min(startDepth, START_OCTREE_DEPTH);

    const uint32_t numTriangles = trianglesData.size();
    std::vector<std::vector<uint32_t>> triangles(maxDepth - startOctreeDepth + 2);
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
                    trianglesInfluence.calculateVerticesInfo(n.center, n.size, triangles[0], childrens,
                                                             0u, n.distanceToVertices,
                                                             n.distanceToVertices, n.verticesInfo,
                                                             mesh, trianglesData);
                }
            }
        }
    }

    std::array<glm::vec3, 3> triangle;

    const std::vector<glm::vec3>& vertices = mesh.getVertices();
    const std::vector<uint32_t>& indices = mesh.getIndices();

#ifdef PRINT_STATISTICS
    std::vector<std::pair<uint32_t, uint32_t>> verticesStatistics(maxDepth + 1, std::make_pair(0, 0));
    verticesStatistics[0] = std::make_pair(trianglesData.size(), 1);
    std::vector<uint32_t> endedNodes(maxDepth + 1, 0);
    std::vector<float> elapsedTime(maxDepth + 1, 0.0f);
    std::vector<uint32_t> numTrianglesEvaluated(maxDepth + 1, 0);
    uint32_t numTrianglesInLeafs = 0;
    uint32_t numLeafs = 0;
    uint32_t maxTrianglesInLeaf = 0;
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

        trianglesInfluence.filterTriangles(node.center, node.size, triangles[rDepth-1], 
                                            triangles[rDepth], node.distanceToVertices, node.verticesInfo,
                                            mesh, trianglesData);

        std::array<float, 19> minDistToPoints;
        std::array<TrianglesInfluenceStrategy::VertexInfo, 19> pointsInfo;

        trianglesInfluence.calculateVerticesInfo(node.center, node.size, triangles[rDepth], nodeSamplePoints,
                                                    0u, node.distanceToVertices,
                                                    minDistToPoints, pointsInfo,
                                                    mesh, trianglesData);
        
        bool isTerminalNode = false;
        if(node.depth >= startDepth)
        {
            isTerminalNode = triangles[rDepth].size() <= minTrianglesPerNode;
        }

        if(!isTerminalNode && node.depth < maxDepth)
        {
            // Generate new childrens
            const float newSize = 0.5f * node.size;

            uint32_t childIndex = (node.depth >= startDepth) ? mOctreeData.size() : std::numeric_limits<uint32_t>::max();
            uint32_t childOffsetMask = (node.depth >= startDepth) ? ~0 : 0;
            if(octreeNode != nullptr) octreeNode->setValues(false, childIndex);

            if(node.depth >= startDepth) mOctreeData.resize(mOctreeData.size() + 8);

            // Low Z children
            nodes.push(NodeInfo(childIndex, node.depth + 1, node.center + glm::vec3(-newSize, -newSize, -newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.distanceToVertices[0] = node.distanceToVertices[0]; child.distanceToVertices[1] = minDistToPoints[0]; 
                child.distanceToVertices[2] = minDistToPoints[1]; child.distanceToVertices[3] = minDistToPoints[2];
                child.distanceToVertices[4] = minDistToPoints[5]; child.distanceToVertices[5] = minDistToPoints[6];
                child.distanceToVertices[6] = minDistToPoints[8]; child.distanceToVertices[7] = minDistToPoints[9];

                child.verticesInfo[0] = node.verticesInfo[0]; child.verticesInfo[1] = pointsInfo[0]; 
                child.verticesInfo[2] = pointsInfo[1]; child.verticesInfo[3] = pointsInfo[2];
                child.verticesInfo[4] = pointsInfo[5]; child.verticesInfo[5] = pointsInfo[6];
                child.verticesInfo[6] = pointsInfo[8]; child.verticesInfo[7] = pointsInfo[9];
            }

            nodes.push(NodeInfo(childIndex + (childOffsetMask & 1), node.depth + 1, node.center + glm::vec3(newSize, -newSize, -newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.distanceToVertices[0] = minDistToPoints[0]; child.distanceToVertices[1] = node.distanceToVertices[1];
                child.distanceToVertices[2] = minDistToPoints[2]; child.distanceToVertices[3] = minDistToPoints[3];
                child.distanceToVertices[4] = minDistToPoints[6]; child.distanceToVertices[5] = minDistToPoints[7];
                child.distanceToVertices[6] = minDistToPoints[9]; child.distanceToVertices[7] = minDistToPoints[10];

                child.verticesInfo[0] = pointsInfo[0]; child.verticesInfo[1] = node.verticesInfo[1];
                child.verticesInfo[2] = pointsInfo[2]; child.verticesInfo[3] = pointsInfo[3];
                child.verticesInfo[4] = pointsInfo[6]; child.verticesInfo[5] = pointsInfo[7];
                child.verticesInfo[6] = pointsInfo[9]; child.verticesInfo[7] = pointsInfo[10];
            }

            nodes.push(NodeInfo(childIndex + (childOffsetMask & 2), node.depth + 1, node.center + glm::vec3(-newSize, newSize, -newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.distanceToVertices[0] = minDistToPoints[1]; child.distanceToVertices[1] = minDistToPoints[2];
                child.distanceToVertices[2] = node.distanceToVertices[2]; child.distanceToVertices[3] = minDistToPoints[4];
                child.distanceToVertices[4] = minDistToPoints[8]; child.distanceToVertices[5] = minDistToPoints[9];
                child.distanceToVertices[6] = minDistToPoints[11]; child.distanceToVertices[7] = minDistToPoints[12];

                child.verticesInfo[0] = pointsInfo[1]; child.verticesInfo[1] = pointsInfo[2];
                child.verticesInfo[2] = node.verticesInfo[2]; child.verticesInfo[3] = pointsInfo[4];
                child.verticesInfo[4] = pointsInfo[8]; child.verticesInfo[5] = pointsInfo[9];
                child.verticesInfo[6] = pointsInfo[11]; child.verticesInfo[7] = pointsInfo[12];
            }

            nodes.push(NodeInfo(childIndex + (childOffsetMask & 3), node.depth + 1, node.center + glm::vec3(newSize, newSize, -newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.distanceToVertices[0] = minDistToPoints[2]; child.distanceToVertices[1] = minDistToPoints[3];
                child.distanceToVertices[2] = minDistToPoints[4]; child.distanceToVertices[3] = node.distanceToVertices[3];
                child.distanceToVertices[4] = minDistToPoints[9]; child.distanceToVertices[5] = minDistToPoints[10];
                child.distanceToVertices[6] = minDistToPoints[12]; child.distanceToVertices[7] = minDistToPoints[13];

                child.verticesInfo[0] = pointsInfo[2]; child.verticesInfo[1] = pointsInfo[3];
                child.verticesInfo[2] = pointsInfo[4]; child.verticesInfo[3] = node.verticesInfo[3];
                child.verticesInfo[4] = pointsInfo[9]; child.verticesInfo[5] = pointsInfo[10];
                child.verticesInfo[6] = pointsInfo[12]; child.verticesInfo[7] = pointsInfo[13];
            }

            // High Z children
            nodes.push(NodeInfo(childIndex + (childOffsetMask & 4), node.depth + 1, node.center + glm::vec3(-newSize, -newSize, newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.distanceToVertices[0] = minDistToPoints[5]; child.distanceToVertices[1] = minDistToPoints[6];
                child.distanceToVertices[2] = minDistToPoints[8]; child.distanceToVertices[3] = minDistToPoints[9];
                child.distanceToVertices[4] = node.distanceToVertices[4]; child.distanceToVertices[5] = minDistToPoints[14];
                child.distanceToVertices[6] = minDistToPoints[15]; child.distanceToVertices[7] = minDistToPoints[16];

                child.verticesInfo[0] = pointsInfo[5]; child.verticesInfo[1] = pointsInfo[6];
                child.verticesInfo[2] = pointsInfo[8]; child.verticesInfo[3] = pointsInfo[9];
                child.verticesInfo[4] = node.verticesInfo[4]; child.verticesInfo[5] = pointsInfo[14];
                child.verticesInfo[6] = pointsInfo[15]; child.verticesInfo[7] = pointsInfo[16];
            }

            nodes.push(NodeInfo(childIndex + (childOffsetMask & 5), node.depth + 1, node.center + glm::vec3(newSize, -newSize, newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.distanceToVertices[0] = minDistToPoints[6]; child.distanceToVertices[1] = minDistToPoints[7];
                child.distanceToVertices[2] = minDistToPoints[9]; child.distanceToVertices[3] = minDistToPoints[10];
                child.distanceToVertices[4] = minDistToPoints[14]; child.distanceToVertices[5] = node.distanceToVertices[5];
                child.distanceToVertices[6] = minDistToPoints[16]; child.distanceToVertices[7] = minDistToPoints[17];

                child.verticesInfo[0] = pointsInfo[6]; child.verticesInfo[1] = pointsInfo[7];
                child.verticesInfo[2] = pointsInfo[9]; child.verticesInfo[3] = pointsInfo[10];
                child.verticesInfo[4] = pointsInfo[14]; child.verticesInfo[5] = node.verticesInfo[5];
                child.verticesInfo[6] = pointsInfo[16]; child.verticesInfo[7] = pointsInfo[17];
            }

            nodes.push(NodeInfo(childIndex + (childOffsetMask & 6), node.depth + 1, node.center + glm::vec3(-newSize, newSize, newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.distanceToVertices[0] = minDistToPoints[8]; child.distanceToVertices[1] = minDistToPoints[9];
                child.distanceToVertices[2] = minDistToPoints[11]; child.distanceToVertices[3] = minDistToPoints[12];
                child.distanceToVertices[4] = minDistToPoints[15]; child.distanceToVertices[5] = minDistToPoints[16];
                child.distanceToVertices[6] = node.distanceToVertices[6]; child.distanceToVertices[7] = minDistToPoints[18];

                child.verticesInfo[0] = pointsInfo[8]; child.verticesInfo[1] = pointsInfo[9];
                child.verticesInfo[2] = pointsInfo[11]; child.verticesInfo[3] = pointsInfo[12];
                child.verticesInfo[4] = pointsInfo[15]; child.verticesInfo[5] = pointsInfo[16];
                child.verticesInfo[6] = node.verticesInfo[6]; child.verticesInfo[7] = pointsInfo[18];
            }

            nodes.push(NodeInfo(childIndex + (childOffsetMask & 7), node.depth + 1, node.center + glm::vec3(newSize, newSize, newSize), newSize));
            {
                NodeInfo& child = nodes.top();
                child.distanceToVertices[0] = minDistToPoints[9]; child.distanceToVertices[1] = minDistToPoints[10];
                child.distanceToVertices[2] = minDistToPoints[12]; child.distanceToVertices[3] = minDistToPoints[13];
                child.distanceToVertices[4] = minDistToPoints[16]; child.distanceToVertices[5] = minDistToPoints[17];
                child.distanceToVertices[6] = minDistToPoints[18]; child.distanceToVertices[7] = node.distanceToVertices[7];

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

            mOctreeData.resize(mOctreeData.size() + triangles[rDepth].size() + 1);
            mOctreeData[childIndex++].size = triangles[rDepth].size();

            for(const uint32_t& idx : triangles[rDepth])
            {
                mOctreeData[childIndex++].triangleIndex = idx;
            }

            #ifdef PRINT_STATISTICS
                numTrianglesInLeafs += triangles[rDepth].size();
                numLeafs++;
                maxTrianglesInLeaf = glm::max(maxTrianglesInLeaf, static_cast<uint32_t>(triangles[rDepth].size()));
                endedNodes[node.depth]++;
            #endif
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
    SPDLOG_INFO("Maximum triangles in a leaf: {}", maxTrianglesInLeaf);

    trianglesInfluence.printStatistics();
#endif
}

#endif