#include "SdfLib/OctreeSdf.h"
#include "SdfLib/utils/Timer.h"
#include "SdfLib/utils/GJK.h"
#include "SdfLib/OctreeSdfUtils.h"
#include <array>
#include <stack>

namespace sdflib
{
namespace OctreeUniformData
{
	struct NodeInfo
	{
		NodeInfo(uint32_t nodeIndex, uint16_t depth, glm::vec3 center, float size, bool isTerminalNode = false)
			: nodeIndex(nodeIndex), depth(depth), center(center), size(size), isTerminalNode(isTerminalNode) {}
		uint32_t nodeIndex;
		uint16_t depth;
		glm::vec3 center;
		float size;
		bool isTerminalNode;
		std::array<float, 8> distanceToVertices;
	};
}

using namespace OctreeUniformData;

void OctreeSdf::initUniformOctree(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth)
{
    std::vector<TriangleUtils::TriangleData> trianglesData(TriangleUtils::calculateMeshTriangleData(mesh));

    const uint32_t startOctreeDepth = glm::min(startDepth, START_OCTREE_DEPTH);

    const uint32_t numTriangles = trianglesData.size();
    std::vector<std::vector<std::pair<float, uint32_t>>> triangles(maxDepth - START_OCTREE_DEPTH + 1);
	triangles[0].resize(numTriangles);
    for(uint32_t i=0; i < numTriangles; i++)
    {
        triangles[0][i] = std::make_pair(0.0, i);
    }

    std::array<glm::vec3, 8> childrens = 
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
                }
            }
        }
    }

    mValueRange = 0.0f;

    std::array<glm::vec3, 3> triangle;

    const std::vector<glm::vec3>& vertices = mesh.getVertices();
    const std::vector<uint32_t>& indices = mesh.getIndices();
    const float voxelDiagonal = glm::sqrt(3.0f); // Voxel diagonal when the voxels has size one

    std::vector<std::pair<uint32_t, uint32_t>> verticesStatistics(maxDepth, std::make_pair(0, 0));
    verticesStatistics[0] = std::make_pair(trianglesData.size(), 1);

    std::vector<float> elapsedTime(maxDepth);
    std::vector<uint32_t> numTrianglesEvaluated(maxDepth, 0);
    Timer timer;

    while(!nodes.empty())
    {
        timer.start();
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
        
        if(node.depth < maxDepth)
        {
            triangles[rDepth].resize(0);
            float minMaxDist = INFINITY;

            for(const std::pair<float, uint32_t>& p : triangles[rDepth-1])
            {
                triangle[0] = vertices[indices[3 * p.second]] - node.center;
                triangle[1] = vertices[indices[3 * p.second + 1]] - node.center;
                triangle[2] = vertices[indices[3 * p.second + 2]] - node.center;

                float minDist = GJK::getMinDistance(glm::vec3(node.size), triangle);
                float maxDist = glm::min(GJK::getMinMaxDistance(glm::vec3(node.size), triangle), minDist + voxelDiagonal * 2.0f * node.size);
                minMaxDist = glm::min(minMaxDist, maxDist);

                if(minDist <= minMaxDist)
                {
                    triangles[rDepth].push_back(std::make_pair(minDist, p.second));
                }
            }

            std::sort(triangles[rDepth].begin(), triangles[rDepth].end());

            int s=0;
            for(; s < triangles[rDepth].size() && triangles[rDepth][s].first <= minMaxDist; s++);

            assert(s > 0);

            triangles[rDepth].resize(s);

            const float newSize = 0.5f * node.size;

            uint32_t childIndex = (node.depth >= startDepth) ? mOctreeData.size() : std::numeric_limits<uint32_t>::max();
            uint32_t childOffsetMask = (node.depth >= startDepth) ? ~0 : 0;
            if(octreeNode != nullptr) octreeNode->setValues(false, childIndex);

            if(node.depth >= startDepth) mOctreeData.resize(mOctreeData.size() + 8);

            for(uint32_t c=0; c < 8; c++)
            {
                nodes.push(NodeInfo(childIndex + (childOffsetMask & c), node.depth + 1, node.center + childrens[c] * newSize, newSize, false));
            }

            verticesStatistics[node.depth].first += triangles[rDepth].size();
            verticesStatistics[node.depth].second += 1;
            elapsedTime[node.depth] += timer.getElapsedSeconds();
            numTrianglesEvaluated[node.depth] += triangles[rDepth-1].size();
        }
        else
        {     
            assert(node.depth >= startDepth);
            uint32_t childIndex = mOctreeData.size();
            assert(octreeNode != nullptr);
			octreeNode->setValues(true, childIndex);

            mOctreeData.resize(mOctreeData.size() + 8);

            float* values = reinterpret_cast<float*>(&mOctreeData[childIndex]);
            for(uint32_t i=0; i < 8; i++)
            {
                values[i] = INFINITY;
            }
            std::array<uint32_t, 8> minIndex;

            for(std::pair<float, uint32_t> p : triangles[rDepth-1])
            {
                for(uint32_t i=0; i < 8; i++)
                {
                    const float dist = TriangleUtils::getSqDistPointAndTriangle(node.center + childrens[i] * node.size, trianglesData[p.second]);
                    if(dist < values[i])
                    {
                        minIndex[i] = p.second;
                        values[i] = dist;
                    }
                }
            }

            for(uint32_t i=0; i < 8; i++)
            {
                values[i] = TriangleUtils::getSignedDistPointAndTriangle(node.center + childrens[i] * node.size, trianglesData[minIndex[i]]);
                mValueRange = glm::max(mValueRange, glm::abs(values[i]));
            }
        }
    }

    SPDLOG_INFO("Used an octree of depth {}", maxDepth);
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
}
}