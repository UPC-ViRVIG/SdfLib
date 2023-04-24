#include "SdfLib/UniformGridSdf.h"
#include "SdfLib/utils/TriangleUtils.h"
#include "SdfLib/utils/GJK.h"
#include "SdfLib/utils/Timer.h"
#include <stack>
#include <algorithm>

#include "SdfLib/RealSdf.h"

namespace sdflib
{
void UniformGridSdf::evalNode(glm::vec3 center, glm::vec3 size, 
                  std::vector<std::pair<float, uint32_t>>& parentTriangles, 
                  const std::vector<TriangleUtils::TriangleData>& trianglesData,
                  uint32_t depth)
{
    std::vector<std::pair<float, uint32_t>> triangles;
    triangles.reserve(parentTriangles.size());

    float maxDist = INFINITY;

    for(const std::pair<float, uint32_t>& p : parentTriangles)
    {
        
    }
}

struct OctreeNode
{
    OctreeNode(uint32_t depth, glm::vec3 center, float size) :
        depth(depth),
        center(center),
        size(size)
    {}
    uint32_t depth;
    glm::vec3 center;
    float size;
};

constexpr uint32_t START_OCTREE_DEPTH = 1;

void UniformGridSdf::octreeInit(const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
{
    // Calculate octree properties
    int octreeSize = glm::max(glm::max(mGridSize.x, mGridSize.y), mGridSize.z);
    uint32_t maxDepth = static_cast<uint32_t>(glm::ceil(glm::log2(static_cast<float>(octreeSize))));
	octreeSize = 1 << maxDepth;

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

    std::stack<OctreeNode> nodes;
    {
        BoundingBox b(mBox.min, mBox.min + glm::vec3((octreeSize - 1) * mCellSize));
        float newSize = 0.5f * b.getSize().x;
        for(uint32_t i = 0; i < START_OCTREE_DEPTH; i++) newSize = 0.5f * (newSize - 0.5f * mCellSize);
        const float voxelSpacing = 2.0f * newSize + mCellSize;
        const glm::vec3 startCenter = b.min + newSize;
        const uint32_t voxlesPerAxis = 1 << START_OCTREE_DEPTH;

        for(uint32_t k=0; k < voxlesPerAxis; k++)
        {
            for(uint32_t j=0; j < voxlesPerAxis; j++)
            {
                for(uint32_t i=0; i < voxlesPerAxis; i++)
                {
                    nodes.push(OctreeNode(START_OCTREE_DEPTH, startCenter + glm::vec3(i, j, k) * voxelSpacing, newSize));
                }
            }
        }
    }

    std::array<glm::vec3, 3> triangle;

    const std::vector<glm::vec3>& vertices = mesh.getVertices();
    const std::vector<uint32_t>& indices = mesh.getIndices();
    const float voxelDiagonal = glm::sqrt(3.0f); // Voxel diagonal when the voxels has size one

    std::vector<std::pair<uint32_t, uint32_t>> verticesStatistics(maxDepth, std::make_pair(0, 0));
    verticesStatistics[0] = std::make_pair(trianglesData.size(), 1);

    std::vector<float> elapsedTime(maxDepth);
    std::vector<uint32_t> numTrianglesEvaluated(maxDepth, 0);
    Timer timer;

    uint32_t numVoxelsCalculated = 0;
    uint32_t lastPercentatge = 0;
    uint32_t numVoxelsToCalculate = mGrid.size();

    while(!nodes.empty())
    {
        timer.start();
        const OctreeNode node = nodes.top();
        nodes.pop();

        if(glm::any(glm::greaterThan(node.center - glm::vec3(node.size), mBox.max))) continue;

        const uint32_t rDepth = node.depth - START_OCTREE_DEPTH + 1;
        
        if(node.depth + 1 < maxDepth)
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

            const float newSize = 0.5f * (node.size - 0.5f * mCellSize);
            for(glm::vec3& c : childrens)
            {
                nodes.push(OctreeNode(node.depth + 1, node.center + c * (newSize + 0.5f * mCellSize), newSize));
            }
        }
        else
        {
			const float newSize = 0.5f * (node.size - 0.5f * mCellSize);
            glm::vec3 fracPart = (node.center - newSize - mBox.min) / mCellSize;
            glm::ivec3 arrayPos = glm::floor(fracPart);

			std::array<float, 8> minDists = { INFINITY, INFINITY, INFINITY, INFINITY,
                                              INFINITY, INFINITY, INFINITY, INFINITY };
            std::array<int, 8> minIndices;
        
            float aux;
            const uint32_t size = triangles[rDepth-1].size();
            for(uint32_t i=0; i < size; i++)
            {
                const uint32_t idx = triangles[rDepth-1][i].second;
                for(uint32_t n=0; n < 8; n++)
                {
                    aux = TriangleUtils::getSqDistPointAndTriangle(node.center + childrens[n] * (0.5f * mCellSize), trianglesData[idx]);
                    if(aux < minDists[n])
                    {
                        minDists[n] = aux; 
                        minIndices[n] = idx;
                    }
                }
            }

            for(uint32_t n=0; n < 8; n++)
            {
                if((arrayPos.x + (n & 0b01)) >= mGridSize.x || (arrayPos.y + ((n >> 1) & 0b01)) >= mGridSize.y || (arrayPos.z + (n >> 2)) >= mGridSize.z) continue;
				
				mGrid[(arrayPos.z + (n >> 2)) * mGridXY + (arrayPos.y + ((n >> 1) & 0b01)) * mGridSize.x + arrayPos.x + (n & 0b01)] =
					    TriangleUtils::getSignedDistPointAndTriangle(node.center + childrens[n] * (0.5f * mCellSize), trianglesData[minIndices[n]]);

                numVoxelsCalculated++;
            }

            if(numVoxelsCalculated & 0x10000)
            {
                const uint32_t p = (numVoxelsCalculated * 100) / numVoxelsToCalculate;
				if (p > lastPercentatge + 4)
				{
					SPDLOG_INFO("Done {}%", p);
					lastPercentatge = p;
				}
            }
        }

        verticesStatistics[node.depth].first += triangles[rDepth].size();
        verticesStatistics[node.depth].second += 1;
        elapsedTime[node.depth] += timer.getElapsedSeconds();
        numTrianglesEvaluated[node.depth] += triangles[rDepth-1].size();
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