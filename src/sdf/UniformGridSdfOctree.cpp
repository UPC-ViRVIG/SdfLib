#include "UniformGridSdf.h"
#include "utils/TriangleUtils.h"
#include "utils/GJK.h"
#include <stack>
#include <algorithm>

#include "RealSdf.h"


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

void UniformGridSdf::octreeInit(const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData)
{
    // Calculate octree properties
    int octreeSize = glm::max(glm::max(mGridSize.x, mGridSize.y), mGridSize.z);
    uint32_t maxDepth = static_cast<uint32_t>(glm::ceil(glm::log2(static_cast<float>(octreeSize))));
	octreeSize = 1 << maxDepth;

    const uint32_t numTriangles = trianglesData.size();
    std::vector<std::vector<std::pair<float, uint32_t>>> triangles(maxDepth);
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
        BoundingBox b(mBox.min - 0.5f * mCellSize, mBox.min - 0.5f * mCellSize + glm::vec3(octreeSize * mCellSize));
        glm::vec3 center = b.getCenter();
        float newSize = 0.25f * b.getSize().x;
        for(glm::vec3& c : childrens)
        {
            nodes.push(OctreeNode(1, center + c * newSize, newSize));
        }
    }

    std::array<glm::vec3, 3> triangle;

    const std::vector<glm::vec3>& vertices = mesh.getVertices();
    const std::vector<uint32_t>& indices = mesh.getIndices();
    const float voxelDiagonal = glm::sqrt(3.0f); // Voxel diagonal when the voxels has size one

    std::vector<std::pair<uint32_t, uint32_t>> verticesStatistics(maxDepth, std::make_pair(0, 0));
    verticesStatistics[0] = std::make_pair(trianglesData.size(), 1);

    uint32_t numVoxelsCalculated = 0;
    uint32_t lastPercentatge = 0;
    uint32_t numVoxelsToCalculate = mGrid.size();

    while(!nodes.empty())
    {
        const OctreeNode node = nodes.top();
        nodes.pop();

        if(glm::any(glm::greaterThan(node.center - glm::vec3(node.size), mBox.max))) continue;
        
        if(node.depth + 1 < maxDepth)
        {
            triangles[node.depth].resize(0);
            float minMaxDist = INFINITY;

            for(const std::pair<float, uint32_t>& p : triangles[node.depth-1])
            {
                triangle[0] = vertices[indices[3 * p.second]] - node.center;
                triangle[1] = vertices[indices[3 * p.second + 1]] - node.center;
                triangle[2] = vertices[indices[3 * p.second + 2]] - node.center;

                float minDist = GJK::getMinDistance(glm::vec3(node.size), triangle);
                float maxDist = glm::min(GJK::getMaxDistance(glm::vec3(node.size), triangle), minDist + voxelDiagonal * 2.0f * node.size);
                minMaxDist = glm::min(minMaxDist, maxDist);

                if(minDist <= minMaxDist)
                {
                    triangles[node.depth].push_back(std::make_pair(minDist, p.second));
                }
            }

            std::sort(triangles[node.depth].begin(), triangles[node.depth].end());

            int s=0;
            for(; s < triangles[node.depth].size() && triangles[node.depth][s].first <= minMaxDist; s++);

            assert(s > 0);

            triangles[node.depth].resize(s);

            const float newSize = 0.5 * node.size;
            for(glm::vec3& c : childrens)
            {
                nodes.push(OctreeNode(node.depth + 1, node.center + c * newSize, newSize));
            }
        }
        else
        {
			const float newSize = 0.5 * node.size;
            glm::vec3 fracPart = (node.center - mBox.min) / mCellSize;
            glm::ivec3 arrayPos = glm::floor(fracPart);

			std::array<float, 8> minDists = { INFINITY, INFINITY, INFINITY, INFINITY,
                                              INFINITY, INFINITY, INFINITY, INFINITY };
            std::array<int, 8> minIndices;
        
            float aux;
            const uint32_t size = triangles[node.depth-1].size();
            for(uint32_t i=0; i < size; i++)
            {
                const uint32_t idx = triangles[node.depth-1][i].second;
                for(uint32_t n=0; n < 8; n++)
                {
                    aux = TriangleUtils::getSqDistPointAndTriangle(node.center + childrens[n] * newSize, trianglesData[idx]);
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
					    TriangleUtils::getSignedDistPointAndTriangle(node.center + childrens[n] * newSize, trianglesData[minIndices[n]]);

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

        verticesStatistics[node.depth].first += triangles[node.depth].size();
        verticesStatistics[node.depth].second += 1;
    }

    SPDLOG_INFO("Used an octree of depth {}", maxDepth);
    for(uint32_t d=0; d < maxDepth; d++)
    {
        const float mean = static_cast<float>(verticesStatistics[d].first) / static_cast<float>(verticesStatistics[d].second);
        SPDLOG_INFO("Depth {}, mean of triangles per node: {}", d, mean);
    }
}