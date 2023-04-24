#include "SdfLib/utils/PrimitivesFactory.h"

#include <map>
#include <array>

namespace sdflib
{
namespace PrimitivesFactory
{

const std::array<unsigned int, 60> isosphereIndices =
{
    0,4,1,0,9,4,9,5,4,4,5,8,4,8,1,
    8,10,1,8,3,10,5,3,8,5,2,3,2,7,3,
    7,10,3,7,6,10,7,11,6,11,0,6,0,1,6,
    6,1,10,9,0,11,9,11,2,9,2,5,7,2,11
};

std::shared_ptr<Mesh> getIsosphere(uint32_t subdivisions)
{
    constexpr float X = 0.525731112119133606;
    constexpr float Z = 0.850650808352039932;
    std::array<glm::vec3, 12> isosphereVertices = 
    {
        glm::vec3(-X, 0.0, Z),
        glm::vec3(X, 0.0, Z),
        glm::vec3(-X, 0.0, -Z),
        glm::vec3(X, 0.0, -Z),
        glm::vec3(0.0, Z, X),
        glm::vec3(0.0, Z, -X),
        glm::vec3(0.0, -Z, X),
        glm::vec3(0.0, -Z, -X),
        glm::vec3(Z, X, 0.0),
        glm::vec3(-Z, X, 0.0),
        glm::vec3(Z, -X, 0.0),
        glm::vec3(-Z, -X, 0.0),
    };

    auto mesh = std::make_shared<Mesh>();
    mesh->getIndices().assign(isosphereIndices.begin(), isosphereIndices.end());
    mesh->getVertices().assign(isosphereVertices.begin(), isosphereVertices.end());

    if(subdivisions > 0)
    {
        std::map<std::pair<uint32_t, uint32_t>, uint32_t> edgeMidPointCache;

        std::vector<glm::vec3>& vertices = mesh->getVertices();
        std::vector<uint32_t>& indices = mesh->getIndices();

        float goldenRatio = (1.0f + glm::sqrt(5.0f)) / 2.0f;
        goldenRatio = glm::sqrt(goldenRatio * goldenRatio + 1.0f);

        auto getMidEdgeId = [&](uint32_t a, uint32_t b) -> uint32_t
        {
            auto edgeId = std::make_pair(glm::min(a, b), glm::max(a, b));
            auto it = edgeMidPointCache.find(edgeId);
            if(it != edgeMidPointCache.end())
            {
                const uint32_t edgeIndex = it->second;
                edgeMidPointCache.erase(it);
                return edgeIndex;
            }
            else
            {
                glm::vec3 point = 0.5f * (vertices[a] + vertices[b]);
                vertices.push_back(glm::normalize(point));
                const uint32_t edgeIndex = vertices.size()-1;
                edgeMidPointCache[edgeId] = edgeIndex;
                return edgeIndex;
            }
        };

        for(uint32_t s=0; s < subdivisions; s++)
        {
			uint32_t oldIndicesSize = indices.size();
            for(uint32_t t=0; t < oldIndicesSize; t += 3)
            {
                uint32_t nv1 = getMidEdgeId(indices[t], indices[t+1]);
                uint32_t nv2 = getMidEdgeId(indices[t+1], indices[t+2]);
                uint32_t nv3 = getMidEdgeId(indices[t+2], indices[t]);

                indices.push_back(indices[t]);
                indices.push_back(nv1);
                indices.push_back(nv3);

                indices.push_back(nv1);
                indices.push_back(indices[t+1]);
                indices.push_back(nv2);

                indices.push_back(nv2);
                indices.push_back(indices[t+2]);
                indices.push_back(nv3);

                indices[t] = nv1;
                indices[t+1] = nv2;
                indices[t+2] = nv3;
            }

            edgeMidPointCache.clear();
        }
    }

    return mesh;
}

// Plane
std::array<unsigned int, 6> planeIndices = 
{
    0, 1, 2,
    2, 3, 0
};

std::array<glm::vec3, 4> planeData = 
{
    glm::vec3(-0.5f, -0.5f, 0.0f),
    glm::vec3(0.5f, -0.5f, 0.0f),
    glm::vec3(0.5f, 0.5f, 0.0f),
    glm::vec3(-0.5f, 0.5f, 0.0f)
};

std::shared_ptr<Mesh> getPlane()
{
    auto mesh = std::make_shared<Mesh>();
    mesh->getIndices().assign(planeIndices.begin(), planeIndices.end());
    mesh->getVertices().assign(planeData.begin(), planeData.end());

    return mesh;
}

// Cube
std::array<unsigned int, 36> cubeIndices = 
{
    0, 1, 2,
    2, 3, 0,

    4, 5, 6,
    6, 7, 4,

    8, 9, 10,
    10, 11, 8,

    12, 13, 14,
    14, 15, 12,

    16, 17, 18,
    18, 19, 16,

    20, 21, 22,
    22, 23, 20
};

std::array<glm::vec3, 24> cubeData = 
{
    // Z axis faces
    glm::vec3(-0.5f, -0.5f, 0.5f),
    glm::vec3(0.5f, -0.5f, 0.5f),
    glm::vec3(0.5f, 0.5f, 0.5f),
    glm::vec3(-0.5f, 0.5f, 0.5f),

    glm::vec3(-0.5f, -0.5f, -0.5f),
    glm::vec3(-0.5f, 0.5f, -0.5f),
    glm::vec3(0.5f, 0.5f, -0.5f),
    glm::vec3(0.5f, -0.5f, -0.5f),

    // Y axis faces
    glm::vec3(-0.5f, 0.5f, -0.5f),
    glm::vec3(-0.5f, 0.5f, 0.5f),
    glm::vec3(0.5f, 0.5f, 0.5f),
    glm::vec3(0.5f, 0.5f, -0.5f),

    glm::vec3(-0.5f, -0.5f, -0.5f),
    glm::vec3(0.5f, -0.5f, -0.5f),
    glm::vec3(0.5f, -0.5f, 0.5f),
    glm::vec3(-0.5f, -0.5f, 0.5f),

    // X axis face
    glm::vec3(0.5f, -0.5f, -0.5f),
    glm::vec3(0.5f, 0.5f, -0.5f),
    glm::vec3(0.5f, 0.5f, 0.5f),
    glm::vec3(0.5f, -0.5f, 0.5f),

    glm::vec3(-0.5f, -0.5f, -0.5f),
    glm::vec3(-0.5f, -0.5f, 0.5f),
    glm::vec3(-0.5f, 0.5f, 0.5f),
    glm::vec3(-0.5f, 0.5f, -0.5f),
};


std::shared_ptr<Mesh> getCube()
{
    auto mesh = std::make_shared<Mesh>();
    mesh->getIndices().assign(cubeIndices.begin(), cubeIndices.end());
    mesh->getVertices().assign(cubeData.begin(), cubeData.end());

    return mesh;
}
}
}