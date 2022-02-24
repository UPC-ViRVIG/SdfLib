#include "PrimitivesFactory.h"

#include <array>

namespace PrimitivesFactory
{

const std::array<unsigned int, 60> isosphereIndices =
{
    0,4,1,0,9,4,9,5,4,4,5,8,4,8,1,
    8,10,1,8,3,10,5,3,8,5,2,3,2,7,3,
    7,10,3,7,6,10,7,11,6,11,0,6,0,1,6,
    6,1,10,9,0,11,9,11,2,9,2,5,7,2,11
};

std::shared_ptr<Mesh> getIsosphere()
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