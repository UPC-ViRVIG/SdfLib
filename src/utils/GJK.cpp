#include "GJK.h"

namespace GJK
{

enum SimplexType
{
    POINT = 1,
    LINE = 2,
    TRIANGLE = 3,
    TETRAHEDRON = 4
};

struct Simplex
{
    std::array<glm::vec3, 4> points;
    SimplexType type;
};

bool getLineOriginDirection(Simplex& simplex, glm::vec3& outDirection)
{
    const glm::vec3 ab = simplex.points[1] - simplex.points[0];
    const glm::vec3 ao = -simplex.points[0];

    if(glm::dot(ab, ao) > 0.0f)
    {
        // The origin is in one side of the line
        outDirection = glm::cross(ab, glm::cross(ao, ab));
    }
    else
    {
        // The origin is near the point a
        simplex.type = SimplexType::POINT;
        outDirection = ao;
    }
    
    return false;
}

bool getTriangleOriginDirection(Simplex& simplex, glm::vec3& outDirection)
{
    glm::vec3 ab = simplex.points[1] - simplex.points[0];
    glm::vec3 ac = simplex.points[2] - simplex.points[0];
    glm::vec3 ao = -simplex.points[0];

    glm::vec3 n = glm::cross(ab, ac);

    if(glm::dot(glm::cross(n, ac), ao) > 0.0f)
    {
        if(glm::dot(ac, ao) > 0.0f)
        {
            // The origin is in the ac line
            simplex.type = SimplexType::LINE;
            simplex.points[1] = simplex.points[2];
			outDirection = glm::cross(ac, glm::cross(ao, ac));
        }
        else
        {
            // The origin is near the point a or in one side of the line ab
            simplex.type = SimplexType::LINE;
            return getLineOriginDirection(simplex, outDirection);
        }
    }
    else if(glm::dot(glm::cross(ab, n), ao) > 0.0f)
    {
        // The origin is near the point a or in one side of the line ab
        simplex.type = SimplexType::LINE;
        return getLineOriginDirection(simplex, outDirection);
    }
    else
    {
        // The origin towards the triangle normal
        if(glm::dot(n, ao) > 0.0f)
        {
            outDirection = n;
        }
        else
        {
            glm::vec3 aux = simplex.points[1];
            simplex.points[1] = simplex.points[2];
            simplex.points[2] = aux;
            outDirection = -n;
        }
    }
    return false;
}

bool getTetrahedronOriginDirection(Simplex& simplex, glm::vec3& outDirection)
{
    glm::vec3 ab = simplex.points[1] - simplex.points[0];
    glm::vec3 ac = simplex.points[2] - simplex.points[0];
    glm::vec3 ad = simplex.points[3] - simplex.points[0];
    glm::vec3 ao = -simplex.points[0];

    glm::vec3 n = glm::cross(ab, ac);
    if(glm::dot(n, ao) > 0.0f)
    {
        simplex.type = SimplexType::TRIANGLE;
        return getTriangleOriginDirection(simplex, outDirection);
    }

    n = glm::cross(ac, ad);
    if(glm::dot(n, ao) > 0.0f)
    {
        simplex.type = SimplexType::TRIANGLE;
        simplex.points[1] = simplex.points[2];
        simplex.points[2] = simplex.points[3];
        return getTriangleOriginDirection(simplex, outDirection);
    }

    n = glm::cross(ad, ab);
    if(glm::dot(n, ao) > 0.0f)
    {
        simplex.type = SimplexType::TRIANGLE;
        simplex.points[2] = simplex.points[1];
        simplex.points[1] = simplex.points[3];
        return getTriangleOriginDirection(simplex, outDirection);
    }

    return true;
}

bool getOriginDirection(Simplex& simplex, glm::vec3& outDirection)
{
    switch(simplex.type)
    {
        case SimplexType::LINE:
            return getLineOriginDirection(simplex, outDirection);
        case SimplexType::TRIANGLE:
            return getTriangleOriginDirection(simplex, outDirection);
        case SimplexType::TETRAHEDRON:
            return getTetrahedronOriginDirection(simplex, outDirection);
    }

    assert(false);
    return false;
}

glm::vec3 findFurthestPoint(const std::vector<glm::vec3>& e, glm::vec3 direction)
{
    float maxValue = glm::dot(e[0], direction);
    int maxIndex = 0;
    for(int i=1; i < e.size(); i++)
    {
        const float value = glm::dot(e[i], direction);
        if(value > maxValue)
        {
            maxValue = value;
            maxIndex = i;
        }
    }

    return e[maxIndex];
}

glm::vec3 findFurthestPoint(const std::vector<glm::vec3>& e1, const std::vector<glm::vec3>& e2, glm::vec3 direction)
{
    return findFurthestPoint(e1, direction) - findFurthestPoint(e2, -direction);
}

float getMinDistance(const std::vector<glm::vec3>& e1, const std::vector<glm::vec3>& e2)
{
    Simplex simplex;
    simplex.type = SimplexType::POINT;
    simplex.points[0] = findFurthestPoint(e1, e2, glm::vec3(1.0, 0.0, 0.0));

    glm::vec3 direction = -simplex.points[0];

    do {
        glm::vec3 p = findFurthestPoint(e1, e2, direction);

        if(glm::dot(p, direction) - glm::dot(simplex.points[0], direction) <= 0.0001f)
        {
            return glm::dot(p, glm::normalize(-direction));
            // return glm::length(direction);
        }

        // Insert new point to the simplex
        simplex.type = static_cast<SimplexType>(simplex.type + 1);
        simplex.points[3] = simplex.points[2];
        simplex.points[2] = simplex.points[1];
        simplex.points[1] = simplex.points[0];
        simplex.points[0] = p;

    } while(!getOriginDirection(simplex, direction));

    // It does not have next origin direction because the origin is inside the simplex
    return 0.0f;
}

float getMaxDistance(const std::vector<glm::vec3>& e1, const std::vector<glm::vec3>& e2)
{
    float maxDist = 0.0f;

    for(const glm::vec3& v1 : e1)
    {
        for(const glm::vec3& v2 : e2)
        {
            const glm::vec3 diff = v2 - v1;
            maxDist = glm::max(maxDist, glm::dot(diff, diff));
        }
    }

    return glm::sqrt(maxDist);
}

}