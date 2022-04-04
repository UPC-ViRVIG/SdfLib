#include "GJK.h"

#include <spdlog/spdlog.h>

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

bool getTetrahedronOriginDirectionCase1(Simplex& simplex, glm::vec3& outDirection)
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

bool getTetrahedronOriginDirectionCase2(Simplex& simplex, glm::vec3& outDirection)
{
    glm::vec3 ab = simplex.points[1] - simplex.points[0];
    glm::vec3 ac = simplex.points[2] - simplex.points[0];
    glm::vec3 ad = simplex.points[3] - simplex.points[0];
    glm::vec3 ao = -simplex.points[0];

    glm::vec3 abc = glm::cross(ab, ac);
    glm::vec3 acd = glm::cross(ac, ad);
    glm::vec3 adb = glm::cross(ad, ab);

    bool enter = false;

    bool acOutAbc = glm::dot(glm::cross(abc, ac), ao) > 0.0f;
    bool acOutAcd = glm::dot(glm::cross(ac, acd), ao) > 0.0f;

    if(acOutAbc && acOutAcd)
    {
        // Edge ac
        if(glm::dot(ac, ao) > 0.0f)
        {
            simplex.type = SimplexType::LINE;
            simplex.points[1] = simplex.points[2];
            outDirection = glm::cross(ac, glm::cross(ao, ac));
            return false;
        }
        enter = true;
    }

    bool abOutAbc = glm::dot(glm::cross(ab, abc), ao) > 0.0f;
    bool abOutAdb = glm::dot(glm::cross(adb, ab), ao) > 0.0f;

    if(abOutAbc && abOutAdb)
    {
        // Edge ab
        if(glm::dot(ab, ao) > 0.0f)
        {
            simplex.type = SimplexType::LINE;
            outDirection = glm::cross(ab, glm::cross(ao, ab));
            return false;
        }
        enter = true;
    }

    if(glm::dot(abc, ao) > 0.0f && !acOutAbc && !abOutAbc)
    {
        // Triangle abc
        simplex.type = SimplexType::TRIANGLE;
        outDirection = abc;
        return false;
    }

    bool adOutAcd = glm::dot(glm::cross(acd, ad), ao) > 0.0f;
    bool adOutAdb = glm::dot(glm::cross(ad, adb), ao) > 0.0f;
    if(adOutAcd && adOutAdb)
    {
        // Edge ad
        if(glm::dot(ad, ao) > 0.0f)
        {
            simplex.type = SimplexType::LINE;
            simplex.points[1] = simplex.points[3];
            outDirection = glm::cross(ad, glm::cross(ao, ad));
            return false;
        }
        enter = true;
    }

    if(glm::dot(acd, ao) > 0.0f && !acOutAcd && !adOutAcd)
    {
        simplex.type = SimplexType::TRIANGLE;
        simplex.points[1] = simplex.points[2];
        simplex.points[2] = simplex.points[3];
        outDirection = acd;
        return false;
    }

    if(glm::dot(adb, ao) > 0.0f && !adOutAdb && !abOutAdb)
    {
        // Triangle adb
        simplex.type = SimplexType::TRIANGLE;
        simplex.points[2] = simplex.points[1];
        simplex.points[1] = simplex.points[3];
        outDirection = adb;
        return false;
    }

    if(enter)
    {
        simplex.type = SimplexType::POINT;
        outDirection = ao;
        return false;
    }

    return true;
}

bool getOriginDirection(Simplex& simplex, glm::vec3& outDirection, float dotLastEnterPoint)
{
    switch(simplex.type)
    {
        case SimplexType::LINE:
            return getLineOriginDirection(simplex, outDirection);
        case SimplexType::TRIANGLE:
            return getTriangleOriginDirection(simplex, outDirection);
        case SimplexType::TETRAHEDRON:
            return getTetrahedronOriginDirectionCase2(simplex, outDirection);
            // if(dotLastEnterPoint > 0.0f)
            //     return getTetrahedronOriginDirectionCase1(simplex, outDirection);
            // else
                
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
    float dotLastEnterPoint = 0.0f;
    uint32_t iter = 0;
    do {
        direction = glm::normalize(direction);
        glm::vec3 p = findFurthestPoint(e1, e2, direction);

        dotLastEnterPoint = glm::dot(p, direction);
        if(dotLastEnterPoint - glm::dot(simplex.points[0], direction) <= 1.0e-5f)
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

    } while(!getOriginDirection(simplex, direction, dotLastEnterPoint) && ++iter < 100);

    assert(iter < 100);

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

inline glm::vec3 findFurthestPoint(const glm::vec3& quadSize, const std::array<glm::vec3, 3>& triangle, const glm::vec3& direction)
{
    const float d1 = glm::dot(triangle[0], direction);
    const float d2 = glm::dot(triangle[1], direction);
    const float d3 = glm::dot(triangle[2], direction);

    const glm::vec3 quadPos = glm::sign(-direction) * quadSize;

    if(d1 > d2)
    {
        return (d1 > d3) ? triangle[0] - quadPos : triangle[2] - quadPos;
    }
    else
    {
        return (d2 > d3) ? triangle[1] - quadPos : triangle[2] - quadPos;
    }
}

float getMinDistance(glm::vec3 quadSize, const std::array<glm::vec3, 3>& triangle)
{
    Simplex simplex;
    simplex.type = SimplexType::POINT;
    simplex.points[0] = findFurthestPoint(quadSize, triangle, glm::vec3(1.0, 0.0, 0.0));

    glm::vec3 direction = -simplex.points[0];
    float dotLastEnterPoint = 0.0f;
    uint32_t iter = 0;
    do {
        direction = glm::normalize(direction);
        glm::vec3 p = findFurthestPoint(quadSize, triangle, direction);
        
        dotLastEnterPoint = glm::dot(p, direction);
        if(dotLastEnterPoint - glm::dot(simplex.points[0], direction) <= 5.0e-5f * glm::abs(dotLastEnterPoint))
        {
            return glm::dot(p, glm::normalize(-direction));
        }

        // Insert new point to the simplex
        simplex.type = static_cast<SimplexType>(simplex.type + 1);
        simplex.points[3] = simplex.points[2];
        simplex.points[2] = simplex.points[1];
        simplex.points[1] = simplex.points[0];
        simplex.points[0] = p;

    } while(!getOriginDirection(simplex, direction, dotLastEnterPoint) && ++iter < 100);

    //assert(iter < 100);
    if(iter >= 100)
    {
        SPDLOG_ERROR("GJK has done maximum iterations without solving the shape");
    }

    // It does not have next origin direction because the origin is inside the simplex
    return 0.0f;
}

inline float sqMaxDistToQuad(const glm::vec3& point, const glm::vec3& quadSize)
{    
    glm::vec3 aux = point - glm::sign(-point) * quadSize;
    return glm::dot(aux, aux);
}

float getMaxDistance(glm::vec3 quadSize, const std::array<glm::vec3, 3>& triangle)
{
    const float maxDist = glm::max(glm::max(
        sqMaxDistToQuad(triangle[0], quadSize),
        sqMaxDistToQuad(triangle[1], quadSize)),
        sqMaxDistToQuad(triangle[2], quadSize)
    );
    return glm::sqrt(maxDist);
}

float getMinMaxDistance(glm::vec3 quadSize, const std::array<glm::vec3, 3>& triangle)
{
    const float maxDist = glm::min(glm::min(
        sqMaxDistToQuad(triangle[0], quadSize),
        sqMaxDistToQuad(triangle[1], quadSize)),
        sqMaxDistToQuad(triangle[2], quadSize)
    );
    return glm::sqrt(maxDist);
}

}