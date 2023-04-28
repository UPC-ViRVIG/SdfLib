#include "SdfLib/utils/GJK.h"

#include <spdlog/spdlog.h>

namespace sdflib
{
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
    typedef glm::vec3 Point;
    std::array<glm::vec3, 4> points;
    SimplexType type;

    inline glm::vec3 operator [](uint32_t idx) const { return points[idx]; }
};

struct TrackedSimplex
{
    struct Point
    {
        glm::vec3 value;
        std::pair<uint32_t, uint32_t> shapePoints;
    };
    std::array<Point, 4> points;
    SimplexType type;

    inline glm::vec3 operator [](uint32_t idx) const { return points[idx].value; }

    bool contains(const Point& p)
    {
        bool res = false;
        switch(type)
        {
            case SimplexType::TETRAHEDRON:
                res = res || (p.shapePoints.first == points[3].shapePoints.first &&
                              p.shapePoints.second == points[3].shapePoints.second);
            case SimplexType::TRIANGLE:
                res = res || (p.shapePoints.first == points[2].shapePoints.first &&
                             p.shapePoints.second == points[2].shapePoints.second);
            case SimplexType::LINE:
                res = res || (p.shapePoints.first == points[1].shapePoints.first &&
                             p.shapePoints.second == points[1].shapePoints.second);
            case SimplexType::POINT:
                res = res || (p.shapePoints.first == points[0].shapePoints.first &&
                             p.shapePoints.second == points[0].shapePoints.second);
        }

        return res;
    }
};

template<typename S>
inline bool getLineOriginDirection(S& simplex, glm::vec3& outDirection)
{
    const glm::vec3 ab = simplex[1] - simplex[0];
    const glm::vec3 ao = -simplex[0];

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

template<typename S>
inline bool getTriangleOriginDirection(S& simplex, glm::vec3& outDirection)
{
    glm::vec3 ab = simplex[1] - simplex[0];
    glm::vec3 ac = simplex[2] - simplex[0];
    glm::vec3 ao = -simplex[0];

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
            typename S::Point aux = simplex.points[1];
            simplex.points[1] = simplex.points[2];
            simplex.points[2] = aux;
            outDirection = -n;
        }
    }
    return false;
}

template<typename S>
inline bool getTetrahedronOriginDirectionCase1(S& simplex, glm::vec3& outDirection)
{
    glm::vec3 ab = simplex[1] - simplex[0];
    glm::vec3 ac = simplex[2] - simplex[0];
    glm::vec3 ad = simplex[3] - simplex[0];
    glm::vec3 ao = -simplex[0];

    float minDist = INFINITY;
    S tmpSimplex;
    S resSimplex;

    glm::vec3 n = glm::cross(ab, ac);
    if(glm::dot(n, ao) > 0.0f)
    {
        resSimplex.type = SimplexType::TRIANGLE;
        resSimplex.points = simplex.points;
        getTriangleOriginDirection(resSimplex, outDirection);
        minDist = glm::dot(resSimplex[0], glm::normalize(-outDirection));
    }

    n = glm::cross(ac, ad);
    if(glm::dot(n, ao) > 0.0f)
    {
        tmpSimplex.type = SimplexType::TRIANGLE;
        tmpSimplex.points[0] = simplex.points[0];
        tmpSimplex.points[1] = simplex.points[2];
        tmpSimplex.points[2] = simplex.points[3];
        glm::vec3 dir;
        getTriangleOriginDirection(tmpSimplex, dir);
        const float dist = glm::dot(tmpSimplex[0], glm::normalize(-dir));
        if(dist < minDist)
        {
            minDist = dist;
            resSimplex = tmpSimplex;
            outDirection = dir;
        }
    }

    n = glm::cross(ad, ab);
    if(glm::dot(n, ao) > 0.0f)
    {
        tmpSimplex.type = SimplexType::TRIANGLE;
        tmpSimplex.points[0] = simplex.points[0];
        tmpSimplex.points[2] = simplex.points[1];
        tmpSimplex.points[1] = simplex.points[3];
        glm::vec3 dir;
        getTriangleOriginDirection(tmpSimplex, dir);
        const float dist = glm::dot(tmpSimplex[0], glm::normalize(-dir));
        if(dist < minDist)
        {
            minDist = dist;
            resSimplex = tmpSimplex;
            outDirection = dir;
        }
    }

    simplex.type = resSimplex.type;
	simplex.points = resSimplex.points;

    return minDist == INFINITY;
}

template<typename S>
inline bool getTetrahedronOriginDirectionCase2(S& simplex, glm::vec3& outDirection)
{
    glm::vec3 ab = simplex[1] - simplex[0];
    glm::vec3 ac = simplex[2] - simplex[0];
    glm::vec3 ad = simplex[3] - simplex[0];
    glm::vec3 ao = -simplex[0];

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

template<typename S>
bool getOriginDirection(S& simplex, glm::vec3& outDirection, float dotLastEnterPoint)
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
        if(dotLastEnterPoint - glm::dot(simplex.points[0], direction) <= 5.0e-7f)
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

inline glm::vec3 fsign(glm::vec3 a)
{
	return glm::vec3(
		(a.x > 0.0f) ? 1.0f : -1.0f,
		(a.y > 0.0f) ? 1.0f : -1.0f,
		(a.z > 0.0f) ? 1.0f : -1.0f
		);
}

inline glm::vec3 findFurthestPoint(const glm::vec3& quadSize, const std::array<glm::vec3, 3>& triangle, const glm::vec3& direction)
{
    const float d1 = glm::dot(triangle[0], direction);
    const float d2 = glm::dot(triangle[1], direction);
    const float d3 = glm::dot(triangle[2], direction);

    const glm::vec3 quadPos = fsign(-direction) * quadSize;

    if(d1 > d2)
    {
        return (d1 > d3) ? triangle[0] - quadPos : triangle[2] - quadPos;
    }
    else
    {
        return (d2 > d3) ? triangle[1] - quadPos : triangle[2] - quadPos;
    }
}

inline TrackedSimplex::Point findFurthestPointAndIndices(const glm::vec3& quadSize, const std::array<glm::vec3, 3>& triangle, const glm::vec3& direction)
{
    const float d1 = glm::dot(triangle[0], direction);
    const float d2 = glm::dot(triangle[1], direction);
    const float d3 = glm::dot(triangle[2], direction);

    const glm::vec3 quadPos = fsign(-direction) * quadSize;
    const uint32_t quadIndex = ((direction.z < 0) ? 4 : 0) + 
                               ((direction.y < 0) ? 2 : 0) + 
                               ((direction.x < 0) ? 1 : 0);

    if(d1 > d2)
    {
        return (d1 > d3) 
                ? TrackedSimplex::Point{triangle[0] - quadPos, std::make_pair(0, quadIndex)}
                : TrackedSimplex::Point{triangle[2] - quadPos, std::make_pair(2, quadIndex)};
    }
    else
    {
        return (d2 > d3) 
                ? TrackedSimplex::Point{triangle[1] - quadPos, std::make_pair(1, quadIndex)} 
                : TrackedSimplex::Point{triangle[2] - quadPos, std::make_pair(2, quadIndex)};
    }
}

float getMinDistanceNewMethod(glm::vec3 quadSize, const std::array<glm::vec3, 3>& triangle)
{
    TrackedSimplex simplex;
    simplex.type = SimplexType::POINT;
    simplex.points[0] = findFurthestPointAndIndices(quadSize, triangle, glm::vec3(1.0, 0.0, 0.0));

    glm::vec3 direction = -simplex[0];
    glm::vec3 lastDirection;
    uint32_t iter = 0;
    do {
        direction = glm::normalize(direction);
        TrackedSimplex::Point p = findFurthestPointAndIndices(quadSize, triangle, direction);
        
        if(simplex.contains(p) || glm::abs(glm::dot(direction, lastDirection) - 1.0f) < 1e-6)
        {
            return glm::dot(p.value, glm::normalize(-direction));
        }

        // Insert new point to the simplex
        simplex.type = static_cast<SimplexType>(simplex.type + 1);
        simplex.points[3] = simplex.points[2];
        simplex.points[2] = simplex.points[1];
        simplex.points[1] = simplex.points[0];
        simplex.points[0] = p;

		lastDirection = direction;

    } while(!getOriginDirection(simplex, direction, 0.0f) && ++iter < 100);

    //assert(iter < 100);
    if(iter >= 100)
    {
        SPDLOG_ERROR("GJK has done maximum iterations without solving the shape");
    }

    // It does not have next origin direction because the origin is inside the simplex
    return 0.0f;
}

float getMinDistance(glm::vec3 quadSize, 
                     const std::array<glm::vec3, 3>& triangle,
                     uint32_t* pIter)
{
    uint32_t dIter;
    uint32_t& iter = (pIter == nullptr) ? dIter : *pIter;
	iter = 0;

    Simplex simplex;
    simplex.type = SimplexType::POINT;
    simplex.points[0] = findFurthestPoint(quadSize, triangle, glm::vec3(1.0, 0.0, 0.0));

    glm::vec3 direction = -simplex.points[0];
    float dotLastEnterPoint = 0.0f;
    do {
        direction = glm::normalize(direction);
        glm::vec3 p = findFurthestPoint(quadSize, triangle, direction);
        
        dotLastEnterPoint = glm::dot(p, direction);
        if(dotLastEnterPoint - glm::dot(simplex.points[0], direction) <= 1.0e-5f)
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

bool IsNear(glm::vec3 quadSize, 
                 const std::array<glm::vec3, 3>& triangle,
                 float distThreshold,
                 uint32_t* pIter)
{
    uint32_t dIter;
    uint32_t& iter = (pIter == nullptr) ? dIter : *pIter;
	iter = 0;

    Simplex simplex;
    simplex.type = SimplexType::POINT;
    simplex.points[0] = findFurthestPoint(quadSize, triangle, glm::vec3(1.0, 0.0, 0.0));

    glm::vec3 direction = -simplex.points[0];
    float dotLastEnterPoint = 0.0f;
    do {
        direction = glm::normalize(direction);
        glm::vec3 p = findFurthestPoint(quadSize, triangle, direction);
        
        //if(dotLastEnterPoint - glm::dot(simplex.points[0], direction) <= 1.0e-5f)
        float dist = glm::dot(simplex.points[0], -direction);
        if(dist - glm::dot(p, -direction) <= 1.0e-5f || dist <= distThreshold)
        {
            return dist <= distThreshold;
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
    return true;
}

bool IsNearMinimize(glm::vec3 quadSize, 
                    const std::array<glm::vec3, 3>& triangle,
                    float distThreshold,
                    uint32_t* pIter)
{
    constexpr uint32_t MAX_ITER = 15;

    uint32_t dIter;
    uint32_t& iter = (pIter == nullptr) ? dIter : *pIter;
	iter = 0;
	float distToP;
	float distToO;
	bool isNear = false;
    const float sqDistThreshold = distThreshold * distThreshold;
    glm::vec3 currentPoint = triangle[0];

    do {
        glm::vec3 gradient = glm::normalize(-currentPoint);
        glm::vec3 p = findFurthestPoint(quadSize, triangle, gradient);
        distToP = glm::dot(gradient, p - currentPoint);
		distToO = glm::dot(gradient, -currentPoint);
        
        glm::vec3 dir = p - currentPoint;
		const float d = glm::dot(dir, -currentPoint);
		if (d < 1.0e-5) return distToO <= distToP + distThreshold;
        currentPoint += dir * glm::min(d / glm::dot(dir, dir), 1.0f);
		isNear = glm::dot(currentPoint, currentPoint) < sqDistThreshold;
    } while(!isNear && distToO <= distToP + distThreshold && ++iter < MAX_ITER);

    if(iter >= 100)
    {
        SPDLOG_ERROR("Frank-Wolfe has done maximum iterations without solving the shape");
    }

    return isNear || iter >= MAX_ITER;
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

glm::vec3 findFurthestPoint(const std::vector<std::pair<glm::vec3, float>>& spheresShape, const glm::vec3& direction)
{
    float maxValue = glm::dot(spheresShape[0].first, direction) + spheresShape[0].second;
    int maxIndex = 0;
    for(int i=1; i < spheresShape.size(); i++)
    {
        const float value = glm::dot(spheresShape[i].first, direction) + spheresShape[i].second;
        if(value > maxValue)
        {
            maxValue = value;
            maxIndex = i;
        }
    }

    return spheresShape[maxIndex].first + spheresShape[maxIndex].second * direction;
}

glm::vec3 findFurthestPoint(const std::array<glm::vec3, 3>& triangle, const glm::vec3& direction)
{
    const float d1 = glm::dot(triangle[0], direction);
    const float d2 = glm::dot(triangle[1], direction);
    const float d3 = glm::dot(triangle[2], direction);

    if(d1 > d2) return (d1 > d3) ? triangle[0] : triangle[2];
    else return (d2 > d3) ? triangle[1] : triangle[2];
}

glm::vec3 findFurthestPoint(const std::vector<std::pair<glm::vec3, float>>& spheresShape, 
                            const std::array<glm::vec3, 3>& triangle,
                            const glm::vec3& direction)
{
    return findFurthestPoint(spheresShape, direction) - findFurthestPoint(triangle, -direction);
}

bool isInsideConvexHull(const std::vector<std::pair<glm::vec3, float>>& spheresShape, 
                        const std::array<glm::vec3, 3>& triangle,
                        uint32_t* pIter)
{
    uint32_t dIter;
    uint32_t& iter = (pIter == nullptr) ? dIter : *pIter;
	iter = 0;

    Simplex simplex;
    simplex.type = SimplexType::POINT;
    simplex.points[0] = findFurthestPoint(spheresShape, triangle, glm::vec3(1.0, 0.0, 0.0));

    glm::vec3 direction = -simplex.points[0];
    do {
        direction = glm::normalize(direction);
        glm::vec3 p = findFurthestPoint(spheresShape, triangle, direction);
        
        if(glm::dot(p, direction) <= 0.0f)
        {
            return false;
        }

        // Insert new point to the simplex
        simplex.type = static_cast<SimplexType>(simplex.type + 1);
        simplex.points[3] = simplex.points[2];
        simplex.points[2] = simplex.points[1];
        simplex.points[1] = simplex.points[0];
        simplex.points[0] = p;

    } while(!getOriginDirection(simplex, direction, 0.0f) && ++iter < 100);

    //assert(iter < 100);
    if(iter >= 100)
    {
        SPDLOG_ERROR("GJK has done maximum iterations without solving the shape");
    }

    // It does not have next origin direction because the origin is inside the simplex
    return true;
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

glm::vec3 findFurthestPoint(float halfNodeSize, const std::array<float, 8>& vertRadius, const glm::vec3& direction)
{
    float maxValue = glm::dot(glm::vec3(-halfNodeSize), direction) + vertRadius[0];
    int maxIndex = 0;
    for(int i=1; i < 8; i++)
    {
        const float value = glm::dot(childrens[i] * halfNodeSize, direction) + vertRadius[i];
        if(value > maxValue)
        {
            maxValue = value;
            maxIndex = i;
        }
    }

    return childrens[maxIndex] * halfNodeSize + vertRadius[maxIndex] * direction;
}

glm::vec3 findFurthestPoint(float halfNodeSize,
                            const std::array<float, 8>& vertRadius, 
                            const std::array<glm::vec3, 3>& triangle,
                            const glm::vec3& direction)
{
    return findFurthestPoint(halfNodeSize, vertRadius, direction) - findFurthestPoint(triangle, -direction);
}

bool isInsideConvexHull(float halfNodeSize,
                        const std::array<float, 8>& vertRadius,
                        const std::array<glm::vec3, 3>& triangle,
                        glm::vec3 startDir,
                        uint32_t* pIter)
{
    uint32_t dIter;
    uint32_t& iter = (pIter == nullptr) ? dIter : *pIter;
	iter = 0;

    Simplex simplex;
    simplex.type = SimplexType::POINT;
    simplex.points[0] = findFurthestPoint(halfNodeSize, vertRadius, triangle, startDir);

    glm::vec3 direction = -simplex.points[0];
    do {
        direction = glm::normalize(direction);
        glm::vec3 p = findFurthestPoint(halfNodeSize, vertRadius, triangle, direction);
        
        if(glm::dot(p, direction) <= 0.0f)
        {
            return false;
        }

        // Insert new point to the simplex
        simplex.type = static_cast<SimplexType>(simplex.type + 1);
        simplex.points[3] = simplex.points[2];
        simplex.points[2] = simplex.points[1];
        simplex.points[1] = simplex.points[0];
        simplex.points[0] = p;

    } while(!getOriginDirection(simplex, direction, 0.0f) && ++iter < 100);

    //assert(iter < 100);
    if(iter >= 100)
    {
        SPDLOG_ERROR("GJK has done maximum iterations without solving the shape");
    }

    // It does not have next origin direction because the origin is inside the simplex
    return true;
}

bool IsNear(float halfNodeSize,
                const std::array<float, 8>& vertRadius, 
                const std::array<glm::vec3, 3>& triangle,
                float distThreshold,
                glm::vec3 startDir,
                uint32_t* pIter)
{
    uint32_t dIter;
    uint32_t& iter = (pIter == nullptr) ? dIter : *pIter;
	iter = 0;

    Simplex simplex;
    simplex.type = SimplexType::POINT;
    simplex.points[0] = findFurthestPoint(halfNodeSize, vertRadius, triangle, startDir);

    glm::vec3 direction = -simplex.points[0];
    float dotLastEnterPoint = 0.0f;
    do {
        direction = glm::normalize(direction);
        glm::vec3 p = findFurthestPoint(halfNodeSize, vertRadius, triangle, direction);
        
        //if(dotLastEnterPoint - glm::dot(simplex.points[0], direction) <= 1.0e-5f)
        float dist = glm::dot(simplex.points[0], -direction);
        if(dist - glm::dot(p, -direction) <= 1.0e-5f || dist <= distThreshold)
        {
            return dist <= distThreshold;
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
    return true;
}

bool IsNearMinimize(float halfNodeSize,
                        const std::array<float, 8>& vertRadius, 
                        const std::array<glm::vec3, 3>& triangle,
                        float distThreshold,
                        uint32_t* pIter)
{
    constexpr uint32_t MAX_ITER = 15;

    uint32_t dIter;
    uint32_t& iter = (pIter == nullptr) ? dIter : *pIter;
	iter = 0;
	float distToP;
	float distToO;
	bool isNear = false;
    const float sqDistThreshold = distThreshold * distThreshold;
    glm::vec3 currentPoint = -triangle[0];

    do {
        glm::vec3 gradient = glm::normalize(-currentPoint);
        glm::vec3 p = findFurthestPoint(halfNodeSize, vertRadius, triangle, gradient);
        distToP = glm::dot(gradient, p - currentPoint);
		distToO = glm::dot(gradient, -currentPoint);

        glm::vec3 dir = p - currentPoint;
		const float d = glm::dot(dir, -currentPoint);
		if (d < 1.0e-5) return distToO <= distToP + distThreshold;
        currentPoint += dir * glm::min(d / glm::dot(dir, dir), 1.0f);
		isNear = glm::dot(currentPoint, currentPoint) < sqDistThreshold;
    } while(!isNear && distToO <= distToP + distThreshold && ++iter < MAX_ITER);

    if(iter >= 100)
    {
        SPDLOG_ERROR("Frank-Wolfe has done maximum iterations without solving the shape");
    }

    return isNear || iter >= MAX_ITER;
}

}
}