#include "GJK.h"

namespace GJK
{

bool getOriginDirection(std::array<glm::vec3, 4> points, glm::vec3 outDirection)
{

    return false;
}

bool getOriginDirection(std::array<glm::vec3, 3> points, glm::vec3 outDirection)
{


    return false;
}

bool getOriginDirection(std::array<glm::vec3, 2> points, glm::vec3 outDirection)
{
    glm::vec3 ab = points[1] - points[0];

    return false;
}

glm::vec3 findFurthestPoint(const std::vector<glm::vec3>& e, glm::vec3 direction)
{
    float maxValue = -INFINITY;
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

void getMinDistance(const std::vector<glm::vec3>& e1, const std::vector<glm::vec3>& e2)
{

}

}