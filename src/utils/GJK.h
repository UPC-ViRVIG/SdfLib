#ifndef GJK_H
#define GJK_H

#include <array>
#include <vector>
#include <glm/glm.hpp>

namespace GJK
{
    float getMinDistance(const std::vector<glm::vec3>& e1, const std::vector<glm::vec3>& e2);
    float getMaxDistance(const std::vector<glm::vec3>& e1, const std::vector<glm::vec3>& e2);
}

#endif