#ifndef GJK_H
#define GJK_H

#include <array>
#include <vector>
#include <glm/glm.hpp>

namespace GJK
{
    float getMinDistance(const std::vector<glm::vec3>& e1, const std::vector<glm::vec3>& e2);
    float getMaxDistance(const std::vector<glm::vec3>& e1, const std::vector<glm::vec3>& e2);

    // Triangle quad specialization
    float getMinDistance(glm::vec3 quadSize, const std::array<glm::vec3, 3>& triangle);
    float getMaxDistance(glm::vec3 quadSize, const std::array<glm::vec3, 3>& triangle);
}

#endif