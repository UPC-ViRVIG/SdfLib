#ifndef GJK_H
#define GJK_H

#include <array>
#include <vector>
#include <glm/glm.hpp>

namespace GJK
{
    void getMinDistance(std::vector<glm::vec3>& e1, std::vector<glm::vec3>& e2);
    void getMaxDistance(std::vector<glm::vec3>& e1, std::vector<glm::vec3>& e2);
}

#endif