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
    float getMinDistance(glm::vec3 quadSize, 
                         const std::array<glm::vec3, 3>& triangle, 
                         uint32_t* pIter = nullptr);
    float getMinDistanceNewMethod(glm::vec3 quadSize, const std::array<glm::vec3, 3>& triangle);
    float getMaxDistance(glm::vec3 quadSize, const std::array<glm::vec3, 3>& triangle);

    // Returns the minimum of the maximum distance between the vertices triangles and the quad
    float getMinMaxDistance(glm::vec3 quadSize, 
                            const std::array<glm::vec3, 3>& triangle);

    // Returns if the triangles is inside the convex hull defined by the spheres
    bool isInsideConvexHull(const std::vector<std::pair<glm::vec3, float>>& spheresShape, 
                            const std::array<glm::vec3, 3>& triangle,
                            uint32_t* pIter = nullptr);
}

#endif