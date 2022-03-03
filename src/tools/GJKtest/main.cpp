#include <spdlog/spdlog.h>
#include <array>
#include "utils/GJK.h"

float distToQuad(glm::vec3 point, glm::vec3 quadMin, glm::vec3 quadMax)
{
    const glm::vec3 center = 0.5f * (quadMin + quadMax);
    const glm::vec3 size = 0.5f * (quadMax - quadMin);
    
    glm::vec3 aux = glm::abs(point - center) - size;

    aux = glm::max(glm::vec3(0.0f), aux);
    return glm::length(aux);
}

int main()
{
    auto getRandomNumber = [] () -> float
	{
		return 2.0f*(static_cast<float>(rand())/static_cast<float>(RAND_MAX)) - 1.0f;
	};

    auto getRandomPoint = [&] () -> glm::vec3
    {
        return glm::vec3(
			getRandomNumber(),
			getRandomNumber(),
			getRandomNumber()
		);
    };

    // Test quad with a point
    for(size_t sample=0; sample < 1000; sample++)
    {
        std::vector<glm::vec3> quad(8);
        const glm::vec3 p1 = getRandomPoint();
        const glm::vec3 p2 = getRandomPoint();
        quad[0] = glm::min(p1, p2);
        quad[7] = glm::max(p1, p2);

        quad[1] = glm::vec3(quad[0].x, quad[0].y, quad[7].z);
        quad[2] = glm::vec3(quad[0].x, quad[7].y, quad[0].z);
        quad[3] = glm::vec3(quad[0].x, quad[7].y, quad[7].z);
        quad[4] = glm::vec3(quad[7].x, quad[0].y, quad[7].z);
        quad[5] = glm::vec3(quad[7].x, quad[7].y, quad[0].z);
        quad[6] = glm::vec3(quad[7].x, quad[7].y, quad[7].z);

        std::vector<glm::vec3> point(1);
        point[0] = getRandomPoint();
        
        float expectedDist = distToQuad(point[0], quad[0], quad[7]);
        float gjkDist = GJK::getMinDistance(quad, point);

        assert(glm::abs(expectedDist - gjkDist) < 0.001);
    }
}