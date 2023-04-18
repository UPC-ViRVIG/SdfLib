#include <spdlog/spdlog.h>
#include <array>
#include "utils/GJK.h"
#include <iostream>

using namespace sdflib;

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

	//Test quad with a point
	// for(size_t sample=0; sample < 2000000; sample++)
	// {
	// 	std::vector<glm::vec3> quad(8);
	// 	const glm::vec3 p1 = getRandomPoint();
	// 	const glm::vec3 p2 = getRandomPoint();
	// 	quad[0] = glm::min(p1, p2);
	// 	quad[7] = glm::max(p1, p2);

	// 	quad[1] = glm::vec3(quad[0].x, quad[0].y, quad[7].z);
	// 	quad[2] = glm::vec3(quad[0].x, quad[7].y, quad[0].z);
	// 	quad[3] = glm::vec3(quad[0].x, quad[7].y, quad[7].z);
	// 	quad[4] = glm::vec3(quad[7].x, quad[0].y, quad[0].z);
	// 	quad[5] = glm::vec3(quad[7].x, quad[0].y, quad[7].z);
	// 	quad[6] = glm::vec3(quad[7].x, quad[7].y, quad[0].z);

	// 	std::vector<glm::vec3> point(1);
	// 	point[0] = getRandomPoint();
      
	// 	float expectedDist = distToQuad(point[0], quad[0], quad[7]);
	// 	float gjkDist = GJK::getMinDistance(quad, point);

	// 	if (glm::abs(expectedDist - gjkDist) > 0.001)
	// 	{
	// 		float expectedDist1 = distToQuad(point[0], quad[0], quad[7]);
	// 		float gjkDist1 = GJK::getMinDistance(quad, point);
	// 		assert(false);
	// 	}
	// 	assert(glm::abs(expectedDist - gjkDist) < 0.001);
	// }

	// // Test quad with a triangle
	// for(size_t sample=0; sample < 1000; sample++)
	// {
	// 	std::vector<glm::vec3> quad(8);
	// 	const glm::vec3 p1 = getRandomPoint();
	// 	const glm::vec3 p2 = getRandomPoint();
	// 	quad[0] = glm::min(p1, p2);
	// 	quad[7] = glm::max(p1, p2);

	// 	quad[1] = glm::vec3(quad[0].x, quad[0].y, quad[7].z);
	// 	quad[2] = glm::vec3(quad[0].x, quad[7].y, quad[0].z);
	// 	quad[3] = glm::vec3(quad[0].x, quad[7].y, quad[7].z);
	// 	quad[4] = glm::vec3(quad[7].x, quad[0].y, quad[0].z);
	// 	quad[5] = glm::vec3(quad[7].x, quad[0].y, quad[7].z);
	// 	quad[6] = glm::vec3(quad[7].x, quad[7].y, quad[0].z);

	// 	std::vector<glm::vec3> triangle(3);
	// 	triangle[0] = getRandomPoint();
	// 	triangle[1] = getRandomPoint();
	// 	triangle[2] = getRandomPoint();
      
	// 	float expectedDist = INFINITY;
	// 	for(size_t i=0; i < 100000; i++)
	// 	{
	// 		glm::vec3 c = 0.5f * (getRandomPoint() + 1.0f);
	// 		c = c / (c.x + c.y + c.z);
	// 		assert(glm::abs((c.x + c.y + c.z) - 1.0) < 0.001);

	// 		glm::vec3 p = triangle[0] * c.x + triangle[1] * c.y + triangle[2] * c.z;
	// 		expectedDist = glm::min(expectedDist, distToQuad(p, quad[0], quad[7]));
	// 	}

	// 	expectedDist = glm::min(expectedDist, distToQuad(triangle[0], quad[0], quad[7]));
	// 	expectedDist = glm::min(expectedDist, distToQuad(triangle[1], quad[0], quad[7]));
	// 	expectedDist = glm::min(expectedDist, distToQuad(triangle[2], quad[0], quad[7]));
      
	// 	float gjkDist = GJK::getMinDistance(quad, triangle);

	// 	if (std::isnan(gjkDist))
	// 	{
	// 		float gjkDist1 = GJK::getMinDistance(quad, triangle);
	// 		assert(false);
	// 	}

	// 	assert(gjkDist - expectedDist < 1e-4);
	// 	assert(gjkDist - expectedDist > -0.02);
	// }

	// Test if GJK is cummutative
	for(size_t sample=0; sample < 2000000; sample++)
	{
		std::vector<glm::vec3> quad(8);
		const glm::vec3 p1 = getRandomPoint();
		const glm::vec3 p2 = getRandomPoint();
		quad[0] = glm::min(p1, p2);
		quad[7] = glm::max(p1, p2);

		glm::vec3 quadSize = 0.5f * (quad[7] - quad[0]);
		glm::vec3 quadCenter = 0.5f * (quad[7] + quad[0]);

		quad[1] = glm::vec3(quad[0].x, quad[0].y, quad[7].z);
		quad[2] = glm::vec3(quad[0].x, quad[7].y, quad[0].z);
		quad[3] = glm::vec3(quad[0].x, quad[7].y, quad[7].z);
		quad[4] = glm::vec3(quad[7].x, quad[0].y, quad[0].z);
		quad[5] = glm::vec3(quad[7].x, quad[0].y, quad[7].z);
		quad[6] = glm::vec3(quad[7].x, quad[7].y, quad[0].z);

		std::vector<glm::vec3> triangle(3);
		triangle[0] = getRandomPoint();
		triangle[1] = getRandomPoint();
		triangle[2] = getRandomPoint();

		std::array<glm::vec3, 3> triangleArray;
		triangleArray[0] = triangle[0] - quadCenter;
		triangleArray[1] = triangle[1] - quadCenter;
		triangleArray[2] = triangle[2] - quadCenter;
        

		float gjkDist1 = GJK::getMinDistance(quad, triangle);
		float gjkDist2 = GJK::getMinDistance(triangle, quad);

		if (glm::abs(gjkDist1 - gjkDist2) > 0.001)
		{
			float gjkDist11 = GJK::getMinDistance(quad, triangle);
			float gjkDist21 = GJK::getMinDistance(triangle, quad);
			assert(false);
		}
		assert(glm::abs(gjkDist1 - gjkDist2) < 0.001);

		float newGjkDist = GJK::getMinDistance(quadSize, triangleArray);
		if(glm::abs(gjkDist1 - newGjkDist) > 0.001)
		{
			float gjkDist11 = GJK::getMinDistance(quad, triangle);
			float newGjkDist1 = GJK::getMinDistance(quadSize, triangleArray);
			assert(false);
		}
		assert(glm::abs(gjkDist1 - newGjkDist) < 0.001);

		newGjkDist = GJK::getMinDistanceNewMethod(quadSize, triangleArray);
		if (glm::abs(gjkDist1 - newGjkDist) > 0.001)
		{
			float gjkDist11 = GJK::getMinDistance(quadSize, triangleArray);
			float newGjkDist1 = GJK::getMinDistanceNewMethod(quadSize, triangleArray);
			assert(false);
		}
		assert(glm::abs(gjkDist1 - newGjkDist) < 0.001);
	}
}