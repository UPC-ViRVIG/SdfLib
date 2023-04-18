#include <iostream>
#include <random>
#include <algorithm>
#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "utils/Timer.h"

using namespace sdflib;

int main()
{
	srand (2222);

	glm::vec3 v1(-0.5, -0.5f, 0.0f);
	glm::vec3 v2(0.5, -0.5f, 0.0f);
	glm::vec3 v3(0.0, 0.5f, 0.0f);

	TriangleUtils::TriangleData triData(v1, v2, v3);

	auto getRandomNumber = [] () -> float
	{
		return 2.0f*(static_cast<float>(rand())/static_cast<float>(RAND_MAX)) - 1.0f;
	};

	std::vector<glm::vec3> samplePoints(1000000);
	std::generate(samplePoints.begin(), samplePoints.end(), [&] ()
	{
		return glm::vec3(
			getRandomNumber(),
			getRandomNumber(),
			getRandomNumber()
		);
	});

	Timer timer;
	timer.start();
	float total = 0.0f;
	int iter = 0;
	for(const glm::vec3& sample : samplePoints)
	{
		total += TriangleUtils::getSqDistPointAndTriangle(sample, v1, v2, v3);
	}

	std::cout << "Elapsed Time: " << timer.getElapsedSeconds() << std::endl;
	std::cout << "Iter: " << total << std::endl;

	timer.start();
	total = 0.0f;
	iter = 0;
	for(const glm::vec3& sample : samplePoints)
	{
		total += TriangleUtils::getSqDistPointAndTriangle(sample, triData);
	}

	
	std::cout << "Elapsed Time: " << timer.getElapsedSeconds() << std::endl;
	std::cout << "Iter: " << total << std::endl;

	for(const glm::vec3& sample : samplePoints)
	{
		assert(glm::abs(TriangleUtils::getSqDistPointAndTriangle(sample, v1, v2, v3) - TriangleUtils::getSqDistPointAndTriangle(sample, triData)) < 0.001f);
		const float aux = TriangleUtils::getSignedDistPointAndTriangle(sample, triData);
		assert(glm::abs(aux * aux - TriangleUtils::getSqDistPointAndTriangle(sample, triData)) < 0.001f);
	}
}