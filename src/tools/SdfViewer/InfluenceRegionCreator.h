#ifndef INFLUENCE_REGION_CREATOR_H
#define INFLUENCE_REGION_CREATOR_H

#include "SdfLib/utils/Mesh.h"
#include "SdfLib/utils/TriangleUtils.h"
#include "SdfLib/utils/PrimitivesFactory.h"

using namespace sdflib;

namespace InfluenceRegionCreator
{

namespace
{
    glm::vec3 findSupportPoint(const std::vector<std::pair<glm::vec3, float>> region, glm::vec3 direction)
    {
		glm::vec3 maxPoint(0.0f);
        float regionDist = -INFINITY;
        for(const std::pair<glm::vec3, float>& sphere : region)
        {
            const float d = glm::dot(sphere.first, direction) + sphere.second;
            if(regionDist < d)
            {
                regionDist = d; maxPoint = sphere.first + direction * sphere.second;
            }
        }

        return maxPoint;
    }

    float getMinDistance(glm::vec3 point, const std::vector<uint32_t>& triangles, 
                         const std::vector<TriangleUtils::TriangleData>& trianglesData)
    {
        float minDist = INFINITY;
        uint32_t minIndex = 0;
        for(uint32_t t : triangles)
        {
            const float dist = glm::abs(TriangleUtils::getSqDistPointAndTriangle(point, trianglesData[t]));
            if(minDist > dist)
            {
                minDist = glm::min(minDist, dist);
                minIndex = t;
            }
        }

        return glm::sqrt(minDist);
    }

    bool intersection(glm::vec3 rayPos, glm::vec3 rayDir, glm::vec3 spherePos, float sphereRadius, float& outT)
    {
        glm::vec3 m = rayPos - spherePos;
        float a = glm::dot(rayDir, rayDir);
        float b = 2.0f * glm::dot(m, rayDir);
        float c = glm::dot(m, m) - sphereRadius * sphereRadius;
        
        float disc = b * b - 4.0f * a * c;
        if(disc < 0.0f) return false;

        outT = glm::max(-0.5 * (b + glm::sqrt(disc)) / a, -0.5 * (b - glm::sqrt(disc)) / a);
        return true;
    }
}

void createConvexHull(Mesh& inOutMesh, glm::vec3 center, const std::vector<std::pair<glm::vec3, float>> region)
{
    for(glm::vec3& vert : inOutMesh.getVertices())
    {
        vert = center + findSupportPoint(region, glm::normalize(vert));
    }
	inOutMesh.computeBoundingBox();
	inOutMesh.computeNormals();
}

void createOptimalRegion(Mesh& inOutMesh, glm::vec3 center, float halfNodeSize, 
                         const std::vector<uint32_t>& triangles, 
                         const std::vector<TriangleUtils::TriangleData>& trianglesData)
{
    auto samplePoints = PrimitivesFactory::getIsosphere(8);
    std::vector<float> radius(samplePoints->getVertices().size());
    for(uint32_t i=0; i < samplePoints->getVertices().size(); i++)
    {
        glm::vec3& vec = samplePoints->getVertices()[i];
        vec = glm::normalize(vec);
        glm::vec3 aux = glm::abs(vec);
        if(aux.x > aux.y)
        {
            if(aux.x > aux.z) vec = vec * glm::abs(halfNodeSize / aux.x);
            else vec = vec * glm::abs(halfNodeSize / aux.z);
        }
        else
        {
            if(aux.y > aux.z) vec = vec * glm::abs(halfNodeSize / aux.y);
            else vec = vec * glm::abs(halfNodeSize / aux.z);
        }
        vec += center;
        radius[i] = getMinDistance(vec, triangles, trianglesData);
    }

    for(glm::vec3& vec : inOutMesh.getVertices())
    {
        // Calculate node gradient in vertex position
        vec = glm::normalize(vec);
        glm::vec3 aux = glm::abs(vec);
		   glm::vec3 distToSmallNode = glm::sign(vec) * glm::max(glm::vec3(0.0f), aux - 0.25f);
        glm::vec3 gradient = glm::normalize(distToSmallNode);

		glm::vec3 samplePoint = center + (vec - distToSmallNode) * 4.0f * halfNodeSize;

        // Search minimum distance
        float maxT = 0.0f;
        for(uint32_t i=0; i < samplePoints->getVertices().size(); i++)
        {
            float t;
            if(intersection(samplePoint, gradient, samplePoints->getVertices()[i], radius[i], t))
            {
                maxT = glm::max(maxT, t);
            }
        }

        vec = samplePoint + gradient * maxT;
    }

    inOutMesh.computeBoundingBox();
	inOutMesh.computeNormals();
}
}
#endif