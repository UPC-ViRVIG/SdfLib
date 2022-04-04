#ifndef TRIANGLE_UTILS_H
#define TRIANGLE_UTILS_H

#include <glm/glm.hpp>
#include <vector>
#include <array>
#include <map>
#include <unordered_map>
#include <iostream>
#include <spdlog/spdlog.h>
#include "Mesh.h"

namespace TriangleUtils
{
    struct TriangleData
    {
        TriangleData() {}
        TriangleData(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3)
        {
            origin = v1;

            glm::vec3 sx = glm::normalize(v2 - v1);
            glm::vec3 sz = glm::normalize(glm::cross(v2 - v1, v3 - v1));
            glm::vec3 sy = glm::cross(sz, sx);

            transform = glm::inverse(glm::mat3x3(sx, sy, sz));

            b = glm::normalize(glm::vec2(transform * (v3 - v2)));

            c = glm::normalize(glm::vec2(transform * (v1 - v3)));

            this->v2 = (transform * (v2-origin)).x;
            this->v3 = glm::vec2(transform * (v3-origin));

            edgesNormal = {glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f) };
            verticesNormal = { glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f) };
        }

        glm::vec3 getTriangleNormal() const
        {
            return glm::vec3(transform[0][2], transform[1][2], transform[2][2]);
        }

        glm::vec3 origin;
        glm::mat3x3 transform;

        // Edge direction normalized in the triangle space
        // The a vector is always the x-axis
        glm::vec2 b;
        glm::vec2 c;

        // Vertices position in triangle space
        // v1 is always at the origin
        float v2; // In x-axis
        glm::vec2 v3;

        // Triangle normals transformed
        std::array<glm::vec3, 3> edgesNormal;
        std::array<glm::vec3, 3> verticesNormal;
    };

    inline std::vector<TriangleData> calculateMeshTriangleData(const Mesh& mesh)
    {
        const std::vector<glm::vec3>& vertices = mesh.getVertices();
        const std::vector<uint32_t> indices = mesh.getIndices();

        std::vector<TriangleData> triangles(indices.size()/3);

        // Cache structures
        std::map<std::pair<uint32_t, uint32_t>, uint32_t> edgesNormal;
        std::vector<glm::vec3> verticesNormal(vertices.size(), glm::vec3(0.0f));

        // Init triangles
		for (int i = 0, tIndex = 0; i < indices.size(); i += 3, tIndex++)
		{
			triangles[tIndex] = TriangleUtils::TriangleData(vertices[indices[i]], vertices[indices[i + 1]], vertices[indices[i + 2]]);
		}

        for(int i = 0, tIndex = 0; i < indices.size(); i += 3, tIndex++)
        {
            for(int k=0; k < 3; k++)
            {
                const uint32_t v1 = indices[i + k];
                const uint32_t v2 = indices[i + ((k+1) % 3)];
                const uint32_t v3 = indices[i + ((k+2) % 3)];
                std::pair<std::map<std::pair<uint32_t, uint32_t>, uint32_t>::iterator, bool> ret;
                ret = edgesNormal.insert(std::make_pair(
                                            std::make_pair(glm::min(v1, v2), glm::max(v1, v2)), i + k));
                if(!ret.second)
                {
					const uint32_t t2Index = ret.first->second / 3;
                    glm::vec3 edgeNormal = triangles[tIndex].getTriangleNormal() + triangles[t2Index].getTriangleNormal();
                    triangles[tIndex].edgesNormal[k] = triangles[tIndex].transform * edgeNormal;
                    triangles[t2Index].edgesNormal[ret.first->second % 3] = triangles[t2Index].transform * edgeNormal;
                    edgesNormal.erase(ret.first);
                }

                const float angle = glm::acos(glm::clamp(glm::dot(glm::normalize(vertices[v2] - vertices[v1]), glm::normalize(vertices[v3] - vertices[v1])), -1.0f, 1.0f));
                verticesNormal[v1] += angle * triangles[tIndex].getTriangleNormal();
            }
        }

        if(edgesNormal.size() > 0)
        {
            SPDLOG_INFO("The mesh has {} non-maifold edges, trying to merge near vertices", edgesNormal.size());
            std::map<uint32_t, uint32_t> verticesMap;
            auto findVertexParent = [&] (uint32_t vId) -> uint32_t
            {
                auto it = verticesMap.find(vId);
                while(it != verticesMap.end() && it->second != vId)
                {
                    vId = it->second;
                    it = verticesMap.find(vId);
                }
                return vId;
            };

            std::vector<uint32_t> nonManifoldVertices(2 * edgesNormal.size());
            uint32_t index = 0;
            for(auto& elem : edgesNormal)
            {
                nonManifoldVertices[index++] = elem.first.first;
                nonManifoldVertices[index++] = elem.first.second;
            }

            std::sort(nonManifoldVertices.begin(), nonManifoldVertices.end());
            auto newEnd = std::unique(nonManifoldVertices.begin(), nonManifoldVertices.end());
            nonManifoldVertices.erase(newEnd, nonManifoldVertices.end());

            // Generate a possible vertex mapping
            for(uint32_t i=0; i < nonManifoldVertices.size(); i++)
            {
                const glm::vec3& v1 = vertices[nonManifoldVertices[i]];
                for(uint32_t ii=i+1; ii < nonManifoldVertices.size(); ii++)
                {
                    const glm::vec3& diff = v1 - vertices[nonManifoldVertices[ii]];
                    if(glm::dot(diff, diff) < 0.000001f)
                    {
                        uint32_t p1 = findVertexParent(nonManifoldVertices[i]);
                        uint32_t p2 = findVertexParent(nonManifoldVertices[ii]);

                        if(nonManifoldVertices[i] == p1) verticesMap[p1] = p1;
                        verticesMap[p2] = p1;
                        break;
                    }
                }
            }

            std::map<std::pair<uint32_t, uint32_t>, uint32_t> newEdgesNormals;
            auto it = edgesNormal.begin();
            for(; it != edgesNormal.end(); it++)
            {
                const uint32_t v1 = findVertexParent(it->first.first);
                const uint32_t v2 = findVertexParent(it->first.second);

                std::pair<std::map<std::pair<uint32_t, uint32_t>, uint32_t>::iterator, bool> ret;
                ret = newEdgesNormals.insert(std::make_pair(
                                            std::make_pair(glm::min(v1, v2), glm::max(v1, v2)), it->second));
                if(!ret.second)
                {
                    const uint32_t tIndex = it->second / 3;
					const uint32_t t2Index = ret.first->second / 3;
                    glm::vec3 edgeNormal = triangles[tIndex].getTriangleNormal() + triangles[t2Index].getTriangleNormal();
                    triangles[tIndex].edgesNormal[it->second % 3] = triangles[tIndex].transform * edgeNormal;
                    triangles[t2Index].edgesNormal[ret.first->second % 3] = triangles[t2Index].transform * edgeNormal;
                    newEdgesNormals.erase(ret.first);
                }
            }

            // Calculate parent real normals
            for(uint32_t vId : nonManifoldVertices)
            {
                uint32_t p = findVertexParent(vId);
                if(p != vId) verticesNormal[p] += verticesNormal[vId];
            }

            // Propagate parent normals
            for(uint32_t vId : nonManifoldVertices)
            {
                uint32_t p = findVertexParent(vId);
                verticesNormal[vId] = verticesNormal[p];
            }

            if(newEdgesNormals.size() > 0)
            {
                SPDLOG_ERROR("The mesh has {} non-maifold edges that cannot be merged", edgesNormal.size());
            }
            else
            {
                SPDLOG_INFO("All the non-manifold vertices merged correctly");
            }
        }

        for(int i = 0; i < indices.size(); i++)
        {
            triangles[i/3].verticesNormal[i % 3] = triangles[i/3].transform * verticesNormal[indices[i]];
        }

        return triangles;
    }

    inline float getSqDistPointAndTriangle(glm::vec3 point, const TriangleData& data)
    {
        glm::vec3 projPoint = data.transform * (point - data.origin);

        const float de1 = -projPoint.y;
        const float de2 = (projPoint.x - data.v2) * data.b.y - projPoint.y * data.b.x;
        const float de3 = projPoint.x * data.c.y - projPoint.y * data.c.x;

        if(de1 >= 0)
        {
            if(projPoint.x <= 0) // Its near v1
            {
                return glm::dot(projPoint, projPoint);
            }
            else if(projPoint.x >= data.v2) // Its near v2
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v2, 0.0, 0.0);
                return glm::dot(p, p);
            }
            else // Its near edge 1
            {
                return de1 * de1 + projPoint.z * projPoint.z;
            }
        }
        else if(de2 >= 0)
        {
            if((projPoint.x - data.v2) * data.b.x + projPoint.y * data.b.y <= 0) // Its near v2
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v2, 0.0, 0.0);
                return glm::dot(p, p);
            }
            else if((projPoint.x - data.v3.x) * data.b.x + (projPoint.y - data.v3.y) * data.b.y >= 0) // Its near v3
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v3.x, data.v3.y, 0.0);
                return glm::dot(p, p);
            }
            else // Its near edge 2
            {
                return de2 * de2 + projPoint.z * projPoint.z;
            }
        }
        else if(de3 >= 0)
        {
            if(projPoint.x * data.c.x + projPoint.y * data.c.y >= 0) // Its near v1
            {
                return glm::dot(projPoint, projPoint);
            }
            else if((projPoint.x - data.v3.x) * data.c.x + (projPoint.y - data.v3.y) * data.c.y <= 0) // Its near v3
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v3.x, data.v3.y, 0.0);
                return glm::dot(p, p);
            }
            else // Its near edge 3
            {
                return de3 * de3 + projPoint.z * projPoint.z;
            }
        }

        return projPoint.z * projPoint.z;
    }

    inline float getSignedDistPointAndTriangle(glm::vec3 point, const TriangleData& data)
    {
        glm::vec3 projPoint = data.transform * (point - data.origin);

        const float de1 = -projPoint.y;
        const float de2 = (projPoint.x - data.v2) * data.b.y - projPoint.y * data.b.x;
        const float de3 = projPoint.x * data.c.y - projPoint.y * data.c.x;

        if(de1 >= 0)
        {
            if(projPoint.x <= 0) // Its near v1
            {
                return glm::sign(glm::dot(data.verticesNormal[0], projPoint)) * glm::sqrt(glm::dot(projPoint, projPoint));
            }
            else if(projPoint.x >= data.v2) // Its near v2
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v2, 0.0, 0.0);
                return glm::sign(glm::dot(data.verticesNormal[1], p)) * glm::sqrt(glm::dot(p, p));
            }
            else // Its near edge 1
            {
                return glm::sign(glm::dot(data.edgesNormal[0], projPoint)) * glm::sqrt(de1 * de1 + projPoint.z * projPoint.z);
            }
        }
        else if(de2 >= 0)
        {
            if((projPoint.x - data.v2) * data.b.x + projPoint.y * data.b.y <= 0) // Its near v2
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v2, 0.0, 0.0);
                return glm::sign(glm::dot(data.verticesNormal[1], p)) * glm::sqrt(glm::dot(p, p));
            }
            else if((projPoint.x - data.v3.x) * data.b.x + (projPoint.y - data.v3.y) * data.b.y >= 0) // Its near v3
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v3.x, data.v3.y, 0.0);
                return glm::sign(glm::dot(data.verticesNormal[2], p)) * glm::sqrt(glm::dot(p, p));
            }
            else // Its near edge 2
            {
                return glm::sign(glm::dot(data.edgesNormal[1], projPoint - glm::vec3(data.v2, 0.0f, 0.0f))) * glm::sqrt(de2 * de2 + projPoint.z * projPoint.z);
            }
        }
        else if(de3 >= 0)
        {
            if(projPoint.x * data.c.x + projPoint.y * data.c.y >= 0) // Its near v1
            {
                return glm::sign(glm::dot(data.verticesNormal[0], projPoint)) * glm::sqrt(glm::dot(projPoint, projPoint));
            }
            else if((projPoint.x - data.v3.x) * data.c.x + (projPoint.y - data.v3.y) * data.c.y <= 0) // Its near v3
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v3.x, data.v3.y, 0.0);
                return glm::sign(glm::dot(data.verticesNormal[2], p)) * glm::sqrt(glm::dot(p, p));
            }
            else // Its near edge 3
            {
                return glm::sign(glm::dot(data.edgesNormal[2], projPoint)) * glm::sqrt(de3 * de3 + projPoint.z * projPoint.z);
            }
        }

        return projPoint.z;
    }

    inline float dot2(glm::vec3 v)
    {
        return glm::dot(v, v);
    }

    inline float getSqDistPointAndTriangle(glm::vec3 p, glm::vec3 a, glm::vec3 b, glm::vec3 c)
    {
        glm::vec3 ba = b - a; glm::vec3 pa = p - a;
        glm::vec3 cb = c - b; glm::vec3 pb = p - b;
        glm::vec3 ac = a - c; glm::vec3 pc = p - c;

        glm::vec3 normal = glm::cross(ba, ac);

        return (glm::sign(glm::dot(glm::cross(ba, normal), pa)) +
                glm::sign(glm::dot(glm::cross(cb, normal), pb)) +
                glm::sign(glm::dot(glm::cross(ac, normal), pc)) < 2.0f)
                ?
                glm::min(glm::min(
                    dot2(ba * glm::clamp(glm::dot(ba, pa) / dot2(ba), 0.0f, 1.0f)-pa),
                    dot2(cb * glm::clamp(glm::dot(cb, pb) / dot2(cb), 0.0f, 1.0f)-pb)),
                    dot2(ac * glm::clamp(glm::dot(ac, pc) / dot2(ac), 0.0f, 1.0f)-pc))
                :
                glm::dot(normal, pa) * glm::dot(normal, pa) / glm::dot(normal, normal);
    }
}

#endif