#include "TriangleUtils.h"

namespace TriangleUtils
{
    std::vector<TriangleData> calculateMeshTriangleData(const Mesh& mesh)
    {
        const std::vector<glm::vec3>& vertices = mesh.getVertices();
        const std::vector<uint32_t> indices = mesh.getIndices();

        std::vector<TriangleData> triangles(indices.size()/3);
        std::vector<uint32_t> degeneratedTriangles;

        // Cache structures
        std::map<std::pair<uint32_t, uint32_t>, uint32_t> edgesNormal;
        std::vector<glm::vec3> verticesNormal(vertices.size(), glm::vec3(0.0f));

        // Init triangles
        uint32_t numAreaZeroTriangles = 0;
		for (int i = 0, tIndex = 0; i < indices.size(); i += 3, tIndex++)
		{
            // Mark area zero triangles
            const double zeroAngleThreshold = 5e-5;
            double triangleArea = 0.5f * glm::length(glm::cross(static_cast<glm::dvec3>(vertices[indices[i + 1]] - vertices[indices[i]]), static_cast<glm::dvec3>(vertices[indices[i + 2]] - vertices[indices[i]])));
            for(int k=0; k < 3; k++)
            {
                const uint32_t v1 = indices[i + k];
                const uint32_t v2 = indices[i + ((k+1) % 3)];
            }
            isAngleZeroTriangle[tIndex] = area < zeroAngleThreshold;
            if(area < zeroAngleThreshold) numAreaZeroTriangles++;

			triangles[tIndex] = TriangleUtils::TriangleData(vertices[indices[i]], vertices[indices[i + 1]], vertices[indices[i + 2]]);
		}

        if(numAreaZeroTriangles > 0)
        {
            SPDLOG_INFO("The mesh has {} zero angle triangles", numAreaZeroTriangles);
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

                    const bool t1IsAreaZero = isAngleZeroTriangle[tIndex] && !isAngleZeroTriangle[t2Index];
                    const bool t2IsAreaZero = isAngleZeroTriangle[t2Index] && !isAngleZeroTriangle[tIndex];
                    glm::vec3 edgeNormal = ((!t1IsAreaZero) ? triangles[tIndex].getTriangleNormal() : glm::vec3(0.0f)) + 
                                           ((!t2IsAreaZero) ? triangles[t2Index].getTriangleNormal() : glm::vec3(0.0f));

                    triangles[tIndex].edgesNormal[k] = triangles[tIndex].transform * edgeNormal;
                    triangles[t2Index].edgesNormal[ret.first->second % 3] = triangles[t2Index].transform * edgeNormal;
                    edgesNormal.erase(ret.first);
                }

                if(!isAngleZeroTriangle[tIndex])
                {
                    const float angle = glm::acos(glm::clamp(glm::dot(glm::normalize(vertices[v2] - vertices[v1]), glm::normalize(vertices[v3] - vertices[v1])), -1.0f, 1.0f));
                    verticesNormal[v1] += angle * triangles[tIndex].getTriangleNormal();
                }
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
                    if(glm::dot(diff, diff) < 0.00001f)
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
}