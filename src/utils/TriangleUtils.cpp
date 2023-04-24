#include "SdfLib/utils/TriangleUtils.h"

namespace sdflib
{
namespace TriangleUtils
{
    std::vector<TriangleData> calculateMeshTriangleData(const Mesh& mesh)
    {
        const std::vector<glm::vec3>& vertices = mesh.getVertices();
        const std::vector<uint32_t> indices = mesh.getIndices();

        std::vector<TriangleData> triangles(indices.size()/3);
        std::vector<bool> isTriangleDegenerated(indices.size()/3, false);
        std::vector<std::pair<uint32_t, uint32_t>> degeneratedTriangles; // Stores triangle index and vertex index with bigger angle

        // Cache structures
        std::map<std::pair<uint32_t, uint32_t>, uint32_t> edgesNormal;
        std::vector<glm::vec3> verticesNormal(vertices.size(), glm::vec3(0.0f));

        // Init triangles
		for (int i = 0, tIndex = 0; i < indices.size(); i += 3, tIndex++)
		{
            // Mark area zero triangles
            const double zeroAngleThreshold = 1e-6;
            const double degeneratedTriangleValue = 0.006;
            double triangleArea = 0.5f * glm::length(glm::cross(static_cast<glm::dvec3>(vertices[indices[i + 1]] - vertices[indices[i]]), static_cast<glm::dvec3>(vertices[indices[i + 2]] - vertices[indices[i]])));
            double maxTriangleBase = 0.0f;
            uint32_t degeneratedVertex = 0;
            for(int k=0; k < 3; k++)
            {
                const uint32_t v1 = indices[i + k];
                const uint32_t v2 = indices[i + ((k+1) % 3)];

                double triangleBase = glm::length(vertices[v2] - vertices[v1]);
                if(triangleBase > maxTriangleBase)
                {
                    maxTriangleBase = triangleBase;
                    degeneratedVertex = k;
                }
            }

            const double maxTriangleHeighValue = glm::tan(glm::radians(30.0));
            double triangleDegerancyValue = 2.0f * triangleArea * maxTriangleHeighValue / (maxTriangleBase * maxTriangleBase);

            if(false && triangleArea < zeroAngleThreshold && triangleDegerancyValue < degeneratedTriangleValue)
            {
                degeneratedTriangles.push_back(std::make_pair(tIndex, degeneratedVertex));
                isTriangleDegenerated[tIndex] = true;
                triangles[tIndex] = TriangleUtils::TriangleData(vertices[indices[i]], vertices[indices[i + 1]], vertices[indices[i + 2]]);
                triangles[tIndex].transform[0][2] = 0.0f; triangles[tIndex].transform[1][2] = 0.0f; triangles[tIndex].transform[2][2] = 0.0f;
            }
            else
            {
                triangles[tIndex] = TriangleUtils::TriangleData(vertices[indices[i]], vertices[indices[i + 1]], vertices[indices[i + 2]]);
            }
		}

        if(degeneratedTriangles.size() > 0)
        {
            SPDLOG_INFO("The mesh has {} degenerated triangles", degeneratedTriangles.size());
        }

        for(int i = 0, tIndex = 0; i < indices.size(); i += 3, tIndex++)
        {
            if(isTriangleDegenerated[tIndex]) continue;
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

        uint32_t iter = 0;
        std::map<std::pair<uint32_t, uint32_t>, glm::vec3> validDegeneratedEdgeNormals;
        while(degeneratedTriangles.size() > 0 && iter++ <= 10)
        {
            std::vector<std::pair<uint32_t, uint32_t>> newDegeneratedTriangles(0);
            std::map<std::pair<uint32_t, uint32_t>, glm::vec3> newValidDegeneratedEdgeNormals;
            for(std::pair<uint32_t, uint32_t> degTri : degeneratedTriangles)
            {
                const uint32_t tIndex = degTri.first;

                // Array for storing edge info (edge length, edge vertices indices)
                std::array<std::pair<float, std::pair<uint32_t, uint32_t>>, 3> triVertices; 
                for(int k=0; k < 3; k++)
                {
                    const uint32_t v1 = indices[3 * tIndex + k];
                    const uint32_t v2 = indices[3 * tIndex + ((k+1) % 3)];

                    float triangleBase = glm::length(vertices[v2] - vertices[v1]);
                    triVertices[k] = std::make_pair(triangleBase, std::make_pair(v1, v2));
                }

                std::sort(triVertices.begin(), triVertices.end(), std::greater<std::pair<float, std::pair<uint32_t, uint32_t>>>());

                // Iterate the two larger edges
                glm::vec3 edgeNormal(0.0f);
                bool validNeighbours = true;
                bool someValidNeighbour = false;
                std::array<uint32_t, 3> adjEdgeFaceIndices;
                adjEdgeFaceIndices.fill(std::numeric_limits<uint32_t>::max());
                std::array<std::map<std::pair<uint32_t, uint32_t>, uint32_t>::iterator, 3> adjEdgeFaceIterators;
                for(int k=0; k < 3; k++)
                {
                    uint32_t idx = 0;
                    bool foundEdge = false;
                    const uint32_t v1 = triVertices[k].second.first;
                    const uint32_t v2 = triVertices[k].second.second;
                    auto tIt = edgesNormal.find(std::make_pair(glm::min(v1, v2), glm::max(v1, v2)));
                    foundEdge = tIt != edgesNormal.end();
                    if(foundEdge) 
                    {
                        idx = tIt->second;
                        adjEdgeFaceIndices[k] = idx;
                        adjEdgeFaceIterators[k] = tIt;
                        if(k < 2) edgeNormal += triangles[idx/3].getTriangleNormal();
                    }
                    else if(k < 2)
                    {
                        auto tdIt = validDegeneratedEdgeNormals.find(triVertices[k].second);
                        foundEdge = tdIt != validDegeneratedEdgeNormals.end();
                        if(foundEdge) edgeNormal += tdIt->second;
                    }

                    if(k < 2)
                    {
                        validNeighbours = validNeighbours && foundEdge;
                        someValidNeighbour = someValidNeighbour || foundEdge;
                    }
                }

                edgeNormal = glm::normalize(edgeNormal);

                if(glm::isnan(edgeNormal.x) || glm::isnan(edgeNormal.y) || glm::isnan(edgeNormal.z))
                {
                    edgeNormal = glm::vec3(0.0f);
                }

                if(validNeighbours || iter >= 10)
                {
                    for(uint32_t k=0; k < 3; k++)
                    {                        
                        const uint32_t v1 = indices[3 * tIndex + k];
                        const uint32_t v2 = indices[3 * tIndex + ((k+1) % 3)];
                        const uint32_t v3 = indices[3 * tIndex + ((k+2) % 3)];

                        // Set degenerated triangle normal
                        newValidDegeneratedEdgeNormals.insert(std::make_pair(std::make_pair(v2, v1), edgeNormal));

                        // Assign vertex normal
                        const float angle = glm::acos(glm::clamp(glm::dot(glm::normalize(vertices[v2] - vertices[v1]), glm::normalize(vertices[v3] - vertices[v1])), -1.0f, 1.0f));
                        verticesNormal[v1] += angle * edgeNormal;


                        // Assign edge normal
                        const uint32_t idx = adjEdgeFaceIndices[k];
                        const uint32_t t2Index = idx/3;
                        if(t2Index >= triangles.size()) continue;
                        triangles[t2Index].edgesNormal[idx % 3] = triangles[t2Index].transform * edgeNormal;
                        edgesNormal.erase(adjEdgeFaceIterators[k]);
                    }
                }
                else
                {
                    if(someValidNeighbour)
                    {
                        for(int k=0; k < 3; k++)
                        {
                            const uint32_t v1 = indices[3 * tIndex + k];
                            const uint32_t v2 = indices[3 * tIndex + ((k+1) % 3)];

                            newValidDegeneratedEdgeNormals.insert(std::make_pair(std::make_pair(v2, v1), edgeNormal));
                        }
                    }
                    newDegeneratedTriangles.push_back(degTri);
                }
            }

            degeneratedTriangles = std::move(newDegeneratedTriangles);
            validDegeneratedEdgeNormals = std::move(newValidDegeneratedEdgeNormals);
        }

        if(degeneratedTriangles.size() > 0)
        {
            SPDLOG_INFO("{} degenerated triangles have been merges", degeneratedTriangles.size());
        }

        // Interpret degenerated triangles as edges
        for(std::pair<uint32_t, uint32_t> degTri : degeneratedTriangles)
        {
            const uint32_t tIndex = degTri.first;

            const uint32_t v1 = indices[3 * tIndex + degTri.second];
            const uint32_t v2 = indices[3 * tIndex + ((degTri.second + 1) % 3)];
            const uint32_t v3 = indices[3 * tIndex + ((degTri.second + 2) % 3)];
            
            glm::vec3 upperEdgeNormal;
            glm::vec3 lowerEdgeNormal(0.0f);
            std::array<uint32_t, 3> adjEdgeFaceIndices;
            adjEdgeFaceIndices.fill(std::numeric_limits<uint32_t>::max());

            auto it = edgesNormal.find(std::make_pair(glm::min(v1, v2), glm::max(v1, v2)));
            if(it != edgesNormal.end())
            {
                const uint32_t t2Index = it->second / 3;
                if(!isTriangleDegenerated[t2Index]) upperEdgeNormal = triangles[t2Index].getTriangleNormal();
                else upperEdgeNormal = triangles[tIndex].getTriangleNormal();
                adjEdgeFaceIndices[0] = it->second;
                edgesNormal.erase(it);
            }

            bool someLowerEdge = false;
            it = edgesNormal.find(std::make_pair(glm::min(v2, v3), glm::max(v2, v3)));
            if(it != edgesNormal.end())
            {
                const uint32_t t2Index = it->second / 3;
                if(!isTriangleDegenerated[t2Index])
                {
                    lowerEdgeNormal += triangles[t2Index].getTriangleNormal();
                    someLowerEdge = true;
                } 
                    
                adjEdgeFaceIndices[1] = it->second;
                edgesNormal.erase(it);
            }

            it = edgesNormal.find(std::make_pair(glm::min(v3, v1), glm::max(v3, v1)));
            if(it != edgesNormal.end())
            {
                const uint32_t t2Index = it->second / 3;
                if(!isTriangleDegenerated[t2Index]) 
                {
                    lowerEdgeNormal += triangles[t2Index].getTriangleNormal();
                    someLowerEdge = true;
                }
                adjEdgeFaceIndices[2] = it->second;
                edgesNormal.erase(it);
            }
            
            if(!someLowerEdge) lowerEdgeNormal = triangles[tIndex].getTriangleNormal();

            glm::vec3 edgeNormal = glm::normalize(upperEdgeNormal + glm::normalize(lowerEdgeNormal));
            if(glm::isnan(edgeNormal.x) ||
               glm::isnan(edgeNormal.y) ||
               glm::isnan(edgeNormal.z))
            {
                std::cout << "is nan" << std::endl;
            }

            for(uint32_t k=0; k < 3; k++)
            {
                const uint32_t idx = adjEdgeFaceIndices[k];
                const uint32_t t2Index = idx/3;
                if(t2Index >= triangles.size()) continue;
                if(!isTriangleDegenerated[t2Index])
                    triangles[t2Index].edgesNormal[idx % 3] = triangles[t2Index].transform * edgeNormal;
                triangles[tIndex].edgesNormal[k] = glm::vec3(0.0f);
            }

            for(uint32_t k=0; k < 3; k++)
            {
                const uint32_t v1 = indices[3 * tIndex + k];
                const uint32_t v2 = indices[3 * tIndex + ((k+1) % 3)];
                const uint32_t v3 = indices[3 * tIndex + ((k+2) % 3)];

                const float angle = glm::acos(glm::clamp(glm::dot(glm::normalize(vertices[v2] - vertices[v1]), glm::normalize(vertices[v3] - vertices[v1])), -1.0f, 1.0f));
                verticesNormal[v1] += angle * edgeNormal;
            }

            triangles[tIndex].transform[0][2] = 0.0f;
            triangles[tIndex].transform[1][2] = 0.0f;
            triangles[tIndex].transform[2][2] = 0.0f;
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

            const glm::vec3 bbSize = mesh.getBoundingBox().getSize();
            const glm::vec3 gridStartPos = mesh.getBoundingBox().min;
            const uint32_t axisRes = 2048; 
            const float gridScale = static_cast<float>(axisRes) / glm::max(bbSize.x, glm::max(bbSize.y, bbSize.z));
            const float threshold = 1e-5 / glm::max(bbSize.x, glm::max(bbSize.y, bbSize.z));
            const float sqThreshold = threshold * threshold;
            std::map<uint64_t, std::vector<uint32_t>> pointSet1;
            std::map<uint64_t, std::vector<uint32_t>> pointSet2;

            auto getId = [axisRes](glm::ivec3 id)
            {
                return id.x + id.y * axisRes + id.z * axisRes * axisRes;
            };

            for(uint32_t i=0; i < nonManifoldVertices.size(); i++)
            {
                const glm::vec3& v1 = vertices[nonManifoldVertices[i]];
                const glm::ivec3 id1 = glm::ivec3((v1-gridStartPos) * gridScale);
                auto it1 = pointSet1.insert(std::make_pair(getId(id1), std::vector<uint32_t>())).first;
                it1->second.push_back(nonManifoldVertices[i]);

                const glm::ivec3 id2 = glm::ivec3((v1-gridStartPos) * gridScale + 0.5f);
                auto it2 = pointSet2.insert(std::make_pair(getId(id2), std::vector<uint32_t>())).first;
                it2->second.push_back(nonManifoldVertices[i]);
            }

            std::array<std::map<uint64_t, std::vector<uint32_t>>*, 2> pointSets = {&pointSet1, &pointSet2};

            // Generate a possible vertex mapping
            for(uint32_t i=0; i < nonManifoldVertices.size(); i++)
            {
                float offset = 0.0f;
                for(std::map<uint64_t, std::vector<uint32_t>>* pointSet : pointSets)
                {
                    const glm::vec3& v1 = vertices[nonManifoldVertices[i]];
                    const glm::ivec3 id = glm::ivec3((v1-gridStartPos) * gridScale + offset);
                    auto it = pointSet->find(getId(id));
                    if(it != pointSet->end())
                    {
                        const std::vector<uint32_t>& indices = it->second;
                        for(uint32_t idx : indices)
                        {
                            const glm::vec3& diff = v1 - vertices[idx];
                            if(glm::dot(diff, diff) < sqThreshold)
                            {
                                uint32_t p1 = findVertexParent(nonManifoldVertices[i]);
                                uint32_t p2 = findVertexParent(idx);

                                if(nonManifoldVertices[i] == p1) verticesMap[p1] = p1;
                                verticesMap[p2] = p1;
                                break;
                            }
                        }   
                    }
                    offset += 0.5f;
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
}