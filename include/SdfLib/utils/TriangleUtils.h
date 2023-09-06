#ifndef TRIANGLE_UTILS_H
#define TRIANGLE_UTILS_H

#include <glm/glm.hpp>
#include <algorithm>
#include <vector>
#include <array>
#include <map>
#include <unordered_map>
#include <iostream>
#include <spdlog/spdlog.h>
#include <cereal/types/array.hpp>
#include "Mesh.h"
#include "UsefullSerializations.h"

namespace sdflib
{
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

        // Returns the normalized triangle normal
        glm::vec3 getTriangleNormal() const
        {
            return glm::vec3(transform[0][2], transform[1][2], transform[2][2]);
        }

        template<class Archive>
        void serialize(Archive & archive)
        {
            archive(origin, transform, b, c, v2, v3, edgesNormal, verticesNormal);
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

    std::vector<TriangleData> calculateMeshTriangleData(const Mesh& mesh);

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

    inline float getSignedDistPointAndTriangle(glm::vec3 point, const TriangleData& data, 
                                               glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, glm::vec3& outNormal)
    {
        glm::vec3 projPoint = data.transform * (point - data.origin);

        const float de1 = -projPoint.y;
        const float de2 = (projPoint.x - data.v2) * data.b.y - projPoint.y * data.b.x;
        const float de3 = projPoint.x * data.c.y - projPoint.y * data.c.x;


        auto normalizeVector = [&data](glm::vec3 vec)
        {
            glm::vec3 n = glm::normalize(vec);
            return glm::isnan(n.x + n.y + n.z) ? data.getTriangleNormal() : n;
        };

        if(de1 >= 0)
        {
            if(projPoint.x <= 0) // Its near v1
            {
                const float sign = glm::sign(glm::dot(data.verticesNormal[0], projPoint));
                outNormal = sign * normalizeVector(point - v1);
                return sign * glm::sqrt(glm::dot(projPoint, projPoint));
            }
            else if(projPoint.x >= data.v2) // Its near v2
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v2, 0.0, 0.0);
                const float sign = glm::sign(glm::dot(data.verticesNormal[1], p));
                outNormal = sign * normalizeVector(point - v2);
                return  sign * glm::sqrt(glm::dot(p, p));
            }
            else // Its near edge 1
            {
                const float sign = glm::sign(glm::dot(data.edgesNormal[0], projPoint));
                outNormal = sign * normalizeVector(glm::transpose(data.transform) * glm::vec3(0.0, projPoint.y, projPoint.z));
                return  sign * glm::sqrt(de1 * de1 + projPoint.z * projPoint.z);
            }
        }
        else if(de2 >= 0)
        {
            if((projPoint.x - data.v2) * data.b.x + projPoint.y * data.b.y <= 0) // Its near v2
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v2, 0.0, 0.0);
                const float sign = glm::sign(glm::dot(data.verticesNormal[1], p));
                outNormal = sign * normalizeVector(point - v2);
                return sign * glm::sqrt(glm::dot(p, p));
            }
            else if((projPoint.x - data.v3.x) * data.b.x + (projPoint.y - data.v3.y) * data.b.y >= 0) // Its near v3
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v3.x, data.v3.y, 0.0);
                const float sign = glm::sign(glm::dot(data.verticesNormal[2], p));
                outNormal = sign * normalizeVector(point - v3);
                return sign * glm::sqrt(glm::dot(p, p));
            }
            else // Its near edge 2
            {
                const float sign = glm::sign(glm::dot(data.edgesNormal[1], projPoint - glm::vec3(data.v2, 0.0f, 0.0f)));
                const float dot = (projPoint.x - data.v2) * data.b.x + projPoint.y * data.b.y;
                outNormal = sign * normalizeVector(glm::transpose(data.transform) * glm::vec3((projPoint.x - data.v2) - dot * data.b.x,
                                                                                              projPoint.y - dot * data.b.y, 
                                                                                              projPoint.z));
                return sign * glm::sqrt(de2 * de2 + projPoint.z * projPoint.z);
            }
        }
        else if(de3 >= 0)
        {
            if(projPoint.x * data.c.x + projPoint.y * data.c.y >= 0) // Its near v1
            {
                const float sign = glm::sign(glm::dot(data.verticesNormal[0], projPoint));
                outNormal = sign * normalizeVector(point - v1);
                return sign * glm::sqrt(glm::dot(projPoint, projPoint));
            }
            else if((projPoint.x - data.v3.x) * data.c.x + (projPoint.y - data.v3.y) * data.c.y <= 0) // Its near v3
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v3.x, data.v3.y, 0.0);
                const float sign = glm::sign(glm::dot(data.verticesNormal[2], p));
                outNormal = sign * normalizeVector(point - v3);
                return sign * glm::sqrt(glm::dot(p, p));
            }
            else // Its near edge 3
            {
                const float sign = glm::sign(glm::dot(data.edgesNormal[2], projPoint));
				const float dot = projPoint.x * data.c.x + projPoint.y * data.c.y;
                outNormal = sign * normalizeVector(glm::transpose(data.transform) * glm::vec3(projPoint.x - dot * data.c.x,
                                                                                             projPoint.y - dot * data.c.y, 
                                                                                             projPoint.z));
                return sign * glm::sqrt(de3 * de3 + projPoint.z * projPoint.z);
            }
        }

        outNormal = data.getTriangleNormal();
        return projPoint.z;
    }

    inline float getSignedDistPointAndTriangle(glm::vec3 point, const TriangleData& data, glm::vec3& outNormal)
    {
        glm::vec3 projPoint = data.transform * (point - data.origin);

        const float de1 = -projPoint.y;
        const float de2 = (projPoint.x - data.v2) * data.b.y - projPoint.y * data.b.x;
        const float de3 = projPoint.x * data.c.y - projPoint.y * data.c.x;

        if(de1 >= 0)
        {
            if(projPoint.x <= 0) // Its near v1
            {
                const float sign = glm::sign(glm::dot(data.verticesNormal[0], projPoint));
                outNormal = sign * glm::normalize(point - data.origin);
                return sign * glm::sqrt(glm::dot(projPoint, projPoint));
            }
            else if(projPoint.x >= data.v2) // Its near v2
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v2, 0.0, 0.0);
                const float sign = glm::sign(glm::dot(data.verticesNormal[1], p));
                outNormal = sign * glm::normalize(point - data.origin - glm::transpose(data.transform) * glm::vec3(data.v2, 0.0f, 0.0f));
                return  sign * glm::sqrt(glm::dot(p, p));
            }
            else // Its near edge 1
            {
                const float sign = glm::sign(glm::dot(data.edgesNormal[0], projPoint));
                outNormal = sign * glm::normalize(glm::transpose(data.transform) * glm::vec3(0.0, projPoint.y, projPoint.z));
                return  sign * glm::sqrt(de1 * de1 + projPoint.z * projPoint.z);
            }
        }
        else if(de2 >= 0)
        {
            if((projPoint.x - data.v2) * data.b.x + projPoint.y * data.b.y <= 0) // Its near v2
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v2, 0.0, 0.0);
                const float sign = glm::sign(glm::dot(data.verticesNormal[1], p));
                outNormal = sign * glm::normalize(point - data.origin - glm::transpose(data.transform) * glm::vec3(data.v2, 0.0f, 0.0f));
                return sign * glm::sqrt(glm::dot(p, p));
            }
            else if((projPoint.x - data.v3.x) * data.b.x + (projPoint.y - data.v3.y) * data.b.y >= 0) // Its near v3
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v3.x, data.v3.y, 0.0);
                const float sign = glm::sign(glm::dot(data.verticesNormal[2], p));
                outNormal = sign * glm::normalize(point - data.origin - glm::transpose(data.transform) * glm::vec3(data.v3.x, data.v3.y, 0.0f));
                return sign * glm::sqrt(glm::dot(p, p));
            }
            else // Its near edge 2
            {
                const float sign = glm::sign(glm::dot(data.edgesNormal[1], projPoint - glm::vec3(data.v2, 0.0f, 0.0f)));
                const float dot = (projPoint.x - data.v2) * data.b.x + projPoint.y * data.b.y;
                outNormal = sign * glm::normalize(glm::transpose(data.transform) * glm::vec3((projPoint.x - data.v2) - dot * data.b.x, 
                                                                                              projPoint.y - dot * data.b.y, 
                                                                                              projPoint.z));
                return sign * glm::sqrt(de2 * de2 + projPoint.z * projPoint.z);
            }
        }
        else if(de3 >= 0)
        {
            if(projPoint.x * data.c.x + projPoint.y * data.c.y >= 0) // Its near v1
            {
                const float sign = glm::sign(glm::dot(data.verticesNormal[0], projPoint));
                outNormal = sign * glm::normalize(point - data.origin);
                return sign * glm::sqrt(glm::dot(projPoint, projPoint));
            }
            else if((projPoint.x - data.v3.x) * data.c.x + (projPoint.y - data.v3.y) * data.c.y <= 0) // Its near v3
            {
                const glm::vec3 p = projPoint - glm::vec3(data.v3.x, data.v3.y, 0.0);
                const float sign = glm::sign(glm::dot(data.verticesNormal[2], p));
                outNormal = sign * glm::normalize(point - data.origin - glm::transpose(data.transform) * glm::vec3(data.v3.x, data.v3.y, 0.0f));
                return sign * glm::sqrt(glm::dot(p, p));
            }
            else // Its near edge 3
            {
                const float sign = glm::sign(glm::dot(data.edgesNormal[2], projPoint));
				const float dot = projPoint.x * data.c.x + projPoint.y * data.c.y;
                outNormal = sign * glm::normalize(glm::transpose(data.transform) * glm::vec3(projPoint.x - dot * data.c.x, 
                                                                                             projPoint.y - dot * data.c.y, 
                                                                                             projPoint.z));
                return sign * glm::sqrt(de3 * de3 + projPoint.z * projPoint.z);
            }
        }

        outNormal = data.getTriangleNormal();
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
}

#endif
