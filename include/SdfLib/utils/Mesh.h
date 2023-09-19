#ifndef MESH_H
#define MESH_H

#include <string>
#include <vector>
#include <glm/glm.hpp>
#ifdef SDFLIB_ASSIMP_AVAILABLE
#include <assimp/Importer.hpp>
#include <assimp/scene.h>      
#include <assimp/postprocess.h>
#endif
#include "SdfLib/utils/UsefullSerializations.h"

namespace sdflib
{
struct BoundingBox
{
    BoundingBox() : min(INFINITY), max(-INFINITY) {}
    BoundingBox(glm::vec3 min, glm::vec3 max)
        : min(min),
          max(max)
    {}
    glm::vec3 min;
    glm::vec3 max;

    glm::vec3 getSize() const
    {
        return max - min;
    }

    glm::vec3 getCenter() const
    {
        return min + 0.5f * getSize();
    }

    void addMargin(float margin)
    {
        min -= glm::vec3(margin);
        max += glm::vec3(margin);
    }

    float getDistance(glm::vec3 point) const
    {
        glm::vec3 q = glm::abs(point - getCenter()) - 0.5f * getSize();
        return glm::length(glm::max(q,glm::vec3(0.0f))) + glm::min(glm::max(q.x, glm::max(q.y,q.z)),0.0f);
    }

    float getDistance(glm::vec3 point, glm::vec3& outGradient) const
    {
        glm::vec3 a = glm::abs(point) - getSize();
        int k = a[0] > a[1] ? 0 : 1;
        int l = a[2] > a[k] ? 2 : k;
        if (a[l] < 0) {
            outGradient[l] = point[l] / glm::abs(point[l]);
        } else {
            glm::vec3 b = glm::max(a, glm::vec3(0.0f));
            float c = glm::length(b);
            outGradient[0] = a[0] > 0 ? b[0] / c * point[0] / glm::abs(point[0]) : 0;
            outGradient[1] = a[1] > 0 ? b[1] / c * point[1] / glm::abs(point[1]) : 0;
            outGradient[2] = a[2] > 0 ? b[2] / c * point[2] / glm::abs(point[2]) : 0;
        }
        return getDistance(point);
    }

    template<class Archive>
    void serialize(Archive & archive)
    {
        archive(min, max); 
    }
};

class Mesh
{
public:
    Mesh() {}
#ifdef SDFLIB_ASSIMP_AVAILABLE
    Mesh(std::string filePath);
    Mesh(const aiMesh* mesh);
#endif
    Mesh(glm::vec3* vertices, uint32_t numVertices,
         uint32_t* indices, uint32_t numIndices);

    std::vector<glm::vec3>& getVertices() { return mVertices; }
    const std::vector<glm::vec3>& getVertices() const { return mVertices; }

    std::vector<uint32_t>& getIndices() { return mIndices; }
    const std::vector<uint32_t>& getIndices() const { return mIndices; }

    std::vector<glm::vec3>& getNormals() { return mNormals; }
    const std::vector<glm::vec3>& getNormals() const { return mNormals; }

    const BoundingBox& getBoundingBox() const { return mBBox; }

    void computeBoundingBox();
    void computeNormals();
    void applyTransform(glm::mat4 trans);
private:
#ifdef SDFLIB_ASSIMP_AVAILABLE
    void initMesh(const aiMesh* mesh);
#endif

    std::vector<glm::vec3> mVertices;
    std::vector<uint32_t> mIndices;
    std::vector<glm::vec3> mNormals;
    BoundingBox mBBox;
};
}

#endif
