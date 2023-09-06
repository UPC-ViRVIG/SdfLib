#ifndef MESH_H
#define MESH_H

#include <string>
#include <vector>
#include <glm/glm.hpp>
#ifdef ASSIMP_AVAILABLE
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
#ifdef ASSIMP_AVAILABLE
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
#ifdef ASSIMP_AVAILABLE
    void initMesh(const aiMesh* mesh);
#endif

    std::vector<glm::vec3> mVertices;
    std::vector<uint32_t> mIndices;
    std::vector<glm::vec3> mNormals;
    BoundingBox mBBox;
};
}

#endif
