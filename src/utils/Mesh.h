#ifndef MESH_H
#define MESH_H

#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>      
#include <assimp/postprocess.h>

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
};

class Mesh
{
public:
    Mesh() {}
    Mesh(std::string filePath);

    std::vector<glm::vec3>& getVertices() { return mVertices; }
    const std::vector<glm::vec3>& getVertices() const { return mVertices; }

    std::vector<uint32_t>& getIndices() { return mIndices; }
    const std::vector<uint32_t>& getIndices() const { return mIndices; }

    std::vector<glm::vec3>& getNormals() { return mNormals; }
    const std::vector<glm::vec3>& getNormals() const { return mNormals; }

    const BoundingBox& getBoudingBox() const { return mBBox; }

    void computeBoundingBox();
    void computeNormals();
    void applyTransform(glm::mat4 trans);
private:
    std::vector<glm::vec3> mVertices;
    std::vector<uint32_t> mIndices;
    std::vector<glm::vec3> mNormals;
    BoundingBox mBBox;
};

#endif