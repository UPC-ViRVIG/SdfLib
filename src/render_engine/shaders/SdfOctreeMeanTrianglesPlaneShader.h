#ifndef SDF_OCTREE_MEAN_TRIANGLES_PLANE_SHADER_H
#define SDF_OCTREE_MEAN_TRIANGLES_PLANE_SHADER_H

#include "Shader.h"
#include "sdf/ExactOctreeSdf.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>

class SdfOctreeMeanTrianglesPlaneShader : public Shader<SdfOctreeMeanTrianglesPlaneShader> 
{
public:
    SdfOctreeMeanTrianglesPlaneShader(sdflib::ExactOctreeSdf& octreeSdf, const sdflib::BoundingBox& viewBB) : Shader(SHADER_PATH + "sdfOctreeMeanTrianglesPlane.vert", SHADER_PATH + "sdfOctreeMeanTrianglesPlane.frag") 
    {
        worldToStartGridMatrixLocation = glGetUniformLocation(getProgramId(), "worldToStartGridMatrix");
        worldToStartGridMatrix = glm::scale(glm::mat4x4(1.0f), 1.0f / viewBB.getSize()) *
                                 glm::translate(glm::mat4x4(1.0f), -viewBB.min);

        planeNormalLocation = glGetUniformLocation(getProgramId(), "planeNormal");

        minNumTrianglesLocation = glGetUniformLocation(getProgramId(), "minNumTriangles");
        minNumTriangles = static_cast<float>(octreeSdf.getMinTrianglesInLeafs());

        maxNumTrianglesLocation = glGetUniformLocation(getProgramId(), "maxNumTriangles");
        maxNumTriangles = static_cast<float>(octreeSdf.getMaxTrianglesInLeafs());

        startGridSizeLocation = glGetUniformLocation(getProgramId(), "startGridSize");
        startGridSize = glm::vec3(octreeSdf.getStartGridSize());

        // Set octree data
        glGenBuffers(1, &mOctreeSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, mOctreeSSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER, octreeSdf.getOctreeData().size() * sizeof(sdflib::ExactOctreeSdf::OctreeNode), octreeSdf.getOctreeData().data(), GL_STATIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, mOctreeSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }

    void setNormal(glm::vec3 normal) { planeNormal = normal; }

    void setMinNumTriangles(uint32_t minTriangles)
    {
        minNumTriangles = minTriangles;
    }
    
    void setMaxNumTriangles(uint32_t maxTriangles)
    {
        maxNumTriangles = maxTriangles;
    }

    void bind() override
    {
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, mOctreeSSBO);
        glUniformMatrix4fv(worldToStartGridMatrixLocation, 1, GL_FALSE, glm::value_ptr(worldToStartGridMatrix));
        glUniform1f(minNumTrianglesLocation, minNumTriangles);
        glUniform1f(maxNumTrianglesLocation, maxNumTriangles);
        glUniform3f(startGridSizeLocation, startGridSize.x, startGridSize.y, startGridSize.z);
        glUniform3f(planeNormalLocation, planeNormal.x, planeNormal.y, planeNormal.z);
    }
private:
    unsigned int mOctreeSSBO;
    glm::mat4x4 worldToStartGridMatrix;
    unsigned int worldToStartGridMatrixLocation;
    float minNumTriangles;
    unsigned int minNumTrianglesLocation;
    float maxNumTriangles;
    unsigned int maxNumTrianglesLocation;
    glm::vec3 startGridSize;
    unsigned int startGridSizeLocation;
    unsigned int planeNormalLocation;
    glm::vec3 planeNormal;
};

#endif