#ifndef SDF_OCTREE_LIGHT_SHADER_H
#define SDF_OCTREE_LIGHT_SHADER_H

#include "Shader.h"
#include "SdfLib/OctreeSdf.h"
#include "SdfLib/utils/Timer.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <algorithm>

class SdfOctreeLightShader : public Shader<SdfOctreeLightShader>
{
public:
    SdfOctreeLightShader(sdflib::OctreeSdf& octreeSdf) : Shader(SHADER_PATH + "sdfOctreeLight.vert", SHADER_PATH + "sdfOctreeLight.frag")
    {
        worldToStartGridMatrixLocation = glGetUniformLocation(getProgramId(), "worldToStartGridMatrix");
        worldToStartGridMatrix = glm::scale(glm::mat4x4(1.0f), 1.0f / octreeSdf.getGridBoundingBox().getSize()) *
                                 glm::translate(glm::mat4x4(1.0f), -octreeSdf.getGridBoundingBox().min);

        normalWorldToStartGridMatrixLocation = glGetUniformLocation(getProgramId(), "normalWorldToStartGridMatrix");
        normalWorldToStartGridMatrix = glm::inverseTranspose(glm::mat3(worldToStartGridMatrix));

        startGridSizeLocation = glGetUniformLocation(getProgramId(), "startGridSize");
        startGridSize = glm::vec3(octreeSdf.getStartGridSize());

        minBorderValueLocation = glGetUniformLocation(getProgramId(), "minBorderValue");
        minBorderValue = octreeSdf.getOctreeMinBorderValue();
        distanceScaleLocation = glGetUniformLocation(getProgramId(), "distanceScale");
        distanceScale = 1.0f / octreeSdf.getGridBoundingBox().getSize().x;
        materialAlbedoColorLocation = glGetUniformLocation(getProgramId(), "materialAlbedoColor");
        materialAlbedoColor = glm::vec3(1.0, 0.0, 0.0);
        timeLocation = glGetUniformLocation(getProgramId(), "time");
        timer.start();

        // Set octree data
        glGenBuffers(1, &mOctreeSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, mOctreeSSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER, octreeSdf.getOctreeData().size() * sizeof(sdflib::OctreeSdf::OctreeNode), octreeSdf.getOctreeData().data(), GL_STATIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, mOctreeSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }

    void setAlbedoColor(glm::vec3 color)
    {
        materialAlbedoColor = color;
    }

    void bind() override
    {
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, mOctreeSSBO);
        glUniformMatrix4fv(worldToStartGridMatrixLocation, 1, GL_FALSE, glm::value_ptr(worldToStartGridMatrix));
        glUniformMatrix3fv(normalWorldToStartGridMatrixLocation, 1, GL_FALSE, glm::value_ptr(normalWorldToStartGridMatrix));
        glUniform3f(startGridSizeLocation, startGridSize.x, startGridSize.y, startGridSize.z);
        glUniform1f(minBorderValueLocation, minBorderValue);
        glUniform1f(distanceScaleLocation, distanceScale);
        glUniform3f(materialAlbedoColorLocation, materialAlbedoColor.x, materialAlbedoColor.y, materialAlbedoColor.z);
        glUniform1f(timeLocation, timer.getElapsedSeconds());
    }
private:
    unsigned int mOctreeSSBO;

    glm::mat4x4 worldToStartGridMatrix;
    unsigned int worldToStartGridMatrixLocation;

    glm::mat3 normalWorldToStartGridMatrix;
    unsigned int normalWorldToStartGridMatrixLocation;

    glm::vec3 startGridSize;
    unsigned int startGridSizeLocation;

    float minBorderValue;
    unsigned int minBorderValueLocation;

    float distanceScale;
    unsigned int distanceScaleLocation;

    unsigned int materialAlbedoColorLocation;
    glm::vec3 materialAlbedoColor;

    sdflib::Timer timer;
    unsigned int timeLocation;
};

#endif