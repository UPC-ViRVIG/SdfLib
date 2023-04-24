#ifndef SDF_OCTREE_PLANE_SHADER_H
#define SDF_OCTREE_PLANE_SHADER_H

#include "Shader.h"
#include "SdfLib/OctreeSdf.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>

class SdfOctreePlaneShader : public Shader<SdfOctreePlaneShader> 
{
public:
    SdfOctreePlaneShader(sdflib::OctreeSdf& octreeSdf, const sdflib::BoundingBox& viewBB) : Shader(SHADER_PATH + "sdfOctreePlane.vert", SHADER_PATH + "sdfOctreePlane.frag") 
    {
        worldToStartGridMatrixLocation = glGetUniformLocation(getProgramId(), "worldToStartGridMatrix");
        worldToStartGridMatrix = glm::scale(glm::mat4x4(1.0f), 1.0f / viewBB.getSize()) *
                                 glm::translate(glm::mat4x4(1.0f), -viewBB.min);

        planeNormalLocation = glGetUniformLocation(getProgramId(), "planeNormal");

        octreeValueRangeLocation = glGetUniformLocation(getProgramId(), "octreeValueRange");
        octreeValueRange = octreeSdf.getOctreeValueRange();

        startGridSizeLocation = glGetUniformLocation(getProgramId(), "startGridSize");
        startGridSize = glm::vec3(octreeSdf.getStartGridSize());

        printGridLocation = glGetUniformLocation(getProgramId(), "printGrid");
        printGrid = true;

        printIsolinesLocation = glGetUniformLocation(getProgramId(), "printIsolines");
        printIsolines = true;

        // Set octree data
        glGenBuffers(1, &mOctreeSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, mOctreeSSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER, octreeSdf.getOctreeData().size() * sizeof(sdflib::OctreeSdf::OctreeNode), octreeSdf.getOctreeData().data(), GL_STATIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, mOctreeSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }

    void setNormal(glm::vec3 normal) { planeNormal = normal; }

    void drawGrid(bool draw) { printGrid = draw; }
    bool isDrawingGrid() { return printGrid; }

    void drawIsolines(bool draw) { printIsolines = draw; }
    bool isDrawingIsolines() { return printIsolines; }

    void bind() override
    {
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, mOctreeSSBO);
        glUniformMatrix4fv(worldToStartGridMatrixLocation, 1, GL_FALSE, glm::value_ptr(worldToStartGridMatrix));
        glUniform1f(octreeValueRangeLocation, octreeValueRange);
        glUniform3f(startGridSizeLocation, startGridSize.x, startGridSize.y, startGridSize.z);
        glUniform3f(planeNormalLocation, planeNormal.x, planeNormal.y, planeNormal.z);
        glUniform1i(printGridLocation, printGrid);
        glUniform1i(printIsolinesLocation, printIsolines);
    }
private:
    unsigned int mOctreeSSBO;
    glm::mat4x4 worldToStartGridMatrix;
    unsigned int worldToStartGridMatrixLocation;
    float octreeValueRange;
    unsigned int octreeValueRangeLocation;
    glm::vec3 startGridSize;
    unsigned int startGridSizeLocation;
    unsigned int planeNormalLocation;
    glm::vec3 planeNormal;
    unsigned int printGridLocation;
    bool printGrid;
    unsigned int printIsolinesLocation;
    bool printIsolines;
};

#endif