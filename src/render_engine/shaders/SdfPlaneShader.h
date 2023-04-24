#ifndef SDF_PLANE_SHADER_H
#define SDF_PLANE_SHADER_H

#include "Shader.h"
#include "SdfLib/UniformGridSdf.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>

class SdfPlaneShader : public Shader<SdfPlaneShader> 
{
public:
    SdfPlaneShader(sdflib::UniformGridSdf& sdfGrid, const sdflib::BoundingBox& viewBB) : Shader(SHADER_PATH + "sdfPlane.vert", SHADER_PATH + "sdfPlane.frag") 
    {   
        // Get uniform locations
        worldToGridMatrixLocation = glGetUniformLocation(getProgramId(), "worldToGridMatrix");
        const glm::vec3 texUnit = 1.0f / glm::vec3(sdfGrid.getGridSize());
        worldToGridMatrix = glm::translate(glm::mat4x4(1.0f), 0.5f * texUnit) *
                            glm::scale(glm::mat4x4(1.0f), (1.0f - texUnit) / viewBB.getSize()) *
                            glm::translate(glm::mat4x4(1.0f), -viewBB.min);

        planeNormalLocation = glGetUniformLocation(getProgramId(), "planeNormal");

        gridValueRangeLocation = glGetUniformLocation(getProgramId(), "gridValueRange");
        gridValueRange = 0.0f;
        for(const float& val : sdfGrid.getGrid())
        {
            gridValueRange = glm::max(gridValueRange, glm::abs(val));
        }

        gridSizeLocation = glGetUniformLocation(getProgramId(), "gridSize");
        gridSize = glm::vec3(sdfGrid.getGridSize());

        printGridLocation = glGetUniformLocation(getProgramId(), "printGrid");
        printGrid = true;

        printIsolinesLocation = glGetUniformLocation(getProgramId(), "printIsolines");
        printIsolines = true;

        // Set texture
        glGenTextures(1, &gpu3DTexture);
        glBindTexture(GL_TEXTURE_3D, gpu3DTexture);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glTexImage3D(GL_TEXTURE_3D, 0, GL_R32F, 
                     sdfGrid.getGridSize().x, sdfGrid.getGridSize().y, sdfGrid.getGridSize().z,
                     0, GL_RED, GL_FLOAT, sdfGrid.getGrid().data());
    }

    void setNormal(glm::vec3 normal) { planeNormal = normal; }

    void drawGrid(bool draw) { printGrid = draw; }
    bool isDrawingGrid() { return printGrid; }

    void drawIsolines(bool draw) { printIsolines = draw; }
    bool isDrawingIsolines() { return printIsolines; }

    void bind() override
    {
        glBindTexture(GL_TEXTURE_3D, gpu3DTexture);
        glUniformMatrix4fv(worldToGridMatrixLocation, 1, GL_FALSE, glm::value_ptr(worldToGridMatrix));
        glUniform1f(gridValueRangeLocation, gridValueRange);
        glUniform3f(gridSizeLocation, gridSize.x, gridSize.y, gridSize.z);
        glUniform3f(planeNormalLocation, planeNormal.x, planeNormal.y, planeNormal.z);
        glUniform1i(printGridLocation, printGrid);
        glUniform1i(printIsolinesLocation, printIsolines);
    }

private:
    glm::mat4x4 worldToGridMatrix;
    unsigned int worldToGridMatrixLocation;
    float gridValueRange;
    unsigned int gridValueRangeLocation;
    glm::vec3 gridSize;
    unsigned int gridSizeLocation;
    unsigned int gpu3DTexture;
    unsigned int planeNormalLocation;
    glm::vec3 planeNormal;
    unsigned int printGridLocation;
    bool printGrid;
    unsigned int printIsolinesLocation;
    bool printIsolines;
    
};

#endif