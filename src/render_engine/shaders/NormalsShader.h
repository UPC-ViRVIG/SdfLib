#ifndef NORMALS_SHADER_H
#define NORMALS_SHADER_H

#include "Shader.h"

class NormalsShader : public Shader<NormalsShader>
{
public:
    NormalsShader() : Shader(SHADER_PATH + "normals.vert", SHADER_PATH + "normals.frag")
    {
        outColorLocation = glGetUniformLocation(getProgramId(), "outColor");
        outColor = glm::vec3(0.8, 0.0, 0.0);
    }
    
    void setDrawColor(glm::vec3 color)
    {
        outColor = color;
    }

    void bind() override
    {
        glUniform3f(outColorLocation, outColor.x, outColor.y, outColor.z);
    }

private:
    unsigned int outColorLocation;
    glm::vec3 outColor;
    
};


#endif
