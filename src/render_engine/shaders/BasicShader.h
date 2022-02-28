#ifndef BASIC_SHADER_H
#define BASIC_SHADER_H

#include "Shader.h"

class BasicShader : public Shader<BasicShader>
{
public:
    BasicShader() : Shader(SHADER_PATH + "basic.vert", SHADER_PATH + "basic.frag") 
    {
        outColorLocation = glGetUniformLocation(getProgramId(), "outColor");
        outColor = glm::vec4(0.8, 0.0, 0.0, 1.0);
    }
    
    void setDrawColor(glm::vec4 color)
    {
        outColor = color;
    }

    void bind() override
    {
        glUniform4f(outColorLocation, outColor.x, outColor.y, outColor.z, outColor.w);
    }

private:
    unsigned int outColorLocation;
    glm::vec4 outColor;
};

#endif