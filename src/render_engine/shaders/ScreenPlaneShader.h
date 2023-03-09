#ifndef SCREEN_PLANE_SHADER_H
#define SCREEN_PLANE_SHADER_H

#include "Shader.h"

class ScreenPlaneShader : public Shader<ScreenPlaneShader>
{
public:
    ScreenPlaneShader() : Shader(SHADER_PATH + "screenPlane.vert", SHADER_PATH + "screenPlane.frag") {}

    void setInputTexture(unsigned int inputTexture) { mInputTexture = inputTexture; }
    void bind() override 
    {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, mInputTexture);
    }
private:
    unsigned int mInputTexture;
};

#endif
