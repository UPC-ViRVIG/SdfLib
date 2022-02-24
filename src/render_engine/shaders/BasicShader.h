#ifndef BASIC_SHADER_H
#define BASIC_SHADER_H

#include "Shader.h"

class BasicShader : public Shader<BasicShader>
{
public:
    BasicShader() : Shader(SHADER_PATH + "basic.vert", SHADER_PATH + "basic.frag") {}
};

#endif