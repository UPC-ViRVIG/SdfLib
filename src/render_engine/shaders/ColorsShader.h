#ifndef COLORS_SHADER_H
#define COLORS_SHADER_H

#include "Shader.h"

class ColorsShader : public Shader<ColorsShader>
{
public:
    ColorsShader() : Shader(SHADER_PATH + "colors.vert", SHADER_PATH + "colors.frag") {}
};

#endif