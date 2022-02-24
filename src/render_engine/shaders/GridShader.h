#ifndef GRID_SHADER_H
#define GRID_SHADER_H

#include "Shader.h"

class GridShader : public Shader<GridShader>
{
public:
    GridShader() : Shader(SHADER_PATH + "grid.vert", SHADER_PATH + "grid.frag") {}
};

#endif