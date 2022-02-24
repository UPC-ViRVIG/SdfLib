#ifndef NORMALS_SHADER_H
#define NORMALS_SHADER_H

#include "Shader.h"

class NormalsShader : public Shader<NormalsShader>
{
public:
    NormalsShader() : Shader(SHADER_PATH + "normals.vert", SHADER_PATH + "normals.frag") {}
};


#endif
