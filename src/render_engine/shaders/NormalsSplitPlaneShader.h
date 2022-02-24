#ifndef NORMALS_SPLIT_PLANE_SHADER_H
#define NORMALS_SPLIT_PLANE_SHADER_H

#include "Shader.h"

class NormalsSplitPlaneShader : public Shader<NormalsSplitPlaneShader>
{
public:
    NormalsSplitPlaneShader(glm::vec4 cutPlane) : Shader(SHADER_PATH + "normalsSplitPlane.vert", SHADER_PATH + "normalsSplitPlane.frag"),
                                            cutPlane(cutPlane)
    {
        cutPlaneLocation = glGetUniformLocation(getProgramId(), "cutPlane");
    }

    void setCutPlane(glm::vec4 plane) { cutPlane = plane; }

    void bind() override
    {
        glUniform4f(cutPlaneLocation, cutPlane.x, cutPlane.y, cutPlane.z, cutPlane.w);
    }
private:
    glm::vec4 cutPlane;
    unsigned int cutPlaneLocation;
};

#endif