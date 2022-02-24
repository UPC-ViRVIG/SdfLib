#ifndef SDF_FUNCTION_H
#define SDF_FUNCTION_H

#include <glm/glm.hpp>

class SdfFunction
{
    virtual float getDistance(glm::vec3 sample) const = 0;
};

#endif