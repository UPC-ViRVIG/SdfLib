#ifndef PRIMITIVES_FACTORY_H
#define PRIMITIVES_FACTORY_H

#include "Mesh.h"

namespace PrimitivesFactory
{
    std::shared_ptr<Mesh> getIsosphere();
    std::shared_ptr<Mesh> getPlane();
    std::shared_ptr<Mesh> getCube();
}

#endif