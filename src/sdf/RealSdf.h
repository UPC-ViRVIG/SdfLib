#ifndef REAL_SDF_H
#define REAL_SDF_H

#include <vector>
#include "SdfFunction.h"
#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"

class RealSdf : public SdfFunction
{
public:
    RealSdf(const Mesh& mesh);
    float getDistance(glm::vec3 sample) const override;
private:
    std::vector<TriangleUtils::TriangleData> mTriangles;
};

#endif