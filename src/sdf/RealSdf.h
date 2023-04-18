#ifndef REAL_SDF_H
#define REAL_SDF_H

#include <vector>
#include "SdfFunction.h"
#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"

namespace sdflib
{
class RealSdf : public SdfFunction
{
public:
    RealSdf(const Mesh& mesh);
    float getDistance(glm::vec3 sample) const override;
    float getDistance(glm::vec3 sample, glm::vec3& outGradient) const override;
    BoundingBox getSampleArea() const override { return BoundingBox(glm::vec3(-INFINITY), glm::vec3(INFINITY)); }
private:
    std::vector<TriangleUtils::TriangleData> mTriangles;
};
}

#endif