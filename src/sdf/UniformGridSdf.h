#ifndef UNIFORM_GRID_SDF_H
#define UNIFORM_GRID_SDF_H

#include <vector>

#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "SdfFunction.h"

class UniformGridSdf : SdfFunction
{
public:
    enum InitAlgorithm {
        BASIC,
        OCTREE
    };

    UniformGridSdf(const Mesh& mesh, BoundingBox box, uint32_t depth, 
                   InitAlgorithm initAlgorithm = InitAlgorithm::BASIC);
    UniformGridSdf(const Mesh& mesh, BoundingBox box, float cellSize, 
                   InitAlgorithm initAlgorithm = InitAlgorithm::BASIC);
    float getDistance(glm::vec3 sample) const override;
    const BoundingBox& getGridBoundingBox() const { return mBox; }
    float getGridCellSize() const { return mCellSize; }
    glm::ivec3 getGridSize() const { return mGridSize; }
    const std::vector<float>& getGrid() const { return mGrid; }
private:
    BoundingBox mBox;
    float mCellSize;

    glm::ivec3 mGridSize;
    int mGridXY;
    std::vector<float> mGrid;

    void basicInit(const std::vector<TriangleUtils::TriangleData>& trianglesData);
    void octreeInit(const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData);
    void evalNode(glm::vec3 center, glm::vec3 size, 
                  std::vector<std::pair<float, uint32_t>>& parentTriangles, 
                  const std::vector<TriangleUtils::TriangleData>& trianglesData,
                  uint32_t depth);
};

#endif