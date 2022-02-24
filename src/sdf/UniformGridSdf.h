#ifndef UNIFORM_GRID_SDF_H
#define UNIFORM_GRID_SDF_H

#include "utils/Mesh.h"
#include "SdfFunction.h"

class UniformGridSdf : SdfFunction
{
public:
    UniformGridSdf(Mesh& mesh, BoundingBox box, float cellSize);
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
};

#endif