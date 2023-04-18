#ifndef UNIFORM_GRID_SDF_H
#define UNIFORM_GRID_SDF_H

#include <vector>

#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "utils/UsefullSerializations.h"
#include "SdfFunction.h"

#include <cereal/types/vector.hpp>

namespace sdflib
{
class UniformGridSdf : public SdfFunction
{
public:
    enum InitAlgorithm {
        BASIC,
        OCTREE
    };

    UniformGridSdf() {}
    UniformGridSdf(const Mesh& mesh, BoundingBox box, uint32_t depth, 
                   InitAlgorithm initAlgorithm = InitAlgorithm::OCTREE);
    UniformGridSdf(const Mesh& mesh, BoundingBox box, float cellSize, 
                   InitAlgorithm initAlgorithm = InitAlgorithm::OCTREE);
    
    float getDistance(glm::vec3 sample) const override;
    float getDistance(glm::vec3 sample, glm::vec3& outGradient) const override;
    SdfFormat getFormat() const override { return SdfFormat::GRID; }

    const BoundingBox& getGridBoundingBox() const { return mBox; }
    BoundingBox getSampleArea() const override { return mBox; }
    float getGridCellSize() const { return mCellSize; }
    glm::ivec3 getGridSize() const { return mGridSize; }
    const std::vector<float>& getGrid() const { return mGrid; }

    template<class Archive>
    void save(Archive & archive) const
    { 
        archive(mBox, mGridSize, mGrid);
    }

    template<class Archive>
    void load(Archive & archive)
    {
        archive(mBox, mGridSize, mGrid);

        glm::vec3 cellSize = mBox.getSize() / glm::vec3(mGridSize - 1);
        assert(
            glm::abs(cellSize.x - cellSize.y) < 0.00001f &&
            glm::abs(cellSize.x - cellSize.z) < 0.00001f &&
            glm::abs(cellSize.y - cellSize.z) < 0.00001f
        );
        mCellSize = (cellSize.x + cellSize.y + cellSize.z) / 3.0f;
        mGridXY = mGridSize.x * mGridSize.y;
    } 

private:
    BoundingBox mBox;
    float mCellSize = 0.0;

    glm::ivec3 mGridSize = glm::ivec3(0);
    int mGridXY = 0;
    std::vector<float> mGrid;

    void basicInit(const std::vector<TriangleUtils::TriangleData>& trianglesData);
    void octreeInit(const Mesh& mesh, const std::vector<TriangleUtils::TriangleData>& trianglesData);
    void evalNode(glm::vec3 center, glm::vec3 size, 
                  std::vector<std::pair<float, uint32_t>>& parentTriangles, 
                  const std::vector<TriangleUtils::TriangleData>& trianglesData,
                  uint32_t depth);
};
}

#endif