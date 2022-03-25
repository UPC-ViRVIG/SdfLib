#ifndef OCTREE_SDF_H
#define OCTREE_SDF_H

#include <array>

#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "utils/UsefullSerializations.h"
#include "SdfFunction.h"

class OctreeSdf : public SdfFunction
{
public:
    struct OctreeNode
    {
        static constexpr uint32_t IS_LEAF_MASK = 1 << 31;
        static constexpr uint32_t CHILDREN_INDEX_MASK = ~IS_LEAF_MASK;
        union
        {
            uint32_t childrenIndex;
            float value;
        };

        inline bool isLeaf() const
        {
            return childrenIndex & IS_LEAF_MASK;
        }

        inline uint32_t getChildrenIndex() const
        {
            return childrenIndex & CHILDREN_INDEX_MASK;
        }

        inline void setValues(bool isLeaf, uint32_t index)
        {
            childrenIndex = (index & CHILDREN_INDEX_MASK) | 
                            ((isLeaf) ? IS_LEAF_MASK : 0);
        }

    };

    OctreeSdf() {}
    OctreeSdf(const Mesh& mesh, BoundingBox box, uint32_t depth, uint32_t startDepth);

    // Returns the maximum distance in absulute value contained by the octree
    float getOctreeValueRange() const { return mValueRange; }

    glm::ivec3 getStartGridSize() const { return glm::ivec3(mStartGridSize); }
    const BoundingBox& getGridBoundingBox() const { return mBox; }
    const std::vector<OctreeNode>& getOctreeData() const { return mOctreeData; }

    float getDistance(glm::vec3 sample) const override;
private:
    BoundingBox mBox;

    float mValueRange;

    int mStartGridSize = 0;
    int mStartGridXY = 0;
    float mStartGridCellSize = 0.0f;

    std::vector<OctreeNode> mOctreeData;

    void initOctree(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth);
};

#endif