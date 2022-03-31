#ifndef OCTREE_SDF_H
#define OCTREE_SDF_H

#include <array>
#include <optional>

#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "utils/UsefullSerializations.h"
#include "SdfFunction.h"

#include <cereal/types/vector.hpp>

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

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(childrenIndex);
        }
    };

    enum TerminationRule
    {
        NONE,
        TRAPEZOIDAL_RULE,
        SIMPSONS_RULE
    };

    static std::optional<TerminationRule> stringToTerminationRule(std::string text)
    {
        if(text == "none" || text == "NONE")
        {
            return std::optional<TerminationRule>(TerminationRule::NONE);
        }
        else if(text == "trapezoidal_rule" || text == "TRAPEZOIDAL_RULE")
        {
            return std::optional<TerminationRule>(TerminationRule::TRAPEZOIDAL_RULE);
        }
        else if(text == "simpsons_rule" || text == "SIMPSONS_RULE")
        {
            return std::optional<TerminationRule>(TerminationRule::SIMPSONS_RULE);
        }

        return std::optional<TerminationRule>();
    }

    OctreeSdf() {}
    OctreeSdf(const Mesh& mesh, BoundingBox box, uint32_t depth, uint32_t startDepth, 
              float terminationThreshold = 1e-3,
              TerminationRule terminationRule = TerminationRule::TRAPEZOIDAL_RULE);

    // Returns the maximum distance in absulute value contained by the octree
    float getOctreeValueRange() const { return mValueRange; }

    glm::ivec3 getStartGridSize() const { return glm::ivec3(mStartGridSize); }
    const BoundingBox& getGridBoundingBox() const { return mBox; }
    uint32_t getOctreeMaxDepth() const { return mMaxDepth; }
    const std::vector<OctreeNode>& getOctreeData() const { return mOctreeData; }

    float getDistance(glm::vec3 sample) const override;
	SdfFunction::SdfFormat getFormat() const override { return SdfFunction::SdfFormat::OCTREE; }

    template<class Archive>
    void save(Archive & archive) const
    { 
        archive(mBox, mStartGridSize, mMaxDepth, mValueRange, mOctreeData);
    }

    template<class Archive>
    void load(Archive & archive)
    {
        archive(mBox, mStartGridSize, mMaxDepth, mValueRange, mOctreeData);
        
        mStartGridCellSize = mBox.getSize().x / static_cast<float>(mStartGridSize);
        mStartGridXY = mStartGridSize * mStartGridSize;
    } 

private:
    BoundingBox mBox;

    float mValueRange;
    
    int mStartGridSize = 0;
    int mStartGridXY = 0;
    float mStartGridCellSize = 0.0f;

    uint32_t mMaxDepth;
    std::vector<OctreeNode> mOctreeData;

    void initOctree(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                    float terminationThreshold, TerminationRule terminationRule);
};

#endif