#ifndef I_OCTREE_SDF_H
#define I_OCTREE_SDF_H

#include <array>
#include <optional>

#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "utils/UsefullSerializations.h"

#include "SdfFunction.h"

namespace sdflib
{

class IOctreeSdf : public SdfFunction
{
public:
    enum InitAlgorithm
    {
        UNIFORM, // All zones are subdivided to the maximum depth
        NO_CONTINUITY, // Not preserve continuity
        CONTINUITY // Preserve continuity
    };
    
    /**
     * @brief Structure representing a node of the octree.
     * 
     *        If it is an inner node, it stores a index pointing 
     *          to the start of an array containing its 8 children.
     * 
     *        If it is a leaf node, it only stores a index pointing 
     *          to the array of polynomial coefficients.
     **/ 
    struct OctreeNode
    {
        static inline OctreeNode getLeafNode()
        {
            OctreeNode n; n.childrenIndex = IS_LEAF_MASK;
            return n;
        }

        static inline OctreeNode getInnerNode()
        {
            OctreeNode n; n.childrenIndex = 0;
            return n;
        }

        static constexpr uint32_t IS_LEAF_MASK = 1 << 31;
        static constexpr uint32_t MARK_MASK = 1 << 30;
        static constexpr uint32_t CHILDREN_INDEX_MASK = ~(IS_LEAF_MASK | MARK_MASK);
        union
        {
            uint32_t childrenIndex;
            float value;
        };

        inline bool isLeaf() const
        {
            return childrenIndex & IS_LEAF_MASK;
        }

        inline void removeMark()
        {
            childrenIndex = childrenIndex & (~MARK_MASK);
        }

        inline void markNode()
        {
            childrenIndex |= MARK_MASK;
        }

        inline bool isMarked()
        {
            return childrenIndex & MARK_MASK;
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
        NONE, // Subdivide always
        TRAPEZOIDAL_RULE, // Estimate error integral by the trapezoidal rule
        SIMPSONS_RULE, // Estimate error integral by the simpson rule
        BY_DISTANCE_RULE, // The minimum error is defined regarding the distance to the surface
        ISOSURFACE // Subdivide only at parts containing the isosurface
    };

    class TerminationRuleParams
    {
        public:
            std::array<float, 2> params;

            static TerminationRuleParams setNoneRuleParams() { return TerminationRuleParams();}
            static TerminationRuleParams setTrapezoidalRuleParams(float expectedError) { return TerminationRuleParams {expectedError}; }
            static TerminationRuleParams setSimpsonRuleParams(float expectedError) { return TerminationRuleParams {expectedError}; }
            static TerminationRuleParams setIsosurfaceRuleParams(float expectedError) { return TerminationRuleParams {expectedError}; }
            static TerminationRuleParams setByDistanceRuleParams(float baseError, float errorDecayByDistance) { return TerminationRuleParams {baseError, errorDecayByDistance}; }

            float& operator[](int p)
            {
                return params[p];
            }
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
        else if(text == "by_distance_rule" || text == "BY_DISTANCE_RULE")
        {
            return std::optional<TerminationRule>(TerminationRule::BY_DISTANCE_RULE);
        }
        else if(text == "isosurface" || text == "ISOSURFACE")
        {
            return std::optional<TerminationRule>(TerminationRule::ISOSURFACE);
        }

        return std::optional<TerminationRule>();
    }

    /**
     * @return Returns the maximum distance in absulute value contained by the octree
     **/
    float getOctreeValueRange() const { return mValueRange; }
    /**
     * @return Returns the minimum distance in the border of the octree box.
     **/
    float getOctreeMinBorderValue() const { return mMinBorderValue; }

    /**
     * @return The size of the start grid containing all 
     *          the nodes of the start depth stored sequentially 
     **/
    glm::ivec3 getStartGridSize() const { return glm::ivec3(mStartGridSize); }
    
    /**
     * @return The octree bounding box
     **/
    const BoundingBox& getGridBoundingBox() const { return mBox; }
    BoundingBox getSampleArea() const override { return mBox; }
    
    /**
     * @return The octree maximum depth
     **/
    uint32_t getOctreeMaxDepth() const { return mMaxDepth; }

    /**
     * @return The array containing all the octree structure
     **/
    const std::vector<OctreeNode>& getOctreeData() const { return mOctreeData; }

    /**
     * @return The array containing all the octree structure
     **/
    std::vector<OctreeNode>& getOctreeData() { return mOctreeData; }

    bool hasSdfOnlyAtSurface() const { return mSdfOnlyAySurface; }

    /**
     * @brief Computes the area covered by the leaves at different depths, 
     *          supposing that the hole octree has area 1.
     * @param depthsDensity Array that is filled with the areas covered by each depth.
     **/
    void getDepthDensity(std::vector<float>& depthsDensity)
    {
        depthsDensity.resize(mMaxDepth + 1);
        std::vector<uint32_t> nodesPerDepth(depthsDensity.size(), 0);
        uint32_t numLeaves = 0;
        std::function<void(OctreeNode&, uint32_t)> vistNode;
        vistNode = [&](OctreeNode& node, uint32_t depth)
        {
            node.removeMark();

            // Iterate children
            if(!node.isLeaf())
            {
                for(uint32_t i = 0; i < 8; i++)
                {
                    vistNode(mOctreeData[node.getChildrenIndex() + i], depth+1);
                }
            }
            else
            {
                nodesPerDepth[depth]++;
                numLeaves++;
            }
        };

        const uint32_t startDepth = glm::round(glm::log2(static_cast<float>(mStartGridSize)));

        for(uint32_t k=0; k < mStartGridSize; k++)
        {
            for(uint32_t j=0; j < mStartGridSize; j++)
            {
                for(uint32_t i=0; i < mStartGridSize; i++)
                {
                    const uint32_t nodeStartIndex = k * mStartGridSize * mStartGridSize + j * mStartGridSize + i;
                    vistNode(mOctreeData[nodeStartIndex], startDepth);
                }
            }
        }

        float size = 1.0f;
        for(uint32_t d=0; d < depthsDensity.size(); d++)
        {
            depthsDensity[d] = size * static_cast<float>(nodesPerDepth[d]);
            size *= 0.125f;
        }
    }
protected:
    // Octree bounding box
    BoundingBox mBox;

    // Stores the maximum distance in absulute value contained by the octree
    float mValueRange;
    // Stores the minimum distance in the border of the octree box
    float mMinBorderValue;
    
    // Octree start grid
    int mStartGridSize = 0;
    int mStartGridXY = 0;
    float mStartGridCellSize = 0.0f;

    uint32_t mMaxDepth;
    bool mSdfOnlyAySurface;
    // Array storing the octree nodes and the arrays of coefficients
    std::vector<OctreeNode> mOctreeData;
};
}

#endif