#ifndef OCTREE_SDF_H
#define OCTREE_SDF_H

#include <array>
#include <optional>

#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "utils/UsefullSerializations.h"
#include "SdfFunction.h"

#include <cereal/types/vector.hpp>

namespace sdflib
{
/**
 * @brief The class constructs and stores a structure for accelerating queries to distance fields with an expected error.
 *        The structure is an octree where each leaf stores a polynomial representing the field behavior inside the node.
 **/
class OctreeSdf : public SdfFunction
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
        SIMPSONS_RULE // Estimate error integral by the simpson rule
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

    // Constructors
    OctreeSdf() {}
    /**
     * @param mesh The input mesh.
     * @param box The area that the structure must cover.
     * @param maxDepth The maximum octree depth.
     * @param startDepth The start depth of the octree.
     * @param minTrianglesPerNode The minimum error expected in a node.
     *                            All the leaves before the maximum depth must have an error less than 
     *                            this minimum.
     * @param initAlgorithm The building algorithm.
     **/
    OctreeSdf(const Mesh& mesh, BoundingBox box, uint32_t depth, uint32_t startDepth, 
              float minimumError = 1e-3,
              InitAlgorithm initAlgorithm = InitAlgorithm::NO_CONTINUITY,
              uint32_t numThreads = 1);

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

    /**
     * @brief Computes the area covered by the leaves at different depths, 
     *          supposing that the hole octree has area 1.
     * @param depthsDensity Array that is filled with the areas covered by each depth.
     **/
    void getDepthDensity(std::vector<float>& depthsDensity);

    float getDistance(glm::vec3 sample) const override;
    float getDistance(glm::vec3 sample, glm::vec3& outGradient) const override;
	SdfFunction::SdfFormat getFormat() const override { return SdfFunction::SdfFormat::OCTREE; }

    // Load and save function for storing the structure on disk
    template<class Archive>
    void save(Archive & archive) const
    { 
        archive(mBox, mStartGridSize, mMaxDepth, mValueRange, mMinBorderValue, mOctreeData);
    }

    template<class Archive>
    void load(Archive & archive)
    {
        archive(mBox, mStartGridSize, mMaxDepth, mValueRange, mMinBorderValue, mOctreeData);
        
        mStartGridCellSize = mBox.getSize().x / static_cast<float>(mStartGridSize);
        mStartGridXY = mStartGridSize * mStartGridSize;

        float total = mOctreeData.size() * sizeof(OctreeNode);
        SPDLOG_INFO("Octree Sdf Total: {}MB", total/1048576.0f);
    } 

private:
    // Option to delay the node termination and recyle the distances already calculated
    static constexpr bool DELAY_NODE_TERMINATION = false;

    // The depth in which the process start the subdivision
    static constexpr uint32_t START_OCTREE_DEPTH = 1;

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
    // Array storing the octree nodes and the arrays of coefficients
    std::vector<OctreeNode> mOctreeData;

    // Functions to construct the structure with different strategies
    template<typename TrianglesInfluenceStrategy>
    void initOctree(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                    float terminationThreshold, TerminationRule terminationRule,
                    uint32_t numThreads = 1);

    template<typename TrianglesInfluenceStrategy>
    void initOctreeWithContinuity(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                                  float terminationThreshold, TerminationRule terminationRule);

    template<typename TrianglesInfluenceStrategy>
    void initOctreeWithContinuityNoDelay(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                                         float terminationThreshold, OctreeSdf::TerminationRule terminationRule,
                                         uint32_t numThreads = 1);
    
    void initUniformOctree(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth); // For testing propouses

    void initOctreeInGPU(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                         float terminationThreshold, TerminationRule terminationRule);

    void computeMinBorderValue();
};
}

#endif