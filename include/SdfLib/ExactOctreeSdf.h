#ifndef EXACT_OCTREE_SDF_H
#define EXACT_OCTREE_SDF_H

#include <array>

#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "utils/UsefullSerializations.h"
#include "SdfFunction.h"

namespace sdflib
{
/**
 * @brief The class constructs and stores a structure for accelerating exact queries to distance fields.
 *        The structure is an octree where each leaf stores the triangles influencing it.
 **/
class ExactOctreeSdf : public SdfFunction
{
public:

    /**
     * @brief Structure representing a node of the octree.
     * 
     *        If it is an inner node, it stores a index pointing 
     *          to the start of an array containing its 8 children.
     *        Moreover, if it is in the bit encoding start depth, it stores 
     *          a index pointing to the set of triangles containing the node.
     *        Also, if it is in a lower depth than the bit encoding start depth,
     *          it stores a index pointing the bit mask set that stores which triangles
     *          influence the node regarding its parent triangles.
     * 
     *        If it is a leaf node, it only stores a index pointing 
     *          to the set of triangles influencing the node.
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
        static constexpr uint32_t CHILDREN_INDEX_MASK = ~IS_LEAF_MASK;
        
        uint32_t childrenIndex;
        uint32_t trianglesArrayIndex;

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
            ar(trianglesArrayIndex);
        }
    };

    // Constructors
    ExactOctreeSdf() {}
    /**
     * @param mesh The input mesh.
     * @param box The area that the structure must cover.
     * @param maxDepth The maximum octree depth.
     * @param startDepth The start depth of the octree.
     * @param minTrianglesPerNode The minimum number of triangles influencing a node.
     *                            All the leaves before the maximum depth must have less than 
     *                            this minimum influencing them.
     * @param numThreads The maximum number of threads to use during the structure construction.
     **/
    ExactOctreeSdf(const Mesh& mesh, BoundingBox box, uint32_t maxDepth,
                   uint32_t startDepth=1, uint32_t minTrianglesPerNode = 128,
                   uint32_t numThreads=1);

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
     * @return Returns the maximum number of triangles influencing a leaf
     **/
    uint32_t getMaxTrianglesInLeafs() const { return mMaxTrianglesInLeafs; }

    /**
     * @return Returns the minimum number of triangles influencing a leaf
     **/
    uint32_t getMinTrianglesInLeafs() const { return mMinTrianglesInLeafs; }

    /**
     * @return The octree maximum depth
     **/
    uint32_t getOctreeMaxDepth() const { return mMaxDepth; }

    /**
     * @return The array containing all the octree structure
     **/
    const std::vector<OctreeNode>& getOctreeData() const { return mOctreeData; }

    /**
     * @return The array of triangles properties used to compute distances to triangles
     **/
    const std::vector<TriangleUtils::TriangleData>& getTrianglesData() { return mTrianglesData; }

    float getDistance(glm::vec3 sample) const override;
    float getDistance(glm::vec3 sample, glm::vec3& outGradient) const override;
    SdfFormat getFormat() const override { return SdfFormat::EXACT_OCTREE; }


    // Load and save function for storing the structure on disk
    template<class Archive>
    void save(Archive & archive) const
    { 
        archive(mBox, mStartGridSize, mStartDepth, mMinTrianglesInLeafs, mMaxTrianglesInLeafs, mMaxTrianglesEncodedInLeafs, mBitEncodingStartDepth, mBitsPerIndex, mMaxDepth, mOctreeData, mTrianglesSets, mTrianglesMasks, mTrianglesData);
    }

    template<class Archive>
    void load(Archive & archive)
    {
        archive(mBox, mStartGridSize, mStartDepth, mMinTrianglesInLeafs, mMaxTrianglesInLeafs, mMaxTrianglesEncodedInLeafs, mBitEncodingStartDepth, mBitsPerIndex, mMaxDepth, mOctreeData, mTrianglesSets, mTrianglesMasks, mTrianglesData);
        
        mStartGridCellSize = mBox.getSize().x / static_cast<float>(mStartGridSize);
        mStartGridXY = mStartGridSize * mStartGridSize;

        mTrianglesCache[0].resize(mMaxTrianglesEncodedInLeafs);
        mTrianglesCache[1].resize(mMaxTrianglesEncodedInLeafs);
        
        // Print structure size
        SPDLOG_INFO("Octree Data: {}", mOctreeData.size() * sizeof(OctreeNode));
        SPDLOG_INFO("Triangle Sets: {}", mTrianglesSets.size() * sizeof(uint32_t));
        SPDLOG_INFO("Triangle Masks: {}", mTrianglesMasks.size());
        SPDLOG_INFO("Triangle Data: {}", mTrianglesData.size() * sizeof(TriangleUtils::TriangleData));

        float total = mOctreeData.size() * sizeof(OctreeNode) + mTrianglesSets.size() * sizeof(uint32_t) + mTrianglesMasks.size() + mTrianglesData.size() * sizeof(TriangleUtils::TriangleData);
        SPDLOG_INFO("Total: {}MB", total/1048576.0f);
        total = mOctreeData.size() * sizeof(OctreeNode) + mTrianglesSets.size() * sizeof(uint32_t) + mTrianglesMasks.size();
        SPDLOG_INFO("Octree: {}MB", total/1048576.0f);
    }

private:

    // The depth in which the process starts the subdivision
    static constexpr uint32_t START_OCTREE_DEPTH = 1;
    // The last levels in which the triangles are bit encoded
    static constexpr uint32_t BIT_ENCODING_DEPTH = 2;

    // Octree bounding box
    BoundingBox mBox;

    // Array used to decode the bit encoding during the structure queries
    mutable std::array<std::vector<uint32_t>, 2> mTrianglesCache;

    // Structure properties
    uint32_t mMinTrianglesInLeafs;
    uint32_t mMaxTrianglesInLeafs;

    uint32_t mMaxTrianglesEncodedInLeafs;
    uint32_t mBitEncodingStartDepth;
    uint32_t mBitsPerIndex;

    uint32_t mMaxDepth;

    // Octree start grid
    int mStartGridSize = 0;
    int mStartGridXY = 0;
    uint32_t mStartDepth = 0;
    float mStartGridCellSize = 0.0f;

    // Structure arrays
    std::vector<OctreeNode> mOctreeData; // List of nodes
    std::vector<uint32_t> mTrianglesSets; // List storing sets of triangles
                                          // The first element of each set is the size of the set
                                          // Each triangle is stored using only a specific number of bits (mBitsPerIndex attribute)
    std::vector<uint8_t> mTrianglesMasks; // List storing sets of triangles bit encoded
    std::vector<TriangleUtils::TriangleData> mTrianglesData; // Triangle properties

    template<typename TrianglesInfluenceStrategy>
    void initOctree(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                    uint32_t minTrianglesPerNode, uint32_t numThreads = 1);

    std::vector<uint32_t> evalNode(uint32_t nodeIndex, uint32_t depth, 
                                    std::vector<uint32_t>& mergedTriangles, 
                                    std::vector<uint32_t>& nodesMerged,
                                    std::vector<uint32_t>& differentTriangles);

    void calculateStatistics();
};
}

#include "ExactOctreeSdfDepthFirst.h"

#endif