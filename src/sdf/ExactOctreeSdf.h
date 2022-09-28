#ifndef EXACT_OCTREE_SDF_H
#define EXACT_OCTREE_SDF_H

#include <array>

#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "utils/UsefullSerializations.h"
#include "SdfFunction.h"

class ExactOctreeSdf : public SdfFunction
{
public:
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

    ExactOctreeSdf() {}
    ExactOctreeSdf(const Mesh& mesh, BoundingBox box, uint32_t maxDepth,
                   uint32_t startDepth=1, uint32_t minTrianglesPerNode = 128);

    glm::ivec3 getStartGridSize() const { return glm::ivec3(mStartGridSize); }
    const BoundingBox& getGridBoundingBox() const { return mBox; }
    BoundingBox getSampleArea() const override { return mBox; }
    uint32_t getMaxTrianglesInLeafs() const { return mMaxTrianglesInLeafs; }
    uint32_t getMinTrianglesInLeafs() const { return mMinTrianglesInLeafs; }
    uint32_t getOctreeMaxDepth() const { return mMaxDepth; }
    const std::vector<OctreeNode>& getOctreeData() const { return mOctreeData; }
    const std::vector<TriangleUtils::TriangleData>& getTrianglesData() { return mTrianglesData; }

    float getDistance(glm::vec3 sample) const override;
    float getDistance(glm::vec3 sample, glm::vec3& outGradient) const override;
    SdfFormat getFormat() const override { return SdfFormat::EXACT_OCTREE; }

    void test(glm::vec3 sample)
    {
        float dist = getDistance(sample);
        float minDist = INFINITY;
        uint32_t minIndex = 0;
        for(uint32_t t=0; t < mTrianglesData.size(); t++)
        {
            const float dist = TriangleUtils::getSqDistPointAndTriangle(sample, mTrianglesData[t]);
            if(dist < minDist)
            {
                minIndex = t;
                minDist = dist;
            }
        }

        float dist2 = TriangleUtils::getSignedDistPointAndTriangle(sample, mTrianglesData[minIndex]);
        
        std::cout << dist << " // " << dist2 << std::endl;
    }

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

        std::cout << "Octree Data: " << mOctreeData.size() * sizeof(OctreeNode) << std::endl;
        std::cout << "Triangle Sets: " << mTrianglesSets.size() * sizeof(uint32_t) << std::endl;
        std::cout << "Triangle Masks: " << mTrianglesMasks.size() << std::endl;
        std::cout << "Triangle Data: " << mTrianglesData.size() * sizeof(TriangleUtils::TriangleData) << std::endl;

        float total = mOctreeData.size() * sizeof(OctreeNode) + mTrianglesSets.size() * sizeof(uint32_t) + mTrianglesMasks.size() + mTrianglesData.size() * sizeof(TriangleUtils::TriangleData);
        std::cout << "Total in MB: " << (total/1048576.0f) << std::endl;
        total = mOctreeData.size() * sizeof(OctreeNode) + mTrianglesSets.size() * sizeof(uint32_t) + mTrianglesMasks.size();
        std::cout << "Octree MB: " << (total/1048576.0f) << std::endl;
    } 

private:

    // The depth in which the process start the subdivision
    static constexpr uint32_t START_OCTREE_DEPTH = 1;
    static constexpr uint32_t BIT_ENCODING_DEPTH = 2;

    BoundingBox mBox;

    mutable std::array<std::vector<uint32_t>, 2> mTrianglesCache;

    // Store the node with the leaf node with the maximum number of triangles
    uint32_t mMinTrianglesInLeafs;
    uint32_t mMaxTrianglesInLeafs;

    uint32_t mMaxTrianglesEncodedInLeafs;
    uint32_t mBitEncodingStartDepth;
    uint32_t mBitsPerIndex;

    int mStartGridSize = 0;
    int mStartGridXY = 0;
    uint32_t mStartDepth = 0;
    float mStartGridCellSize = 0.0f;

    uint32_t mMaxDepth;
    std::vector<OctreeNode> mOctreeData;
    std::vector<uint32_t> mTrianglesSets;
    std::vector<uint8_t> mTrianglesMasks;
    std::vector<TriangleUtils::TriangleData> mTrianglesData;

    template<typename TrianglesInfluenceStrategy>
    void initOctree(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                    uint32_t minTrianglesPerNode);

    std::vector<uint32_t> ExactOctreeSdf::evalNode(uint32_t nodeIndex, uint32_t depth, 
                                                   std::vector<uint32_t>& mergedTriangles, 
                                                   std::vector<uint32_t>& nodesMerged,
                                                   std::vector<uint32_t>& differentTriangles);

    void calculateStatistics();
};

#include "ExactOctreeSdfDepthFirst.h"

#endif