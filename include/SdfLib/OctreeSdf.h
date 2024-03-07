#ifndef OCTREE_SDF_H
#define OCTREE_SDF_H

#include <array>
#include <optional>

#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "utils/UsefullSerializations.h"

#include "SdfLib/TrianglesInfluence.h"
#include "SdfLib/InterpolationMethods.h"
#include "IOctreeSdf.h"

#include <cereal/types/vector.hpp>

namespace sdflib
{
/**
 * @brief The class constructs and stores a structure for accelerating queries to distance fields with an expected error.
 *        The structure is an octree where each leaf stores a polynomial representing the field behavior inside the node.
 **/
template <typename InterpolationMethod=TriLinearInterpolation>
class TOctreeSdf : public IOctreeSdf
{
public:
    // Constructors
    TOctreeSdf() {}
    /**
     * @param mesh The input mesh.
     * @param box The area that the structure must cover.
     * @param maxDepth The maximum octree depth.
     * @param startDepth The start depth of the octree.
     * @param minimumError The minimum error expected in a node.
     *                     All the leaves before the maximum depth must have an error less than 
     *                     this minimum.
     * @param initAlgorithm The building algorithm.
     * @param terminationRule The heuristic used to decide if one node has to be subdivided
     **/
    TOctreeSdf(const Mesh& mesh, BoundingBox box, uint32_t depth, uint32_t startDepth, 
              float maxError = 1e-3,
              InitAlgorithm initAlgorithm = InitAlgorithm::NO_CONTINUITY,
              uint32_t numThreads = 1)
    {
        buildOctree(mesh, box, depth, startDepth, 
                TOctreeSdf::TerminationRule::TRAPEZOIDAL_RULE, 
                TerminationRuleParams::setTrapezoidalRuleParams(maxError),
                initAlgorithm, numThreads);
    }

    /**
     * @param mesh The input mesh.
     * @param box The area that the structure must cover.
     * @param maxDepth The maximum octree depth.
     * @param startDepth The start depth of the octree.
     * @param terminationRule The algorithm termination rule
     * @param params The parameters of the termination rule chosen
     * @param initAlgorithm The building algorithm.
     **/
    TOctreeSdf(const Mesh& mesh, BoundingBox box, uint32_t depth, uint32_t startDepth, 
              TerminationRule terminationRule, TerminationRuleParams params,
              InitAlgorithm initAlgorithm, uint32_t numThreads = 1)
    {
        buildOctree(mesh, box, depth, startDepth, terminationRule, params, initAlgorithm, numThreads);
    }

    float getDistance(glm::vec3 sample) const override;
    float getDistance(glm::vec3 sample, glm::vec3& outGradient) const override;

    OctreeNode getGridNode(glm::vec3 sample, glm::vec3& leafPos, float& leafSize) const;
    OctreeNode getLeaf(glm::vec3 sample, glm::vec3& leafPos, float& leafSize) const;
	SdfFunction::SdfFormat getFormat() const override { return SdfFunction::SdfFormat::NONE; }

    // Load and save function for storing the structure on disk
    template<class Archive>
    void save(Archive & archive) const
    { 
        archive(mBox, mStartGridSize, mMaxDepth, mSdfOnlyAySurface, mValueRange, mMinBorderValue, mOctreeData);
    }

    template<class Archive>
    void load(Archive & archive)
    {
        archive(mBox, mStartGridSize, mMaxDepth, mSdfOnlyAySurface, mValueRange, mMinBorderValue, mOctreeData);
        
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

    void buildOctree(const Mesh& mesh, BoundingBox box, uint32_t depth, uint32_t startDepth, 
                     TerminationRule terminationRule, TerminationRuleParams params,
                     InitAlgorithm initAlgorithm, uint32_t numThreads = 1)
    {
        mMaxDepth = depth;

        const glm::vec3 bbSize = box.getSize();
        const float maxSize = glm::max(glm::max(bbSize.x, bbSize.y), bbSize.z);
        mBox.min = box.getCenter() - 0.5f * maxSize;
        mBox.max = box.getCenter() + 0.5f * maxSize;

        mStartGridSize = 1 << startDepth;
        mStartGridXY = mStartGridSize * mStartGridSize;

        mStartGridCellSize = maxSize / static_cast<float>(mStartGridSize);

        mSdfOnlyAySurface = terminationRule == TerminationRule::ISOSURFACE; 

        switch(initAlgorithm)
        {
            case TOctreeSdf::InitAlgorithm::UNIFORM:
                // initUniformOctree(mesh, startDepth, depth);
                std::cerr << "ERROR: Uniform algoirthm not currently supported" << std::endl;
                break;
            case TOctreeSdf::InitAlgorithm::NO_CONTINUITY:
                initOctree<VHQueries<InterpolationMethod>>(mesh, startDepth, depth, terminationRule, params, numThreads);
                break;
            case TOctreeSdf::InitAlgorithm::CONTINUITY:
                if(DELAY_NODE_TERMINATION)
                {
                    initOctreeWithContinuity<VHQueries<InterpolationMethod>>(mesh, startDepth, depth,terminationRule, params);
                }
                else
                {
                    initOctreeWithContinuityNoDelay<VHQueries<InterpolationMethod>>(mesh, startDepth, depth, terminationRule, params, numThreads);
                }
                break;
            // case TOctreeSdf::InitAlgorithm::GPU_IMPLEMENTATION:
            //     initOctreeInGPU(mesh, startDepth, depth, terminationThreshold, terminationRule);
            //     break;
        }

        computeMinBorderValue();
        if(terminationRule == TOctreeSdf::TerminationRule::ISOSURFACE)
        {
            reduceTree();
        }
    }

    // Functions to construct the structure with different strategies
    template<typename TrianglesInfluenceStrategy>
    void initOctree(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                    TerminationRule terminationRule,
                    TerminationRuleParams terminationRuleParams,
                    uint32_t numThreads = 1);

    template<typename TrianglesInfluenceStrategy>
    void initOctreeWithContinuity(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                                  TerminationRule terminationRule,
                                  TerminationRuleParams terminationRuleParams);

    template<typename TrianglesInfluenceStrategy>
    void initOctreeWithContinuityNoDelay(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                                         TOctreeSdf::TerminationRule terminationRule,
                                         TerminationRuleParams terminationRuleParams,
                                         uint32_t numThreads = 1);
    
    // Not supported
    // void initUniformOctree(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth); // For testing propouses

    void initOctreeInGPU(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                         float terminationThreshold, TerminationRule terminationRule);

    void computeMinBorderValue();

    // Function reduces leafs that do not contain the isosurface
    void reduceTree();
};

template<>
SdfFunction::SdfFormat TOctreeSdf<TriLinearInterpolation>::getFormat() const
{
    return SdfFunction::SdfFormat::TRILINEAR_OCTREE;
}

template<>
SdfFunction::SdfFormat TOctreeSdf<TriCubicInterpolation>::getFormat() const
{
    return SdfFunction::SdfFormat::TRICUBIC_OCTREE;
}

typedef TOctreeSdf<> OctreeSdf;

// --- Public method definition --- //
template<typename InterpolationMethod>
float TOctreeSdf<InterpolationMethod>::getDistance(glm::vec3 sample) const
{
    auto roundFloat = [](float a) -> uint32_t
    {
        return (a >= 0.5f) ? 1 : 0;
    };

    glm::vec3 fracPart = (sample - mBox.min) / mStartGridCellSize;
    glm::ivec3 startArrayPos = glm::floor(fracPart);
    fracPart = glm::fract(fracPart);

    if(startArrayPos.x < 0 || startArrayPos.x >= mStartGridSize ||
       startArrayPos.y < 0 || startArrayPos.y >= mStartGridSize ||
       startArrayPos.z < 0 || startArrayPos.z >= mStartGridSize)
    {
        return mBox.getDistance(sample) + mMinBorderValue;
    }

    const OctreeNode* currentNode = &mOctreeData[startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x];

    while(!currentNode->isLeaf())
    {
        const uint32_t childIdx = (roundFloat(fracPart.z) << 2) + 
                                  (roundFloat(fracPart.y) << 1) + 
                                   roundFloat(fracPart.x);

        currentNode = &mOctreeData[currentNode->getChildrenIndex() + childIdx];
        fracPart = glm::fract(2.0f * fracPart);
    }

    if(currentNode->getChildrenIndex() >= mOctreeData.size()) return 10.0;

    auto& values = *reinterpret_cast<const std::array<float, InterpolationMethod::NUM_COEFFICIENTS>*>(&mOctreeData[currentNode->getChildrenIndex()]);

    return InterpolationMethod::interpolateValue(values, fracPart);
}

template<typename InterpolationMethod>
float TOctreeSdf<InterpolationMethod>::getDistance(glm::vec3 sample, glm::vec3& outGradient) const
{
    auto roundFloat = [](float a) -> uint32_t
    {
        return (a >= 0.5f) ? 1 : 0;
    };

    glm::vec3 fracPart = (sample - mBox.min) / mStartGridCellSize;
    glm::ivec3 startArrayPos = glm::floor(fracPart);
    fracPart = glm::fract(fracPart);

    if(startArrayPos.x < 0 || startArrayPos.x >= mStartGridSize ||
       startArrayPos.y < 0 || startArrayPos.y >= mStartGridSize ||
       startArrayPos.z < 0 || startArrayPos.z >= mStartGridSize)
    {
        return mBox.getDistance(sample, outGradient) + mMinBorderValue;
    }

    const OctreeNode* currentNode = &mOctreeData[startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x];

    while(!currentNode->isLeaf())
    {
        const uint32_t childIdx = (roundFloat(fracPart.z) << 2) + 
                                  (roundFloat(fracPart.y) << 1) + 
                                   roundFloat(fracPart.x);

        currentNode = &mOctreeData[currentNode->getChildrenIndex() + childIdx];
        fracPart = glm::fract(2.0f * fracPart);
    }

    auto& values = *reinterpret_cast<const std::array<float, InterpolationMethod::NUM_COEFFICIENTS>*>(&mOctreeData[currentNode->getChildrenIndex()]);

    outGradient = glm::normalize(InterpolationMethod::interpolateGradient(values, fracPart));
    return InterpolationMethod::interpolateValue(values, fracPart);
}

template<typename InterpolationMethod>
typename TOctreeSdf<InterpolationMethod>::OctreeNode TOctreeSdf<InterpolationMethod>::getLeaf(glm::vec3 sample, glm::vec3& leafPos, float& leafSize) const
{
    auto roundFloat = [](float a) -> uint32_t
    {
        return (a >= 0.5f) ? 1 : 0;
    };
    
    glm::vec3 fracPart = (sample - mBox.min) / mStartGridCellSize;
    glm::ivec3 startArrayPos = glm::floor(fracPart);
    fracPart = glm::fract(fracPart);

    const OctreeNode* currentNode = &mOctreeData[startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x];

    leafPos = glm::vec3(0.0f);
    leafSize = 1.0f;

    while (!currentNode->isLeaf())
    {
        const uint32_t childIdx = (roundFloat(fracPart.z) << 2) +
            (roundFloat(fracPart.y) << 1) +
            roundFloat(fracPart.x);

        currentNode = &mOctreeData[currentNode->getChildrenIndex() + childIdx];
        leafSize *= 0.5f;
        leafPos += glm::vec3(leafSize * roundFloat(fracPart.x), leafSize * roundFloat(fracPart.y), leafSize * roundFloat(fracPart.z));
        fracPart = glm::fract(2.0f * fracPart);
    }

    leafPos = mBox.min + (glm::vec3(startArrayPos) + leafPos) * mStartGridCellSize;
    leafSize *= mStartGridCellSize;

    return *currentNode;
}

template<typename InterpolationMethod>
typename TOctreeSdf<InterpolationMethod>::OctreeNode TOctreeSdf<InterpolationMethod>::getGridNode(glm::vec3 sample, glm::vec3& nodePos, float& nodeSize) const
{
    glm::vec3 fracPart = (sample - mBox.min) / mStartGridCellSize;
    glm::ivec3 startArrayPos = glm::floor(fracPart);
    fracPart = glm::fract(fracPart);

    nodePos = mBox.min + glm::vec3(startArrayPos) * mStartGridCellSize;
    nodeSize = mStartGridCellSize;

    const OctreeNode* currentNode = &mOctreeData[startArrayPos.z * mStartGridXY + startArrayPos.y * mStartGridSize + startArrayPos.x];

    return *currentNode;
}

template<typename InterpolationMethod>
void TOctreeSdf<InterpolationMethod>::computeMinBorderValue()
{
    const std::array<glm::vec3, 8> childrens = 
    {
        glm::vec3(-1.0f, -1.0f, -1.0f),
        glm::vec3(1.0f, -1.0f, -1.0f),
        glm::vec3(-1.0f, 1.0f, -1.0f),
        glm::vec3(1.0f, 1.0f, -1.0f),

        glm::vec3(-1.0f, -1.0f, 1.0f),
        glm::vec3(1.0f, -1.0f, 1.0f),
        glm::vec3(-1.0f, 1.0f, 1.0f),
        glm::vec3(1.0f, 1.0f, 1.0f)
    };

    std::function<float(uint32_t nIdx, glm::vec3 pos, float halfSize)> processNode;
    processNode = [&](uint32_t nIdx, glm::vec3 pos, float halfSize) -> float
    {
        if(!mOctreeData[nIdx].isLeaf())
        {
            float minValue = INFINITY;
            for(uint32_t i=0; i < 8; i++)
            {
                const glm::vec3 cp = pos + 0.5f * halfSize * childrens[i];
                if(cp.x < halfSize || cp.y < halfSize || cp.z < halfSize ||
                   cp.x > (1.0f-halfSize) || cp.y > (1.0f-halfSize) || cp.z > (1.0f-halfSize))
                {
                    minValue = glm::min(minValue, processNode(mOctreeData[nIdx].getChildrenIndex() + i, cp, 0.5f * halfSize));
                }
            }

            return minValue;
        }
        else
        {
            float minValue = INFINITY;
            for(uint32_t i=0; i < 8; i++)
            {
                const glm::vec3 sp = pos + halfSize * childrens[i];
                if(sp.x < 1e-4 || sp.y < 1e-4 || sp.z < 1e-4 ||
                   sp.x > (1.0f-1e-4) || sp.y > (1.0f-1e-4) || sp.z > (1.0f-1e-4))
                {
                    std::array<float, InterpolationMethod::NUM_COEFFICIENTS>* coeff = reinterpret_cast<std::array<float, InterpolationMethod::NUM_COEFFICIENTS>*>(
                                                                                            &mOctreeData[mOctreeData[nIdx].getChildrenIndex()]);
                    minValue = glm::min(minValue, InterpolationMethod::interpolateValue(*coeff, 0.5f * childrens[i] + glm::vec3(0.5f)));

                    float val = InterpolationMethod::interpolateValue(*coeff, 0.5f * childrens[i] + glm::vec3(0.5f));
                }
            }

            return minValue;
        }

        return INFINITY;
    };

    const float gridCellSize = 1.0f / static_cast<float>(mStartGridSize);

    float minValue = INFINITY;
    for(uint32_t k=0; k < mStartGridSize; k++)
    {
        for(uint32_t j=0; j < mStartGridSize; j++)
        {
            for(uint32_t i=0; i < mStartGridSize; i++)
            {
                const uint32_t idx = k * mStartGridXY + j * mStartGridSize + i;
                const glm::vec3 pos((static_cast<float>(i) + 0.5f) * gridCellSize,
                                    (static_cast<float>(j) + 0.5f) * gridCellSize,
                                    (static_cast<float>(k) + 0.5f) * gridCellSize);
                minValue = glm::min(minValue, processNode(idx, pos, 0.5f * gridCellSize));
            }
        }
    }

    mMinBorderValue = minValue;
}

template<typename InterpolationMethod>
void TOctreeSdf<InterpolationMethod>::reduceTree()
{
    std::vector<TOctreeSdf::OctreeNode> octreeTmp = mOctreeData;
    uint32_t currentIndex = mStartGridSize * mStartGridSize * mStartGridSize;
    std::function<bool(OctreeNode&, uint32_t)> vistNode;
    vistNode = [&](OctreeNode& node, uint32_t nodeIndex)
    {
        // Iterate children
        if(!node.isLeaf())
        {
            bool reduceNode = true;
            uint32_t childrenIndices = currentIndex;
            currentIndex += 8;
            for(uint32_t i = 0; i < 8; i++)
            {
                reduceNode = vistNode(octreeTmp[node.getChildrenIndex() + i], childrenIndices + i) && reduceNode;
            }

            if(reduceNode)
            {
                mOctreeData[nodeIndex].setValues(true, std::numeric_limits<uint32_t>::max());
                currentIndex = childrenIndices;
            } 
            else
            {
                mOctreeData[nodeIndex].setValues(false, childrenIndices);
            }

            return reduceNode;
        }
        else
        {
            auto& values = *reinterpret_cast<const std::array<float, InterpolationMethod::NUM_COEFFICIENTS>*>(&octreeTmp[node.getChildrenIndex()]);
            if(InterpolationMethod::isIsosurfaceInside(values))
            {
                mOctreeData[nodeIndex].setValues(true, currentIndex);
                mOctreeData[nodeIndex].markNode();
                for(uint32_t i=0; i < InterpolationMethod::NUM_COEFFICIENTS; i++)
                {
                    mOctreeData[currentIndex++].value = values[i];
                }
                return false;
            }
            else
            {
                mOctreeData[nodeIndex].setValues(true, std::numeric_limits<uint32_t>::max());
                return true;
            }
        }

        return false;
    };

    for(uint32_t k=0; k < mStartGridSize; k++)
    {
        for(uint32_t j=0; j < mStartGridSize; j++)
        {
            for(uint32_t i=0; i < mStartGridSize; i++)
            {
                const uint32_t nodeStartIndex = k * mStartGridSize * mStartGridSize + j * mStartGridSize + i;
                vistNode(octreeTmp[nodeStartIndex], nodeStartIndex);
            }
        }
    }

    mOctreeData.resize(currentIndex);
}
}

#include "OctreeSdfDepthFirst.h"
#include "OctreeSdfBreadthFirst.h"
#include "OctreeSdfBreadthFirstNoDelay.h"

#endif