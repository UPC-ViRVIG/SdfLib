#include "SdfExportFunc.h"
#include "spdlog/sinks/rotating_file_sink.h"

EXPORT SdfFunction* createExactOctreeSdf(glm::vec3* vertices, uint32_t numVertices, 
                                         uint32_t* indices, uint32_t numIndices,
                                         float bbMinX, float bbMinY, float bbMinZ,
                                         float bbMaxX, float bbMaxY, float bbMaxZ,
                                         uint32_t startOctreeDepth,
                                         uint32_t maxOctreeDepth,
                                         uint32_t minTrianglesPerNode)
{
    BoundingBox octreeBox(
        glm::vec3(bbMinX, bbMinY, bbMinZ),
        glm::vec3(bbMaxX, bbMaxY, bbMaxZ)
    );

    Mesh mesh(vertices, numVertices,
              indices, numIndices);

    ExactOctreeSdf* sdf = new ExactOctreeSdf(mesh, octreeBox, maxOctreeDepth, startOctreeDepth, minTrianglesPerNode);

    return sdf;
}

EXPORT float getDistance(SdfFunction* sdfPointer, float pointX, float pointY, float pointZ)
{
    float res = sdfPointer->getDistance(glm::vec3(pointX, pointY, pointZ));
	return res;
}

EXPORT void deleteSdf(SdfFunction* sdfPointer)
{
    switch(sdfPointer->getFormat())
    {
        case SdfFunction::SdfFormat::EXACT_OCTREE:
            delete reinterpret_cast<ExactOctreeSdf*>(sdfPointer);
            break;
        default:
            break;
    }
}