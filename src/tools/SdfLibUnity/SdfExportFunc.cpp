#include "SdfExportFunc.h"
#include "spdlog/sinks/rotating_file_sink.h"

#include <InteractiveComputerGraphics/TriangleMeshDistance.h>

using namespace sdflib;

class ICG
{
public:
    ICG(Mesh& mesh)
    : mesh_distance(toDoubleVector(mesh.getVertices()),
                    *reinterpret_cast<std::vector<std::array<int, 3>>*>(&mesh.getIndices()))
    {}

    inline float getDistance(glm::vec3 samplePoint)
    {
        tmd::Result result = mesh_distance.signed_distance({ samplePoint.x, samplePoint.y, samplePoint.z });
        return result.distance;
    }

    inline float getDistance(glm::vec3 samplePoint, glm::vec3* gradient)
    {
        tmd::Result result = mesh_distance.signed_distance({ samplePoint.x, samplePoint.y, samplePoint.z });
        *gradient = glm::normalize(samplePoint - glm::vec3(result.nearest_point[0], result.nearest_point[1], result.nearest_point[2]));
        return result.distance;
    }
private:
    tmd::TriangleMeshDistance mesh_distance;

    std::vector<std::array<double, 3>> toDoubleVector(const std::vector<glm::vec3>& vec)
    {
        std::vector<std::array<double, 3>> res(vec.size());
        for(uint32_t i=0; i < vec.size(); i++)
        {
            res[i] = { static_cast<double>(vec[i].x), static_cast<double>(vec[i].y), static_cast<double>(vec[i].z) };
        }

        return res;
    }
};

EXPORT void saveSdf(SdfFunction* sdfPointer, char* path)
{
    std::string p(path);
    sdfPointer->saveToFile(p);
}

EXPORT SdfFunction* loadSdf(char* path)
{
    std::string p(path);
    std::unique_ptr<SdfFunction> sdf = SdfFunction::loadFromFile(p);
    return sdf.release();
}

EXPORT SdfFunction* createExactOctreeSdf(glm::vec3* vertices, uint32_t numVertices, 
                                         uint32_t* indices, uint32_t numIndices,
                                         float bbMinX, float bbMinY, float bbMinZ,
                                         float bbMaxX, float bbMaxY, float bbMaxZ,
                                         uint32_t startOctreeDepth,
                                         uint32_t maxOctreeDepth,
                                         uint32_t minTrianglesPerNode,
                                         uint32_t numThreads)
{
    BoundingBox octreeBox(
        glm::vec3(bbMinX, bbMinY, bbMinZ),
        glm::vec3(bbMaxX, bbMaxY, bbMaxZ)
    );

    Mesh mesh(vertices, numVertices,
              indices, numIndices);

    ExactOctreeSdf* sdf = new ExactOctreeSdf(mesh, octreeBox, maxOctreeDepth, startOctreeDepth, minTrianglesPerNode, numThreads);

    return sdf;

    // Mesh mesh(vertices, numVertices,
    //           indices, numIndices);

    // ICG* icg = new ICG(mesh);
    // return reinterpret_cast<SdfFunction*>(icg);
}

EXPORT SdfFunction* createOctreeSdf(glm::vec3* vertices, uint32_t numVertices,
                                    uint32_t* indices, uint32_t numIndices,
                                    float bbMinX, float bbMinY, float bbMinZ,
                                    float bbMaxX, float bbMaxY, float bbMaxZ,
                                    uint32_t startOctreeDepth,
                                    uint32_t maxOctreeDepth,
                                    float maxError,
                                    uint32_t numThreads)
{
    BoundingBox octreeBox(
        glm::vec3(bbMinX, bbMinY, bbMinZ),
        glm::vec3(bbMaxX, bbMaxY, bbMaxZ)
    );

    Mesh mesh(vertices, numVertices,
        indices, numIndices);
       
    OctreeSdf* sdf = new OctreeSdf(mesh, octreeBox, maxOctreeDepth, startOctreeDepth, maxError, 
                                    OctreeSdf::InitAlgorithm::CONTINUITY, 
                                    numThreads);
    //ExactOctreeSdf* sdf = new ExactOctreeSdf(mesh, octreeBox, maxOctreeDepth, startOctreeDepth, minTrianglesPerNode);

    return sdf;

    // Mesh mesh(vertices, numVertices,
    //           indices, numIndices);

    // ICG* icg = new ICG(mesh);
    // return reinterpret_cast<SdfFunction*>(icg);
}

EXPORT float getDistance(SdfFunction* sdfPointer, float pointX, float pointY, float pointZ)
{
    float res = sdfPointer->getDistance(glm::vec3(pointX, pointY, pointZ));
	return res;

    // return reinterpret_cast<ICG*>(sdfPointer)->getDistance(glm::vec3(pointX, pointY, pointZ));
}

EXPORT float getDistanceAndGradient(SdfFunction* sdfPointer, float pointX, float pointY, float pointZ, glm::vec3* outGradient)
{
    float res = sdfPointer->getDistance(glm::vec3(pointX, pointY, pointZ), *outGradient);
	return res;

    // return reinterpret_cast<ICG*>(sdfPointer)->getDistance(glm::vec3(pointX, pointY, pointZ), outGradient);
}

EXPORT glm::vec3 getBBMinPoint(SdfFunction* sdfPointer)
{
    return sdfPointer->getSampleArea().min;
}

EXPORT glm::vec3 getBBSize(SdfFunction* sdfPointer)
{
    return sdfPointer->getSampleArea().getSize();
}

EXPORT uint32_t getStartGridSize(SdfFunction* sdfPointer)
{
    OctreeSdf* octreeSdf = dynamic_cast<OctreeSdf*>(sdfPointer);
    if(octreeSdf != nullptr) return octreeSdf->getStartGridSize().x;
    else return 0;
}

EXPORT uint32_t getOctreeDataSize(SdfFunction* sdfPointer)
{
    if(OctreeSdf* octreeSdf = dynamic_cast<OctreeSdf*>(sdfPointer))
    {
        return octreeSdf->getOctreeData().size();
    }
    
    return 0;
}

EXPORT void getOctreeData(SdfFunction* sdfPointer, uint32_t* data)
{
    if(OctreeSdf* octreeSdf = dynamic_cast<OctreeSdf*>(sdfPointer))
    {
        std::memcpy(data, octreeSdf->getOctreeData().data(), octreeSdf->getOctreeData().size() * sizeof(uint32_t));
    }
    else if(ExactOctreeSdf* exactSdf = dynamic_cast<ExactOctreeSdf*>(sdfPointer))
    {
        // TODO: We need to return more things
    }
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

    // delete reinterpret_cast<ICG*>(sdfPointer);
}