#include "SdfExportFunc.h"
#include "spdlog/sinks/rotating_file_sink.h"

#include <InteractiveComputerGraphics/TriangleMeshDistance.h>

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

EXPORT void saveExactOctreeSdf(SdfFunction* sdfPointer, char* path)
{
    std::string p(path);
    sdfPointer->saveToFile(p);
}

EXPORT SdfFunction* loadExactOctreeSdf(char* path)
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
                                         uint32_t minTrianglesPerNode)
{
    auto file_logger = spdlog::rotating_logger_mt("file_logger", "logs/mylogfile", 1048576 * 5, 3);
    spdlog::set_default_logger(file_logger);
    
    BoundingBox octreeBox(
        glm::vec3(bbMinX, bbMinY, bbMinZ),
        glm::vec3(bbMaxX, bbMaxY, bbMaxZ)
    );

    Mesh mesh(vertices, numVertices,
              indices, numIndices);

    ExactOctreeSdf* sdf = new ExactOctreeSdf(mesh, octreeBox, maxOctreeDepth, startOctreeDepth, minTrianglesPerNode);

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