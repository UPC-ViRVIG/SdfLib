#ifndef SDF_FUNCTION_H
#define SDF_FUNCTION_H

#include <glm/glm.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <fstream>
#include <InteractiveComputerGraphics/TriangleMeshDistance.h>

#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"

namespace sdflib
{
class SdfFunction
{
public:
    // Supported formats
    enum SdfFormat
    {
        GRID,
        TRILINEAR_OCTREE,
        TRILINEAR_OCTREE2,
        TRICUBIC_OCTREE,
        HYBRID_OCTREE,
        EXACT_OCTREE,
        NONE
    };

    virtual ~SdfFunction() = default;

    /**
     * @return The signed distance to the mesh at the point
     **/
    virtual float getDistance(glm::vec3 sample) const = 0;
    /**
     * @return The signed distance and gradient field at the point
     * @param outGradient Returns the gradient of the field
     **/
    virtual float getDistance(glm::vec3 sample, glm::vec3& outGradient) const = 0;
    /**
     * @return The bounding box that can be queried
     **/
    virtual BoundingBox getSampleArea() const 
    {
        return BoundingBox(glm::vec3(-INFINITY), glm::vec3(INFINITY));
    }
    
    /**
     * @return The format of the structure
     **/
    virtual SdfFormat getFormat() const { return SdfFormat::NONE; }
    
    /**
     * @brief Stores the structure to disk.
     * @param outputPath The file path where the structure should be stored
     * @return If the structure has been stored successfully
     **/
    bool saveToFile(const std::string& outputPath);

    /**
     * @brief Load a structure from disk
     * @param inputPath The file path where the structure is stored
     * @return The pointer to the loaded structure. 
     *          If the file cannot be successfully loaded, it returns nullptr.
     **/
    static std::unique_ptr<SdfFunction> loadFromFile(const std::string& inputPath);
};

class MeshSvhSdf : public SdfFunction
{
public:
    MeshSvhSdf(const Mesh& mesh)
    : mesh_distance(toDoubleVector(mesh.getVertices()),
                    *reinterpret_cast<const std::vector<std::array<int, 3>>*>(&mesh.getIndices())),
      trianglesData(TriangleUtils::calculateMeshTriangleData(mesh)), 
      mesh(mesh)
    {}

    inline float getDistance(glm::vec3 sample) const override
    {
        uint32_t tId = getNearestTriangle(sample);
        return TriangleUtils::getSignedDistPointAndTriangle(sample, trianglesData[tId]);
    }
    
    inline float getDistance(glm::vec3 sample, glm::vec3& outGradient) const override
    {
        uint32_t tId = getNearestTriangle(sample);
        return TriangleUtils::getSignedDistPointAndTriangle(sample, trianglesData[tId],
                                                            mesh.getVertices()[mesh.getIndices()[3 * tId]],
                                                            mesh.getVertices()[mesh.getIndices()[3 * tId + 1]],
                                                            mesh.getVertices()[mesh.getIndices()[3 * tId + 2]],
                                                            outGradient);
    }    

    inline float getDistance(glm::vec3 samplePoint)
    {
        tmd::Result result = mesh_distance.signed_distance({ samplePoint.x, samplePoint.y, samplePoint.z });
        return result.distance;
    }

    inline uint32_t getNearestTriangle(glm::vec3 samplePoint) const
    {
        tmd::Result result = mesh_distance.signed_distance({ samplePoint.x, samplePoint.y, samplePoint.z });
        return static_cast<uint32_t>(result.triangle_id);
    }

private:
    tmd::TriangleMeshDistance mesh_distance;
    std::vector<TriangleUtils::TriangleData> trianglesData;
    const Mesh& mesh;

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

}

#endif