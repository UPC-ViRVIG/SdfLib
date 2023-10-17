#ifndef SDF_FUNCTION_H
#define SDF_FUNCTION_H

#include <glm/glm.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <fstream>

#include "utils/Mesh.h"

namespace sdflib
{
class SdfFunction
{
public:
    // Supported formats
    enum SdfFormat
    {
        GRID,
        OCTREE,
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
    virtual BoundingBox getSampleArea() const = 0;
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
}

#endif