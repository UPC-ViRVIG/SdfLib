#ifndef SDF_FUNCTION_H
#define SDF_FUNCTION_H

#include <glm/glm.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <fstream>

class SdfFunction
{
public:
    enum SdfFormat
    {
        GRID,
        OCTREE,
        NONE
    };
    virtual float getDistance(glm::vec3 sample) const = 0;
    virtual SdfFormat getFormat() const { return SdfFormat::NONE; }
    
    bool saveToFile(const std::string& outputPath);

    static std::unique_ptr<SdfFunction> loadFromFile(const std::string& inputPath);
};

#endif