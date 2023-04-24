#include <iostream>
#include <random>
#include <algorithm>
#include "sdf/UniformGridSdf.h"
#include "sdf/RealSdf.h"
#include "utils/Mesh.h"
#include <iostream>
#include <random>
#include <args.hxx>
#include "utils/TriangleUtils.h"
#include "utils/Timer.h"
#include <spdlog/spdlog.h>

using namespace sdflib;

int main(int argc, char** argv)
{
    #ifdef SDFLIB_PRINT_STATISTICS
        spdlog::set_pattern("[%^%l%$] [%s:%#] %v");
    #else
        spdlog::set_pattern("[%^%l%$] %v");
    #endif

    args::ArgumentParser parser("UniformGridSdfTest prints the distance difference using the method", "");
    args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
    args::Positional<std::string> modelPathArg(parser, "model_path", "The model path");
    args::Positional<float> cellSizeArg(parser, "cell_size", "The voxel size of the voxelization");

    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch(args::Help)
    {
        std::cerr << parser;
        return 0;
    }

    std::string modelPath = (modelPathArg) ? args::get(modelPathArg) : "../models/sphere.glb";
    float cellSize = (cellSizeArg) ? args::get(cellSizeArg) : 0.1f;

    Mesh meshSphere(modelPath);
    
    BoundingBox box = meshSphere.getBoundingBox();
    SPDLOG_INFO("BB min: {}, {}, {}", box.min.x, box.min.y, box.min.z);
    SPDLOG_INFO("BB max: {}, {}, {}", box.max.x, box.max.y, box.max.z);

    box.addMargin(1.0f);

    Timer timer;
    timer.start();
    UniformGridSdf grid(meshSphere, box, cellSize);
    RealSdf sdf(meshSphere);
    
    SPDLOG_INFO("Generation time: {}s", timer.getElapsedSeconds());

    auto getRandomVec3 = [] () -> glm::vec3
	{
		return glm::vec3(static_cast<float>(rand())/static_cast<float>(RAND_MAX),
                         static_cast<float>(rand())/static_cast<float>(RAND_MAX),
                         static_cast<float>(rand())/static_cast<float>(RAND_MAX));
	};

    constexpr size_t numSamples = 10000;
    float maxDistanceDiff = 0.0f;
    float meanDistanceDiff = 0.0f;
    for(size_t i = 0; i < numSamples; i++)
    {
        glm::vec3 samplePos = getRandomVec3();
        const float distDiff = glm::abs(grid.getDistance(samplePos) - sdf.getDistance(samplePos));
        maxDistanceDiff = glm::max(maxDistanceDiff, distDiff);
        meanDistanceDiff += distDiff;
    }
    meanDistanceDiff /= static_cast<float>(numSamples);

    std::cout << "Mean difference: " << meanDistanceDiff << std::endl;
    std::cout << "Max difference: " << maxDistanceDiff << std::endl;
}