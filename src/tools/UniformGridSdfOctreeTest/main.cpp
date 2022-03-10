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

int main(int argc, char** argv)
{
    spdlog::set_pattern("[%^%l%$] [%s:%#] %v");

    args::ArgumentParser parser("UniformGridSdfOctreeTest test the result with the basic one", "");
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
    float cellSize = (cellSizeArg) ? args::get(cellSizeArg) : 0.05f;

    Mesh meshSphere(modelPath);
    
    BoundingBox box = meshSphere.getBoudingBox();
    SPDLOG_INFO("BB min: {}, {}, {}", box.min.x, box.min.y, box.min.z);
    SPDLOG_INFO("BB max: {}, {}, {}", box.max.x, box.max.y, box.max.z);

    box.addMargin(1.0f);

    Timer timer;
    timer.start();
    UniformGridSdf uniformGridBasic(meshSphere, box, cellSize, UniformGridSdf::InitAlgorithm::BASIC);
    
    SPDLOG_INFO("Basic algorithm time: {}s", timer.getElapsedSeconds());

    timer.start();
    UniformGridSdf uniformGridOctree(meshSphere, box, cellSize, UniformGridSdf::InitAlgorithm::OCTREE);
    SPDLOG_INFO("Octree algorithm time {}s", timer.getElapsedSeconds());

    const std::vector<float>& grid1 = uniformGridBasic.getGrid();
    const std::vector<float>& grid2 = uniformGridOctree.getGrid();

    for(uint32_t i=0; i < grid1.size(); i++)
    {
        assert(glm::abs(grid1[i] - grid2[i]) < 0.001f);
    }
}