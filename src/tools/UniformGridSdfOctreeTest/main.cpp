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

    args::ArgumentParser parser("UniformGridSdfOctreeTest test the result with the basic one", "");
    args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
    args::Positional<std::string> modelPathArg(parser, "model_path", "The model path");
    args::Positional<uint32_t> depthArg(parser, "depth", "The octree depth");

    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch(args::Help)
    {
        std::cerr << parser;
        return 0;
    }

    std::string modelPath = (modelPathArg) ? args::get(modelPathArg) : "../models/frog.ply";
    uint32_t depth = (depthArg) ? args::get(depthArg) : 5;

    Mesh meshSphere(modelPath);
    
    BoundingBox box = meshSphere.getBoundingBox();
    const glm::vec3 modelBBSize = box.getSize();
    box.addMargin(0.12f * glm::max(glm::max(modelBBSize.x, modelBBSize.y), modelBBSize.z));

    Timer timer;
    timer.start();
    UniformGridSdf uniformGridBasic(meshSphere, box, depth, UniformGridSdf::InitAlgorithm::BASIC);
    
    SPDLOG_INFO("Basic algorithm time: {}s", timer.getElapsedSeconds());

    timer.start();
    UniformGridSdf uniformGridOctree(meshSphere, box, depth, UniformGridSdf::InitAlgorithm::OCTREE);
    SPDLOG_INFO("Octree algorithm time {}s", timer.getElapsedSeconds());

    const std::vector<float>& grid1 = uniformGridBasic.getGrid();
    const std::vector<float>& grid2 = uniformGridOctree.getGrid();

    for(uint32_t i=0; i < grid1.size(); i++)
    {
        assert(glm::abs(grid1[i] - grid2[i]) < 0.001f);
    }
}