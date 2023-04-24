#include <spdlog/spdlog.h>
#include <args.hxx>
#include "sdf/UniformGridSdf.h"
#include "sdf/OctreeSdf.h"
#include "utils/Timer.h"

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
    args::Positional<uint32_t> startDepthArg(parser, "start_depth", "The resulting octree start depth");

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
    uint32_t depth = (depthArg) ? args::get(depthArg) : 5;
    uint32_t startDepth = (startDepthArg) ? args::get(startDepthArg) : 1;

    Mesh meshSphere(modelPath);

    BoundingBox box = meshSphere.getBoundingBox();
    const glm::vec3 modelBBSize = box.getSize();
    box.addMargin(0.12f * glm::max(glm::max(modelBBSize.x, modelBBSize.y), modelBBSize.z));

    Timer timer;
    timer.start();
    UniformGridSdf uniformSdf(meshSphere, box, depth);

    SPDLOG_INFO("Uniform Grid algorithm time {}s", timer.getElapsedSeconds());

    timer.start();
    OctreeSdf octreeSdf(meshSphere, box, depth, startDepth);

    SPDLOG_INFO("Octree algorithm time {}s", timer.getElapsedSeconds());
    SPDLOG_INFO("Octree size: {} nodes", octreeSdf.getOctreeData().size());

	BoundingBox modelBox = meshSphere.getBoundingBox();
    auto getRandomVec3 = [&] () -> glm::vec3
    {
        glm::vec3 p =  glm::vec3(static_cast<float>(rand())/static_cast<float>(RAND_MAX),
                            static_cast<float>(rand())/static_cast<float>(RAND_MAX),
                            static_cast<float>(rand())/static_cast<float>(RAND_MAX));
        return modelBox.min + p * modelBox.getSize();
    };

    for(uint32_t sample=0; sample < 1000; sample++)
    {
        glm::vec3 point = getRandomVec3();
        float dist1 = uniformSdf.getDistance(point);
        float dist2 = octreeSdf.getDistance(point);
        if(glm::abs(dist1 - dist2) > 0.001f)
        {
			float dist11 = uniformSdf.getDistance(point);
			float dist21 = octreeSdf.getDistance(point);
            SPDLOG_INFO("Sdf in center {} and {}", dist1, dist2);
            assert(false);
        }
    }
}