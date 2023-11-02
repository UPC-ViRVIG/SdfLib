#include <iostream>
#include <random>
#include <algorithm>
#include <optional>
#include "SdfLib/UniformGridSdf.h"
#include "SdfLib/RealSdf.h"
#include "SdfLib/OctreeSdf.h"
#include "SdfLib/ExactOctreeSdf.h"
#include "SdfLib/utils/Mesh.h"
#include <iostream>
#include <random>
#include <args.hxx>
#include "SdfLib/utils/TriangleUtils.h"
#include "SdfLib/utils/Timer.h"
#include <spdlog/spdlog.h>
#include <glm/gtc/matrix_transform.hpp>

using namespace sdflib;

int main(int argc, char** argv)
{
    #ifdef SDFLIB_PRINT_STATISTICS
        spdlog::set_pattern("[%^%l%$] [%s:%#] %v");
    #else
        spdlog::set_pattern("[%^%l%$] %v");
    #endif

    args::ArgumentParser parser("SdfExporter export an sdf", "");
    args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
    args::Positional<std::string> modelPathArg(parser, "model_path", "The model path");
    args::Positional<std::string> outputPathArg(parser, "output_path", "Output path");
    args::ValueFlag<float> cellSizeArg(parser, "cell_size", "The voxel size of the voxelization", {'c', "cell_size"});
    args::ValueFlag<uint32_t> depthArg(parser, "depth", "The octree subdivision depth", {'d', "depth"});
    args::ValueFlag<uint32_t> startDepthArg(parser, "start_depth", "The octree start depth", {"start_depth"});
	args::ValueFlag<float> terminationThresholdArg(parser, "termination_threshold", "Octree generation termination threshold", {"termination_threshold"});
	args::ValueFlag<std::string> sdfFormatArg(parser, "sdf_format", "It supports two formats: octree, grid, exact_octree", {"sdf_format"});
    args::ValueFlag<std::string> octreeAlgorithmArg(parser, "algorithm", "Select the algoirthm to generate the octree. It supports: uniform, no_continuity, continuity", {"algorithm"});
    args::ValueFlag<uint32_t> minTrianglesPerNode(parser, "min_triangles_per_node", "The minimum acceptable number of triangles per leaf in the octree", {"min_triangles_per_node"});
    args::Flag normalizeBBArg(parser, "normalize_model", "Normalize the model coordinates", {'n', "normalize"});
    args::ValueFlag<uint32_t> numThreadsArg(parser, "num_threads", "Set the application maximum number of threads", {"num_threads"});
    args::ValueFlag<float> bbMarginArg(parser, "bb_margin", "Percentage of margin added between the structure BB and the model BB", {"bb_margin"});

    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch (const args::Completion& e)
    {
        std::cout << e.what();
        return 0;
    }
    catch(const args::Help&)
    {
        std::cerr << parser;
        return 0;
    }catch (const args::ParseError& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }

    if(!modelPathArg)
    {
        std::cerr << "Error: No model_path specified" << std::endl;
        std::cerr << parser;
        return 1;
    }

    std::string sdfFormat = (sdfFormatArg) ? args::get(sdfFormatArg) : "octree";
    std::string modelPath = (modelPathArg) ? args::get(modelPathArg) : "../models/bunny.ply";
    std::string outputPath = (outputPathArg) ? args::get(outputPathArg) : "../output/sdfOctreeBunny.bin";

    Mesh mesh(modelPath);
    BoundingBox box = mesh.getBoundingBox();
    if(normalizeBBArg) {
        // Normalize model units
        const glm::vec3 boxSize = box.getSize();
        const float maxSize = glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z);
        mesh.applyTransform(	glm::scale(glm::mat4(1.0), glm::vec3(2.0f/maxSize)) *
                                glm::translate(glm::mat4(1.0), -box.getCenter()));
        box = mesh.getBoundingBox();
    }

    const glm::vec3 modelBBSize = box.getSize();
    // box.addMargin(0.12f * glm::max(glm::max(modelBBSize.x, modelBBSize.y), modelBBSize.z));
    const float margin = ((bbMarginArg) ? args::get(bbMarginArg) : 20.0f) / 100.0f;
    box.addMargin(margin * glm::max(glm::max(modelBBSize.x, modelBBSize.y), modelBBSize.z));
    // box.addMargin(0.8f * glm::max(glm::max(modelBBSize.x, modelBBSize.y), modelBBSize.z));

    Timer timer;
    std::unique_ptr<SdfFunction> sdfFunc;

    if(sdfFormat == "grid")
    {
        timer.start();
        sdfFunc = std::unique_ptr<UniformGridSdf>((cellSizeArg) ? 
                    new UniformGridSdf(mesh, box, args::get(cellSizeArg), UniformGridSdf::InitAlgorithm::OCTREE) :
                    new UniformGridSdf(mesh, box, (depthArg) ? args::get(depthArg) : 6, UniformGridSdf::InitAlgorithm::OCTREE));
        
    }
    else if(sdfFormat == "octree")
    {
        std::string initAlgorithmStr = (octreeAlgorithmArg) ? args::get(octreeAlgorithmArg) : "continuity";
        OctreeSdf::InitAlgorithm initAlgorithm;
        if(initAlgorithmStr == "uniform") initAlgorithm = OctreeSdf::InitAlgorithm::UNIFORM;
        else if(initAlgorithmStr == "no_continuity") initAlgorithm = OctreeSdf::InitAlgorithm::NO_CONTINUITY;
        else if(initAlgorithmStr == "continuity") initAlgorithm = OctreeSdf::InitAlgorithm::CONTINUITY;
        else
        {
            std::cerr << initAlgorithmStr << " is not a valid supported octree generation algorithm" << std::endl;
            return 0;
        }

        timer.start();
        sdfFunc = std::unique_ptr<OctreeSdf>(new OctreeSdf(
            mesh, box, 
            (depthArg) ? args::get(depthArg) : 8,
            (startDepthArg) ? args::get(startDepthArg) : 1,
            (terminationThresholdArg) ? args::get(terminationThresholdArg) : 1e-3f,
            initAlgorithm,
            (numThreadsArg) ? args::get(numThreadsArg) : 1
        ));
    }
    else if(sdfFormat == "exact_octree")
    {
        timer.start();
        sdfFunc = std::unique_ptr<ExactOctreeSdf>(new ExactOctreeSdf(
            mesh, box,
            (depthArg) ? args::get(depthArg) : 5,
            (startDepthArg) ? args::get(startDepthArg) : 1,
            (minTrianglesPerNode) ? args::get(minTrianglesPerNode) : 32,
            (numThreadsArg) ? args::get(numThreadsArg) : 1
        ));
    }
    else
    {
        std::cerr << "The sdf_format can only be octree or grid";
    }

    SPDLOG_INFO("Computation time {}s", timer.getElapsedSeconds());
    
    SPDLOG_INFO("Saving the model");
    sdfFunc->saveToFile(outputPath);
}