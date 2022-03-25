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
#include <glm/gtc/matrix_transform.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <fstream>

int main(int argc, char** argv)
{
    spdlog::set_pattern("[%^%l%$] [%s:%#] %v");

    args::ArgumentParser parser("SdfExporter export an sdf", "");
    args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
    args::Positional<std::string> modelPathArg(parser, "model_path", "The model path");
    args::Positional<std::string> outputPathArg(parser, "output_path", "Output path");
    args::ValueFlag<float> cellSizeArg(parser, "cell_size", "The voxel size of the voxelization", {'c', "cell_size"});
    args::ValueFlag<uint32_t> depthArg(parser, "depth", "The octree subdivision depth", {'d', "depth"});
    args::Flag normalizeBBArg(parser, "normalize_model", "Normalize the model coordinates", {'n', "normalize"});

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
    std::string outputPath = (outputPathArg) ? args::get(outputPathArg) : "../output/sdf.bin";

    Mesh mesh(modelPath);
    BoundingBox box = mesh.getBoudingBox();
    if(normalizeBBArg) {
        // Normalize model units
        const glm::vec3 boxSize = box.getSize();
        mesh.applyTransform(	glm::scale(glm::mat4(1.0), glm::vec3(2.0f/glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z))) *
                                glm::translate(glm::mat4(1.0), -box.getCenter()));
        box = mesh.getBoudingBox();
    }

    const glm::vec3 modelBBSize = box.getSize();
    box.addMargin(0.12f * glm::max(glm::max(modelBBSize.x, modelBBSize.y), modelBBSize.z));

    Timer timer;
    timer.start();
    UniformGridSdf uniformGridOctree = (cellSizeArg) ? 
                    UniformGridSdf(mesh, box, args::get(cellSizeArg), UniformGridSdf::InitAlgorithm::OCTREE) :
                    UniformGridSdf(mesh, box, (depthArg) ? args::get(depthArg) : 6, UniformGridSdf::InitAlgorithm::OCTREE);

    SPDLOG_INFO("Computation time {}s", timer.getElapsedSeconds());
    
    SPDLOG_INFO("Saving the model");
    std::ofstream os(outputPath, std::ios::out | std::ios::binary);
    if(!os.is_open())
    {
        SPDLOG_ERROR("Cannot open file {}", outputPath);
        return 1;
    }
    cereal::PortableBinaryOutputArchive archive(os);
    archive(uniformGridOctree);
}