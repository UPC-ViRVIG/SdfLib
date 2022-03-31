#include <iostream>
#include <random>
#include <algorithm>
#include <optional>
#include "sdf/UniformGridSdf.h"
#include "sdf/RealSdf.h"
#include "sdf/OctreeSdf.h"
#include "utils/Mesh.h"
#include <iostream>
#include <random>
#include <args.hxx>
#include "utils/TriangleUtils.h"
#include "utils/Timer.h"
#include <spdlog/spdlog.h>
#include <glm/gtc/matrix_transform.hpp>

int main(int argc, char** argv)
{
    spdlog::set_pattern("[%^%l%$] [%s:%#] %v");

    args::ArgumentParser parser("SdfExporter export an sdf", "");
    args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
    args::Positional<std::string> modelPathArg(parser, "model_path", "The model path");
    args::Positional<std::string> outputPathArg(parser, "output_path", "Output path");
    args::ValueFlag<float> cellSizeArg(parser, "cell_size", "The voxel size of the voxelization", {'c', "cell_size"});
    args::ValueFlag<uint32_t> depthArg(parser, "depth", "The octree subdivision depth", {'d', "depth"});
    args::ValueFlag<uint32_t> startDepthArg(parser, "start_depth", "The octree start depth", {"start_depth"});
	args::ValueFlag<std::string> terminationRuleArg(parser, "termination_rule", "Octree generation termination rule", {"termination_rule"});
	args::ValueFlag<float> terminationThresholdArg(parser, "termination_threshold", "Octree generation termination threshold", {"termination_threshold"});
	args::ValueFlag<std::string> sdfFormatArg(parser, "sdf_format", "It supports two formats: octree or grid", {"sdf_format"});
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

    std::string sdfFormat = (sdfFormatArg) ? args::get(sdfFormatArg) : "octree";
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
        std::optional<OctreeSdf::TerminationRule> terminationRule((terminationRuleArg) ? 
                    OctreeSdf::stringToTerminationRule(args::get(terminationRuleArg)) : 
                    std::optional<OctreeSdf::TerminationRule>(OctreeSdf::TerminationRule::TRAPEZOIDAL_RULE));
        if(!terminationRule.has_value())
        {
            std::cerr << args::get(terminationRuleArg) << " is not a valid termination rule";
            return 0;
        }
        timer.start();
        sdfFunc = std::unique_ptr<OctreeSdf>(new OctreeSdf(
            mesh, box, 
            (depthArg) ? args::get(depthArg) : 6,
            (startDepthArg) ? args::get(startDepthArg) : 1,
            (terminationThresholdArg) ? args::get(terminationThresholdArg) : 1e-3f,
            terminationRule.value()));
    }
    else
    {
        std::cerr << "The sdf_format can only be octree or grid";
    }

    SPDLOG_INFO("Computation time {}s", timer.getElapsedSeconds());
    
    SPDLOG_INFO("Saving the model");
    sdfFunc->saveToFile(outputPath);
}