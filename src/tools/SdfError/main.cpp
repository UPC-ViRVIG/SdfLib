#include <random>
#include <vector>
#include <args.hxx>
#include <spdlog/spdlog.h>
#include <algorithm>

#include "SdfLib/SdfFunction.h"
#include "SdfLib/utils/Timer.h"

using namespace sdflib;

int main(int argc, char** argv)
{
    #ifdef SDFLIB_PRINT_STATISTICS
        spdlog::set_pattern("[%^%l%$] [%s:%#] %v");
    #else
        spdlog::set_pattern("[%^%l%$] %v");
    #endif

    args::ArgumentParser parser("Calculate the error of a sdf", "");
    args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
    args::Positional<std::string> sdfPathArg(parser, "sdf_path", "Sdf path");
    args::Positional<std::string> exactSdfPathArg(parser, "exact_sdf_path", "Exact sdf path");
    args::Positional<uint32_t> millionsOfSamplesArg(parser, "num_samples_in_millions", "Number of samples to made in millions");
    
    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch(args::Help)
    {
        std::cerr << parser;
        return 0;
    }

    std::unique_ptr<SdfFunction> sdf = SdfFunction::loadFromFile(args::get(sdfPathArg));
    std::unique_ptr<SdfFunction> exactSdf = SdfFunction::loadFromFile(args::get(exactSdfPathArg));

    // std::unique_ptr<SdfFunction> sdf = SdfFunction::loadFromFile("../output/OctreeDepth7Bunny.bin");
    // std::unique_ptr<SdfFunction> exactSdf = SdfFunction::loadFromFile("../output/exactOctreeBunny.bin");

	SPDLOG_INFO("Models Loaded");

    const uint32_t numSamples = 1000000 * ((millionsOfSamplesArg) ? args::get(millionsOfSamplesArg) : 1);
    std::vector<glm::vec3> samples(numSamples);
    
    glm::vec3 center = sdf->getSampleArea().getCenter();
    glm::vec3 size = sdf->getSampleArea().getSize() - glm::vec3(1e-5);
    auto getRandomSample = [&] () -> glm::vec3
    {
        glm::vec3 p =  glm::vec3(static_cast<float>(rand())/static_cast<float>(RAND_MAX),
                            static_cast<float>(rand())/static_cast<float>(RAND_MAX),
                            static_cast<float>(rand())/static_cast<float>(RAND_MAX));
        return center + (p - 0.5f) * size;
    };

    std::generate(samples.begin(), samples.end(), getRandomSample);

    std::vector<float> sdfDist(numSamples);
    Timer timer; timer.start();
    for(uint32_t s=0; s < numSamples; s++)
    {
        sdfDist[s] = sdf->getDistance(samples[s]);
    }
    float sdfTimePerSample = (timer.getElapsedSeconds() * 1.0e6f) / static_cast<float>(numSamples);
	SPDLOG_INFO("Sdf us per query: {}", sdfTimePerSample, timer.getElapsedSeconds());

    std::vector<float> exactSdfDist(numSamples);
    timer.start();
    for(uint32_t s=0; s < numSamples; s++)
    {
        exactSdfDist[s] = exactSdf->getDistance(samples[s]);
    }
    float exactSdfTimePerSample = (timer.getElapsedSeconds() * 1.0e6f) / static_cast<float>(numSamples);
	SPDLOG_INFO("Exact Sdf us per query: {}", exactSdfTimePerSample, timer.getElapsedSeconds());

    // Calculate error

    auto pow2 = [](float a) { return a * a; };
    
    double rmseError = 0.0;
    double maeError = 0.0;
    float maxError = 0.0;
    for(uint32_t s=0; s < numSamples; s++)
    {
        rmseError += static_cast<double>(pow2(sdfDist[s] - exactSdfDist[s]));
        maeError += static_cast<double>(glm::abs(sdfDist[s] - exactSdfDist[s]));
        maxError = glm::max(maxError, glm::abs(sdfDist[s] - exactSdfDist[s]));
    }
    rmseError = glm::sqrt(rmseError / static_cast<double>(numSamples));
    maeError = maeError / static_cast<double>(numSamples);

    SPDLOG_INFO("RMSE: {}", rmseError);
    SPDLOG_INFO("MAE: {}", maeError);
    SPDLOG_INFO("Max error: {}", maxError);
}