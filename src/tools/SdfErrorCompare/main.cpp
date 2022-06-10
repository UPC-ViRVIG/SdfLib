#include <random>
#include <vector>
#include <args.hxx>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "sdf/SdfFunction.h"
#include "utils/Timer.h"
#include "utils/Mesh.h"

#define TEST_METHODS 
#ifdef TEST_METHODS
#include <InteractiveComputerGraphics/TriangleMeshDistance.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#endif

class ICG
{
public:
    ICG(Mesh& mesh)
    : mesh_distance(toDoubleVector(mesh.getVertices()),
                    *reinterpret_cast<std::vector<std::array<int, 3>>*>(&mesh.getIndices()))
    {}

    inline float getDistance(glm::vec3 samplePoint)
    {
        tmd::Result result = mesh_distance.signed_distance({ samplePoint.x, samplePoint.y, samplePoint.z });
        return result.distance;
    }
private:
    tmd::TriangleMeshDistance mesh_distance;

    std::vector<std::array<double, 3>> toDoubleVector(const std::vector<glm::vec3>& vec)
    {
        std::vector<std::array<double, 3>> res(vec.size());
        for(uint32_t i=0; i < vec.size(); i++)
        {
            res[i] = { static_cast<double>(vec[i].x), static_cast<double>(vec[i].y), static_cast<double>(vec[i].z) };
        }

        return res;
    }
};

class CGALtree
{
public:
    typedef CGAL::Simple_cartesian<float> K;
    typedef K::FT FT;
    typedef K::Ray_3 Ray;
    typedef K::Line_3 Line;
    typedef K::Point_3 Point;
    typedef K::Triangle_3 Triangle;
    typedef std::vector<Triangle>::iterator Iterator;
    typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
    typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
    typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

    CGALtree(Mesh& mesh)
    {
        const std::vector<uint32_t>& indices = mesh.getIndices();
        const std::vector<glm::vec3>& vertices = mesh.getVertices();
        triangles.resize(indices.size()/3);
        for(uint32_t i=0; i < indices.size(); i += 3)
        {
            const uint32_t tIndex = i/3;
            triangles[tIndex] = Triangle(Point(vertices[indices[i]].x, vertices[indices[i]].y, vertices[indices[i]].z),
                                         Point(vertices[indices[i+1]].x, vertices[indices[i+1]].y, vertices[indices[i+1]].z),
                                         Point(vertices[indices[i+2]].x, vertices[indices[i+2]].y, vertices[indices[i+2]].z));
        }

        tree = Tree(triangles.begin(), triangles.end());
    }

    inline float getDistance(glm::vec3 samplePoint)
    {
        return glm::sqrt(tree.squared_distance(Point(samplePoint.x, samplePoint.y, samplePoint.z)));
    }
private:
    Tree tree;
	std::vector<Triangle> triangles;
};

int main(int argc, char** argv)
{
    spdlog::set_pattern("[%^%l%$] [%s:%#] %v");

    args::ArgumentParser parser("Calculate the error of a sdf", "");
    args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
    args::Positional<std::string> sdfPathArg(parser, "sdf_path", "Sdf path");
    args::Positional<std::string> exactSdfPathArg(parser, "exact_sdf_path", "Exact sdf path");
    args::Positional<std::string> modelPathArg(parser, "model_path", "Mesh model path");
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

    Mesh mesh(args::get(modelPathArg));
    // Mesh mesh("../models/bunny.ply");
    // Normalize model units
    const glm::vec3 boxSize = mesh.getBoudingBox().getSize();
    mesh.applyTransform( glm::scale(glm::mat4(1.0), glm::vec3(2.0f/glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z))) *
                                    glm::translate(glm::mat4(1.0), -mesh.getBoudingBox().getCenter()));
    std::unique_ptr<SdfFunction> sdf = SdfFunction::loadFromFile(args::get(sdfPathArg));
    std::unique_ptr<SdfFunction> exactSdf = SdfFunction::loadFromFile(args::get(exactSdfPathArg));
	
	Timer timer; timer.start();
    ICG icg(mesh);
	SPDLOG_INFO("ICG init time: {}", timer.getElapsedSeconds());

	timer.start();
    CGALtree cgalTree(mesh);
	SPDLOG_INFO("CGAL init time: {}", timer.getElapsedSeconds());

    // std::unique_ptr<SdfFunction> sdf = SdfFunction::loadFromFile("../output/OctreeDepth7Bunny.bin");
    // std::unique_ptr<SdfFunction> exactSdf = SdfFunction::loadFromFile("../output/exactOctreeBunny.bin");

	SPDLOG_INFO("Models Loaded");

    //const uint32_t numSamples = 1000000 * ((millionsOfSamplesArg) ? args::get(millionsOfSamplesArg) : 1);
    const uint32_t numSamples = 100000 * ((millionsOfSamplesArg) ? args::get(millionsOfSamplesArg) : 1);
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
    timer.start();
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

    std::vector<float> exactSdfDist1(numSamples);
    timer.start();
    for(uint32_t s=0; s < numSamples; s++)
    {
        exactSdfDist1[s] = icg.getDistance(samples[s]);
    }
    float exact1SdfTimePerSample = (timer.getElapsedSeconds() * 1.0e6f) / static_cast<float>(numSamples);
	SPDLOG_INFO("ICG us per query: {}", exact1SdfTimePerSample, timer.getElapsedSeconds());

    std::vector<float> exactSdfDist2(numSamples);
    timer.start();
    for(uint32_t s=0; s < numSamples; s++)
    {
        exactSdfDist2[s] = cgalTree.getDistance(samples[s]);
    }
    float exact2SdfTimePerSample = (timer.getElapsedSeconds() * 1.0e6f) / static_cast<float>(numSamples);
	SPDLOG_INFO("CGal us per query: {}", exact2SdfTimePerSample, timer.getElapsedSeconds());

    // Calculate error

    auto pow2 = [](float a) { return a * a; };
    
    double accError = 0.0;
    double method1Error = 0.0f;
    double method2Error = 0.0f;
    for(uint32_t s=0; s < numSamples; s++)
    {
        accError += static_cast<double>(pow2(sdfDist[s] - exactSdfDist[s]));
        method1Error += static_cast<double>(pow2(exactSdfDist1[s] - exactSdfDist[s]));
        method2Error += static_cast<double>(pow2(glm::abs(exactSdfDist2[s]) - glm::abs(exactSdfDist[s])));
    }
    accError = glm::sqrt(accError / static_cast<double>(numSamples));

    SPDLOG_INFO("Method1 RMSE: {}", method1Error);
    SPDLOG_INFO("Method2 RMSE: {}", method2Error);
    SPDLOG_INFO("RMSE: {}", accError);
}