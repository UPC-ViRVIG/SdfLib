#include <random>
#include <vector>
#include <args.hxx>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cstring>

#include "SdfLib/SdfFunction.h"
#include "SdfLib/OctreeSdf.h"
#include "SdfLib/ExactOctreeSdf.h"
#include "SdfLib/utils/Timer.h"
#include "SdfLib/utils/Mesh.h"

using namespace sdflib;

//#define TEST_ICG
//#define TEST_CGAL
#define TEST_OCTREE_SDF
#define TEST_EXACT_OCTREE_SDF
// #define TEST_OPENVDB

#ifdef TEST_ICG
#include <InteractiveComputerGraphics/TriangleMeshDistance.h>
#endif
#ifdef TEST_CGAL
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#endif
#ifdef TEST_OPENVDB
#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/Interpolation.h>
#endif

#ifdef TEST_ICG
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
#endif

#ifdef TEST_CGAL
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
#endif


// int main(int argc, char** argv)
// {
//     spdlog::set_pattern("[%^%l%$] [%s:%#] %v");

//     args::ArgumentParser parser("Calculate the error of a sdf", "");
//     args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
//     // args::Positional<std::string> sdfPathArg(parser, "sdf_path", "Sdf path");
//     // args::Positional<std::string> exactSdfPathArg(parser, "exact_sdf_path", "Exact sdf path");
//     args::Positional<std::string> modelPathArg(parser, "model_path", "Mesh model path");
//     args::Positional<uint32_t> millionsOfSamplesArg(parser, "num_samples_in_millions", "Number of samples to made in millions");
    
//     try
//     {
//         parser.ParseCLI(argc, argv);
//     }
//     catch(args::Help)
//     {
//         std::cerr << parser;
//         return 0;
//     }

//     Mesh mesh(args::get(modelPathArg));

    
//     float value = sampler.wsSample(openvdb::Vec3R(0.25, 1.4, -1.1));
//     std::cout << value << std::endl;
// }

#define MAKE_HISTOGRAM

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

#ifdef MAKE_HISTOGRAM
    std::array<float, 40> histMeanTimeExact;
	histMeanTimeExact.fill(0.0f);

    std::array<float, 40> histMeanTimeICG;
	histMeanTimeICG.fill(0.0f);

    std::array<float, 40> histMeanTimeCGAL;
	histMeanTimeCGAL.fill(0.0f);

    std::array<float, 40> histRSMEOctreeSdf;
    histRSMEOctreeSdf.fill(0.0f);
    std::array<uint32_t, 40> histNumSamplesOctreeSdf;
    histNumSamplesOctreeSdf.fill(0);

    std::array<float, 40> histRSMEOpenVdb;
    histRSMEOpenVdb.fill(0.0f);
    std::array<uint32_t, 40> histNumSamplesOpenVdb;
    histNumSamplesOpenVdb.fill(0);
	
#endif
    Mesh mesh(args::get(modelPathArg));

    // Normalize model units
    const glm::vec3 boxSize = mesh.getBoundingBox().getSize();
    mesh.applyTransform( glm::scale(glm::mat4(1.0), glm::vec3(2.0f/glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z))) *
                                    glm::translate(glm::mat4(1.0), -mesh.getBoundingBox().getCenter()));
    
    BoundingBox box = mesh.getBoundingBox();
    float invModelDiagonal = 1.0f / glm::length(box.getSize());
    // float invModelDiagonal = 1.0f;
#ifdef TEST_OCTREE_SDF
    std::unique_ptr<SdfFunction> sdf = SdfFunction::loadFromFile(args::get(sdfPathArg));
    box = sdf->getSampleArea();
#endif

#ifdef TEST_EXACT_OCTREE_SDF
    std::unique_ptr<SdfFunction> exactSdf = SdfFunction::loadFromFile(args::get(exactSdfPathArg));
    box = exactSdf->getSampleArea();
    exactSdf->getDistance(glm::vec3(0.0f, 0.0f, 0.0f));
#endif

    float invDiag = 1.0f / glm::length(box.max - box.min);

    Timer timer;

#ifdef TEST_OPENVDB
    openvdb::initialize();
    const uint32_t gridSize = 64;
    const float voxelSize = box.getSize().x / gridSize;
    float exteriorNarrowBand = gridSize;
    float interiorNarrowBand = gridSize;
#ifdef TEST_EXACT_OCTREE_SDF
    // Compute the smallest narrow bands possible
    exteriorNarrowBand = 0.0f;
    interiorNarrowBand = 0.0f;
    for(uint32_t k=0; k < 32; k++)
    {
        for(uint32_t j=0; j < 32; j++)
        {
            for(uint32_t i=0; i < 32; i++)
            {
                const glm::vec3 texCoords((static_cast<float>(i) + 0.5f) / 32.0f,
                                    (static_cast<float>(j) + 0.5f) / 32.0f,
                                    (static_cast<float>(k) + 0.5f) / 32.0f);
                const glm::vec3 point = box.min + texCoords * box.getSize();
                const float dist = exactSdf->getDistance(point);
                if(dist >= 0.0f)
                {
                    exteriorNarrowBand = glm::max(exteriorNarrowBand, dist);
                }
                else
                {
                    interiorNarrowBand = glm::max(interiorNarrowBand, -dist);
                }
            }
        }
    }
    // Add error margin and transform to voxel space
    exteriorNarrowBand = (exteriorNarrowBand + box.getSize().x / 32.0f) / voxelSize;
    interiorNarrowBand = (interiorNarrowBand + box.getSize().x / 32.0f) / voxelSize;
#endif

    openvdb::math::Transform::Ptr linearTransform = openvdb::math::Transform::createLinearTransform(voxelSize);
    glm::vec3 gridCenter = box.getCenter() + 0.5f * voxelSize;
    linearTransform->postTranslate(openvdb::Vec3R(static_cast<double>(gridCenter.x), static_cast<double>(gridCenter.y), static_cast<double>(gridCenter.z)));

    std::vector<openvdb::Vec4I> emptyMeshQuadIndices;
    std::vector<openvdb::Vec3I> meshTraingleIndices(mesh.getIndices().size()/3);
    std::memcpy(meshTraingleIndices.data(), mesh.getIndices().data(), mesh.getIndices().size() * sizeof(uint32_t));

    timer.start();    
    openvdb::FloatGrid::Ptr vdbGrid = openvdb::tools::meshToSignedDistanceField<openvdb::FloatGrid>(*linearTransform,
            *reinterpret_cast<std::vector<openvdb::Vec3s>*>(&mesh.getVertices()),
            meshTraingleIndices,
            emptyMeshQuadIndices,
            exteriorNarrowBand,
            interiorNarrowBand);
    
    SPDLOG_INFO("OpenVDB init time: {}", timer.getElapsedSeconds());
    vdbGrid->print();
    SPDLOG_INFO("OpenVDB mem usage: {}", vdbGrid->memUsage());

    openvdb::tools::GridSampler<openvdb::FloatGrid, openvdb::tools::BoxSampler> vdbSampler(*vdbGrid);
#endif

#ifdef TEST_ICG
	timer.start();
    ICG icg(mesh);
	SPDLOG_INFO("ICG init time: {}", timer.getElapsedSeconds());
#endif

#ifdef TEST_CGAL
	timer.start();
    CGALtree cgalTree(mesh);
	SPDLOG_INFO("CGAL init time: {}", timer.getElapsedSeconds());
#endif

	SPDLOG_INFO("Models Loaded");

    const uint32_t numSamples = 1000000 * ((millionsOfSamplesArg) ? args::get(millionsOfSamplesArg) : 1);
    // const uint32_t numSamples = 100000 * ((millionsOfSamplesArg) ? args::get(millionsOfSamplesArg) : 1);
    std::vector<glm::vec3> samples(numSamples);
    
    glm::vec3 center = box.getCenter();
    glm::vec3 size = box.getSize() - glm::vec3(1e-5);

    auto getRandomSample = [&] () -> glm::vec3
    {
        glm::vec3 p =  glm::vec3(static_cast<float>(rand())/static_cast<float>(RAND_MAX),
                            static_cast<float>(rand())/static_cast<float>(RAND_MAX),
                            static_cast<float>(rand())/static_cast<float>(RAND_MAX));
        return center + (p - 0.5f) * size;
    };

    std::generate(samples.begin(), samples.end(), getRandomSample);

#ifdef TEST_OCTREE_SDF
    std::vector<float> sdfDist(numSamples);
    timer.start();
    for(uint32_t s=0; s < numSamples; s++)
    {
        sdfDist[s] = sdf->getDistance(samples[s]);
    }
    float sdfTimePerSample = (timer.getElapsedSeconds() * 1.0e6f) / static_cast<float>(numSamples);
	SPDLOG_INFO("Sdf us per query: {}", sdfTimePerSample, timer.getElapsedSeconds());
#endif

#ifdef TEST_EXACT_OCTREE_SDF
    std::vector<float> exactSdfDist(numSamples);
    timer.start();
    for(uint32_t s=0; s < numSamples; s++)
    {
        exactSdfDist[s] = exactSdf->getDistance(samples[s]);
    }
    float exactSdfTimePerSample = (timer.getElapsedSeconds() * 1.0e6f) / static_cast<float>(numSamples);
	SPDLOG_INFO("Exact Sdf us per query: {}", exactSdfTimePerSample, timer.getElapsedSeconds());
    SPDLOG_INFO("Exact Sdf: {}s", timer.getElapsedSeconds());
#endif

#ifdef TEST_ICG
    std::vector<float> exactSdfDist1(numSamples);
    timer.start();
    for(uint32_t s=0; s < numSamples; s++)
    {
        exactSdfDist1[s] = icg.getDistance(samples[s]);
    }
    float exact1SdfTimePerSample = (timer.getElapsedSeconds() * 1.0e6f) / static_cast<float>(numSamples);
	SPDLOG_INFO("ICG us per query: {}", exact1SdfTimePerSample, timer.getElapsedSeconds());
    SPDLOG_INFO("ICG: {}s", timer.getElapsedSeconds());
#endif

#ifdef TEST_CGAL
    std::vector<float> exactSdfDist2(numSamples);
    timer.start();
    for(uint32_t s=0; s < numSamples; s++)
    {
        exactSdfDist2[s] = cgalTree.getDistance(samples[s]);
    }
    float exact2SdfTimePerSample = (timer.getElapsedSeconds() * 1.0e6f) / static_cast<float>(numSamples);
	SPDLOG_INFO("CGal us per query: {}", exact2SdfTimePerSample, timer.getElapsedSeconds());
    SPDLOG_INFO("CGal: {}s", timer.getElapsedSeconds());
#endif

#ifdef TEST_OPENVDB
    std::vector<float> sdfDist2(numSamples);
    timer.start();
    for(uint32_t s=0; s < numSamples; s++)
    {
        sdfDist2[s] = vdbSampler.wsSample(openvdb::Vec3R(static_cast<double>(samples[s].x), static_cast<double>(samples[s].y), static_cast<double>(samples[s].z)));
    }
    float exact3SdfTimePerSample = (timer.getElapsedSeconds() * 1.0e6f) / static_cast<float>(numSamples);
	SPDLOG_INFO("OpenVDB us per query: {}", exact3SdfTimePerSample, timer.getElapsedSeconds());
#endif

    // Calculate error
    auto pow2 = [](float a) { return a * a; };
    
    double sdfRMSE = 0.0;
    double icgRMSE = 0.0f;
    double cgalRMSE = 0.0f;
    double vdbRMSE = 0.0f;

    double sdfMAE = 0.0f;
    double icgMAE = 0.0f;
    double cgalMAE = 0.0f;
    double vdbMAE = 0.0f;

    float sdfMaxError = 0.0f;
    float icgMaxError = 0.0f;
    float cgalMaxError = 0.0f;
    float vdbMaxError = 0.0f;

    float maxError = 0.0f;

    std::array<std::vector<glm::vec3>, 40> samplesPerDist;
    samplesPerDist.fill(std::vector<glm::vec3>());

    std::array<std::vector<float>, 40> samplesExactDist;
    samplesExactDist.fill(std::vector<float>());

    for(uint32_t s=0; s < numSamples; s++)
    {
#ifdef TEST_OCTREE_SDF
        sdfRMSE += static_cast<double>(pow2((sdfDist[s] - exactSdfDist[s])));
        sdfMAE += static_cast<double>(glm::abs((sdfDist[s] - exactSdfDist[s])));
        sdfMaxError = glm::max(sdfMaxError, glm::abs((sdfDist[s] - exactSdfDist[s])));
        // sdfRMSE += static_cast<double>(pow2((sdfDist[s] - exactSdfDist1[s])));
        // sdfMAE += static_cast<double>(glm::abs((sdfDist[s] - exactSdfDist1[s])));
        // sdfMaxError = glm::max(sdfMaxError, glm::abs((sdfDist[s] - exactSdfDist1[s])));
#endif
#ifdef TEST_ICG
        // icgRMSE += static_cast<double>(pow2((exactSdfDist1[s] - exactSdfDist[s])));
        // icgMAE += static_cast<double>(glm::abs((exactSdfDist1[s] - exactSdfDist[s])));
        // icgMaxError = glm::max(icgMaxError, glm::abs((exactSdfDist1[s] - exactSdfDist[s])));
#endif
#ifdef TEST_CGAL
        cgalRMSE += static_cast<double>(pow2((exactSdfDist2[s] - exactSdfDist[s])));
        cgalMAE += static_cast<double>(glm::abs((exactSdfDist2[s] - exactSdfDist[s])));
        cgalMaxError = glm::max(cgalMaxError, glm::abs((exactSdfDist2[s] - exactSdfDist[s])));
#endif
#ifdef TEST_OPENVDB
        vdbRMSE += static_cast<double>(pow2((sdfDist2[s] - exactSdfDist[s])));
        vdbMAE += static_cast<double>(glm::abs((sdfDist2[s] - exactSdfDist[s])));
        vdbMaxError = glm::max(vdbMaxError, glm::abs((sdfDist2[s] - exactSdfDist[s])));
#endif
#ifdef TEST_ICG
#ifdef TEST_CGAL
        maxError = glm::max(maxError, glm::abs((exactSdfDist1[s]-exactSdfDist2[s])));
#endif
#endif
#ifdef MAKE_HISTOGRAM
        uint32_t idx = glm::min(static_cast<uint32_t>(glm::round((exactSdfDist[s] * invModelDiagonal + 1.0f) * 20.0f)), 39u);
        // uint32_t idx = static_cast<uint32_t>(glm::round((exactSdfDist[s] * invDiag + 1.0f) * 100.0f)) - 80;
        // samplesPerDist[idx].push_back(samples[s]);
        if(idx >= 0 && idx < 40)
        {
            samplesPerDist[idx].push_back(samples[s]);
            samplesExactDist[idx].push_back(exactSdfDist[s]);
        }
#endif
    }

#ifdef MAKE_HISTOGRAM
    for(uint32_t r=0; r < 40; r++)
    {
        std::vector<glm::vec3>& s = samplesPerDist[r];
        uint32_t len = s.size();

#ifdef TEST_EXACT_OCTREE_SDF
        timer.start();
        for(uint32_t i=0; i < len; i++)
        {
            exactSdf->getDistance(s[i]);
        }
        histMeanTimeExact[r] = timer.getElapsedSeconds() / static_cast<float>(len);
#endif

#ifdef TEST_ICG
        timer.start();
        for(uint32_t i=0; i < len; i++)
        {
            icg.getDistance(s[i]);
        }
        histMeanTimeICG[r] = timer.getElapsedSeconds() / static_cast<float>(len);
#endif

#ifdef TEST_CGAL
        timer.start();
        for(uint32_t i=0; i < len; i++)
        {
            cgalTree.getDistance(s[i]);
        }
        histMeanTimeCGAL[r] = timer.getElapsedSeconds() / static_cast<float>(len);
#endif

#ifdef TEST_OCTREE_SDF
        for(uint32_t i=0; i < len; i++)
        {
            const float dist = sdf->getDistance(s[i]);
            histRSMEOctreeSdf[r] += pow2(dist - samplesExactDist[r][i]);
            histNumSamplesOctreeSdf[r]++;
        }
#endif

#ifdef TEST_OPENVDB
        for(uint32_t i=0; i < len; i++)
        {
            const float dist = vdbSampler.wsSample(openvdb::Vec3R(static_cast<double>(s[i].x), static_cast<double>(s[i].y), static_cast<double>(s[i].z)));
            histRSMEOpenVdb[r] += pow2(dist - samplesExactDist[r][i]);
            histNumSamplesOpenVdb[r]++;
        }
#endif
    }

#ifdef TEST_EXACT_OCTREE_SDF
    for(int i=0; i < 40; i++)
    {
        SPDLOG_INFO("{}%: {}", 5*(i-20), histMeanTimeExact[i] * 1e6);
    }
#endif

#ifdef TEST_ICG
    for(int i=0; i < 40; i++)
    {
        SPDLOG_INFO("{}%: {}", 5*(i-20), histMeanTimeICG[i] * 1e6);
    }
#endif

#ifdef TEST_OCTREE_SDF
    for(int i=0; i < 40; i++)
    {
        SPDLOG_INFO("{}%: {}", 5*(i-20), glm::sqrt(histRSMEOctreeSdf[i] / static_cast<float>(histNumSamplesOctreeSdf[i])) * invModelDiagonal);
    }
#endif

#ifdef TEST_OPENVDB
    for(int i=0; i < 40; i++)
    {
        SPDLOG_INFO("{}%: {}", 5*(i-20), glm::sqrt(histRSMEOpenVdb[i] / static_cast<float>(histNumSamplesOpenVdb[i])) * invModelDiagonal);
    }
#endif


#ifdef TEST_CGAL
    for(int i=0; i < 40; i++)
    {
        SPDLOG_INFO("{}%: {}", 5*(i-20), histMeanTimeCGAL[i] * 1e6);
    }
#endif

    // for(int i=0; i < 40; i++)
    // {
    //     SPDLOG_INFO("{}%: {}", (i-20), histMeanTimeExact[i] * 1e6);
    // }

    // for(int i=0; i < 40; i++)
    // {
    //     SPDLOG_INFO("{}%: {}", (i-20), histMeanTimeICG[i] * 1e6);
    // }

    // for(int i=0; i < 40; i++)
    // {
    //     SPDLOG_INFO("{}%: {}", (i-20), histMeanTimeCGAL[i] * 1e6);
    // }
#endif

#ifdef TEST_OCTREE_SDF
    SPDLOG_INFO("Octree Sdf RMSE: {}", glm::sqrt(sdfRMSE / static_cast<double>(numSamples)) * invModelDiagonal);
    SPDLOG_INFO("Octree Sdf MAE: {}", (sdfMAE / static_cast<double>(numSamples)) * invModelDiagonal);
    SPDLOG_INFO("Octree Sdf max error: {}", sdfMaxError * invModelDiagonal);
#endif
#ifdef TEST_ICG
    SPDLOG_INFO("ICG RMSE: {}", glm::sqrt(icgRMSE / static_cast<double>(numSamples)) * invModelDiagonal);
    SPDLOG_INFO("ICG MAE: {}", (icgMAE / static_cast<double>(numSamples)) * invModelDiagonal);
    SPDLOG_INFO("ICG max error: {}", icgMaxError * invModelDiagonal);
#endif
#ifdef TEST_CGAL
    SPDLOG_INFO("CGAL RMSE: {}", glm::sqrt(cgalRMSE / static_cast<double>(numSamples)) * invModelDiagonal);
    SPDLOG_INFO("CGAL MAE: {}", (cgalMAE / static_cast<double>(numSamples)) * invModelDiagonal);
    SPDLOG_INFO("CGAL max error: {}", cgalMaxError * invModelDiagonal);
#endif
#ifdef TEST_OPENVDB
    SPDLOG_INFO("VDB RMSE: {}", glm::sqrt(vdbRMSE / static_cast<double>(numSamples)) * invModelDiagonal);
    SPDLOG_INFO("VDB MAE: {}", (vdbMAE / static_cast<double>(numSamples)) * invModelDiagonal);
    SPDLOG_INFO("VDB max error: {}", vdbMaxError * invModelDiagonal);
#endif

#ifdef TEST_ICG
#ifdef TEST_CGAL
    SPDLOG_INFO("Methods Max error: {}", maxError);
#endif
#endif
}