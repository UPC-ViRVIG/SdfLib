#include <random>
#include <vector>
#include <args.hxx>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "sdf/SdfFunction.h"
#include "sdf/ExactOctreeSdf.h"
#include "utils/Timer.h"
#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#define TEST_METHODS 
#ifdef TEST_METHODS
#include <InteractiveComputerGraphics/TriangleMeshDistance.h>
// #include <CGAL/Simple_cartesian.h>
// #include <CGAL/AABB_tree.h>
// #include <CGAL/AABB_traits.h>
// #include <CGAL/AABB_triangle_primitive.h>
// #include <openvdb/openvdb.h>
// #include <openvdb/tools/MeshToVolume.h>
// #include <openvdb/tools/Interpolation.h>
#endif

using namespace sdflib;

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

// class CGALtree
// {
// public:
//     typedef CGAL::Simple_cartesian<float> K;
//     typedef K::FT FT;
//     typedef K::Ray_3 Ray;
//     typedef K::Line_3 Line;
//     typedef K::Point_3 Point;
//     typedef K::Triangle_3 Triangle;
//     typedef std::vector<Triangle>::iterator Iterator;
//     typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
//     typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
//     typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

//     CGALtree(Mesh& mesh)
//     {
//         const std::vector<uint32_t>& indices = mesh.getIndices();
//         const std::vector<glm::vec3>& vertices = mesh.getVertices();
//         triangles.resize(indices.size()/3);
//         for(uint32_t i=0; i < indices.size(); i += 3)
//         {
//             const uint32_t tIndex = i/3;
//             triangles[tIndex] = Triangle(Point(vertices[indices[i]].x, vertices[indices[i]].y, vertices[indices[i]].z),
//                                          Point(vertices[indices[i+1]].x, vertices[indices[i+1]].y, vertices[indices[i+1]].z),
//                                          Point(vertices[indices[i+2]].x, vertices[indices[i+2]].y, vertices[indices[i+2]].z));
//         }

//         tree = Tree(triangles.begin(), triangles.end());
//     }

//     inline float getDistance(glm::vec3 samplePoint)
//     {
//         return glm::sqrt(tree.squared_distance(Point(samplePoint.x, samplePoint.y, samplePoint.z)));
//     }
// private:
//     Tree tree;
// 	std::vector<Triangle> triangles;
// };

int main(int argc, char** argv)
{
    #ifdef SDFLIB_PRINT_STATISTICS
        spdlog::set_pattern("[%^%l%$] [%s:%#] %v");
    #else
        spdlog::set_pattern("[%^%l%$] %v");
    #endif

    args::ArgumentParser parser("Calculate the error of a sdf", "");
    args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
    args::Positional<std::string> exactSdfPathArg(parser, "exact_sdf_path", "Exact sdf path");
    args::Positional<std::string> modelPathArg(parser, "model_path", "Mesh model path");
    args::Positional<uint32_t> imageWidthArg(parser, "image_width", "Image width");
    
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

    Timer timer;

    // Normalize model units
    const glm::vec3 boxSize = mesh.getBoundingBox().getSize();
    mesh.applyTransform( glm::scale(glm::mat4(1.0), glm::vec3(2.0f/glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z))) *
                                    glm::translate(glm::mat4(1.0), -mesh.getBoundingBox().getCenter()));

    std::unique_ptr<SdfFunction> exactSdf = SdfFunction::loadFromFile(args::get(exactSdfPathArg));
    BoundingBox box = exactSdf->getSampleArea();

    // openvdb::initialize();
    // const uint32_t gridSize = 64;
    // const float voxelSize = box.getSize().x / gridSize;
    // float invModelDiagonal = 1.0f / voxelSize;
    // float exteriorNarrowBand = gridSize;
    // float interiorNarrowBand = gridSize;

    // // Compute the smallest narrow bands possible
    // exteriorNarrowBand = 0.0f;
    // interiorNarrowBand = 0.0f;
    // for(uint32_t k=0; k < 32; k++)
    // {
    //     for(uint32_t j=0; j < 32; j++)
    //     {
    //         for(uint32_t i=0; i < 32; i++)
    //         {
    //             const glm::vec3 texCoords((static_cast<float>(i) + 0.5f) / 32.0f,
    //                                 (static_cast<float>(j) + 0.5f) / 32.0f,
    //                                 (static_cast<float>(k) + 0.5f) / 32.0f);
    //             const glm::vec3 point = box.min + texCoords * box.getSize();
    //             const float dist = exactSdf->getDistance(point);
    //             if(dist >= 0.0f)
    //             {
    //                 exteriorNarrowBand = glm::max(exteriorNarrowBand, dist);
    //             }
    //             else
    //             {
    //                 interiorNarrowBand = glm::max(interiorNarrowBand, -dist);
    //             }
    //         }
    //     }
    // }
    // // Add error margin and transform to voxel space
    // exteriorNarrowBand = (exteriorNarrowBand + box.getSize().x / 32.0f) / voxelSize;
    // interiorNarrowBand = (interiorNarrowBand + box.getSize().x / 32.0f) / voxelSize;

    // openvdb::math::Transform::Ptr linearTransform = openvdb::math::Transform::createLinearTransform(voxelSize);
    // glm::vec3 gridCenter = box.getCenter() + 0.5f * voxelSize;
    // linearTransform->postTranslate(openvdb::Vec3R(static_cast<double>(gridCenter.x), static_cast<double>(gridCenter.y), static_cast<double>(gridCenter.z)));

    // std::vector<openvdb::Vec4I> emptyMeshQuadIndices;
    // std::vector<openvdb::Vec3I> meshTraingleIndices(mesh.getIndices().size()/3);
    // std::memcpy(meshTraingleIndices.data(), mesh.getIndices().data(), mesh.getIndices().size() * sizeof(uint32_t));

    // timer.start();    
    // openvdb::FloatGrid::Ptr vdbGrid = openvdb::tools::meshToSignedDistanceField<openvdb::FloatGrid>(*linearTransform,
    //         *reinterpret_cast<std::vector<openvdb::Vec3s>*>(&mesh.getVertices()),
    //         meshTraingleIndices,
    //         emptyMeshQuadIndices,
    //         exteriorNarrowBand,
    //         interiorNarrowBand);


    // std::vector<openvdb::Vec3s>& mPointsIn = *reinterpret_cast<std::vector<openvdb::Vec3s>*>(&mesh.getVertices());

    // const size_t numPoints = mPointsIn.size();
    // std::unique_ptr<openvdb::Vec3s[]> indexSpacePoints{ new openvdb::Vec3s[numPoints] };

    // openvdb::Vec3d pos;

    // for (size_t n = 0; n < numPoints; ++n) 
    // {
    //     const openvdb::Vec3s& wsP = mPointsIn[n];
    //     pos[0] = double(wsP[0]);
    //     pos[1] = double(wsP[1]);
    //     pos[2] = double(wsP[2]);

    //     pos = linearTransform->worldToIndex(pos);

    //     openvdb::Vec3s& isP = indexSpacePoints[n];
    //     isP[0] = typename openvdb::Vec3s::value_type(pos[0]);
    //     isP[1] = typename openvdb::Vec3s::value_type(pos[1]);
    //     isP[2] = typename openvdb::Vec3s::value_type(pos[2]);
    // }

    // const size_t numPrimitives = meshTraingleIndices.size();
    // std::unique_ptr<openvdb::Vec4I[]> prims{ new openvdb::Vec4I[numPrimitives] };

    // for (size_t n = 0, N = meshTraingleIndices.size(); n < N; ++n) {
    //     const openvdb::Vec3I& triangle = meshTraingleIndices[n];
    //     openvdb::Vec4I& prim = prims[n];
    //     prim[0] = triangle[0];
    //     prim[1] = triangle[1];
    //     prim[2] = triangle[2];
    //     prim[3] = openvdb::util::INVALID_IDX;
    // }

    // openvdb::tools::QuadAndTriangleDataAdapter<openvdb::Vec3s, openvdb::Vec4I>
    //     meshvdb(indexSpacePoints.get(), numPoints, prims.get(), numPrimitives);

    // openvdb::util::NullInterrupter nullInterrupter;
    // using Int32GridType = typename openvdb::FloatGrid::template ValueConverter<openvdb::Int32>::Type;
    // typename Int32GridType::Ptr vdbIndexGrid;
    // vdbIndexGrid.reset(new Int32GridType(openvdb::Int32(openvdb::util::INVALID_IDX)));

    // openvdb::FloatGrid::Ptr vdbGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(nullInterrupter, meshvdb, *linearTransform,
    //                                     exteriorNarrowBand, interiorNarrowBand, 0, vdbIndexGrid.get());
    
    SPDLOG_INFO("OpenVDB init time: {}", timer.getElapsedSeconds());
    // vdbGrid->print();

    // openvdb::tools::GridSampler<Int32GridType, openvdb::tools::BoxSampler> vdbIndexSampler(*vdbIndexGrid);
    // openvdb::tools::GridSampler<openvdb::FloatGrid, openvdb::tools::BoxSampler> vdbSampler(*vdbGrid);

    ICG icg(mesh);

    // CGALtree cgalTree(mesh);

    // const float z = 0.163f;
    const float z = 0.056f;
    std::array<glm::vec3, 4> samplesQuad = {
        glm::vec3(box.min.x, box.max.y, z),
        glm::vec3(box.max.x, box.max.y, z),
        glm::vec3(box.min.x, box.min.y, z),
        glm::vec3(box.max.x, box.min.y, z),
    };
    
    timer.start();

    uint32_t imageWidth = args::get(imageWidthArg);

    std::vector<float> outImage1(imageWidth * imageWidth);
    std::vector<float> outImage2(imageWidth * imageWidth);

    float invWidth = 1.0f / static_cast<float>(imageWidth);

    auto interpolate = [](glm::vec3 a, glm::vec3 b, float t)
    {
        return (1.0f - t) * a + t * b;
    };

    float minImage1 = INFINITY;
    float maxImage1 = 0.0f;

    float minImage2 = INFINITY;
    float maxImage2 = 0.0f;

    std::array<float, 40> histAccTime1;
    histAccTime1.fill(0.0f);
    std::array<uint32_t, 40> histCount1;
    histCount1.fill(0);

    std::array<float, 40> histAccTime2;
    histAccTime2.fill(0.0f);
    std::array<uint32_t, 40> histCount2;
    histCount2.fill(0);

    float invDiag = 1.0f / glm::length(box.max - box.min);

    float maxError = 0.0f;

    for(uint32_t j=0; j < imageWidth; j++)
    {
        for(uint32_t i=0; i < imageWidth; i++)
        {
            const float tx = invWidth * (0.5f + static_cast<float>(i));
            const float ty = invWidth * (0.5f + static_cast<float>(j));
            // const float tx = invWidth * static_cast<float>(i);
            // const float ty = invWidth * static_cast<float>(j);
            const glm::vec3 pos = interpolate(interpolate(samplesQuad[0], samplesQuad[1], tx), 
                                              interpolate(samplesQuad[2], samplesQuad[3], tx), ty);

            // outImage1[j * imageWidth + i] = glm::abs(exactSdf->getDistance(pos) - 
            //                                 vdbSampler.wsSample(openvdb::Vec3R(static_cast<double>(pos.x), static_cast<double>(pos.y), static_cast<double>(pos.z)))) * invModelDiagonal;
            outImage1[j * imageWidth + i] = exactSdf->getDistance(pos);
            minImage1 = glm::min(minImage1, outImage1[j * imageWidth + i]);
            maxImage1 = glm::max(maxImage1, outImage1[j * imageWidth + i]);
            // const uint32_t samples = 1000;
            // const uint32_t samples = 1;
            // float dist1 = 0.0f;

            // timer.start();
            // for(uint32_t i=0; i < samples; i++)
            // {
            //     dist1 = exactSdf->getDistance(pos);
            // }

            // const float t1 = timer.getElapsedMicroseconds() / static_cast<float>(samples);
            // outImage1[j * imageWidth + i] = t1;
            // uint32_t idx = glm::min(static_cast<uint32_t>(glm::round((dist1 * invDiag + 1.0f) * 20.0f)), 39u);
            // histAccTime1[idx] += t1;
			// histCount1[idx]++;
            // minImage1 = glm::min(minImage1, outImage1[j * imageWidth + i]);
            // maxImage1 = glm::max(maxImage1, outImage1[j * imageWidth + i]);

            // float dist2 = 0.0f;

            // timer.start();
            // for(uint32_t i=0; i < samples; i++)
            // {
            //     dist2 = cgalTree.getDistance(pos);
            // }

            // const float t2 = timer.getElapsedMicroseconds() / static_cast<float>(samples);
            // outImage2[j * imageWidth + i] = t2;
            // histAccTime2[idx] += t2;
			// histCount2[idx]++;
            // minImage2 = glm::min(minImage2, outImage2[j * imageWidth + i]);
            // maxImage2 = glm::max(maxImage2, outImage2[j * imageWidth + i]);

            // maxError = glm::max(maxError, glm::abs(dist1 - dist2));
        }

        // if(maxError > 1e-5)
        // {
        //     break;
        // }
    }

    SPDLOG_INFO("Max error: {}", maxError);

    SPDLOG_INFO("Our method time interval ({},{})", minImage1, maxImage1);
    // SPDLOG_INFO("CGAL time interval ({},{})", minImage2, maxImage2);

    const uint32_t length = outImage1.size();
    std::vector<uint32_t> finalImage1(length);
    std::vector<uint32_t> finalImage2(length);

    // std::array<glm::vec3, 5> colorsPalette = 
    // {
    //     glm::vec3(1.0f, 0.0f, 1.0f), 
    //     glm::vec3(0.0f, 0.0f, 1.0f), 
    //     glm::vec3(0.0f, 1.0f, 0.0f), 
    //     glm::vec3(1.0f, 1.0f, 0.0f), 
    //     glm::vec3(1.0f, 0.0f, 0.0f),
    // };  
    std::array<glm::vec3, 2> colorsPalette = 
    {
        glm::vec3(0.0f, 0.0f, 0.0f), 
        glm::vec3(1.0f, 1.0f, 1.0f)
    };  

    auto printImage = [&](std::string name, float minColorInterval, float maxColorInterval)
    {
        for(uint32_t i=0; i < length; i++)
        {
            {
            const float value = (outImage1[i] - minColorInterval) / (maxColorInterval - minColorInterval);
            float index = glm::clamp(value * (colorsPalette.size()-1), 0.0f, float(colorsPalette.size()-1) - 0.01f);
            const glm::vec3 finalColor = interpolate(colorsPalette[static_cast<uint32_t>(index)], 
                                                    colorsPalette[static_cast<uint32_t>(index)+1], 
                                                    glm::fract(index));

            finalImage1[i] = (255 << 24) | 
                            (static_cast<uint32_t>(255.0f * finalColor.z) << 16) |
                            (static_cast<uint32_t>(255.0f * finalColor.y) << 8) |
                            (static_cast<uint32_t>(255.0f * finalColor.x));
            }

            // {
            // const float value = (outImage2[i] - minColorInterval) / (maxColorInterval - minColorInterval);
            // float index = glm::clamp(value * (colorsPalette.size()-1), 0.0f, float(colorsPalette.size()-1) - 0.01f);
            // const glm::vec3 finalColor = interpolate(colorsPalette[static_cast<uint32_t>(index)], 
            //                                         colorsPalette[static_cast<uint32_t>(index)+1], 
            //                                         glm::fract(index));

            // finalImage2[i] = (255 << 24) | 
            //                 (static_cast<uint32_t>(255.0f * finalColor.z) << 16) |
            //                 (static_cast<uint32_t>(255.0f * finalColor.y) << 8) |
            //                 (static_cast<uint32_t>(255.0f * finalColor.x));
            // }
        }

        stbi_write_png((name + "1.png").c_str(), imageWidth, imageWidth, 4, static_cast<void*>(finalImage1.data()), 4 * imageWidth);
        // stbi_write_png((name + "2.png").c_str(), imageWidth, imageWidth, 4, static_cast<void*>(finalImage2.data()), 4 * imageWidth);
    };

    printImage("image", minImage1, maxImage1);

    // printImage("quarter", 0.0f, 0.25f * glm::max(maxImage1, maxImage2));

    // printImage("low2", 0.0f, 8.0f);

    // printImage("low", 0.0f, 5.0f);

    // for(int i=0; i < 40; i++)
    // {
    //     const float t = histAccTime1[i] / static_cast<float>(histCount1[i]);
    //     SPDLOG_INFO("{}%: {}", 5*(i-20), t);
    // }

    // for(int i=0; i < 40; i++)
    // {
    //     const float t = histAccTime2[i] / static_cast<float>(histCount2[i]);
    //     SPDLOG_INFO("{}%: {}", 5*(i-20), t);
    // }


    /*
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
    */
}