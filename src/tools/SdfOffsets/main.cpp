#include <random>
#include <vector>
#include <args.hxx>
#include <spdlog/spdlog.h>
#include <iostream>
#include <list>
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <InteractiveComputerGraphics/TriangleMeshDistance.h>
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/Complex_2_in_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/IO/facets_in_complex_2_to_triangle_mesh.h>
#include <CGAL/Surface_mesh.h>


#include "sdf/SdfFunction.h"
#include "utils/Timer.h"

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

// Default triangulation for Surface_mesher
typedef CGAL::Surface_mesh_default_triangulation_3 Tr;

// c2t3
typedef CGAL::Complex_2_in_triangulation_3<Tr> C2t3;

typedef Tr::Geom_traits GT;
typedef GT::Sphere_3 Sphere_3;
typedef GT::Point_3 Point_3;
typedef GT::FT FT;

typedef FT(*Function)(Point_3);
typedef CGAL::Implicit_surface_3<GT, Function> Surface_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

float isovalue = 0.0f;

std::shared_ptr<SdfFunction> exactSdf;
FT sphere_function (const Point_3 p)
{ 
	return exactSdf->getDistance(glm::vec3(p.x(), p.y(), p.z())) - isovalue;
}

// ICG* pIcg;
// FT sphere_function (const Point_3 p)
// { 
// 	return pIcg->getDistance(glm::vec3(p.x(), p.y(), p.z())) - isovalue;
// }

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
	args::Positional<float> isovalueArg(parser, "isovalue", "Isovalue");

    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch(args::Help)
    {
        std::cerr << parser;
        return 0;
    }

    exactSdf = SdfFunction::loadFromFile(args::get(sdfPathArg));
	// Mesh mesh(args::get(sdfPathArg));
    // const glm::vec3 boxSize = mesh.getBoudingBox().getSize();
    // mesh.applyTransform( glm::scale(glm::mat4(1.0), glm::vec3(2.0f/glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z))) *
    //                                 glm::translate(glm::mat4(1.0), -mesh.getBoudingBox().getCenter()));
	// ICG icg(mesh);
	// pIcg = &icg;
	isovalue = (isovalueArg) ? -args::get(isovalueArg) : 0.0f;

	Tr tr;            
	C2t3 c2t3(tr);

	Surface_3 surface(sphere_function,            
					  Sphere_3(CGAL::ORIGIN, 1.73f + 2.0f * isovalue));

	CGAL::Surface_mesh_default_criteria_3<Tr> criteria(25.,  // angular bound
														0.01,  // radius bound
														0.01); // distance bound

	// meshing surface
	Timer timer;
	timer.start();
	CGAL::make_surface_mesh(c2t3, surface, criteria, CGAL::Non_manifold_tag());
	Surface_mesh sm;
	CGAL::facets_in_complex_2_to_triangle_mesh(c2t3, sm);

	SPDLOG_INFO("Computation time: {}", timer.getElapsedSeconds());

	std::ofstream mesh_file("out.ply");
	CGAL::IO::write_PLY(mesh_file, sm);
}