#include <random>
#include <vector>
#include <args.hxx>
#include <spdlog/spdlog.h>
#include <iostream>
#include <list>
#include <memory>

#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/Complex_2_in_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/IO/facets_in_complex_2_to_triangle_mesh.h>
#include <CGAL/Surface_mesh.h>


#include "sdf/SdfFunction.h"

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


std::shared_ptr<SdfFunction> exactSdf;
float isovalue = 0.0f;

FT sphere_function (const Point_3 p)
{ 
	return exactSdf->getDistance(glm::vec3(p.x(), p.y(), p.z())) - isovalue;
}

int main(int argc, char** argv)
{
    spdlog::set_pattern("[%^%l%$] [%s:%#] %v");

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
	isovalue = (isovalueArg) ? args::get(isovalueArg) : 0.0f;

	Tr tr;            
	C2t3 c2t3(tr);

	Surface_3 surface(sphere_function,            
					  Sphere_3(CGAL::ORIGIN, glm::sqrt(3.0f)));

	CGAL::Surface_mesh_default_criteria_3<Tr> criteria(25.,  // angular bound
														0.01,  // radius bound
														0.01); // distance bound

	// meshing surface
	CGAL::make_surface_mesh(c2t3, surface, criteria, CGAL::Non_manifold_tag());
	Surface_mesh sm;
	CGAL::facets_in_complex_2_to_triangle_mesh(c2t3, sm);

	std::ofstream mesh_file("out.ply");
	CGAL::IO::write_PLY(mesh_file, sm);
}