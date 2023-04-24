#include <iostream>
#include <random>
#include <algorithm>
#include "render_engine/MainLoop.h"
#include "render_engine/NavigationCamera.h"
#include "render_engine/RenderMesh.h"
#include "render_engine/shaders/NormalsShader.h"
#include "render_engine/shaders/SdfPlaneShader.h"
#include "render_engine/shaders/BasicShader.h"
#include "render_engine/shaders/NormalsSplitPlaneShader.h"
#include "render_engine/Window.h"
#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "utils/PrimitivesFactory.h"
#include "utils/Timer.h"
#include "sdf/InterpolationMethods.h"

#include <spdlog/spdlog.h>
#include <args.hxx>
#include <imgui.h>
#include <ImGuizmo.h>
#include <glm/gtc/type_ptr.hpp>

using namespace sdflib;

class TestScene : public Scene
{
	void start() override
	{
		Window::getCurrentWindow().setBackgroudColor(glm::vec4(0.9, 0.9, 0.9, 1.0));

		auto camera = std::make_shared<NavigationCamera>();
		camera->start();
		setMainCamera(camera);
		addSystem(camera);

		std::shared_ptr<Mesh> isosphereMesh = PrimitivesFactory::getIsosphere(3);
		mCubeRenderer = std::make_shared<RenderMesh>();
		mCubeRenderer->start();
		mCubeRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
									RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
							}, isosphereMesh->getVertices().data(), isosphereMesh->getVertices().size());

		isosphereMesh->computeNormals();	
		mCubeRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
									RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
							}, isosphereMesh->getNormals().data(), isosphereMesh->getNormals().size());

		mCubeRenderer->setIndexData(isosphereMesh->getIndices());
		mCubeRenderer->setShader(Shader<NormalsShader>::getInstance());
		addSystem(mCubeRenderer);
	}

	void update(float deltaTime) override
	{
		Scene::update(deltaTime);

		// ImGuiIO& io = ImGui::GetIO();
    	// ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

		// if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_LEFT_ALT))
		// {
		// 	ImGuizmo::Manipulate(glm::value_ptr(getMainCamera()->getViewMatrix()), 
		// 						glm::value_ptr(getMainCamera()->getProjectionMatrix()),
		// 						ImGuizmo::OPERATION::ROTATE, ImGuizmo::MODE::LOCAL, glm::value_ptr(mGizmoMatrix));
		// }
		// else
		// {
		// 	ImGuizmo::Manipulate(glm::value_ptr(getMainCamera()->getViewMatrix()), 
		// 						glm::value_ptr(getMainCamera()->getProjectionMatrix()),
		// 						ImGuizmo::OPERATION::TRANSLATE_Z, ImGuizmo::MODE::LOCAL, glm::value_ptr(mGizmoMatrix));
		// }
		
		// mCubeRenderer->setTransform(mGizmoMatrix);
	}

private:
	glm::mat4x4 mGizmoMatrix = glm::mat4(1.0f);
	std::shared_ptr<RenderMesh> mCubeRenderer;
};

int main()
{
	#ifdef SDFLIB_PRINT_STATISTICS
        spdlog::set_pattern("[%^%l%$] [%s:%#] %v");
    #else
        spdlog::set_pattern("[%^%l%$] %v");
    #endif

    Mesh mesh("../models/sphere.glb");

	std::vector<TriangleUtils::TriangleData> trianglesData = TriangleUtils::calculateMeshTriangleData(mesh);
	std::vector<uint32_t> triangles(trianglesData.size());

	for(uint32_t i=0; i < trianglesData.size(); i++)
	{
		triangles[i] = i;
	}

	std::array<std::array<float, 8>, 8> coeff;
	typedef TriCubicInterpolation Inter;
	auto f = [](glm::vec3 point)
	{
		std::array<float, 8> c;
		c[0] = point.x;
		c[1] = 1.0f;
		c[2] = 0.0f;
		c[3] = 0.0f;

		c[4] = 0.0f;
		c[5] = 0.0f;
		c[6] = 0.0f;
		c[7] = 0.0f;
		return c;
	};

	glm::vec3 center = glm::vec3(-1.24f, -2.0f, 2.0f);

	coeff[0] = f(5.0f * glm::vec3(0.0f, 0.0f, 0.0f));
	coeff[1] = f(5.0f * glm::vec3(1.0f, 0.0f, 0.0f));
	coeff[2] = f(5.0f * glm::vec3(0.0f, 1.0f, 0.0f));
	coeff[3] = f(5.0f * glm::vec3(1.0f, 1.0f, 0.0f));

	coeff[4] = f(5.0f * glm::vec3(0.0f, 0.0f, 1.0f));
	coeff[5] = f(5.0f * glm::vec3(1.0f, 0.0f, 1.0f));
	coeff[6] = f(5.0f * glm::vec3(0.0f, 1.0f, 1.0f));
	coeff[7] = f(5.0f * glm::vec3(1.0f, 1.0f, 1.0f));

	std::array<float, 64> param;
	Inter::calculateCoefficients(coeff, 5.0f, triangles, mesh, trianglesData, param);
	
	std::array<float, 8> interCoeff;
	Inter::interpolateVertexValues(param, glm::vec3(0.5f, 0.0f, 0.0f), 5.0f, interCoeff);
	
	std::cout << interCoeff[0] << std::endl;
	std::cout << interCoeff[1] << std::endl;
	std::cout << interCoeff[2] << std::endl;
	std::cout << interCoeff[3] << std::endl;
}