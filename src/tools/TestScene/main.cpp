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
	spdlog::set_pattern("[%^%l%$] [%s:%#] %v");

    Mesh mesh("../models/sphere.glb");

	std::vector<TriangleUtils::TriangleData> trianglesData = TriangleUtils::calculateMeshTriangleData(mesh);
	std::vector<uint32_t> triangles(trianglesData.size());

	for(uint32_t i=0; i < trianglesData.size(); i++)
	{
		triangles[i] = i;
	}

	std::array<std::array<float, 4>, 8> coeff;
	typedef TriCubicInterpolation Inter;
	Inter::calculatePointsValues(glm::vec3(0.0, 0.0, 0.0), 0, mesh, trianglesData, coeff[0]);
	Inter::calculatePointsValues(glm::vec3(1.0, 0.0, 0.0), 0, mesh, trianglesData, coeff[1]);
	Inter::calculatePointsValues(glm::vec3(0.0, 1.0, 0.0), 0, mesh, trianglesData, coeff[2]);
	Inter::calculatePointsValues(glm::vec3(1.0, 1.0, 0.0), 0, mesh, trianglesData, coeff[3]);

	Inter::calculatePointsValues(glm::vec3(0.0, 0.0, 1.0), 0, mesh, trianglesData, coeff[4]);
	Inter::calculatePointsValues(glm::vec3(1.0, 0.0, 1.0), 0, mesh, trianglesData, coeff[5]);
	Inter::calculatePointsValues(glm::vec3(0.0, 1.0, 1.0), 0, mesh, trianglesData, coeff[6]);
	Inter::calculatePointsValues(glm::vec3(1.0, 1.0, 1.0), 0, mesh, trianglesData, coeff[7]);


	std::array<float, 64> interpolation;
	Inter::calculateCoefficients(coeff, triangles, mesh, trianglesData, interpolation);

	float value = Inter::interpolateValue(interpolation, glm::vec3(0.0, 0.0, 0.0));
	std::cout << value << ", " << coeff[0][0] << std::endl;
	
	value = Inter::interpolateValue(interpolation, glm::vec3(1.0, 0.0, 0.0));
	std::cout << value << ", " << coeff[1][0] << std::endl;

	value = Inter::interpolateValue(interpolation, glm::vec3(0.0, 1.0, 0.0));
	std::cout << value << ", " << coeff[2][0] << std::endl;

	value = Inter::interpolateValue(interpolation, glm::vec3(1.0, 1.0, 0.0));
	std::cout << value << ", " << coeff[3][0] << std::endl;

	value = Inter::interpolateValue(interpolation, glm::vec3(0.0, 0.0, 1.0));
	std::cout << value << ", " << coeff[4][0] << std::endl;

	value = Inter::interpolateValue(interpolation, glm::vec3(1.0, 0.0, 1.0));
	std::cout << value << ", " << coeff[5][0] << std::endl;

	value = Inter::interpolateValue(interpolation, glm::vec3(0.0, 1.0, 1.0));
	std::cout << value << ", " << coeff[6][0] << std::endl;

	value = Inter::interpolateValue(interpolation, glm::vec3(1.0, 1.0, 1.0));
	std::cout << value << ", " << coeff[7][0] << std::endl;
}