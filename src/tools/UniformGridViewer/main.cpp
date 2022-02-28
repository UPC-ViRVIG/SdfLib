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

#include <spdlog/spdlog.h>
#include <args.hxx>
#include <imgui.h>
#include <ImGuizmo.h>
#include <glm/gtc/type_ptr.hpp>

class MyScene : public Scene
{
public:
    MyScene(std::string modelPath, float cellSize) : mModelPath(modelPath), mCellSize(cellSize) {}

	void start() override
	{
		Window::getCurrentWindow().setBackgroudColor(glm::vec4(0.9, 0.9, 0.9, 1.0));

		auto camera = std::make_shared<NavigationCamera>();
		camera->start();
		setMainCamera(camera);
		addSystem(camera);

		mPlaneRenderer = std::make_shared<RenderMesh>();
		mPlaneRenderer->start();

		// Plane
		std::shared_ptr<Mesh> plane = PrimitivesFactory::getPlane();

		mPlaneRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
									RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
							}, plane->getVertices().data(), plane->getVertices().size());

		mPlaneRenderer->setIndexData(plane->getIndices());

		Mesh sphereMesh(mModelPath);
		BoundingBox box = sphereMesh.getBoudingBox();
		box.addMargin(0.5f);

		Timer timer; timer.start();
		UniformGridSdf sdfGrid(sphereMesh, box, mCellSize);
		SPDLOG_INFO("Uniform grid generation time: {}s", timer.getElapsedSeconds());

		mGizmoStartMatrix = glm::translate(glm::mat4x4(1.0f), sdfGrid.getGridBoundingBox().getCenter()) *
							glm::scale(glm::mat4x4(1.0f), 2.0f * sdfGrid.getGridBoundingBox().getSize());
		mGizmoMatrix = mGizmoStartMatrix;
		mPlaneRenderer->setTransform(mGizmoMatrix);

		mPlaneShader = std::unique_ptr<SdfPlaneShader> (new SdfPlaneShader(sdfGrid));
		mPlaneRenderer->setShader(mPlaneShader.get());
		addSystem(mPlaneRenderer);
		mPlaneRenderer->callDrawGui = false;

		auto sphereMeshRenderer = std::make_shared<RenderMesh>();
		sphereMeshRenderer->systemName = "Object Mesh";
		sphereMeshRenderer->start();
		sphereMeshRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
									RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
							}, sphereMesh.getVertices().data(), sphereMesh.getVertices().size());

        sphereMesh.computeNormals();
		sphereMeshRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
									RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
							}, sphereMesh.getNormals().data(), sphereMesh.getNormals().size());

		sphereMeshRenderer->setIndexData(sphereMesh.getIndices());
		sphereMeshRenderer->setShader(Shader<NormalsShader>::getInstance());
		addSystem(sphereMeshRenderer);
		sphereMeshRenderer->drawSurface(false);

		std::shared_ptr<Mesh> cubeMesh = PrimitivesFactory::getCube();
		mCubeRenderer = std::make_shared<RenderMesh>();
		mCubeRenderer->start();
		mCubeRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
									RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
							}, cubeMesh->getVertices().data(), cubeMesh->getVertices().size());

		cubeMesh->computeNormals();	
		mCubeRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
									RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
							}, cubeMesh->getNormals().data(), cubeMesh->getNormals().size());

		mCubeRenderer->setIndexData(cubeMesh->getIndices());

		mCubeShader = std::unique_ptr<NormalsSplitPlaneShader>(new NormalsSplitPlaneShader(glm::vec4(0.0, 0.0, 1.0, 0.0)));

		mCubeRenderer->setShader(mCubeShader.get());

		mCubeRenderer->setTransform(glm::translate(glm::mat4x4(1.0f), sdfGrid.getGridBoundingBox().getCenter()) *
					   				glm::scale(glm::mat4x4(1.0f), sdfGrid.getGridBoundingBox().getSize()));
		addSystem(mCubeRenderer);

		// Draw normals
		auto meshNormals = std::make_shared<RenderMesh>();
		meshNormals->systemName = "Mesh normals";
		meshNormals->start();
		std::vector<glm::vec3> normals;
		int tIndex = 0;
		for(TriangleUtils::TriangleData& triData : TriangleUtils::calculateMeshTriangleData(sphereMesh))
		{
			const glm::vec3 v1 = sphereMesh.getVertices()[sphereMesh.getIndices()[3 * tIndex]];
			const glm::vec3 v2 = sphereMesh.getVertices()[sphereMesh.getIndices()[3 * tIndex + 1]];
			const glm::vec3 v3 = sphereMesh.getVertices()[sphereMesh.getIndices()[3 * tIndex + 2]];

			glm::mat3 trans = glm::inverse(triData.transform);

			// Vertices normal
			normals.push_back(v1);
			normals.push_back(v1 + 0.25f * trans * glm::normalize(triData.verticesNormal[0]));

			normals.push_back(v2);
			normals.push_back(v2 + 0.25f * trans * glm::normalize(triData.verticesNormal[1]));

			normals.push_back(v3);
			normals.push_back(v3 + 0.25f * trans *glm::normalize(triData.verticesNormal[2]));

			// Edges normal
			glm::vec3 nPos = 0.5f * (v1 + v2);
			normals.push_back(nPos);
			normals.push_back(nPos + 0.25f * trans * glm::normalize(triData.edgesNormal[0]));

			nPos = 0.5f * (v2 + v3);
			normals.push_back(nPos);
			normals.push_back(nPos + 0.25f * trans * glm::normalize(triData.edgesNormal[1]));

			nPos = 0.5f * (v3 + v1);
			normals.push_back(nPos);
			normals.push_back(nPos + 0.25f * trans * glm::normalize(triData.edgesNormal[2]));

			// Triangle normal
			nPos =  (v1 + v2 + v3) / 3.0f;
			normals.push_back(nPos);
			normals.push_back(nPos + 0.25f * glm::normalize(triData.getTriangleNormal()));

			tIndex++;
		}

		meshNormals->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
									RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
								   }, normals.data(), normals.size());
		
		meshNormals->setDataMode(GL_LINES);
		meshNormals->setShader(Shader<BasicShader>::getInstance());
		meshNormals->drawSurface(false);
		addSystem(meshNormals);
	}

	void update(float deltaTime) override
	{
		Scene::update(deltaTime);

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Text("Scene Options");

		bool drawGrid = mPlaneShader->isDrawingGrid();
		ImGui::Checkbox("Print grid", &drawGrid);
		mPlaneShader->drawGrid(drawGrid);

		bool drawIsolines = mPlaneShader->isDrawingIsolines();
		ImGui::Checkbox("Print Isolines", &drawIsolines);
		mPlaneShader->drawIsolines(drawIsolines);

		ImGuiIO& io = ImGui::GetIO();
    	ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

		if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_1))
		{
			mGizmoMatrix = mGizmoStartMatrix;
		} 
		else if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_2))
		{
			mGizmoMatrix = glm::rotate(glm::mat4x4(1.0f), glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f)) * mGizmoStartMatrix;
		} 
		else if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_3))
		{
			mGizmoMatrix = glm::rotate(glm::mat4x4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f)) * mGizmoStartMatrix;
		}

		if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_LEFT_ALT))
		{
			ImGuizmo::Manipulate(glm::value_ptr(getMainCamera()->getViewMatrix()), 
							 glm::value_ptr(getMainCamera()->getProjectionMatrix()),
							 ImGuizmo::OPERATION::ROTATE, ImGuizmo::MODE::LOCAL, glm::value_ptr(mGizmoMatrix));
		}
		else
		{
			ImGuizmo::Manipulate(glm::value_ptr(getMainCamera()->getViewMatrix()), 
							 glm::value_ptr(getMainCamera()->getProjectionMatrix()),
							 ImGuizmo::OPERATION::TRANSLATE_Z, ImGuizmo::MODE::LOCAL, glm::value_ptr(mGizmoMatrix));
		}
		
		mPlaneRenderer->setTransform(mGizmoMatrix);
		glm::vec3 planeNormal = glm::normalize(glm::vec3(mGizmoMatrix * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f)));
		glm::vec3 planePoint = glm::vec3(mGizmoMatrix * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
		mCubeShader->setCutPlane(glm::vec4(planeNormal.x, planeNormal.y, planeNormal.z, -glm::dot(planeNormal, planePoint)));
		mPlaneShader->setNormal(planeNormal);
	}
private:
	std::unique_ptr<SdfPlaneShader> mPlaneShader;
	std::unique_ptr<NormalsSplitPlaneShader> mCubeShader;

	std::shared_ptr<RenderMesh> mPlaneRenderer; 
	std::shared_ptr<RenderMesh> mCubeRenderer;

	glm::mat4x4 mGizmoStartMatrix;
	glm::mat4x4 mGizmoMatrix;

    std::string mModelPath;
    float mCellSize;
};

int main(int argc, char** argv)
{
	spdlog::set_pattern("[%^%l%$] [%s:%#] %v");

    args::ArgumentParser parser("UniformGridViwer reconstructs and draws a uniform grid sdf");
    args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
    args::Positional<std::string> modelPathArg(parser, "model_path", "The model path");
    args::Positional<float> cellSizeArg(parser, "cell_size", "The voxel size of the voxelization");

    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch(args::Help)
    {
        std::cerr << parser;
        return 0;
    }


    MyScene scene(
        (modelPathArg) ? args::get(modelPathArg) : "../models/sphere.glb",
        // (cellSizeArg) ? args::get(cellSizeArg) : 0.1f
		(cellSizeArg) ? args::get(cellSizeArg) : 0.2f
    );
    MainLoop loop;
    loop.start(scene);
}