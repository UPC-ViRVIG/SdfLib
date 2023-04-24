#include <iostream>
#include <random>
#include <algorithm>
#include <optional>
#include "utils/Mesh.h"
#include "utils/TriangleUtils.h"
#include "utils/PrimitivesFactory.h"
#include "utils/Timer.h"
#include "utils/GJK.h"
#include "sdf/TrianglesInfluence.h"
#include "sdf/InterpolationMethods.h"
#include "sdf/ExactOctreeSdf.h"
#include "render_engine/MainLoop.h"
#include "render_engine/NavigationCamera.h"
#include "render_engine/RenderMesh.h"
#include "render_engine/shaders/NormalsShader.h"
#include "render_engine/shaders/SdfPlaneShader.h"
#include "render_engine/shaders/SdfOctreePlaneShader.h"
#include "render_engine/shaders/BasicShader.h"
#include "render_engine/shaders/SdfOctreeMeanTrianglesPlaneShader.h"
#include "render_engine/shaders/NormalsSplitPlaneShader.h"
#include "render_engine/shaders/ColorsShader.h"
#include "render_engine/Window.h"

#include <spdlog/spdlog.h>
#include <args.hxx>
#include <imgui.h>
#include <ImGuizmo.h>
#include <glm/gtc/type_ptr.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <fstream>

using namespace sdflib;

class MyScene : public Scene
{
public:
    MyScene(std::string sdfExactPath) : mSdfExactPath(sdfExactPath)
    {}

    void start() override
    {
		Window::getCurrentWindow().setBackgroudColor(glm::vec4(1.0, 1.0, 1.0, 1.0));

        // Create camera
		{
			auto camera = std::make_shared<NavigationCamera>();
			camera->start();
			setMainCamera(camera);
			addSystem(camera);
		}

        ExactOctreeSdf exactOctreeSdf;

        {
            std::unique_ptr<SdfFunction> sdfFunc = SdfFunction::loadFromFile(mSdfExactPath);
            if(!sdfFunc)
			{
				assert(false);
				return;
			}

            if(sdfFunc->getFormat() == SdfFunction::SdfFormat::EXACT_OCTREE)
            {
                exactOctreeSdf = std::move(*reinterpret_cast<ExactOctreeSdf*>(sdfFunc.get()));
            }
            else
            {
                assert(false);
                return;
            }
        }

		mMinNumTriangles = exactOctreeSdf.getMinTrianglesInLeafs();
		mMinMinNumTriangles = exactOctreeSdf.getMinTrianglesInLeafs();
		mMinMinNumTriangles = 0.0f;
		mMaxNumTriangles = exactOctreeSdf.getMaxTrianglesInLeafs();
		mMaxMaxNumTriangles = exactOctreeSdf.getMaxTrianglesInLeafs();
        
        BoundingBox viewBB = exactOctreeSdf.getGridBoundingBox();

        // Create sdf plane
		{
			mPlaneRenderer = std::make_shared<RenderMesh>();
			mPlaneRenderer->start();

			// Plane
			std::shared_ptr<Mesh> plane = PrimitivesFactory::getPlane();

			mPlaneRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
										RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
								}, plane->getVertices().data(), plane->getVertices().size());

			mPlaneRenderer->setIndexData(plane->getIndices());

			mGizmoStartMatrix = glm::translate(glm::mat4x4(1.0f), viewBB.getCenter()) *
								glm::scale(glm::mat4x4(1.0f), 2.0f * viewBB.getSize());
			mGizmoStartMatrix = glm::translate(glm::mat4x4(1.0f), glm::vec3(0.0f, 0.0f, 0.163f)) * mGizmoStartMatrix;
			mGizmoMatrix = mGizmoStartMatrix;
			mPlaneRenderer->setTransform(mGizmoMatrix);
			
			mOctreePlaneShader = std::unique_ptr<SdfOctreeMeanTrianglesPlaneShader> (new SdfOctreeMeanTrianglesPlaneShader(exactOctreeSdf, viewBB));
			mPlaneRenderer->setShader(mOctreePlaneShader.get());
			addSystem(mPlaneRenderer);
			mPlaneRenderer->callDrawGui = false;
		}

        // Create model BB cube
		{
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

			mCubeRenderer->setTransform(glm::translate(glm::mat4x4(1.0f), viewBB.getCenter()) *
										glm::scale(glm::mat4x4(1.0f), viewBB.getSize()));
			mCubeRenderer->callDrawGui = false;
			addSystem(mCubeRenderer);
		}
    }

    void update(float deltaTime) override
	{
        Scene::update(deltaTime);

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Text("Scene Options");

		int v[2] = { static_cast<int>(mMinNumTriangles),
					 static_cast<int>(mMaxNumTriangles)};
		ImGui::SliderInt2("MinMaxTriangles", v, static_cast<int>(mMinMinNumTriangles), static_cast<int>(mMaxMaxNumTriangles));
		mMinNumTriangles = v[0];
		mMaxNumTriangles = v[1];

		mOctreePlaneShader->setMinNumTriangles(mMinNumTriangles);
		mOctreePlaneShader->setMaxNumTriangles(mMaxNumTriangles);

        ImGuiIO& io = ImGui::GetIO();
    	ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

		// Short keys to change cube cuts
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

		if(!Window::getCurrentWindow().isKeyPressed(GLFW_KEY_LEFT_CONTROL))
		{
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
		}
		
		mPlaneRenderer->setTransform(mGizmoMatrix);
		glm::vec3 planeNormal = glm::normalize(glm::vec3(mGizmoMatrix * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f)));
		glm::vec3 planePoint = glm::vec3(mGizmoMatrix * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
		mCubeShader->setCutPlane(glm::vec4(planeNormal.x, planeNormal.y, planeNormal.z, -glm::dot(planeNormal, planePoint)));
		
		mOctreePlaneShader->setNormal(planeNormal);
    }

private:
    std::string mSdfExactPath;

	uint32_t mMinNumTriangles;
	uint32_t mMaxNumTriangles;

	uint32_t mMaxMaxNumTriangles;
	uint32_t mMinMinNumTriangles;

    std::unique_ptr<NormalsSplitPlaneShader> mCubeShader;
	std::unique_ptr<SdfOctreeMeanTrianglesPlaneShader> mOctreePlaneShader;

    std::shared_ptr<RenderMesh> mPlaneRenderer; 
	std::shared_ptr<RenderMesh> mCubeRenderer;

    glm::mat4x4 mGizmoStartMatrix;
	glm::mat4x4 mGizmoMatrix;
};

int main(int argc, char** argv)
{
	#ifdef SDFLIB_PRINT_STATISTICS
        spdlog::set_pattern("[%^%l%$] [%s:%#] %v");
    #else
        spdlog::set_pattern("[%^%l%$] %v");
    #endif

    args::ArgumentParser parser("UniformGridViwer reconstructs and draws a uniform grid sdf");
    args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
	args::Positional<std::string> sdfPathArg(parser, "sdf_path", " The precalculated sdf to visualize");

    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch(args::Help)
    {
        std::cerr << parser;
        return 0;
    }

	MyScene scene(args::get(sdfPathArg));
	MainLoop loop;
	loop.start(scene);
}