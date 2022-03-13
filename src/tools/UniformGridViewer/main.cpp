#include <iostream>
#include <random>
#include <algorithm>
#include <optional>
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
#include "utils/GJK.h"

#include <spdlog/spdlog.h>
#include <args.hxx>
#include <imgui.h>
#include <ImGuizmo.h>
#include <glm/gtc/type_ptr.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <fstream>

class MyScene : public Scene
{
public:
    MyScene(std::string modelPath, float cellSize) : mModelPath(modelPath), mCellSize(cellSize), mNormalizeModel(true) {}
	MyScene(std::string modelPath, uint32_t depth) : mModelPath(modelPath), mDepth(depth), mNormalizeModel(true) {}
	MyScene(std::string sdfPath, std::optional<std::string> modelPath = std::optional<std::string>(), bool normalizeModel = false) :
		mSdfPath(sdfPath),
		mModelPath(modelPath),
		mNormalizeModel(normalizeModel) {}

	void start() override
	{
		Window::getCurrentWindow().setBackgroudColor(glm::vec4(0.9, 0.9, 0.9, 1.0));

		// Create camera
		{
			auto camera = std::make_shared<NavigationCamera>();
			camera->start();
			setMainCamera(camera);
			addSystem(camera);
		}

		// Create unifrom grid
		UniformGridSdf sdfGrid;
		if(mModelPath.has_value())
		{
			mMesh = Mesh(mModelPath.value());
			if(mNormalizeModel)
			{
				// Normalize model units
				const glm::vec3 boxSize = mMesh.value().getBoudingBox().getSize();
				mMesh.value().applyTransform( glm::scale(glm::mat4(1.0), glm::vec3(2.0f/glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z))) *
											 glm::translate(glm::mat4(1.0), -mMesh.value().getBoudingBox().getCenter()));
			}
		}

		if(mSdfPath.has_value())
		{
			std::ifstream is(mSdfPath.value(), std::ios::binary);
			if(!is.is_open())
			{
				SPDLOG_ERROR("Cannot open file {}", mSdfPath.value());
				assert(false);
			}
			cereal::PortableBinaryInputArchive archive(is);
			archive(sdfGrid);
		}
		else
		{
			assert(mModelPath.has_value());
			BoundingBox box = mMesh.value().getBoudingBox();
			const glm::vec3 modelBBSize = box.getSize();
			box.addMargin(0.12f * glm::max(glm::max(modelBBSize.x, modelBBSize.y), modelBBSize.z));
			Timer timer; timer.start();
			sdfGrid = (mCellSize.has_value()) ? 
							UniformGridSdf(mMesh.value(), box, mCellSize.value(), UniformGridSdf::InitAlgorithm::OCTREE) :
							UniformGridSdf(mMesh.value(), box, mDepth.value(), UniformGridSdf::InitAlgorithm::OCTREE);
			SPDLOG_INFO("Uniform grid generation time: {}s", timer.getElapsedSeconds());
		}

		
		mSelectArea = sdfGrid.getGridBoundingBox();
		mGridSize = sdfGrid.getGridCellSize();

		// glm::vec3 bbRatio = sdfGrid.getGridBoundingBox().getSize() / sdfGrid.getGridBoundingBox().getSize().x;
		// BoundingBox viewBB(-bbRatio, bbRatio);
		BoundingBox viewBB = sdfGrid.getGridBoundingBox();

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
			mGizmoMatrix = mGizmoStartMatrix;
			mPlaneRenderer->setTransform(mGizmoMatrix);

			mPlaneShader = std::unique_ptr<SdfPlaneShader> (new SdfPlaneShader(sdfGrid, viewBB));
			mPlaneRenderer->setShader(mPlaneShader.get());
			addSystem(mPlaneRenderer);
			mPlaneRenderer->callDrawGui = false;
		}

		if(mMesh.has_value())
		{
			mModelRenderer = std::make_shared<RenderMesh>();
			mModelRenderer->systemName = "Object Mesh";
			mModelRenderer->start();
			mModelRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
										RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
								}, mMesh.value().getVertices().data(), mMesh.value().getVertices().size());

			mModelRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
										RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
								}, mMesh.value().getNormals().data(), mMesh.value().getNormals().size());

			mModelRenderer->setIndexData(mMesh.value().getIndices());
			mModelRenderer->setShader(Shader<NormalsShader>::getInstance());
			mModelRenderer->setTransform(
				glm::translate(glm::mat4(1.0f), viewBB.getCenter()) * 
				glm::scale(glm::mat4(1.0f), viewBB.getSize() / sdfGrid.getGridBoundingBox().getSize()) *
				glm::translate(glm::mat4(1.0f), -sdfGrid.getGridBoundingBox().getCenter())
			);
			addSystem(mModelRenderer);
			mModelRenderer->drawSurface(false);
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

		// Create mMesh representing thhe model normals
		if(mMesh.has_value()){
			auto meshNormals = std::make_shared<RenderMesh>();
			meshNormals->systemName = "Mesh normals";
			meshNormals->start();
			std::vector<glm::vec3> normals;
			int tIndex = 0;
			for(TriangleUtils::TriangleData& triData : TriangleUtils::calculateMeshTriangleData(mMesh.value()))
			{
				const glm::vec3 v1 = mMesh.value().getVertices()[mMesh.value().getIndices()[3 * tIndex]];
				const glm::vec3 v2 = mMesh.value().getVertices()[mMesh.value().getIndices()[3 * tIndex + 1]];
				const glm::vec3 v3 = mMesh.value().getVertices()[mMesh.value().getIndices()[3 * tIndex + 2]];

				glm::mat3 trans = glm::inverse(triData.transform);

				// Vertices normal
				normals.push_back(v1);
				normals.push_back(v1 + 0.008f * trans * glm::normalize(triData.verticesNormal[0]));

				normals.push_back(v2);
				normals.push_back(v2 + 0.008f * trans * glm::normalize(triData.verticesNormal[1]));

				normals.push_back(v3);
				normals.push_back(v3 + 0.008f * trans *glm::normalize(triData.verticesNormal[2]));

				// Edges normal
				glm::vec3 nPos = 0.5f * (v1 + v2);
				normals.push_back(nPos);
				normals.push_back(nPos + 0.008f * trans * glm::normalize(triData.edgesNormal[0]));

				nPos = 0.5f * (v2 + v3);
				normals.push_back(nPos);
				normals.push_back(nPos + 0.008f * trans * glm::normalize(triData.edgesNormal[1]));

				nPos = 0.5f * (v3 + v1);
				normals.push_back(nPos);
				normals.push_back(nPos + 0.008f * trans * glm::normalize(triData.edgesNormal[2]));

				// Triangle normal
				nPos =  (v1 + v2 + v3) / 3.0f;
				normals.push_back(nPos);
				normals.push_back(nPos + 0.008f * glm::normalize(triData.getTriangleNormal()));

				tIndex++;
			}

			meshNormals->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
										RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
									}, normals.data(), normals.size());
			
			meshNormals->setDataMode(GL_LINES);
			meshNormals->setShader(Shader<BasicShader>::getInstance());
			meshNormals->drawSurface(false);
			meshNormals->setTransform(
				glm::translate(glm::mat4(1.0f), viewBB.getCenter()) * 
				glm::scale(glm::mat4(1.0f), viewBB.getSize() / sdfGrid.getGridBoundingBox().getSize()) *
				glm::translate(glm::mat4(1.0f), -sdfGrid.getGridBoundingBox().getCenter())
			);
			addSystem(meshNormals);
		}

		// Create selection cube
		{
			std::shared_ptr<Mesh> cubeMesh = PrimitivesFactory::getCube();
			mSelectionCube = std::make_shared<RenderMesh>();
			mSelectionCube->start();
			mSelectionCube->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
										RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
								}, cubeMesh->getVertices().data(), cubeMesh->getVertices().size());

			cubeMesh->computeNormals();	
			mSelectionCube->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
										RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
								}, cubeMesh->getNormals().data(), cubeMesh->getNormals().size());

			mSelectionCube->setIndexData(cubeMesh->getIndices());

			mSelectionCube->setShader(Shader<BasicShader>::getInstance());

			mSelectionCube->callDrawGui = false;
			mSelectionCube->drawSurface(false);
			mSelectionCube->drawWireframe(false);
			addSystem(mSelectionCube);
		}
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


		// Selection option
		if(!mMesh.has_value()) return;
		bool lastSelectZone = mSelectZone;
		ImGui::Checkbox("Visualize zone", &mSelectZone);
		if(mSelectZone) 
		{
			ImGui::InputInt("Select depth: ", reinterpret_cast<int*>(&mSelectedDepth));
			if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_N))
			{
				glm::vec3 cameraPos = getMainCamera()->getPosition();
				glm::vec3 cameraDir = glm::vec3(glm::inverse(getMainCamera()->getViewMatrix()) * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f));
				float t = glm::dot(planePoint - cameraPos, planeNormal) / glm::dot(cameraDir, planeNormal);
				glm::vec3 selPoint = cameraPos + cameraDir * t;
				const float size = mGridSize * static_cast<float>(1 << mSelectedDepth);

				glm::vec3 centerPoint = mSelectArea.min + glm::floor((selPoint - mSelectArea.min + 0.5f * mGridSize) / size) * size - 0.5f * mGridSize + 0.5f * size;

				mSelectionCube->setTransform( 
					glm::translate(glm::mat4(1.0f), centerPoint) * 
					glm::scale(glm::mat4(1.0f), glm::vec3(size))
				);
				mSelectionCube->drawWireframe(true);

				std::vector<uint32_t> newIndices;
				const std::vector<uint32_t>& indices = mMesh.value().getIndices();
				const std::vector<glm::vec3>& vertices = mMesh.value().getVertices();

				std::vector<std::pair<float, uint32_t>> triangles;

				std::vector<glm::vec3> triangle(3);
    			std::vector<glm::vec3> quad(8);

				quad[0] = -glm::vec3(0.5f*size);
				quad[1] = glm::vec3(0.5f*size, -0.5f*size, -0.5f*size);
				quad[2] = glm::vec3(-0.5f*size, 0.5f*size, -0.5f*size);
				quad[3] = glm::vec3(0.5f*size, 0.5f*size, -0.5f*size);

				quad[4] = glm::vec3(-0.5f*size, -0.5f*size, 0.5f*size);
				quad[5] = glm::vec3(0.5f*size, -0.5f*size, 0.5f*size);
				quad[6] = glm::vec3(-0.5f*size, 0.5f*size, 0.5f*size);
				quad[7] = glm::vec3(0.5f*size);

				float minMaxDist = INFINITY;

				for(uint32_t i=0; i < indices.size(); i += 3)
				{
					triangle[0] = vertices[indices[i]] - centerPoint;
					triangle[1] = vertices[indices[i + 1]] - centerPoint;
					triangle[2] = vertices[indices[i + 2]] - centerPoint;

					float minDist = GJK::getMinDistance(quad, triangle);
					float maxDist = minDist > 0.00001f ? GJK::getMaxDistance(quad, triangle) : INFINITY;
					minMaxDist = glm::min(minMaxDist, maxDist);

					if(minDist <= minMaxDist)
					{
						triangles.push_back(std::make_pair(minDist, i));
					}
				}

				std::sort(triangles.begin(), triangles.end());

				int s=0;
				for(; s < triangles.size() && triangles[s].first <= minMaxDist; s++);

				triangles.resize(s);

				for(const std::pair<float, uint32_t>& p : triangles)
				{
					newIndices.push_back(indices[p.second]);
					newIndices.push_back(indices[p.second + 1]);
					newIndices.push_back(indices[p.second + 2]);
				}

				mModelRenderer->setIndexData(newIndices);

				SPDLOG_INFO("Number of selected triangles: {}", triangles.size());
			}
		}
		else
		{
			if(lastSelectZone)
			{
				mSelectionCube->drawWireframe(false);
				mModelRenderer->setIndexData(mMesh.value().getIndices());
			}
		}
	}
private:
	std::unique_ptr<SdfPlaneShader> mPlaneShader;
	std::unique_ptr<NormalsSplitPlaneShader> mCubeShader;

	std::shared_ptr<RenderMesh> mModelRenderer;
	std::shared_ptr<RenderMesh> mPlaneRenderer; 
	std::shared_ptr<RenderMesh> mCubeRenderer;
	std::shared_ptr<RenderMesh> mSelectionCube;

	glm::mat4x4 mGizmoStartMatrix;
	glm::mat4x4 mGizmoMatrix;

	bool mSelectZone = false;
	uint32_t mSelectedDepth = 0;
	BoundingBox mSelectArea;
	float mGridSize;

	std::optional<Mesh> mMesh;

	// Input parameters
    std::optional<std::string> mModelPath;
    std::optional<float> mCellSize;
	std::optional<uint32_t> mDepth;
	std::optional<std::string> mSdfPath;
	bool mNormalizeModel;
};

int main(int argc, char** argv)
{
	spdlog::set_pattern("[%^%l%$] [%s:%#] %v");

    args::ArgumentParser parser("UniformGridViwer reconstructs and draws a uniform grid sdf");
    args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
    args::Positional<std::string> modelPathArg(parser, "model_path", "The model path");
	args::ValueFlag<std::string> sdfPathArg(parser, "sdf_path", " The precalculated sdf to visualize", {"in"});
	args::Flag normalizeBBArg(parser, "normalize_model", "Normalize the model coordinates", {'n', "normalize"});
	args::ValueFlag<float> cellSizeArg(parser, "cell_size", "The voxel size of the voxelization", {"cell_size"});
    args::ValueFlag<uint32_t> depthArg(parser, "depth", "The octree subdivision depth", {"depth"});

    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch(args::Help)
    {
        std::cerr << parser;
        return 0;
    }

	const std::string defaultModel = "../models/quad.ply";

	if(sdfPathArg)
	{
		MyScene scene(
			args::get(sdfPathArg),
			(modelPathArg) ? std::optional<std::string>(args::get(modelPathArg)) : std::optional<std::string>(),
			(normalizeBBArg) ? true : false
		);
		MainLoop loop;
		loop.start(scene);
	}
	else if(cellSizeArg)
	{
		MyScene scene(
			(modelPathArg) ? args::get(modelPathArg) : defaultModel,
			(cellSizeArg) ? args::get(cellSizeArg) : 0.1f
		);
		MainLoop loop;
		loop.start(scene);
	}
	else
	{
		MyScene scene(
			(modelPathArg) ? args::get(modelPathArg) : defaultModel,
			(depthArg) ? args::get(depthArg) : 6
		);
		MainLoop loop;
		loop.start(scene);
	}
}