#include <iostream>
#include <random>
#include <algorithm>
#include <optional>
#include "SdfLib/utils/Mesh.h"
#include "SdfLib/utils/TriangleUtils.h"
#include "SdfLib/utils/PrimitivesFactory.h"
#include "SdfLib/utils/Timer.h"
#include "SdfLib/utils/GJK.h"
#include "SdfLib/OctreeSdfUtils.h"
#include "InfluenceRegionCreator.h"
#include "SdfLib/TrianglesInfluence.h"
#include "SdfLib/InterpolationMethods.h"
#include "render_engine/MainLoop.h"
#include "render_engine/NavigationCamera.h"
#include "render_engine/RenderMesh.h"
#include "render_engine/shaders/NormalsShader.h"
#include "render_engine/shaders/SdfPlaneShader.h"
#include "render_engine/shaders/SdfOctreePlaneShader.h"
#include "render_engine/shaders/BasicShader.h"
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
	enum SdfFormat
	{
		OCTREE,
		GRID
	};

    MyScene(std::string modelPath, float cellSize) : mSdfFormat(SdfFormat::GRID), mModelPath(modelPath), mCellSize(cellSize), mNormalizeModel(true) {}
	MyScene(std::string modelPath, uint32_t maxDepth) : mSdfFormat(SdfFormat::GRID), mModelPath(modelPath), mDepth(maxDepth), mNormalizeModel(true) {}
	MyScene(std::string modelPath, uint32_t maxDepth, uint32_t startDepth,
			float terminationThreshold = 1e-3,
			OctreeSdf::TerminationRule terminationRule = OctreeSdf::TerminationRule::TRAPEZOIDAL_RULE) : 
		mSdfFormat(SdfFormat::OCTREE), 
		mModelPath(modelPath), 
		mDepth(maxDepth), 
		mStartDepth(startDepth),
		mTerminationThreshold(terminationThreshold),
		mTerminationRule(terminationRule),
		mNormalizeModel(true) {}
	MyScene(std::string sdfPath, std::optional<std::string> modelPath = std::optional<std::string>(), bool normalizeModel = false) :
		mSdfPath(sdfPath),
		mModelPath(modelPath),
		mNormalizeModel(normalizeModel) {}

	glm::mat4 invTransform;
	OctreeSdf octreeSdf;
	void start() override
	{
		Window::getCurrentWindow().setBackgroudColor(glm::vec4(0.9, 0.9, 0.9, 1.0));
		// Window::getCurrentWindow().setBackgroudColor(glm::vec4(1.0, 1.0, 1.0, 1.0));

		// Create camera
		std::shared_ptr<NavigationCamera> camera;
		{
			camera = std::make_shared<NavigationCamera>();
			camera->start();
			setMainCamera(camera);
			addSystem(camera);
		}

		// Create unifrom grid
		UniformGridSdf sdfGrid;

		BoundingBox sdfBB;
		BoundingBox viewBB;

		if(mModelPath.has_value())
		{
			mMesh = Mesh(mModelPath.value());
			if(mNormalizeModel)
			{
				// Normalize model units
				const glm::vec3 boxSize = mMesh.value().getBoundingBox().getSize();
				glm::vec3 center = mMesh.value().getBoundingBox().getCenter();
				mMesh.value().applyTransform(glm::scale(glm::mat4(1.0), glm::vec3(2.0f/glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z))) *
											 glm::translate(glm::mat4(1.0), -mMesh.value().getBoundingBox().getCenter()));

				invTransform = glm::inverse(glm::scale(glm::mat4(1.0), glm::vec3(2.0f/glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z))) *
											 glm::translate(glm::mat4(1.0), -center));
				SPDLOG_INFO("Center is {}, {}, {}", center.x, center.y, center.z);
			}
		}

		if(mSdfPath.has_value())
		{
			std::unique_ptr<SdfFunction> sdfFunc = SdfFunction::loadFromFile(mSdfPath.value());
			if(!sdfFunc)
			{
				SPDLOG_ERROR("Could not find the path {}", mSdfPath.value());
				return;
			}
			else if(sdfFunc->getFormat() == SdfFunction::SdfFormat::EXACT_OCTREE)
			{
				SPDLOG_ERROR("Exact octrees are not supported.");
				return;
			}

			switch(sdfFunc->getFormat())
			{
				case SdfFunction::SdfFormat::GRID:
					mSdfFormat = SdfFormat::GRID;
					sdfGrid = std::move(*reinterpret_cast<UniformGridSdf*>(sdfFunc.get()));
					sdfBB = sdfGrid.getGridBoundingBox();
					viewBB = sdfGrid.getGridBoundingBox();
					mGridSize = sdfGrid.getGridCellSize();
					break;
				case SdfFunction::SdfFormat::OCTREE:
					mSdfFormat = SdfFormat::OCTREE;
					octreeSdf = std::move(*reinterpret_cast<OctreeSdf*>(sdfFunc.get()));
					sdfBB = octreeSdf.getGridBoundingBox();
					viewBB = octreeSdf.getGridBoundingBox();
					mGridSize = sdfBB.getSize().x * glm::pow(0.5f, octreeSdf.getOctreeMaxDepth());
					break;
				default:
					assert(false);
					return;
			}
		}
		else
		{
			assert(mModelPath.has_value());
			BoundingBox box = mMesh.value().getBoundingBox();
			const glm::vec3 modelBBSize = box.getSize();
			box.addMargin(0.12f * glm::max(glm::max(modelBBSize.x, modelBBSize.y), modelBBSize.z));
			Timer timer; timer.start();
			switch(mSdfFormat)
			{
				case SdfFormat::GRID:
					sdfGrid = (mCellSize.has_value()) ? 
							UniformGridSdf(mMesh.value(), box, mCellSize.value(), UniformGridSdf::InitAlgorithm::OCTREE) :
							UniformGridSdf(mMesh.value(), box, mDepth.value(), UniformGridSdf::InitAlgorithm::OCTREE);
					sdfBB = sdfGrid.getGridBoundingBox();
					viewBB = sdfGrid.getGridBoundingBox();
					mGridSize = sdfGrid.getGridCellSize();
					break;
				case SdfFormat::OCTREE:
					octreeSdf = OctreeSdf(mMesh.value(), box, mDepth.value(), mStartDepth.value(), mTerminationRule.value());
					sdfBB = octreeSdf.getGridBoundingBox();
					viewBB = octreeSdf.getGridBoundingBox();
					mGridSize = sdfBB.getSize().x * glm::pow(0.5f, mDepth.value());
					break;
			}
			SPDLOG_INFO("Uniform grid generation time: {}s", timer.getElapsedSeconds());
		}

		{
			glm::vec3 pos = glm::vec3(invTransform * glm::vec4(octreeSdf.getGridBoundingBox().min, 1.0f));
			SPDLOG_INFO("Min Point: {}, {}, {}", pos.x, pos.y, pos.z);

			pos = glm::vec3(invTransform * glm::vec4(octreeSdf.getGridBoundingBox().max, 1.0f));
			SPDLOG_INFO("Max Point: {}, {}, {}", pos.x, pos.y, pos.z);
		}

		mSelectArea = viewBB;

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

			switch(mSdfFormat)
			{
				case SdfFormat::GRID:
					mGridPlaneShader = std::unique_ptr<SdfPlaneShader> (new SdfPlaneShader(sdfGrid, viewBB));
					mPlaneRenderer->setShader(mGridPlaneShader.get());
					break;
				case SdfFormat::OCTREE:
					mOctreePlaneShader = std::unique_ptr<SdfOctreePlaneShader> (new SdfOctreePlaneShader(octreeSdf, viewBB));
					mPlaneRenderer->setShader(mOctreePlaneShader.get());
					break;
			}
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
				glm::scale(glm::mat4(1.0f), viewBB.getSize() / sdfBB.getSize()) *
				glm::translate(glm::mat4(1.0f), -sdfBB.getCenter())
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
				glm::scale(glm::mat4(1.0f), viewBB.getSize() / sdfBB.getSize()) *
				glm::translate(glm::mat4(1.0f), -sdfBB.getCenter())
			);
			#ifndef SDFLIB_PRINT_STATISTICS
				meshNormals->callDrawGui = false;
			#endif
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
			mSelectionCube->callDraw = false;
			mSelectionCube->drawSurface(false);
			mSelectionCube->drawWireframe(true);
			addSystem(mSelectionCube);
		}

		// Create colored mesh
		{
			mColoredModelRenderer = std::make_shared<RenderMesh>();
			mColoredModelRenderer->start();
			mColoredModelRendererBufferId =
				mColoredModelRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
														RenderMesh::VertexParameterLayout(GL_FLOAT, 3),
														RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
													}, nullptr, 0);

			mColoredModelRenderer->setDrawMode(GL_TRIANGLES);
			mColoredModelRenderer->setShader(Shader<ColorsShader>::getInstance());
			mColoredModelRenderer->drawSurface(true);
			mColoredModelRenderer->drawWireframe(true);
			mColoredModelRenderer->setTransform(
				glm::translate(glm::mat4(1.0f), viewBB.getCenter()) * 
				glm::scale(glm::mat4(1.0f), viewBB.getSize() / sdfBB.getSize()) *
				glm::translate(glm::mat4(1.0f), -sdfBB.getCenter())
			);
			//mColoredModelRenderer->callDrawGui = false;
			#ifndef SDFLIB_PRINT_STATISTICS
				mColoredModelRenderer->callDrawGui = false;
			#endif
			mColoredModelRenderer->callDraw = false;
			addSystem(mColoredModelRenderer);
		}

		// Influence mesh
		auto addInfluenceRegion = [&] (std::shared_ptr<RenderMesh>& renderMesh, 
									   uint32_t& vertBufferId, uint32_t& normBufferId)
		{
			renderMesh = std::make_shared<RenderMesh>();
			renderMesh->start();
			vertBufferId =
				renderMesh->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
													RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
												}, nullptr, 0);

			normBufferId =
				renderMesh->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
														RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
													}, nullptr, 0);

			renderMesh->setDrawMode(GL_TRIANGLES);
			renderMesh->setShader(Shader<NormalsShader>::getInstance());
			renderMesh->drawSurface(true);
			renderMesh->drawWireframe(true);
			renderMesh->setTransform(
				glm::translate(glm::mat4(1.0f), viewBB.getCenter()) * 
				glm::scale(glm::mat4(1.0f), viewBB.getSize() / sdfBB.getSize()) *
				glm::translate(glm::mat4(1.0f), -sdfBB.getCenter())
			);

			renderMesh->callDraw = false;
		};

		addInfluenceRegion(mInfluenceRegion, mInfluenceRegionBufferIdVert, mInfluenceRegionBufferIdNorm);
		mInfluenceRegion->systemName = "Influence region";
		#ifndef SDFLIB_PRINT_STATISTICS
			mInfluenceRegion->callDrawGui = false;
		#endif
		addSystem(mInfluenceRegion);

		mGreenNormalsShader = std::unique_ptr<NormalsShader>(new NormalsShader());
		mGreenNormalsShader->setDrawColor(glm::vec3(0.0f, 0.8f, 0.0f));

		addInfluenceRegion(mOptimalInfluenceRegion, mOptimalInfluenceRegionBufferIdVert, mOptimalInfluenceRegionBufferIdNorm);
		mOptimalInfluenceRegion->systemName = "Optimal influence region";
		mOptimalInfluenceRegion->setShader(mGreenNormalsShader.get());
		#ifndef SDFLIB_PRINT_STATISTICS
			mOptimalInfluenceRegion->callDrawGui = false;
		#endif
		addSystem(mOptimalInfluenceRegion);

		// Move camera in the z-axis to be able to see the whole model
		{
			float zMovment = 0.5f * glm::max(sdfBB.getSize().x, sdfBB.getSize().y) / glm::tan(glm::radians(0.5f * camera->getFov()));
			camera->setPosition(sdfBB.getCenter() + glm::vec3(0.0f, 0.0f, 0.1f * sdfBB.getSize().z + zMovment));
		}
	}

	void update(float deltaTime) override
	{
		Scene::update(deltaTime);

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Text("Scene Options");

		bool drawGrid;
		bool drawIsolines;
		switch(mSdfFormat)
		{
			case SdfFormat::GRID:
				drawGrid = mGridPlaneShader->isDrawingGrid();
				drawIsolines = mGridPlaneShader->isDrawingIsolines();
				break;
			case SdfFormat::OCTREE:
				drawGrid = mOctreePlaneShader->isDrawingGrid();
				drawIsolines = mOctreePlaneShader->isDrawingIsolines();
				break;
		}

		ImGui::Checkbox("Print grid", &drawGrid);

		ImGui::Checkbox("Print Isolines", &drawIsolines);

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
		else if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_4))
		{
			mGizmoMatrix = glm::rotate(glm::mat4x4(1.0f), glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f)) * mGizmoStartMatrix;
		}
		else if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_5))
		{
			mGizmoMatrix = glm::rotate(glm::mat4x4(1.0f), glm::radians(-90.0f), glm::vec3(-1.0f, 0.0f, 0.0f)) * mGizmoStartMatrix;
		}
		else if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_6))
		{
			mGizmoMatrix = glm::rotate(glm::mat4x4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, -1.0f, 0.0f)) * mGizmoStartMatrix;
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
		
		switch(mSdfFormat)
		{
			case SdfFormat::GRID:
				mGridPlaneShader->setNormal(planeNormal);
				mGridPlaneShader->drawGrid(drawGrid);
				mGridPlaneShader->drawIsolines(drawIsolines);
				break;
			case SdfFormat::OCTREE:
				mOctreePlaneShader->setNormal(planeNormal);
				mOctreePlaneShader->drawGrid(drawGrid);
				mOctreePlaneShader->drawIsolines(drawIsolines);
				break;
		}


		// Selection option
		if(mMesh.has_value())
		{
			bool lastSelectZone = mSelectZone;
			#ifdef SDFLIB_PRINT_STATISTICS
				ImGui::Checkbox("Visualize zone", &mSelectZone);
			#endif
			if(mSelectZone) 
			{
				const char* selectionAlgorithms[] = {
					"Basic",
					"Precise",
					"Per vertex",
					"Per node region"
				};

				if(ImGui::BeginCombo("Selection Algorithm", mSelectionAlgorithm))
				{
					for (int n = 0; n < IM_ARRAYSIZE(selectionAlgorithms); n++)
					{
						bool is_selected = mSelectionAlgorithm == selectionAlgorithms[n];
						if (ImGui::Selectable(selectionAlgorithms[n], is_selected))
							std::strcpy(mSelectionAlgorithm, selectionAlgorithms[n]);
						if (is_selected)
							ImGui::SetItemDefaultFocus();
					}

					ImGui::EndCombo();
				}

				ImGui::Checkbox("Draw influence zone", &mDrawInfluenceZone);
				ImGui::Checkbox("Draw optimal influence zone", &mDrawOptimalZone);
				ImGui::InputInt("Draw influence subdivisions", reinterpret_cast<int*>(&mInfluenceZoneSubdivisions));
				ImGui::Checkbox("Print triangles influence", &mPrintTrianglesInfluence);
				ImGui::Checkbox("Print node error", &mPrintNodeError);
				//ImGui::InputInt("Selected triangle", reinterpret_cast<int*>(&mSelectedTriangle));
				if(mPrintTrianglesInfluence)
				{
					ImGui::InputInt("Num samples for influence computation", reinterpret_cast<int*>(&mInfluenceNumSamples));
					ImGui::Checkbox("Print if it has some influence", &mPrintIfSomeInfluence);
				}
				ImGui::InputInt("Select depth: ", reinterpret_cast<int*>(&mSelectedDepth));
				if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_N))
				{
					// Get selected point and selected region
					glm::vec3 cameraPos = getMainCamera()->getPosition();
					glm::vec3 cameraDir(0.0f);
					{
						glm::vec2 w = Window::getCurrentWindow().getWindowSize();
						glm::vec2 m = Window::getCurrentWindow().getMousePosition();
						float realWidth = glm::tan(glm::radians(getMainCamera()->getFov() / 2.0f));
						glm::vec3 wPos (
							glm::clamp(m.x / w.x, 0.0f, 1.0f) * getMainCamera()->getRatio() * realWidth * 2.0f - realWidth * getMainCamera()->getRatio(),
							-glm::clamp(m.y / w.y, 0.0f, 1.0f) * realWidth * 2.0f + realWidth,
							-1.0f
						);

						cameraDir = glm::vec3(glm::inverse(getMainCamera()->getViewMatrix()) * glm::vec4(wPos, 0.0f));
					}
					float t = glm::dot(planePoint - cameraPos, planeNormal) / glm::dot(cameraDir, planeNormal);
					glm::vec3 selPoint = cameraPos + cameraDir * t;
					const float size = mGridSize * static_cast<float>(1 << mSelectedDepth);

					glm::vec3 centerPoint = mSelectArea.min + glm::floor((selPoint - mSelectArea.min) / size) * size + 0.5f * size;

					// Position cube
					mSelectionCube->setTransform( 
						glm::translate(glm::mat4(1.0f), centerPoint) * 
						glm::scale(glm::mat4(1.0f), glm::vec3(size))
					);
					mSelectionCube->callDraw = true;

					// Serach triangles influencing the selected zone
					const std::vector<uint32_t>& indices = mMesh.value().getIndices();
					const std::vector<glm::vec3>& vertices = mMesh.value().getVertices();

					std::vector<uint32_t> triangles;

					const std::array<glm::vec3, 8> childrens = 
					{
						glm::vec3(-1.0f, -1.0f, -1.0f),
						glm::vec3(1.0f, -1.0f, -1.0f),
						glm::vec3(-1.0f, 1.0f, -1.0f),
						glm::vec3(1.0f, 1.0f, -1.0f),

						glm::vec3(-1.0f, -1.0f, 1.0f),
						glm::vec3(1.0f, -1.0f, 1.0f),
						glm::vec3(-1.0f, 1.0f, 1.0f),
						glm::vec3(1.0f, 1.0f, 1.0f)
					};

					for(uint32_t i=0; i < 8; i++)
					{
						const glm::vec3 pos = glm::vec3(invTransform * glm::vec4((centerPoint + 0.5f * childrens[i] * size), 1.0f));
						std::cout << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
					}

					const std::array<glm::vec3, 19> nodeSamplePoints =
					{
						glm::vec3(0.0f, -1.0f, -1.0f),
						glm::vec3(-1.0f, 0.0f, -1.0f),
						glm::vec3(0.0f, 0.0f, -1.0f),
						glm::vec3(1.0f, 0.0f, -1.0f),
						glm::vec3(0.0f, 1.0f, -1.0f),

						glm::vec3(-1.0f, -1.0f, 0.0f),
						glm::vec3(0.0f, -1.0f, 0.0f),
						glm::vec3(1.0f, -1.0f, 0.0f),
						glm::vec3(-1.0f, 0.0f, 0.0f),
						glm::vec3(0.0f),
						glm::vec3(1.0f, 0.0f, 0.0f),
						glm::vec3(-1.0f, 1.0f, 0.0f),
						glm::vec3(0.0f, 1.0f, 0.0f),
						glm::vec3(1.0f, 1.0f, 0.0f),

						glm::vec3(0.0f, -1.0f, 1.0f),
						glm::vec3(-1.0f, 0.0f, 1.0f),
						glm::vec3(0.0f, 0.0f, 1.0f),
						glm::vec3(1.0f, 0.0f, 1.0f),
						glm::vec3(0.0f, 1.0f, 1.0f),
					};

					std::shared_ptr<Mesh> influenceRegionMesh = (mDrawInfluenceZone)
						? PrimitivesFactory::getIsosphere(mInfluenceZoneSubdivisions)
						: nullptr;

					std::vector<TriangleUtils::TriangleData> trianglesInfo = TriangleUtils::calculateMeshTriangleData(mMesh.value());

					float maxMinDist = 0.0f;

					{
						SPDLOG_INFO("Center point: {}, {}, {}", centerPoint.x, centerPoint.y, centerPoint.z);

						auto getRandomSample = [&] () -> glm::vec3
						{
							glm::vec3 p =  glm::vec3(static_cast<float>(rand())/static_cast<float>(RAND_MAX),
												static_cast<float>(rand())/static_cast<float>(RAND_MAX),
												static_cast<float>(rand())/static_cast<float>(RAND_MAX));
							return centerPoint + (p - 0.5f) * size;
						};

						auto getNearestTriangle = [&](glm::vec3 point)
						{
							uint32_t tIndex;
							float minIndexDist = INFINITY;
							for(uint32_t t=0; t < trianglesInfo.size(); t++)
							{
								float dist = TriangleUtils::getSqDistPointAndTriangle(point, trianglesInfo[t]);
								if(dist < minIndexDist)
								{
									tIndex = t;
									minIndexDist = dist;
								}
							}

							return tIndex;
						};

						auto getDistance = [&](glm::vec3 point)
						{
							uint32_t tIndex;
							float minIndexDist = INFINITY;
							for(uint32_t t=0; t < trianglesInfo.size(); t++)
							{
								float dist = TriangleUtils::getSqDistPointAndTriangle(point, trianglesInfo[t]);
								if(dist < minIndexDist)
								{
									tIndex = t;
									minIndexDist = dist;
								}
							}

							return TriangleUtils::getSignedDistPointAndTriangle(point, trianglesInfo[tIndex]);
						};

						double sdfRMSE = 0.0f;
						double sdfMAE = 0.0f;
						for(uint32_t s=0; s < 64; s++)
						{
							glm::vec3 point = getRandomSample();
							float exactDist = getDistance(point);
							sdfRMSE += static_cast<double>(pow2((octreeSdf.getDistance(point) - exactDist)));
							sdfMAE += static_cast<double>(glm::abs(octreeSdf.getDistance(point) - exactDist));
						}

						SPDLOG_INFO("MC RMSE: {}", glm::sqrt(sdfRMSE / static_cast<double>(64)));
						SPDLOG_INFO("MC MAE: {}", sdfMAE / static_cast<double>(64));

						std::array<std::array<float, TriCubicInterpolation::VALUES_PER_VERTEX>, 8> vertexValues; 
						for(uint32_t s=0; s < 8; s++)
						{
							const glm::vec3 p = centerPoint + childrens[s] * 0.5f * size;
							TriCubicInterpolation::calculatePointValues(p, getNearestTriangle(p), mMesh.value(), trianglesInfo, vertexValues[s]);
						}

						const std::vector<uint32_t> emptyArray;
						std::array<float, TriCubicInterpolation::NUM_COEFFICIENTS> coefficients;
						TriCubicInterpolation::calculateCoefficients(vertexValues, size, emptyArray, mMesh.value(), trianglesInfo, coefficients);

						std::array<std::array<float, TriCubicInterpolation::VALUES_PER_VERTEX>, 19> middlePoints;
						for(uint32_t s=0; s < 19; s++)
						{
							middlePoints[s][0] = getDistance(centerPoint + nodeSamplePoints[s] * 0.5f * size);
						}

						SPDLOG_INFO("Trapezoid RMSE: {}", glm::sqrt(estimateErrorFunctionIntegralByTrapezoidRule<TriCubicInterpolation>(coefficients, middlePoints)));
						SPDLOG_INFO("Max Face Trapezoid RMSE: {}", glm::sqrt(estimateFaceErrorFunctionIntegralByTrapezoidRule<TriCubicInterpolation>(coefficients, middlePoints)));
						SPDLOG_INFO("Max RMSE: {}", glm::sqrt(estimateMaxError<TriCubicInterpolation>(coefficients, middlePoints)));
					}

					// Serach triangles influencing the zone
					if(false) {
						typedef TriLinearInterpolation Inter;

						std::vector<uint32_t> inTriangles(indices.size()/3);
						for(uint32_t i=0; i < indices.size()/3; i++)
						{
							inTriangles[i] = i;
						}

						std::vector<uint32_t> newTriangles;
						// Ground truth process
						{
							std::vector<uint32_t> outTriangles;
							std::array<PreciseTrianglesInfluence<Inter>::VertexInfo, 8> verticesInfo;
							std::array<std::array<float, Inter::VALUES_PER_VERTEX>, 8> minDistToVertices;
							std::array<float, Inter::NUM_COEFFICIENTS> nullArray;

							PreciseTrianglesInfluence<Inter>().calculateVerticesInfo(centerPoint, 0.5f * size,
																					inTriangles,
																					childrens, 0u, nullArray,
																					minDistToVertices, verticesInfo,
																					mMesh.value(), trianglesInfo);
							PreciseTrianglesInfluence<Inter>().filterTriangles(centerPoint, 0.5f * size, 
																		inTriangles, outTriangles,
																		minDistToVertices, verticesInfo,
																		mMesh.value(), trianglesInfo);

							for(const uint32_t idx : outTriangles)
							{
								std::cout << idx << ", ";
							}

							std::cout << std::endl;

							newTriangles = std::move(outTriangles);
							triangles = newTriangles;

							SPDLOG_INFO("PreciseTrianglesInfluence num selected triangles: {}", newTriangles.size());
						}

						std::sort(newTriangles.begin(), newTriangles.end());

						auto printErrors = [&](std::string&& algorithmName, std::vector<uint32_t>& selectedTriangles)
						{
							uint32_t numErrors = glm::abs(newTriangles.size() - selectedTriangles.size());
							std::sort(selectedTriangles.begin(), selectedTriangles.end());
							uint32_t falsePositive = 0;
							uint32_t falseNegative = 0;
							uint32_t i, j;
							for(i=0, j=0; i < selectedTriangles.size() && j < newTriangles.size();)
							{
								if(selectedTriangles[i] < newTriangles[j]) 
								{
									falsePositive++;
									i++;
								}
								else if(selectedTriangles[i] > newTriangles[j]) 
								{
									falseNegative++;
									j++;
								}
								else
								{
									i++; j++;
								}
							}

							if(i < selectedTriangles.size())
							{
								falsePositive += selectedTriangles.size() - i;
							}
							else if(j < newTriangles.size())
							{
								falseNegative += newTriangles.size() - j;
							}

							SPDLOG_INFO("{} results // FP: {}, FN: {}", algorithmName, falsePositive, falseNegative);
							SPDLOG_INFO("{} num selected triangles: {}", algorithmName, selectedTriangles.size());
						};

						{
							std::vector<uint32_t> outTriangles;
							std::array<std::array<float, Inter::VALUES_PER_VERTEX>, 8> verticesDist;
							std::array<BasicTrianglesInfluence<Inter>::VertexInfo, 8> verticesInfo;
							std::array<float, Inter::NUM_COEFFICIENTS> nullArray;
							BasicTrianglesInfluence<Inter>().calculateVerticesInfo(centerPoint, 0.5f * size,
																			inTriangles, 
																			childrens, 0u, nullArray,
																			verticesDist, verticesInfo,
																			mMesh.value(), trianglesInfo);

							for(const auto& d : verticesDist) maxMinDist = glm::max(maxMinDist, d[0]);
							BasicTrianglesInfluence<Inter>().filterTriangles(centerPoint, 0.5f * size, 
																			inTriangles, outTriangles,
																			verticesDist, verticesInfo,
																			mMesh.value(), trianglesInfo);

							printErrors("BasicTrianglesInfluence", outTriangles);

							for(const uint32_t idx : outTriangles)
							{
								std::cout << idx << ", ";
							}

							std::cout << std::endl;

							if(std::strcmp(mSelectionAlgorithm, "Basic") == 0)
							{
								triangles = std::move(outTriangles);
							}
						}

						{
							std::vector<uint32_t> outTriangles;
							std::array<std::array<float, Inter::VALUES_PER_VERTEX>, 8> verticesDist;
							std::array<PerVertexTrianglesInfluence<1, Inter>::VertexInfo, 8> verticesInfo;
							std::array<float, Inter::NUM_COEFFICIENTS> nullArray;
							PerVertexTrianglesInfluence<1, Inter>().calculateVerticesInfo(centerPoint, 0.5f * size,
																				inTriangles,
																				childrens, 0u, nullArray,
																				verticesDist, verticesInfo,
																				mMesh.value(), trianglesInfo);
							PerVertexTrianglesInfluence<1, Inter>().filterTriangles(centerPoint, 0.5f * size, 
																			inTriangles, outTriangles,
																			verticesDist, verticesInfo,
																			mMesh.value(), trianglesInfo);

							printErrors("PerVertexTrianglesInfluence", outTriangles);

							if(std::strcmp(mSelectionAlgorithm, "Per vertex") == 0)
							{
								triangles = std::move(outTriangles);
							}
						}

						{
							std::vector<uint32_t> outTriangles;
							std::array<std::array<float, Inter::VALUES_PER_VERTEX>, 8> verticesDist;
							std::array<PerNodeRegionTrianglesInfluence<Inter>::VertexInfo, 8> verticesInfo;
							std::array<float, Inter::NUM_COEFFICIENTS> nullArray;
							PerNodeRegionTrianglesInfluence<Inter>().calculateVerticesInfo(centerPoint, 0.5f * size,
																				inTriangles,
																				childrens, 0u, nullArray,
																				verticesDist, verticesInfo,
																				mMesh.value(), trianglesInfo);
							PerNodeRegionTrianglesInfluence<Inter>().filterTriangles(centerPoint, 0.5f * size, 
																			inTriangles, outTriangles,
																			verticesDist, verticesInfo,
																			mMesh.value(), trianglesInfo);

							printErrors("PerNodeRegionTrianglesInfluence", outTriangles);

							for(const uint32_t idx : outTriangles)
							{
								std::cout << idx << ", ";
							}

							std::cout << std::endl;

							if(std::strcmp(mSelectionAlgorithm, "Per node region") == 0)
							{
								triangles = std::move(outTriangles);
							}
						}
					}
					
					if(mDrawInfluenceZone)
					{
						std::vector<std::pair<glm::vec3, float>> sphereQuad(8);
						for(uint32_t i=0; i < 8; i++)
						{
							sphereQuad[i] = std::make_pair(childrens[i] * 0.5f * size, maxMinDist);
						}
						InfluenceRegionCreator::createConvexHull(*influenceRegionMesh, centerPoint, sphereQuad);

						mInfluenceRegion->setIndexData(influenceRegionMesh->getIndices());
						mInfluenceRegion->setVertexData(mInfluenceRegionBufferIdVert, 
															influenceRegionMesh->getVertices().data(),
															influenceRegionMesh->getVertices().size());

						mInfluenceRegion->setVertexData(mInfluenceRegionBufferIdNorm, 
															influenceRegionMesh->getNormals().data(),
															influenceRegionMesh->getNormals().size());
						mInfluenceRegion->callDraw = true;
					} else mInfluenceRegion->callDraw = false;


					// Compute optimal region
					if(mDrawOptimalZone)
					{
						std::shared_ptr<Mesh> optimalInfluenceRegionMesh = PrimitivesFactory::getIsosphere(mInfluenceZoneSubdivisions);

						InfluenceRegionCreator::createOptimalRegion(*optimalInfluenceRegionMesh, centerPoint, 0.5f * size, triangles, trianglesInfo);

						mOptimalInfluenceRegion->setIndexData(optimalInfluenceRegionMesh->getIndices());
						mOptimalInfluenceRegion->setVertexData(mOptimalInfluenceRegionBufferIdVert, 
															optimalInfluenceRegionMesh->getVertices().data(),
															optimalInfluenceRegionMesh->getVertices().size());

						mOptimalInfluenceRegion->setVertexData(mOptimalInfluenceRegionBufferIdNorm, 
															optimalInfluenceRegionMesh->getNormals().data(),
															optimalInfluenceRegionMesh->getNormals().size());

						mOptimalInfluenceRegion->callDraw = true;
					} else mOptimalInfluenceRegion->callDraw = false;

					std::vector<std::pair<TriangleUtils::TriangleData, uint32_t>> trianglesData;
					trianglesData.reserve(triangles.size());
					std::vector<uint32_t> newIndices;
					newIndices.reserve(3 * triangles.size());

					for(const uint32_t& idx : triangles)
					{
						newIndices.push_back(indices[3 * idx]);
						newIndices.push_back(indices[3 * idx + 1]);
						newIndices.push_back(indices[3 * idx + 2]);

						trianglesData.push_back(std::make_pair(TriangleUtils::TriangleData(
							vertices[indices[3 * idx]],
							vertices[indices[3 * idx + 1]],
							vertices[indices[3 * idx + 2]]
						), 0));
					}

					mModelRenderer->setIndexData(newIndices);

					auto getRandomVec3 = [&] () -> glm::vec3
					{
						glm::vec3 p =  glm::vec3(static_cast<float>(rand())/static_cast<float>(RAND_MAX),
											static_cast<float>(rand())/static_cast<float>(RAND_MAX),
											static_cast<float>(rand())/static_cast<float>(RAND_MAX));
						return centerPoint + (p - 0.5f) * size;
					};

					if(mPrintTrianglesInfluence)
					{
						for(uint32_t s=0; s < mInfluenceNumSamples; s++)
						{
							glm::vec3 sample = getRandomVec3();
							float minDist = INFINITY;
							uint32_t nearestTriangle = 0;
							for(uint32_t t=0; t < trianglesData.size(); t++)
							{
								const float dist = 
									TriangleUtils::getSqDistPointAndTriangle(sample, trianglesData[t].first);
								if (dist < minDist)
								{
									nearestTriangle = t;
									minDist = dist;
								}
							}

							trianglesData[nearestTriangle].second += 1;
						}

						uint32_t maxValue = 0;
						for(const std::pair<TriangleUtils::TriangleData, uint32_t>& p : trianglesData)
						{
							maxValue = glm::max(maxValue, p.second);
						}
					
						std::vector<glm::vec3> newVertices;
						newVertices.reserve(6 * triangles.size());
						const float maxValueF = static_cast<float>(maxValue);
						std::array<glm::vec3, 4> colorsPalette = {
							glm::vec3(1.0f, 1.0f, 1.0f), 
							glm::vec3(1.0f, 1.0f, 0.0f), 
							glm::vec3(1.0f, 0.5f, 0.0f), 
							glm::vec3(1.0f, 0.0f, 0.0f)
						};

						for(uint32_t t=0; t < trianglesData.size(); t++)
						{
							// Calculate triangle color
							glm::vec3 color(0.0);
							if(mPrintIfSomeInfluence)
							{
								color = (trianglesData[t].second > 0) ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(1.0f);
							}
							else
							{
								const float value = static_cast<float>(trianglesData[t].second) / maxValueF;
								const float colorIndex = glm::clamp(value * static_cast<float>(colorsPalette.size() - 1), 
																	0.0f, static_cast<float>(colorsPalette.size() - 1) - 0.01f);
								color = glm::mix(colorsPalette[static_cast<int>(colorIndex)], 
																colorsPalette[static_cast<int>(colorIndex) + 1],
																glm::fract(colorIndex));
							}

							// if(t == mSelectedTriangle) color = glm::vec3(0.0f, 0.0f, 1.0f);

							// Add triangle
							const uint32_t& idx = triangles[t];
							if(trianglesData[t].second > 0) std::cout << idx << ", ";
							newVertices.push_back(vertices[indices[3 * idx]]);
							newVertices.push_back(color);
							newVertices.push_back(vertices[indices[3 * idx + 1]]);
							newVertices.push_back(color);
							newVertices.push_back(vertices[indices[3 * idx + 2]]);
							newVertices.push_back(color);
						}

						std::cout << std::endl;

						mColoredModelRenderer->setVertexData(mColoredModelRendererBufferId, 
															newVertices.data(), 
															newVertices.size() / 2);

						mModelRenderer->drawSurface(false);
						mModelRenderer->drawWireframe(false);
						mColoredModelRenderer->callDraw = true;
					}
					else
					{
						mModelRenderer->drawWireframe(true);
						mModelRenderer->drawWireframe(true);
						mColoredModelRenderer->callDraw = false;
					}

					if(mPrintNodeError)
					{
						std::array<float, 8> distanceToVertices;
						{
							std::array<glm::vec3, 8> inPos;
							for(uint32_t i=0; i < 8; i++)
							{
								inPos[i] = centerPoint + childrens[i] * 0.5f * size;
							}
							calculateMinDistances(inPos, distanceToVertices, triangles, trianglesInfo);
						}

						std::array<float, 19> distanceToMidPoints;
						{
							std::array<glm::vec3, 19> inPos;
							for(uint32_t i=0; i < 19; i++)
							{
								inPos[i] = centerPoint + nodeSamplePoints[i] * 0.5f * size;
							}
							calculateMinDistances(inPos, distanceToMidPoints, triangles, trianglesInfo);
						}

						const float trapezoidRMSE = glm::sqrt(estimateErrorFunctionIntegralByTrapezoidRule<TriLinearInterpolation>(
																distanceToVertices, *reinterpret_cast<const std::array<std::array<float, 1>, 19>*>(&distanceToMidPoints)));
						SPDLOG_INFO("RMSE using trapezoid method: {}", trapezoidRMSE);

						auto pow2 = [](float a) { return a * a; };

						std::array<float, 1> dist;
						std::array<glm::vec3, 1> pos;
						double accError = 0.0;
						for(uint32_t s=0; s < mInfluenceNumSamples; s++)
						{
							pos[0] = getRandomVec3();
							calculateMinDistances(pos, dist, triangles, trianglesInfo);

							const float d2 = interpolateValue(reinterpret_cast<const float*>(&distanceToVertices), 
															(pos[0] - centerPoint + 0.5f * size) / size);

							accError += static_cast<double>(pow2(dist[0] - d2));
						}
						const float montecarloRMSE = glm::sqrt(static_cast<float>(accError / static_cast<double>(mInfluenceNumSamples)));
						SPDLOG_INFO("MSE uisng monte carlo method: {}", montecarloRMSE);
					}

					SPDLOG_INFO("Number of selected triangles: {}", triangles.size());
				}
			}
			else
			{
				if(lastSelectZone)
				{
					mSelectionCube->callDraw = false;
					mColoredModelRenderer->callDraw = false;
					mInfluenceRegion->callDraw = false;
					mOptimalInfluenceRegion->callDraw = false;
					mModelRenderer->setIndexData(mMesh.value().getIndices());
				}
			}
		}


		// Print shortcuts information
		{
			ImGui::Spacing();
			ImGui::Separator();
			ImGui::Text("-> Shortcuts:");
			ImGui::Spacing();
			ImGui::Spacing();
			ImGui::Text("Keys A/D: Move left/right regarding the camera position");
			ImGui::Text("Keys W/S: Move forward/backwards regarding the camera position");
			ImGui::Text("Keys Space/Shift: Move up/down regarding the world Y-axis");
			ImGui::Spacing();
			ImGui::Text("Key Alt: Change to rotation gizmo to rotate freely the cutting plane");
			ImGui::Text("Key Ctrl: Hide all the plane gizmos");
			ImGui::Spacing();
			ImGui::Text("Key 1: Align plane normal to Z axis");
			ImGui::Text("Key 2: Align plane normal to Y axis");
			ImGui::Text("Key 3: Align plane normal to X axis");
			ImGui::Text("Key 4: Align plane normal to -Z axis");
			ImGui::Text("Key 5: Align plane normal to -Y axis");
			ImGui::Text("Key 6: Align plane normal to -X axis");
		}
	}
private:
	std::unique_ptr<SdfPlaneShader> mGridPlaneShader;
	std::unique_ptr<SdfOctreePlaneShader> mOctreePlaneShader;
	std::unique_ptr<NormalsSplitPlaneShader> mCubeShader;
	std::unique_ptr<NormalsShader> mGreenNormalsShader;

	std::shared_ptr<RenderMesh> mModelRenderer;
	std::shared_ptr<RenderMesh> mColoredModelRenderer;
	uint32_t mColoredModelRendererBufferId;
	std::shared_ptr<RenderMesh> mPlaneRenderer; 
	std::shared_ptr<RenderMesh> mCubeRenderer;
	std::shared_ptr<RenderMesh> mSelectionCube;
	std::shared_ptr<RenderMesh> mInfluenceRegion;
	uint32_t mInfluenceRegionBufferIdVert;
	uint32_t mInfluenceRegionBufferIdNorm;
	std::shared_ptr<RenderMesh> mOptimalInfluenceRegion;
	uint32_t mOptimalInfluenceRegionBufferIdVert;
	uint32_t mOptimalInfluenceRegionBufferIdNorm;

	glm::mat4x4 mGizmoStartMatrix;
	glm::mat4x4 mGizmoMatrix;

	bool mSelectZone = false;
	bool mPrintTrianglesInfluence = false;
	bool mPrintIfSomeInfluence = false;
	bool mComputeMinimalInfluence = false;
	bool mDrawInfluenceZone = false;
	bool mDrawOptimalZone = false;
	bool mPrintNodeError = false;
	uint32_t mInfluenceNumSamples = 10000;
	uint32_t mSelectedDepth = 0;
	BoundingBox mSelectArea;
	float mGridSize;
	uint32_t mInfluenceZoneSubdivisions = 2;
	char mSelectionAlgorithm[128] = "Basic";

	uint32_t mSelectedTriangle = 0;

	std::optional<Mesh> mMesh;

	// Input parameters
	SdfFormat mSdfFormat;
    std::optional<std::string> mModelPath;
    std::optional<float> mCellSize;
	std::optional<uint32_t> mDepth;
	std::optional<uint32_t> mStartDepth;
	std::optional<std::string> mSdfPath;
	std::optional<float> mTerminationThreshold;
	std::optional<OctreeSdf::TerminationRule> mTerminationRule;
	bool mNormalizeModel;
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
    args::Positional<std::string> modelPathArg(parser, "model_path", "The model path");
	args::ValueFlag<std::string> sdfPathArg(parser, "sdf_path", " The precalculated sdf to visualize", {"in"});
	args::Flag normalizeBBArg(parser, "normalize_model", "Normalize the model coordinates", {'n', "normalize"});
	args::ValueFlag<float> cellSizeArg(parser, "cell_size", "The voxel size of the voxelization", {"cell_size"});
    args::ValueFlag<uint32_t> depthArg(parser, "depth", "The octree subdivision depth", {"depth"});
	args::ValueFlag<uint32_t> startDepthArg(parser, "start_depth", "The octree start depth", {"start_depth"});
	args::ValueFlag<std::string> terminationRuleArg(parser, "termination_rule", "Octree generation termination rule", {"termination_rule"});
	args::ValueFlag<float> terminationThresholdArg(parser, "termination_threshold", "Octree generation termination threshold", {"termination_threshold"});
	args::ValueFlag<std::string> sdfFormatArg(parser, "sdf_format", "It supports two formats: octree or grid", {"sdf_format"});

    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch(args::Help)
    {
        std::cerr << parser;
        return 0;
    }

	if(sdfFormatArg && args::get(sdfFormatArg) != "octree" && args::get(sdfFormatArg) != "grid")
	{
		std::cerr << "The sdf_format can only be octree or grid";
		return 0;
	}

	std::optional<OctreeSdf::TerminationRule> terminationRule;
	if(terminationRuleArg)
	{
		terminationRule = OctreeSdf::stringToTerminationRule(args::get(terminationRuleArg));

		if(!terminationRule.has_value())
		{
			std::cerr << args::get(terminationRuleArg) << " is not a valid termination rule";
			return 0;
		}
	}

	const std::string sdfFormat = (sdfFormatArg) ? args::get(sdfFormatArg) : "octree";
	// const std::string defaultModel = "../models/sphere.glb";
	const std::string defaultModel = "../models/bunny.ply";

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
	else if(sdfFormat == "grid")
	{
		if(cellSizeArg)
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
				(depthArg) ? args::get(depthArg) : 5
			);
			MainLoop loop;
			loop.start(scene);
		}
	}
	else if(sdfFormat == "octree")
	{
		MyScene scene(
			(modelPathArg) ? args::get(modelPathArg) : defaultModel,
			(depthArg) ? args::get(depthArg) : 6,
			(startDepthArg) ? args::get(startDepthArg) : 2,
			(terminationThresholdArg) ? args::get(terminationThresholdArg) : 1e-3,
			terminationRule.value_or(OctreeSdf::TerminationRule::TRAPEZOIDAL_RULE)
		);
		MainLoop loop;
		loop.start(scene);
	}	
}