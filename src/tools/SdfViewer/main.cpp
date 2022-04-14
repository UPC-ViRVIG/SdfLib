#include <iostream>
#include <random>
#include <algorithm>
#include <optional>
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
		OctreeSdf octreeSdf;

		BoundingBox sdfBB;
		BoundingBox viewBB;

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
			std::unique_ptr<SdfFunction> sdfFunc = SdfFunction::loadFromFile(mSdfPath.value());
			if(!sdfFunc)
			{
				assert(false);
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
			BoundingBox box = mMesh.value().getBoudingBox();
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
					octreeSdf = OctreeSdf(mMesh.value(), box, mDepth.value(), mStartDepth.value(), 
										  mTerminationThreshold.value(), mTerminationRule.value());
					sdfBB = octreeSdf.getGridBoundingBox();
					viewBB = octreeSdf.getGridBoundingBox();
					mGridSize = sdfBB.getSize().x * glm::pow(0.5f, mDepth.value());
					break;
			}
			SPDLOG_INFO("Uniform grid generation time: {}s", timer.getElapsedSeconds());
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
			mColoredModelRenderer->callDraw = false;
			addSystem(mColoredModelRenderer);
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
		if(!mMesh.has_value()) return;
		bool lastSelectZone = mSelectZone;
		ImGui::Checkbox("Visualize zone", &mSelectZone);
		if(mSelectZone) 
		{
			ImGui::Checkbox("Calculate Minimal Influence", &mComputeMinimalInfluence);
			ImGui::Checkbox("Print triangles influence", &mPrintTrianglesInfluence);
			if(mPrintTrianglesInfluence)
			{
				ImGui::InputInt("Num samples for influence computation", reinterpret_cast<int*>(&mInfluenceNumSamples));
				ImGui::Checkbox("Print If has some influence", &mPrintIfSomeInfluence);
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

				glm::vec3 centerPoint = mSelectArea.min + glm::floor((selPoint - mSelectArea.min + 0.5f * mGridSize) / size) * size - 0.5f * mGridSize + 0.5f * size;

				// Position cube
				mSelectionCube->setTransform( 
					glm::translate(glm::mat4(1.0f), centerPoint) * 
					glm::scale(glm::mat4(1.0f), glm::vec3(size))
				);
				mSelectionCube->callDraw = true;

				// Serach triangles influencing the selected zone
				const std::vector<uint32_t>& indices = mMesh.value().getIndices();
				const std::vector<glm::vec3>& vertices = mMesh.value().getVertices();

				std::vector<std::pair<float, uint32_t>> triangles;

				std::array<glm::vec3, 3> triangle;
				
				const float voxelDiagonal = glm::sqrt(3.0f); // Voxel diagonal when the voxels has size one

				float minMaxDist = INFINITY;

				for(uint32_t i=0; i < indices.size(); i += 3)
				{
					triangle[0] = vertices[indices[i]] - centerPoint;
					triangle[1] = vertices[indices[i + 1]] - centerPoint;
					triangle[2] = vertices[indices[i + 2]] - centerPoint;

					float minDist = GJK::getMinDistance(glm::vec3(0.5f * size), triangle);
					float maxDist = glm::min(GJK::getMinMaxDistance(glm::vec3(0.5f * size), triangle), minDist + voxelDiagonal * size);
					minMaxDist = glm::min(minMaxDist, maxDist);

					if(minDist <= minMaxDist)
					{
						triangles.push_back(std::make_pair(minDist, i/3));
					}
				}

				std::sort(triangles.begin(), triangles.end());

				int s=0;
				for(; s < triangles.size() && triangles[s].first <= minMaxDist; s++);

				triangles.resize(s);

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

				if(mComputeMinimalInfluence) 
				{
					std::vector<TriangleUtils::TriangleData> trianglesInfo = TriangleUtils::calculateMeshTriangleData(mMesh.value());
					std::vector<std::array<float, 8>> trianglesDist(triangles.size());

					uint32_t index = 0;
					for(std::pair<float, uint32_t>& p : triangles)
					{
						for(uint32_t j=0; j < 8; j++)
						{
							trianglesDist[index][j] = glm::abs(TriangleUtils::getSignedDistPointAndTriangle(centerPoint + 0.5f * childrens[j] * size, trianglesInfo[p.second]));
						}
						index++;
					}

					std::vector<std::pair<float, uint32_t>> newTriangles;
					std::vector<std::pair<glm::vec3, float>> spheresQuad(8);

					uint32_t outIndex = 0;
					for(std::pair<float, uint32_t>& p : triangles)
					{
						triangle[0] = vertices[indices[3 * p.second]] - centerPoint;
						triangle[1] = vertices[indices[3 * p.second + 1]] - centerPoint;
						triangle[2] = vertices[indices[3 * p.second + 2]] - centerPoint;
						
						bool inside = true;
						index = 0;
						for(std::pair<float, uint32_t>& p1 : triangles)
						{
							if (outIndex != index)
							{
								for (uint32_t i = 0; i < 8; i++)
								{
									spheresQuad[i] = std::make_pair(childrens[i] * 0.5f * size, trianglesDist[index][i]);
								}

								bool insideRegion = GJK::isInsideConvexHull(spheresQuad, triangle);
								if (!insideRegion)
								{
									inside = false;
									//break;
								}
								else
								{
									int a = 22;
									a++;
								}
							}
							index++;
						}

						if (inside)
						{
							newTriangles.push_back(p);
						}
						outIndex++;
					}

					SPDLOG_INFO("Triangles with new method {} // with all method {}", newTriangles.size(), triangles.size());

					triangles = std::move(newTriangles);
				}


				std::vector<std::pair<TriangleUtils::TriangleData, uint32_t>> trianglesData;
				trianglesData.reserve(triangles.size());
				std::vector<uint32_t> newIndices;
				newIndices.reserve(3 * triangles.size());

				for(const std::pair<float, uint32_t>& p : triangles)
				{
					newIndices.push_back(indices[3 * p.second]);
					newIndices.push_back(indices[3 * p.second + 1]);
					newIndices.push_back(indices[3 * p.second + 2]);

					trianglesData.push_back(std::make_pair(TriangleUtils::TriangleData(
						vertices[indices[3 * p.second]],
						vertices[indices[3 * p.second + 1]],
						vertices[indices[3 * p.second + 2]]
					), 0));
				}

				mModelRenderer->setIndexData(newIndices);

				if(mPrintTrianglesInfluence)
				{

					auto getRandomVec3 = [&] () -> glm::vec3
					{
						glm::vec3 p =  glm::vec3(static_cast<float>(rand())/static_cast<float>(RAND_MAX),
										 static_cast<float>(rand())/static_cast<float>(RAND_MAX),
										 static_cast<float>(rand())/static_cast<float>(RAND_MAX));
						return centerPoint + (p - 0.5f) * size;
					};
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

						// Add triangle
						const std::pair<float, uint32_t>& p = triangles[t];
						newVertices.push_back(vertices[indices[3 * p.second]]);
						newVertices.push_back(color);
						newVertices.push_back(vertices[indices[3 * p.second + 1]]);
						newVertices.push_back(color);
						newVertices.push_back(vertices[indices[3 * p.second + 2]]);
						newVertices.push_back(color);
					}

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

				SPDLOG_INFO("Number of selected triangles: {}", triangles.size());
			}
		}
		else
		{
			if(lastSelectZone)
			{
				mSelectionCube->callDraw = false;
				mColoredModelRenderer->callDraw = false;
				mModelRenderer->setIndexData(mMesh.value().getIndices());
			}
		}
	}
private:
	std::unique_ptr<SdfPlaneShader> mGridPlaneShader;
	std::unique_ptr<SdfOctreePlaneShader> mOctreePlaneShader;
	std::unique_ptr<NormalsSplitPlaneShader> mCubeShader;

	std::shared_ptr<RenderMesh> mModelRenderer;
	std::shared_ptr<RenderMesh> mColoredModelRenderer;
	uint32_t mColoredModelRendererBufferId;
	std::shared_ptr<RenderMesh> mPlaneRenderer; 
	std::shared_ptr<RenderMesh> mCubeRenderer;
	std::shared_ptr<RenderMesh> mSelectionCube;

	glm::mat4x4 mGizmoStartMatrix;
	glm::mat4x4 mGizmoMatrix;

	bool mSelectZone = false;
	bool mPrintTrianglesInfluence = false;
	bool mPrintIfSomeInfluence = false;
	bool mComputeMinimalInfluence = false;
	uint32_t mInfluenceNumSamples = 10000;
	uint32_t mSelectedDepth = 0;
	BoundingBox mSelectArea;
	float mGridSize;

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
	spdlog::set_pattern("[%^%l%$] [%s:%#] %v");

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

	const std::string sdfFormat = (sdfFormatArg) ? args::get(sdfFormatArg) : "grid";
	//const std::string defaultModel = "../models/sphere.glb";
	const std::string defaultModel = "../models/frog.ply";

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
			(startDepthArg) ? args::get(startDepthArg) : 4,
			(terminationThresholdArg) ? args::get(terminationThresholdArg) : 1e-3,
			terminationRule.value_or(OctreeSdf::TerminationRule::TRAPEZOIDAL_RULE)
		);
		MainLoop loop;
		loop.start(scene);
	}	
}