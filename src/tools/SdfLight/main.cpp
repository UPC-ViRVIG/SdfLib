#include "SdfLib/utils/Mesh.h"
#include "SdfLib/utils/PrimitivesFactory.h"
#include "SdfLib/utils/Timer.h"
#include "render_engine/MainLoop.h"
#include "render_engine/NavigationCamera.h"
#include "render_engine/RenderMesh.h"
#include "render_engine/RenderSdf.h"
#include "render_engine/Window.h"
#include "render_engine/shaders/SdfOctreeLightShader.h"
#include "render_engine/shaders/BasicShader.h"
#include <spdlog/spdlog.h>
#include <args.hxx>
#include <imgui.h>

using namespace sdflib;

class MyScene : public Scene
{
public:
    MyScene(std::string modelPath, std::string sdfPath) : mModelPath(modelPath), mSdfPath(sdfPath){}

    void start() override
	{
        Window::getCurrentWindow().setBackgroudColor(glm::vec4(0.9, 0.9, 0.9, 1.0));


        Mesh mesh(mModelPath);
        
        glm::mat4 invTransform;
        if(true)
        {
            // Normalize model units
            const glm::vec3 boxSize = mesh.getBoundingBox().getSize();
            glm::vec3 center = mesh.getBoundingBox().getCenter();
            mesh.applyTransform(glm::scale(glm::mat4(1.0), glm::vec3(2.0f/glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z))) *
                                            glm::translate(glm::mat4(1.0), -mesh.getBoundingBox().getCenter()));

            invTransform = glm::inverse(glm::scale(glm::mat4(1.0), glm::vec3(2.0f/glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z))) *
                                            glm::translate(glm::mat4(1.0), -center));
            SPDLOG_INFO("Center is {}, {}, {}", center.x, center.y, center.z);
        }
        
        // Load Sdf
        std::unique_ptr<SdfFunction> sdfUnique = SdfFunction::loadFromFile(mSdfPath);
        std::shared_ptr<SdfFunction> sdf = std::move(sdfUnique);
        std::shared_ptr<OctreeSdf> octreeSdf = std::dynamic_pointer_cast<OctreeSdf>(sdf);

        mOctreeLightShader = std::make_unique<SdfOctreeLightShader>(*octreeSdf);

        // Model Render
        {
            mModelRenderer = std::make_shared<RenderMesh>();
			mModelRenderer->systemName = "Object Mesh";
			mModelRenderer->start();
            mesh.computeNormals();
			mModelRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
										RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
								}, mesh.getVertices().data(), mesh.getVertices().size());

			mModelRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
										RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
								}, mesh.getNormals().data(), mesh.getNormals().size());

			mModelRenderer->setIndexData(mesh.getIndices());
			mModelRenderer->setShader(mOctreeLightShader.get());
            mModelRenderer->callDraw = false; // Disable the automatic call because we already call the function
			addSystem(mModelRenderer);
        }

        // Plane Render
        {
            mPlaneRenderer = std::make_shared<RenderMesh>();
			mPlaneRenderer->start();

			// Plane
			std::shared_ptr<Mesh> plane = PrimitivesFactory::getPlane();
            plane->computeNormals();

			mPlaneRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
										RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
								}, plane->getVertices().data(), plane->getVertices().size());

            mPlaneRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
										RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
								}, plane->getNormals().data(), plane->getNormals().size());

			mPlaneRenderer->setIndexData(plane->getIndices());
			mPlaneRenderer->setTransform(glm::translate(glm::mat4(1.0f), glm::vec3(mesh.getBoundingBox().getCenter().x, mesh.getBoundingBox().min.y, mesh.getBoundingBox().getCenter().z)) * 
                                         glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f)) * 
                                        glm::scale(glm::mat4(1.0f), glm::vec3(12.0f)));
            mPlaneRenderer->setShader(mOctreeLightShader.get());
            mPlaneRenderer->callDraw = false; // Disable the automatic call because we already call the function
            addSystem(mPlaneRenderer);
        }

        // Create camera
		{
			auto camera = std::make_shared<NavigationCamera>();
        // Move camera in the z-axis to be able to see the whole model
            BoundingBox BB = mesh.getBoundingBox();
			float zMovement = 0.5f * glm::max(BB.getSize().x, BB.getSize().y) / glm::tan(glm::radians(0.5f * camera->getFov()));
			camera->setPosition(glm::vec3(0.0f, 0.0f, 0.1f * BB.getSize().z + zMovement));
            camera->start();
			setMainCamera(camera);
			addSystem(camera);
		}

    }

    void update(float deltaTime) override
	{
        Scene::update(deltaTime);
    }

    virtual void draw() override
    {
        //Model
        mOctreeLightShader->setMaterial(mAlbedo, mRoughness, mMetallic, mF0);
        mOctreeLightShader->setLightNumber(mLightNumber);
        for (int i = 0; i < mLightNumber; i++)
        {
            mOctreeLightShader->setLightInfo(i, mLightPosition[i], mLightColor[i], mLightIntensity[i]);
        }
        mOctreeLightShader->setUseAO(mUseAO);
        mOctreeLightShader->setUseSoftShadows(mUseSoftShadows);
        mOctreeLightShader->setOverRelaxation(mOverRelaxation);
        mOctreeLightShader->setMaxShadowIterations(mMaxShadowIterations);

        mModelRenderer->draw(getMainCamera());

        //Plane
        mOctreeLightShader->setMaterial(glm::vec3(0.7), 0.1, 0.1, glm::vec3(0.07));
        mPlaneRenderer->draw(getMainCamera());

        drawGui();
        Scene::draw();
    }

    void drawGui() 
    {
        if (ImGui::BeginMainMenuBar()) 
        {
            if (ImGui::BeginMenu("Scene")) 
            {
                ImGui::MenuItem("Show scene settings", NULL, &mShowSceneGUI);	
                ImGui::MenuItem("Show lighting settings", NULL, &mShowLightingGUI);
                ImGui::MenuItem("Show algorithm settings", NULL, &mShowAlgorithmGUI);	
                ImGui::MenuItem("Show model settings", NULL, &mShowSdfModelGUI);	
                
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Mesh")) 
            {
                ImGui::MenuItem("Show model settings", NULL, &mShowModelMeshGUI);
                ImGui::MenuItem("Show plane settings", NULL, &mShowPlaneMeshGUI);
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }

        mModelRenderer->setShowGui(mShowModelMeshGUI);
        mPlaneRenderer->setShowGui(mShowPlaneMeshGUI);

        if (mShowSceneGUI) 
        {
            ImGui::Begin("Scene");
            ImGui::Text("Scene Settings");
            ImGui::Checkbox("AO", &mUseAO);
            ImGui::Checkbox("Soft Shadows", &mUseSoftShadows);
            ImGui::End();
        }

        if (mShowLightingGUI)
        {
            ImGui::Begin("Lighting settings");
            ImGui::SliderInt("Lights", &mLightNumber, 1, 4);

            for (int i = 0; i < mLightNumber; ++i) { //DOES NOT WORK, PROBLEM WITH REFERENCES
                ImGui::Text("Light %d", i);
                std::string pos = "Position##"+std::to_string(i+48);
                std::string col = "Color##"+std::to_string(i+48);
                std::string intens = "Intensity##"+std::to_string(i+48);
                ImGui::InputFloat3(pos.c_str(), reinterpret_cast<float*>(&mLightPosition[i]));
                ImGui::ColorEdit3(col.c_str(), reinterpret_cast<float*>(&mLightColor[i]));
                ImGui::SliderFloat(intens.c_str(), &mLightIntensity[i], 0.0f, 20.0f);
            }

            ImGui::End();
        }

        if (mShowSdfModelGUI)
        {
            ImGui::Begin("Model Settings");
            ImGui::Text("Transform");
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Text("Material");
            ImGui::SliderFloat("Metallic", &mMetallic, 0.0f, 1.0f);
            ImGui::SliderFloat("Roughness", &mRoughness, 0.0f, 1.0f);
            ImGui::ColorEdit3("Albedo", reinterpret_cast<float*>(&mAlbedo));
            ImGui::ColorEdit3("F0", reinterpret_cast<float*>(&mF0));
            ImGui::End();
        }

        if (mShowAlgorithmGUI) 
        {
            ImGui::Begin("Algorithm Settings");
            ImGui::InputInt("Max Shadow Iterations", &mMaxShadowIterations);
            ImGui::SliderFloat("Over Relaxation", &mOverRelaxation, 1.0f, 2.0f);
            ImGui::End();
        }
    }

private:
    std::string mSdfPath;
    std::string mModelPath;
    std::shared_ptr<RenderSdf> mRenderSdf;
    std::shared_ptr<RenderMesh> mModelRenderer;
    std::shared_ptr<RenderMesh> mPlaneRenderer;
    std::shared_ptr<RenderMesh> mLightRenderer;

    std::unique_ptr<SdfOctreeLightShader> mOctreeLightShader;

    //Options
    int mMaxShadowIterations = 512;
    bool mUseAO = false;
    bool mUseSoftShadows = false;
    float mOverRelaxation = 1.47f;

    //Lighting
    int mLightNumber = 1;
    glm::vec3 mLightPosition[4] =
    {
        glm::vec3 (1.0f, 2.0f, 1.0f),
        glm::vec3 (-1.0f, 2.0f, 1.0f),
        glm::vec3 (1.0f, 2.0f, -1.0f),
        glm::vec3 (-1.0f, 2.0f, -1.0f)
    };

    glm::vec3 mLightColor[4] =
    {
        glm::vec3(1.0f, 1.0f, 1.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f),
        glm::vec3(1.0f, 0.0f, 1.0f)
    };

    float mLightIntensity[4] = 
    {
        10.0f,
        10.0f,
        10.0f,
        10.0f
    };

    //Material
    float mMetallic = 0.0f;
    float mRoughness = 0.5f;
    glm::vec3 mAlbedo = glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 mF0 = glm::vec3(0.07f, 0.07f, 0.07f);

    //GUI
    bool mShowSceneGUI = false;
    bool mShowLightingGUI = false;
    bool mShowAlgorithmGUI = false;
    bool mShowSdfModelGUI = false;
    bool mShowModelMeshGUI = false;
    bool mShowPlaneMeshGUI = false;
};

int main(int argc, char** argv)
{
    #ifdef SDFLIB_PRINT_STATISTICS
        spdlog::set_pattern("[%^%l%$] [%s:%#] %v");
    #else
        spdlog::set_pattern("[%^%l%$] %v");
    #endif

    args::ArgumentParser parser("UniformGridViwer reconstructs and draws a uniform grid sdf");
    args::Positional<std::string> modelPathArg(parser, "model_path", "The model path");
    args::Positional<std::string> sdfPathArg(parser, "sdf_path", "The sdf model path");
    
    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch(args::Help)
    {
        std::cerr << parser;
        return 0;
    }

    MyScene scene(args::get(modelPathArg), args::get(sdfPathArg));
    MainLoop loop;
    loop.start(scene);
}