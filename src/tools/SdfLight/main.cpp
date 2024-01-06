#include "SdfLib/utils/Mesh.h"
#include "SdfLib/utils/PrimitivesFactory.h"
#include "SdfLib/utils/Timer.h"
#include "render_engine/shaders/SdfOctreeLightShader.h"
#include "render_engine/shaders/BasicShader.h"
#include "render_engine/MainLoop.h"
#include "render_engine/NavigationCamera.h"
#include "render_engine/RenderMesh.h"
#include "render_engine/Window.h"
#include <spdlog/spdlog.h>
#include <args.hxx>
#include <imgui.h>

using namespace sdflib;

class MyScene : public Scene
{
public:
    MyScene(std::string modelPath, std::string sdfPath, bool normalizeModel) : mModelPath(modelPath), mSdfPath(sdfPath), mNormalizeModel(normalizeModel) {}

    void start() override
	{
        Window::getCurrentWindow().setBackgroudColor(glm::vec4(0.9, 0.9, 0.9, 1.0));


        Mesh mesh(mModelPath);
        
        if(mNormalizeModel)
        {
            // Normalize model units
            const glm::vec3 boxSize = mesh.getBoundingBox().getSize();
            glm::vec3 center = mesh.getBoundingBox().getCenter();
            mesh.applyTransform(glm::scale(glm::mat4(1.0), glm::vec3(2.0f/glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z))) *
                                            glm::translate(glm::mat4(1.0), -mesh.getBoundingBox().getCenter()));
        }
        
        // Load Sdf
        std::unique_ptr<SdfFunction> sdfUnique = SdfFunction::loadFromFile(mSdfPath);
        std::shared_ptr<SdfFunction> sdf = std::move(sdfUnique);
        std::shared_ptr<IOctreeSdf> octreeSdf = std::dynamic_pointer_cast<IOctreeSdf>(sdf);
        if(octreeSdf->hasSdfOnlyAtSurface())
        {
            std::cerr << "The octrees with the isosurface termination rule are not supported in this application" << std::endl;
            exit(1);
        }

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
            mModelRenderer->callDrawGui = false;
            mModelRenderer->systemName = "Mesh Model";
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
                                        glm::scale(glm::mat4(1.0f), glm::vec3(64.0f)));
            mPlaneRenderer->setShader(mOctreeLightShader.get());
            mPlaneRenderer->callDraw = false; // Disable the automatic call because we already call the function
            mPlaneRenderer->callDrawGui = false;
            mPlaneRenderer->systemName = "Mesh Plane";
            addSystem(mPlaneRenderer);
        }

        // Create camera
        auto camera = std::make_shared<NavigationCamera>();
        camera->callDrawGui = false;
        // Move camera in the z-axis to be able to see the whole model
        BoundingBox BB = mesh.getBoundingBox();
        float zMovement = 0.5f * glm::max(BB.getSize().x, BB.getSize().y) / glm::tan(glm::radians(0.5f * camera->getFov()));
        camera->setPosition(glm::vec3(0.0f, 0.0f, 0.1f * BB.getSize().z + zMovement));
        camera->start();
        setMainCamera(camera);
        addSystem(camera);
        mCamera = camera;

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
            mOctreeLightShader->setLightInfo(i, mLightPosition[i], mLightColor[i], mLightIntensity[i], mLightRadius[i]);
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
                ImGui::EndMenu();
            }

            ImGui::EndMenuBar();
        }

        mCamera->drawGuiWindow();

        if (mShowSceneGUI) 
        {
            ImGui::Begin("Scene");
            // Print light settings
            ImGui::Spacing();
		    ImGui::Separator();

            ImGui::Text("Lighting settings");
            ImGui::SliderInt("Lights", &mLightNumber, 1, 4);

            for (int i = 0; i < mLightNumber; ++i) { //DOES NOT WORK, PROBLEM WITH REFERENCES
                ImGui::Text("Light %d", i);
                std::string pos = "Position##"+std::to_string(i+48);
                std::string col = "Color##"+std::to_string(i+48);
                std::string intens = "Intensity##"+std::to_string(i+48);
                std::string radius = "Radius##"+std::to_string(i+48);
                ImGui::InputFloat3(pos.c_str(), reinterpret_cast<float*>(&mLightPosition[i]));
                ImGui::ColorEdit3(col.c_str(), reinterpret_cast<float*>(&mLightColor[i]));
                ImGui::SliderFloat(intens.c_str(), &mLightIntensity[i], 0.0f, 20.0f);
                ImGui::SliderFloat(radius.c_str(), &mLightRadius[i], 0.01f, 1.0f);
            }

            // Print meterial settings
            ImGui::Spacing();
		    ImGui::Separator();

            ImGui::Text("Material settings");
            ImGui::SliderFloat("Metallic", &mMetallic, 0.0f, 1.0f);
            ImGui::SliderFloat("Roughness", &mRoughness, 0.0f, 1.0f);
            ImGui::ColorEdit3("Albedo", reinterpret_cast<float*>(&mAlbedo));
            ImGui::ColorEdit3("F0", reinterpret_cast<float*>(&mF0));

            // Print algorithm settings
            ImGui::Spacing();
		    ImGui::Separator();

            ImGui::Text("Algorithm Settings");
            ImGui::InputInt("Max Shadow Iterations", &mMaxShadowIterations);
            ImGui::SliderFloat("Over Relaxation", &mOverRelaxation, 1.0f, 2.0f);
            ImGui::Checkbox("AO", &mUseAO);
            ImGui::Checkbox("Soft Shadows", &mUseSoftShadows);

            // Print model GUI
            ImGui::PushID(mModelRenderer->getSystemId());
            mModelRenderer->drawGui();
            ImGui::PopID();
            ImGui::PushID(mPlaneRenderer->getSystemId());
            mPlaneRenderer->drawGui();
            ImGui::PopID();

            ImGui::End();
        }
    }

private:
    std::shared_ptr<NavigationCamera> mCamera;
    std::string mSdfPath;
    std::string mModelPath;
    bool mNormalizeModel;
    std::shared_ptr<RenderMesh> mModelRenderer;
    std::shared_ptr<RenderMesh> mPlaneRenderer;
    std::shared_ptr<RenderMesh> mLightRenderer;

    std::unique_ptr<SdfOctreeLightShader> mOctreeLightShader;

    //Options
    int mMaxShadowIterations = 512;
    bool mUseAO = true;
    bool mUseSoftShadows = true;
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

    float mLightRadius[4] =
    {
        0.1f,
        0.1f,
        0.1f,
        0.1f
    };

    //Material
    float mMetallic = 0.0f;
    float mRoughness = 0.5f;
    glm::vec3 mAlbedo = glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 mF0 = glm::vec3(0.07f, 0.07f, 0.07f);

    //GUI
    bool mShowSceneGUI = false;
};

int main(int argc, char** argv)
{
    #ifdef SDFLIB_PRINT_STATISTICS
        spdlog::set_pattern("[%^%l%$] [%s:%#] %v");
    #else
        spdlog::set_pattern("[%^%l%$] %v");
    #endif

    args::ArgumentParser parser("SdfLight app for rendering a model using its sdf");
    args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
    args::Positional<std::string> modelPathArg(parser, "model_path", "The model path");
    args::Positional<std::string> sdfPathArg(parser, "sdf_path", "The sdf model path");
    args::Flag normalizeBBArg(parser, "sdf_is_normalized", "Indicates that the sdf model is normalized", {'n', "normalize"});
    
    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch(args::Help)
    {
        std::cerr << parser;
        return 0;
    }

    MyScene scene(args::get(modelPathArg), args::get(sdfPathArg), (normalizeBBArg) ? true : false);
    MainLoop loop;
    loop.start(scene, "SdfLight");
}