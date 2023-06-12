#include "SdfLib/utils/Mesh.h"
#include "SdfLib/utils/PrimitivesFactory.h"
#include "render_engine/MainLoop.h"
#include "render_engine/NavigationCamera.h"
#include "render_engine/RenderMesh.h"
#include "render_engine/RenderSdf.h"
#include "render_engine/Window.h"
#include <spdlog/spdlog.h>
#include <args.hxx>
#include <imgui.h>

using namespace sdflib;

class MyScene : public Scene
{
public:
    MyScene(std::string sdfPath) : mSdfPath(sdfPath){}

    void start() override
	{
        Window::getCurrentWindow().setBackgroudColor(glm::vec4(0.9, 0.9, 0.9, 1.0));

        // Create camera
		
        auto camera = std::make_shared<NavigationCamera>();
        camera->start();
        setMainCamera(camera);
        addSystem(camera);
		

        BoundingBox sdfBB;

        // Load model
        std::unique_ptr<SdfFunction> sdfUnique = SdfFunction::loadFromFile(mSdfPath);
        std::shared_ptr<SdfFunction> sdf = std::move(sdfUnique);
        std::shared_ptr<OctreeSdf> octreeSdf = std::dynamic_pointer_cast<OctreeSdf>(sdf);
        
        sdfBB = octreeSdf->getGridBoundingBox();

        mRenderSdf = std::make_shared<RenderSdf>(octreeSdf);
        mRenderSdf->start();
        addSystem(mRenderSdf);

        // Move camera in the z-axis to be able to see the whole model
		{
			float zMovement = 0.5f * glm::max(sdfBB.getSize().x, sdfBB.getSize().y) / glm::tan(glm::radians(0.5f * camera->getFov()));
			camera->setPosition(glm::vec3(0.0f, 0.0f, 0.1f * sdfBB.getSize().z + zMovement));
		}
    }

    void update(float deltaTime) override
	{
        drawGui();
        Scene::update(deltaTime);
    }

    void drawGui() 
    {
        if (ImGui::BeginMainMenuBar()) 
        {
            if (ImGui::BeginMenu("File")) 
            {
                if (ImGui::MenuItem("Load Sdf")) 
                {
                    strncpy( buf, mSdfPath.c_str(), sizeof(buf)-1 );
                    mShowLoadSdfWindow = true;
                }	
                
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }

        if (mShowLoadSdfWindow) {
            ImGui::Begin("Load Sdf");
            ImGui::InputText("Sdf Path", buf, sizeof(buf));
            if (ImGui::Button("Load")) 
            {   
                mSdfPath = buf;
                std::unique_ptr<SdfFunction> sdfUnique = SdfFunction::loadFromFile(mSdfPath);
                std::shared_ptr<SdfFunction> sdf = std::move(sdfUnique);
                std::shared_ptr<OctreeSdf> octreeSdf = std::dynamic_pointer_cast<OctreeSdf>(sdf);
                mRenderSdf->setSdf(octreeSdf);
                mShowLoadSdfWindow = false;
            }
            if (ImGui::Button("Cancel")) 
            {       
                mShowLoadSdfWindow = false;
            }
            ImGui::End();
        }
    }

private:
    std::string mSdfPath;
    std::shared_ptr<RenderSdf> mRenderSdf;
    char buf[255]{};
    bool mShowLoadSdfWindow = false;
};

int main(int argc, char** argv)
{
    #ifdef SDFLIB_PRINT_STATISTICS
        spdlog::set_pattern("[%^%l%$] [%s:%#] %v");
    #else
        spdlog::set_pattern("[%^%l%$] %v");
    #endif

    args::ArgumentParser parser("UniformGridViwer reconstructs and draws a uniform grid sdf");
    args::Positional<std::string> modelPathArg(parser, "sdf_path", "The model path");
    
    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch(args::Help)
    {
        std::cerr << parser;
        return 0;
    }

    MyScene scene(args::get(modelPathArg));
    MainLoop loop;
    loop.start(scene, "SdfRender");
}