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

const char* models[]{
    "Armadillo",
    "Bunny",
    "Dragon",
    "Frog",
    "Happy",
    "ReliefPlate",
    "Sponza",
    "Temple"
};

const char* isoLinearPaths[]{ 
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeArmadilloIsoLin.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeBunnyIsoLin.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeDragonIsoLin.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeFrogIsoLin.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeHappyIsoLin.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeReliefPlate1MIsoLin.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeSponzaIsoLin.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeTempleIsoLin.bin"
};

const char* linearPaths[]{
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeArmadilloLin.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeBunnyLin.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeDragonLin.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeFrogLin.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeHappyLin.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeReliefPlate1MLin.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeSponzaLin.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeTempleLin.bin"
};

const char* cubicPaths[]{
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeArmadilloCub.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeBunnyCub.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeDragonCub.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeFrogCub.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeHappyCub.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeReliefPlate1MCub.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeSponzaCub.bin",
    "C:/Users/juane/Documents/Github/SdfLib/output/SdfOctreeTempleCub.bin"
};


class MyScene : public Scene
{
public:
    MyScene(std::string sdfPath, std::string sdfTricubicPath) : mSdfPath(sdfPath), mSdfTricubicPath(sdfTricubicPath) {}

    void start() override
	{
        Window::getCurrentWindow().setBackgroudColor(glm::vec4(0.9, 0.9, 0.9, 1.0));

        // Create camera
		
        auto camera = std::make_shared<NavigationCamera>();
        camera->start();
        setMainCamera(camera);
        addSystem(camera);
		

        BoundingBox sdfBB;

        // Load linear model
        std::unique_ptr<SdfFunction> sdfUnique = SdfFunction::loadFromFile(mSdfPath);
        std::shared_ptr<SdfFunction> sdf = std::move(sdfUnique);
        std::shared_ptr<OctreeSdf> octreeSdf = std::dynamic_pointer_cast<OctreeSdf>(sdf);

        // Load tricubic model
        std::unique_ptr<SdfFunction> sdfTriUnique = SdfFunction::loadFromFile(mSdfTricubicPath);
        std::shared_ptr<SdfFunction> sdfTri = std::move(sdfTriUnique);
        std::shared_ptr<OctreeSdf> octreeTriSdf = std::dynamic_pointer_cast<OctreeSdf>(sdfTri);
        
        sdfBB = octreeSdf->getGridBoundingBox();
        glm::vec3 center = sdfBB.getSize();

        SPDLOG_INFO("GridBoundingBox size is {}, {}, {}", center.x, center.y, center.z);


        mRenderSdf = std::make_shared<RenderSdf>(octreeSdf, octreeTriSdf);
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
                    mShowLoadSdfWindow = true;
                }	
                
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }

        if (mShowLoadSdfWindow) {
            ImGui::Begin("Load Sdf");
            ImGui::Checkbox("Use Isosurface", &mUseIsoSurfaceModels);
            ImGui::Combo("Model", &selectedItem, models, IM_ARRAYSIZE(models));
            if (ImGui::Button("Load")) 
            {   
                mSdfPath = mUseIsoSurfaceModels ? isoLinearPaths[selectedItem] : linearPaths[selectedItem];
                mSdfTricubicPath = cubicPaths[selectedItem];
                std::unique_ptr<SdfFunction> sdfUnique = SdfFunction::loadFromFile(mSdfPath);
                std::shared_ptr<SdfFunction> sdf = std::move(sdfUnique);
                std::shared_ptr<OctreeSdf> octreeSdf = std::dynamic_pointer_cast<OctreeSdf>(sdf);
                std::unique_ptr<SdfFunction> sdfTriUnique = SdfFunction::loadFromFile(mSdfTricubicPath);
                std::shared_ptr<SdfFunction> sdfTri = std::move(sdfTriUnique);
                std::shared_ptr<OctreeSdf> octreeTriSdf = std::dynamic_pointer_cast<OctreeSdf>(sdfTri);

                mRenderSdf->setSdf(octreeSdf, octreeTriSdf);
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
    std::string mSdfTricubicPath;
    std::shared_ptr<RenderSdf> mRenderSdf;
    char buf[255]{};
    char bufTri[255]{};
    bool mShowLoadSdfWindow = false;
    bool mUseIsoSurfaceModels = true;
    int selectedItem = 1;
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

    //MyScene scene(args::get(modelPathArg));
    MyScene scene("C:/Users/juane/Documents/Github/SdfLib/output/sdfOctreeArmadilloIsoLin.bin", "C:/Users/juane/Documents/Github/SdfLib/output/sdfOctreeArmadilloCub.bin");
    MainLoop loop;
    loop.start(scene, "SdfRender");
}