#include "SdfLib/utils/Mesh.h"
#include "SdfLib/utils/PrimitivesFactory.h"
#include "render_engine/MainLoop.h"
#include "render_engine/NavigationCamera.h"
#include "render_engine/RenderMesh.h"
#include "render_engine/RenderSdf.h"
#include "render_engine/Window.h"
#include <spdlog/spdlog.h>
#include <args.hxx>

using namespace sdflib;

class MyScene : public Scene
{
public:
    MyScene(std::string sdfPath) : mSdfPath(sdfPath){}

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

        // Load model
        std::unique_ptr<SdfFunction> sdfUnique = SdfFunction::loadFromFile(mSdfPath);
        std::shared_ptr<SdfFunction> sdf = std::move(sdfUnique);
        std::shared_ptr<OctreeSdf> octreeSdf = std::dynamic_pointer_cast<OctreeSdf>(sdf);
        

        mRenderSdf = std::make_shared<RenderSdf>(octreeSdf);
        mRenderSdf->start();
        addSystem(mRenderSdf);
    }

    void update(float deltaTime) override
	{
        Scene::update(deltaTime);
    }

private:
    std::string mSdfPath;
    std::shared_ptr<RenderSdf> mRenderSdf;
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
    loop.start(scene);
}