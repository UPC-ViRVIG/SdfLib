#include "render_engine/MainLoop.h"
#include "render_engine/NavigationCamera.h"
#include "render_engine/RenderMesh.h"
#include "render_engine/Window.h"
#include <spdlog/spdlog.h>
#include <args.hxx>

class MyScene : public Scene
{
public:
    MyScene(std::string sdfPath) : mSdfPath(sdfPath){}

private:
    std::string mSdfPath;
};

int main(int argc, char** argv)
{
    spdlog::set_pattern("[%^%l%$] [%s:%#] %v");

    args::ArgumentParser parser("UniformGridViwer reconstructs and draws a uniform grid sdf");
    args::Positional<std::string> modelPathArg(parser, "sdf_path", "The model path");
    
}