#include "SdfLib/utils/Mesh.h"
#include "SdfLib/utils/PrimitivesFactory.h"
#include "render_engine/RenderSdf.h"
#include "render_engine/MainLoop.h"
#include "render_engine/NavigationCamera.h"
#include "render_engine/RenderMesh.h"
#include "render_engine/Window.h"
#include <spdlog/spdlog.h>
#include <args.hxx>
#include <imgui.h>
#include <glm/gtc/quaternion.hpp>

using namespace sdflib;

class MyScene : public Scene
{
public:
    MyScene(std::string sdfPath, std::optional<std::string> sdfTricubicPath) : mSdfPath(sdfPath), mSdfTricubicPath(sdfTricubicPath) {}

    void start() override
	{
        Window::getCurrentWindow().setBackgroudColor(glm::vec4(0.9, 0.9, 0.9, 1.0));

        // Create camera
        mCamera = std::make_shared<NavigationCamera>();
        mCamera->start();
        mCamera->callDrawGui = false;
        setMainCamera(mCamera);
        addSystem(mCamera);
		

        BoundingBox sdfBB;

        // Load linear model
        std::unique_ptr<SdfFunction> sdfUnique = SdfFunction::loadFromFile(mSdfPath);
        std::shared_ptr<SdfFunction> sdf = std::move(sdfUnique);
        std::shared_ptr<IOctreeSdf> octreeSdf = std::dynamic_pointer_cast<IOctreeSdf>(sdf);

        // Load tricubic model
        std::shared_ptr<IOctreeSdf> octreeTriSdf(nullptr);
        if(mSdfTricubicPath.has_value())
        {
            std::unique_ptr<SdfFunction> sdfTriUnique = SdfFunction::loadFromFile(mSdfTricubicPath.value());
            std::shared_ptr<SdfFunction> sdfTri = std::move(sdfTriUnique);
            octreeTriSdf = std::dynamic_pointer_cast<IOctreeSdf>(sdfTri);
        }
        
        sdfBB = octreeSdf->getGridBoundingBox();
        glm::vec3 center = sdfBB.getSize();

        SPDLOG_INFO("GridBoundingBox size is {}, {}, {}", center.x, center.y, center.z);

        RenderSdf::Algorithm renderSdfAlgorithm;
        if(!octreeSdf->hasSdfOnlyAtSurface())
        {
            // renderSdfAlgorithm = RenderSdf::Algorithm::SPHERE_TRACING;
            renderSdfAlgorithm = RenderSdf::Algorithm::SPHERE_TRACING_SOLVER;
        }
        else
        {
            renderSdfAlgorithm = RenderSdf::Algorithm::OCTREE_TRAVERSAL_SOLVER;
        }

        mRenderSdf = std::make_shared<RenderSdf>(octreeSdf, renderSdfAlgorithm, octreeTriSdf);
        mRenderSdf->start();
        addSystem(mRenderSdf);

        // Move camera in the z-axis to be able to see the whole model
		{
			float zMovement = 0.5f * glm::max(sdfBB.getSize().x, sdfBB.getSize().y) / glm::tan(glm::radians(0.5f * mCamera->getFov()));
			mCamera->setPosition(glm::vec3(0.0f, 0.0f, 0.1f * sdfBB.getSize().z + zMovement));
		}
    }

    void update(float deltaTime) override
	{
        drawGui();
        Scene::update(deltaTime);

        if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_P))
        {
            if(!isPrintPositionPressed)
            {
                std::cout << "std::make_pair(glm::vec3(" << mCamera->getPosition().x << ", " << mCamera->getPosition().y << ", " << mCamera->getPosition().z << "), ";
                std::cout << "glm::quat(" << mCamera->getOrientation().w << ", " << mCamera->getOrientation().x << ", " << mCamera->getOrientation().y << ", " <<  mCamera->getOrientation().z << "))," << std::endl;
            }
            isPrintPositionPressed = true;
        } else isPrintPositionPressed = false;

        auto changeCameraPos = [&](uint32_t pId)
        {
            pId--;
            if(pId >= mCameraPositions.size()) return;
            mCamera->setPosition(mCameraPositions[pId].first);
            mCamera->setOrientation(mCameraPositions[pId].second);
        };

        if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_1)) changeCameraPos(1);
        else if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_2)) changeCameraPos(2);
        else if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_3)) changeCameraPos(3);
        else if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_4)) changeCameraPos(4);
        else if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_5)) changeCameraPos(5);
        else if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_6)) changeCameraPos(6);
        else if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_7)) changeCameraPos(7);
        else if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_8)) changeCameraPos(8);
        else if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_9)) changeCameraPos(9);
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
                    if(mSdfTricubicPath.has_value()) strncpy(bufTri, mSdfTricubicPath.value().c_str(), sizeof(bufTri) - 1);
                    mShowLoadSdfWindow = true;
                }	
                
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }

        mCamera->drawGuiWindow();

        if (mShowLoadSdfWindow) {
            ImGui::Begin("Load Sdf");
            ImGui::InputText("Sdf Linear Path", buf, sizeof(buf));
            if(mSdfTricubicPath.has_value()) ImGui::InputText("Sdf Tricubic Path", bufTri, sizeof(bufTri));
            if (ImGui::Button("Load")) 
            {   
                mSdfPath = buf;
                std::unique_ptr<SdfFunction> sdfUnique = SdfFunction::loadFromFile(mSdfPath);
                std::shared_ptr<SdfFunction> sdf = std::move(sdfUnique);
                std::shared_ptr<OctreeSdf> octreeSdf = std::dynamic_pointer_cast<OctreeSdf>(sdf);

                std::shared_ptr<OctreeSdf> octreeTriSdf(nullptr);
                if(mSdfTricubicPath.has_value())
                {
                    mSdfTricubicPath = bufTri;
                    std::unique_ptr<SdfFunction> sdfTriUnique = SdfFunction::loadFromFile(mSdfTricubicPath.value());
                    std::shared_ptr<SdfFunction> sdfTri = std::move(sdfTriUnique);
                    std::shared_ptr<OctreeSdf> octreeTriSdf = std::dynamic_pointer_cast<OctreeSdf>(sdfTri);
                }

                mRenderSdf->setSdf(octreeSdf, RenderSdf::Algorithm::SPHERE_TRACING, octreeTriSdf);
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
    std::shared_ptr<NavigationCamera> mCamera;
    std::string mSdfPath;
    std::optional<std::string> mSdfTricubicPath;
    std::shared_ptr<RenderSdf> mRenderSdf;
    char buf[255]{};
    char bufTri[255]{};
    bool mShowLoadSdfWindow = false;
    bool mUseIsoSurfaceModels = true;
    int selectedItem = 1;
    bool isPrintPositionPressed = false;
    

    // Temple scene
    // std::vector<std::pair<glm::vec3, glm::quat>> mCameraPositions = {
    //     std::make_pair(glm::vec3(-0.0173418, -0.17709, 1.78989), glm::quat(0.999977, -0.00600002, -0.00300003, -1.80006e-05)),
    //     std::make_pair(glm::vec3(-1.08202, 0.164158, 1.48781), glm::quat(0.927745, -0.0958972, -0.358771, -0.0370847)),
    //     std::make_pair(glm::vec3(1.89197, 0.0315761, 0.334719), glm::quat(0.803655, -0.0717148, 0.588421, 0.0525082)),
    //     std::make_pair(glm::vec3(0.327815, 0.329917, 0.448447), glm::quat(0.954738, -0.256027, 0.146248, 0.0392185)),
    //     std::make_pair(glm::vec3(-1.82276, 0.0571681, -0.736072), glm::quat(0.560121, -0.0156875, -0.827938, -0.0231883))
    // };

    // // Forest scene
    // std::vector<std::pair<glm::vec3, glm::quat>> mCameraPositions = {
    //     std::make_pair(glm::vec3(-0.828697, 0.172521, 0.965966), glm::quat(0.919071, -0.107093, -0.376714, -0.0438959)),
    //     std::make_pair(glm::vec3(-0.981824, -0.165227, -0.0478582), glm::quat(0.675246, 0.0222912, -0.736854, 0.024325)),
    //     std::make_pair(glm::vec3(0.365382, 1.12731, -0.341952), glm::quat(-0.496172, 0.367113, -0.632489, -0.467973)),
    //     std::make_pair(glm::vec3(0.411804, 0.289238, 0.885411), glm::quat(-0.976602, 0.116767, -0.179317, -0.02144))
    // };

    // // Room scene
    // std::vector<std::pair<glm::vec3, glm::quat>> mCameraPositions = {
    //     std::make_pair(glm::vec3(0.591452, 0.636738, 1.75727), glm::quat(0.977337, -0.193034, 0.0852435, 0.0168364)),
    //     std::make_pair(glm::vec3(-0.742403, 1.04158, 0.942762), glm::quat(0.854995, -0.341495, -0.362494, -0.144785)),
    //     std::make_pair(glm::vec3(-1.333, 0.413318, 0.25074), glm::quat(0.706888, -0.160302, -0.671862, -0.152359)),
    //     std::make_pair(glm::vec3(-1.43232, -0.627878, -0.81572), glm::quat(0.542223, 0.0582403, -0.83342, 0.0895179))
    // };

    // Sponza
    // std::vector<std::pair<glm::vec3, glm::quat>> mCameraPositions = {
    //     std::make_pair(glm::vec3(-0.817508, -0.331622, -0.0610357), glm::quat(0.592487, 0.0397562, -0.802793, 0.0538678)),
    //     std::make_pair(glm::vec3(0.865696, -0.316675, 0.0682273), glm::quat(0.831107, 0.0499264, 0.55287, -0.0332121)),
    //     std::make_pair(glm::vec3(0.927203, -0.0170088, 0.124477), glm::quat(0.819555, -0.0122941, 0.572804, 0.00859264)),
    //     std::make_pair(glm::vec3(0.605429, 0.229468, 0.0913712), glm::quat(0.854993, -0.195687, 0.468199, 0.107159)),
    //     std::make_pair(glm::vec3(-0.616771, -0.0108272, 0.348314), glm::quat(0.921911, 0.0359723, -0.385435, 0.0150394)),
    // };

    // Metaballs
    // std::vector<std::pair<glm::vec3, glm::quat>> mCameraPositions = {
    //     std::make_pair(glm::vec3(-1.57497, 1.28039, -0.567486), glm::quat(0.532547, -0.199237, -0.770463, -0.288246)),
    //     std::make_pair(glm::vec3(-1.04957, -0.0998881, 1.71175), glm::quat(0.953809, -0.00858442, -0.30028, -0.00270257)),
    //     std::make_pair(glm::vec3(0.729383, 0.812298, 1.6807), glm::quat(0.963519, -0.207382, 0.165396, 0.0355988)),
    //     std::make_pair(glm::vec3(0.720604, 0.199943, 0.727153), glm::quat(0.958743, -0.125343, 0.252998, 0.0330761)),
    //     std::make_pair(glm::vec3(-0.383302, -0.601878, 0.522696), glm::quat(0.951764, 0.0715169, -0.297543, 0.0223578))
    // };

    // Mugs room
    // std::vector<std::pair<glm::vec3, glm::quat>> mCameraPositions = {
    //     std::make_pair(glm::vec3(-0.420431, 1.57662, -0.282761), glm::quat(0.197488, -0.135399, -0.800779, -0.549016)),
    //     std::make_pair(glm::vec3(-0.746497, 1.10992, 0.442754), glm::quat(0.764859, -0.408947, -0.438946, -0.234691)),
    //     std::make_pair(glm::vec3(-0.842776, 0.170953, 1.10333), glm::quat(0.946372, -0.0978198, -0.306283, -0.0316583))
    // };

    // Monkeys
    std::vector<std::pair<glm::vec3, glm::quat>> mCameraPositions = {
        std::make_pair(glm::vec3(-0.570112, 0.684951, -1.31685), glm::quat(0.156258, -0.046629, -0.945418, -0.282124)),
        std::make_pair(glm::vec3(0.744043, 0.894792, -0.992971), glm::quat(-0.282707, 0.0899354, -0.910042, -0.289504)),
        std::make_pair(glm::vec3(0.561782, 0.579377, 1.12745), glm::quat(-0.952135, 0.223939, -0.202537, -0.0476361))
    };

    // Photo scene
    // std::vector<std::pair<glm::vec3, glm::quat>> mCameraPositions = {
    //     std::make_pair(glm::vec3(-0.675932, 0.210635, 1.82153), glm::quat(0.964288, -0.145738, -0.218673, -0.0330492))
    // };


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
    args::Positional<std::string> modelPathArg(parser, "sdf_path", "The model path");
    args::Positional<std::string> modelTricubicPathArg(parser, "sdftri_path", "The tricubic version of the model");
    
    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch(args::Help)
    {
        std::cerr << parser;
        return 0;
    }

    if(!modelPathArg)
    {
        std::cerr << "Error: No sdf_path specified" << std::endl;
        std::cerr << parser;
        return 1;
    }

    //MyScene scene(args::get(modelPathArg));
    MyScene scene(args::get(modelPathArg), 
                    (modelTricubicPathArg) ? std::optional<std::string>(args::get(modelTricubicPathArg)) : std::optional<std::string>());
    MainLoop loop;
    loop.start(scene, "SdfRender");
}