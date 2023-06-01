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

using namespace sdflib;

class MyScene : public Scene
{
public:
    MyScene(std::string modelPath, std::string sdfPath) : mModelPath(modelPath), mSdfPath(sdfPath){}

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

        // // Light Model
        // {
        //     mLightRenderer = std::make_shared<RenderMesh>();
		// 	mLightRenderer->start();

		// 	// Plane
		// 	std::shared_ptr<Mesh> plane = PrimitivesFactory::getIsosphere(3);
        //     plane->computeNormals();

		// 	mLightRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
		// 								RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
		// 						}, plane->getVertices().data(), plane->getVertices().size());

        //     mLightRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
		// 								RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
		// 						}, plane->getNormals().data(), plane->getNormals().size());

		// 	mLightRenderer->setIndexData(plane->getIndices());
		// 	mLightRenderer->setTransform(glm::translate(glm::mat4(1.0f), glm::vec3(1.046f, 2.6f, 1.661f)) * 
        //                                 glm::scale(glm::mat4(1.0f), glm::vec3(0.08f)));
        //     mLightRenderer->setShader(BasicShader::getInstance());
        //     BasicShader::getInstance()->setDrawColor(glm::vec4(0.95f, 0.95f, 0.1f, 1.0f));
        //     addSystem(mLightRenderer);
        // }
    }

    void update(float deltaTime) override
	{
        Scene::update(deltaTime);
    }

    virtual void draw() override
    {
        // Set albedo color at each object
        mOctreeLightShader->setAlbedoColor(glm::vec3(0.4707, 0.173, 0.554));
        mModelRenderer->draw(getMainCamera());
        mOctreeLightShader->setAlbedoColor(glm::vec3(0.7, 0.7, 0.7));
        mPlaneRenderer->draw(getMainCamera());

        Scene::draw();
    }

private:
    std::string mSdfPath;
    std::string mModelPath;
    std::shared_ptr<RenderSdf> mRenderSdf;
    std::shared_ptr<RenderMesh> mModelRenderer;
    std::shared_ptr<RenderMesh> mPlaneRenderer;
    std::shared_ptr<RenderMesh> mLightRenderer;

    std::unique_ptr<SdfOctreeLightShader> mOctreeLightShader;
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