#include "utils/Mesh.h"
#include "utils/PrimitivesFactory.h"
#include "render_engine/MainLoop.h"
#include "render_engine/NavigationCamera.h"
#include "render_engine/RenderMesh.h"
#include "render_engine/RenderSdf.h"
#include "render_engine/Window.h"
#include "render_engine/shaders/SdfOctreeLightShader.h"
#include <spdlog/spdlog.h>
#include <args.hxx>

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
                                        glm::scale(glm::mat4(1.0f), glm::vec3(8.0f)));
            mPlaneRenderer->setShader(mOctreeLightShader.get());
            addSystem(mPlaneRenderer);
        }
    }

    void update(float deltaTime) override
	{
        Scene::update(deltaTime);
    }

private:
    std::string mSdfPath;
    std::string mModelPath;
    std::shared_ptr<RenderSdf> mRenderSdf;
    std::shared_ptr<RenderMesh> mModelRenderer;
    std::shared_ptr<RenderMesh> mPlaneRenderer;

    std::unique_ptr<SdfOctreeLightShader> mOctreeLightShader;
};

int main(int argc, char** argv)
{
    spdlog::set_pattern("[%^%l%$] [%s:%#] %v");

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