#include "render_engine/MainLoop.h"
#include "render_engine/NavigationCamera.h"
#include "render_engine/RenderMesh.h"
#include "render_engine/shaders/NormalsShader.h"
#include "render_engine/Window.h"
#include "utils/Mesh.h"
#include "sdf/ExactOctreeSdf.h"

#include <map>
#include <spdlog/spdlog.h>
#include <args.hxx>
#include <imgui.h>
#include <ImGuizmo.h>
#include <glm/gtx/string_cast.hpp>

class MyScene : public Scene
{
    public:
        MyScene(std::string scenePath) : mScenePath(scenePath) {}

        void start() override
        {
            Window::getCurrentWindow().setBackgroudColor(glm::vec4(0.9, 0.9, 0.9, 1.0));

            {
                auto camera = std::make_shared<NavigationCamera>();
                camera->start();
                setMainCamera(camera);
                addSystem(camera);
		    }

            Assimp::Importer import;
            const aiScene *scene = import.ReadFile(mScenePath, aiProcess_Triangulate);

            if(!scene || !scene->mRootNode || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE)
            {
                SPDLOG_ERROR("Error with Assimp: {}", import.GetErrorString());
                return;
            }

            // Add Lights
            {
                std::map<std::string, Light> mapLights;

                SPDLOG_INFO("Loading scene with {} meshes and {} lights", scene->mNumMeshes, scene->mNumLights);
            
                for(uint32_t lightIndex=0; lightIndex < scene->mNumLights; lightIndex++)
                {
                    aiLight& aLight = *(scene->mLights[lightIndex]);
                    Light light;
                    SPDLOG_INFO("Light pos: {}, {}, {}", aLight.mPosition[0], aLight.mPosition[1], aLight.mPosition[2]);
                    light.lightPos = *reinterpret_cast<glm::vec3*>(&aLight.mPosition);
                    mapLights.emplace(std::string(aLight.mName.C_Str()), light);
                }

                // Process scene tree
                auto addMesh = [&](aiMesh* mesh, glm::mat4 transform)
                {
                    mMeshes.emplace_back(mesh);
                    mMeshes.back().applyTransform(transform);
                };

                auto addLightIfIllumination = [&](aiNode* node, glm::mat4 transform)
                {
                    auto it = mapLights.find(std::string(node->mName.C_Str()));
                    if(it != mapLights.end())
                    {
                        SPDLOG_INFO("Light found with transform matrix: {}", glm::to_string(transform));
                        it->second.lightPos = glm::vec3(transform * glm::vec4(it->second.lightPos, 1.0));
                        SPDLOG_INFO("Light pos: {}, {}, {}", it->second.lightPos[0], it->second.lightPos[1], it->second.lightPos[2]);
                    }
                };

                std::function<void(aiNode* node, glm::mat4 parentTransform)> processNode = 
                    [&](aiNode* node, glm::mat4 parentTransform)
                {
                    node->mTransformation.Transpose();
                    glm::mat4 transform = *reinterpret_cast<glm::mat4*>(&node->mTransformation);
                    transform = parentTransform * transform;

                    addLightIfIllumination(node, transform);

                    for(uint32_t i=0; i < node->mNumMeshes; i++)
                    {
                        const uint32_t mIdx = node->mMeshes[i];
                        addMesh(scene->mMeshes[mIdx], transform);
                    }

                    for(uint32_t i=0; i < node->mNumChildren; i++)
                    {
                        processNode(node->mChildren[i], transform);
                    }
                };

                processNode(scene->mRootNode, glm::mat4(1.0f));

                mLights.resize(mapLights.size());

                for(auto& it : mapLights)
                {
                    mLights.push_back(it.second);
                }
            }

            for(Mesh& mesh : mMeshes)
            {
                auto meshRenderer = std::make_shared<RenderMesh>();
                meshRenderer->start();

                meshRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
                    RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
					}, mesh.getVertices().data(), mesh.getVertices().size());

                meshRenderer->setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
                    RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
					}, mesh.getNormals().data(), mesh.getNormals().size());

                meshRenderer->setIndexData(mesh.getIndices());

                meshRenderer->setShader(NormalsShader::getInstance());

                addSystem(meshRenderer);

                meshRenderer->callDrawGui = false;
            }

            mObjectsSdf.reserve(mMeshes.size());

            for(Mesh& m : mMeshes)
            {
                BoundingBox box = m.getBoundingBox();
                glm::vec3 size = box.getSize();
                box.addMargin(0.2f * glm::max(size.x, glm::max(size.y, size.z)));
                mObjectsSdf.emplace_back(m, box, 7, 3, 128);
            }

            
        }

        void update(float deltaTime) override
        {
            Scene::update(deltaTime);

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Text("Scene Options");

        }
    private:
        struct Light
        {
            glm::vec3 lightPos;
        };

        std::string mScenePath;
        std::vector<Mesh> mMeshes;
        std::vector<Light> mLights;
        std::vector<ExactOctreeSdf> mObjectsSdf;
};

int main(int argc, char** argv)
{
    spdlog::set_pattern("[%^%l%$] [%s:%#] %v");

    args::ArgumentParser parser("ParticleSimulation basic particle simulation using SDFs");
    args::Positional<std::string> scenePathArg(parser, "scene_path", "The scene path");
    args::Positional<float> simulationStepSizeArg(parser, "scene_path", "The scene path");

    try 
    {
        parser.ParseCLI(argc, argv);
    }
    catch(args::Help)
    {
        std::cerr << parser;
    }

    MyScene scene(
        (scenePathArg) ? args::get(scenePathArg) : "../models/sceneTest.glb"
    );

    MainLoop loop;
    loop.start(scene);
}