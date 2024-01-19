#include "RenderSdf.h"
#include "Window.h"
#include "Camera.h"
#include "SdfLib/utils/PrimitivesFactory.h"
#include "shaders/Shader.h"
#include "SdfLib/OctreeSdf.h"
#include <filesystem>
#include <iostream>
#include <fstream>
#include <spdlog/spdlog.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "shaders/ScreenPlaneShader.h"
#include <imgui.h>

using namespace sdflib;

RenderSdf::~RenderSdf()
{
    glDeleteProgram(mRenderProgramId);
}

void RenderSdf::restart()
{
    glDeleteBuffers(1, &mOctreeSSBO);
    glDeleteProgram(mRenderProgramId);
    glDeleteTextures(1, &mRenderTexture);

    start();
}

void RenderSdf::start()
{
    auto checkForOpenGLErrors = []() -> GLenum
    {
        GLenum errorCode;
        while((errorCode = glGetError()) != GL_NO_ERROR)
        {
            SPDLOG_ERROR("OpenGL error with code {}", errorCode);
            return errorCode;
        }

        return GL_NO_ERROR;
    };

    // Load shader
    {
        unsigned int computeShaderId = glCreateShader(GL_COMPUTE_SHADER);

        auto loadShaderFromFile = [](std::string path, unsigned long* length) -> char*
        {
            std::ifstream file;
            file.open(path, std::ios_base::in | std::ios_base::binary);
            if (!file.good()) return nullptr;
            file.seekg(0, std::ios::end);
            *length = file.tellg();
            (*length)++;
            char* ret = new char[*length];
            file.seekg(0, std::ios::beg);
            file.read(ret, *length);
            file.close();
            ret[(*length) - 1] = 0;
            return ret;
        };

        unsigned long length;
        const char* fileName = "./shaders/sdfOctreeRender.comp";
        char* fileShader = loadShaderFromFile(fileName, &length);
        if (fileShader == nullptr) {
            std::filesystem::path p(fileName);
            fileShader = loadShaderFromFile("../src/render_engine/shaders/" + p.filename().string(), &length);
            if (fileShader == nullptr)
                std::cout << "File " << p.filename().string() << " not found" << std::endl;
        }
        
        // Add headers
        std::string computeShader;
        computeShader.append("#version 460 core\n\n");
        if(mInputOctree->getFormat() == IOctreeSdf::TRILINEAR_OCTREE)
        {
            computeShader.append("#define USE_TRILINEAR_INTERPOLATION\n");
        }
        else if(mInputOctree->getFormat() == IOctreeSdf::TRICUBIC_OCTREE)
        {
            computeShader.append("#define USE_TRICUBIC_INTERPOLATION\n");
        }

        switch(mAlgorithm)
        {
            case Algorithm::SPHERE_TRACING:
                computeShader.append("#define SPHERE_MARCHING\n");
                break;
            case Algorithm::OCTREE_TRAVERSAL_SH:
                computeShader.append("#define OCTREE_TRAVERSAL_AND_SH\n");
                break;
            case Algorithm::OCTREE_TRAVERSAL_SOLVER:
                computeShader.append("#define OCTREE_TRAVERSAL_AND_SOLVER\n");
                break;
            case Algorithm::SPHERE_TRACING_SOLVER:
                computeShader.append("#define SPHERE_MARCHING_SOLVER\n");
                break;
        }

        computeShader.append(fileShader);
        
        const char* computeShaderText = computeShader.c_str();

        glShaderSource(computeShaderId, 1, &computeShaderText, NULL);
        glCompileShader(computeShaderId);

        int success;
        glGetShaderiv(computeShaderId, GL_COMPILE_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glGetShaderInfoLog(computeShaderId, 512, NULL, infoLog);
            std::cout << "-> Vertex Shader error ( " << "sdfOctreeRender.comp" << " ):" << std::endl;
            std::cout << infoLog << std::endl;
            return;
        }

        checkForOpenGLErrors();

        delete[] fileShader;

        mRenderProgramId = glCreateProgram();
        glAttachShader(mRenderProgramId, computeShaderId);
        glLinkProgram(mRenderProgramId);

        glUseProgram(mRenderProgramId);

        mPixelToViewLocation = glGetUniformLocation(mRenderProgramId, "pixelToView");
        mNearPlaneHalfSizeLocation = glGetUniformLocation(mRenderProgramId, "nearPlaneHalfSize");
        mNearAndFarLocation = glGetUniformLocation(mRenderProgramId, "nearAndFarPlane");
        mInvViewModelMatrixLocation = glGetUniformLocation(mRenderProgramId, "invViewModelMatrix");
        mStartGridSizeLocation = glGetUniformLocation(mRenderProgramId, "startGridSize");
        mDistanceScaleLocation = glGetUniformLocation(mRenderProgramId, "distanceScale");
        mOctreeMinBorderValueLocation = glGetUniformLocation(mRenderProgramId, "minBorderValue");


        mEpsilonLocation = glGetUniformLocation(mRenderProgramId, "epsilon");

        //Options
        mUseAOLocation = glGetUniformLocation(mRenderProgramId, "useAO");
        mUseShadowsLocation = glGetUniformLocation(mRenderProgramId, "useShadows");
        mUseSoftShadowsLocation = glGetUniformLocation(mRenderProgramId, "useSoftShadows");
        mMaxIterationsLocation = glGetUniformLocation(mRenderProgramId, "maxIterations");
        mMaxShadowIterationsLocation = glGetUniformLocation(mRenderProgramId, "maxShadowIterations");

        //Lighting
        mLightNumberLocation = glGetUniformLocation(mRenderProgramId, "lightNumber");
        mLightPosLocation = glGetUniformLocation(mRenderProgramId, "lightPos");
        mLightColorLocation = glGetUniformLocation(mRenderProgramId, "lightColor");
        mLightIntensityLocation = glGetUniformLocation(mRenderProgramId, "lightIntensity");
        mLightRadiusLocation = glGetUniformLocation(mRenderProgramId, "lightRadius");

        //Material
        mMetallicLocation = glGetUniformLocation(mRenderProgramId, "matMetallic");
        mRoughnessLocation = glGetUniformLocation(mRenderProgramId, "matRoughness");
        mAlbedoLocation = glGetUniformLocation(mRenderProgramId, "matAlbedo");
        mF0Location = glGetUniformLocation(mRenderProgramId, "matF0");

        checkForOpenGLErrors();
    }

    // CreateTexture
    {
        glGenTextures(1, &mRenderTexture);
        glBindTexture(GL_TEXTURE_2D, mRenderTexture);
        // set the texture wrapping/filtering options (on the currently bound texture object)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        mRenderTextureSize = Window::getCurrentWindow().getWindowSize();
        std::vector<uint32_t> colorImage(mRenderTextureSize.x * mRenderTextureSize.y);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, mRenderTextureSize.x, mRenderTextureSize.y, 0, GL_RGBA, GL_FLOAT, NULL);

        glBindImageTexture(0, mRenderTexture, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    }

    // Set octree data
    {
        // Set octree trilinear data
        glGenBuffers(1, &mOctreeSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, mOctreeSSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER, mInputOctree->getOctreeData().size() * sizeof(IOctreeSdf::OctreeNode), mInputOctree->getOctreeData().data(), GL_STATIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, mOctreeSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

        mOctreeMatrix = glm::scale(glm::mat4(1.0f), 1.0f / mInputOctree->getGridBoundingBox().getSize()) * glm::translate(glm::mat4(1.0f), -mInputOctree->getGridBoundingBox().min);
        mOctreeDistanceScale = 1.0f / mInputOctree->getGridBoundingBox().getSize().x;
        mOctreeMinBorderValue = mInputOctree->getOctreeMinBorderValue();
    }


    if (mFirstLoad) {
        // Set plane render
        std::shared_ptr<Mesh> planeMesh = PrimitivesFactory::getPlane();
        planeMesh->applyTransform(glm::scale(glm::mat4(1.0f), glm::vec3(2.0f)));
        mRenderMesh.start();
        mRenderMesh.setIndexData(planeMesh->getIndices());
        mRenderMesh.setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
                                            RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
                                    }, planeMesh->getVertices().data(), planeMesh->getVertices().size());
        mFirstLoad = false;
    }

    screenPlaneShader.setInputTexture(mRenderTexture);
    mRenderMesh.setShader(&screenPlaneShader);

    mOctreeStartGridSize = mInputOctree->getStartGridSize();
    float minNodeSize = mInputOctree->getSampleArea().getSize().x / float(1 << mInputOctree->getOctreeMaxDepth());
    mEpsilon = minNodeSize / 2048.0f;
    mEpsilon10000 = mEpsilon * 10000;

    sendAlgorithmOptionsValues();
    sendLightingValues();
    sendMaterialValues();
}

void RenderSdf::draw(Camera* camera)
{
    glm::ivec2 currentScreenSize = Window::getCurrentWindow().getWindowSize();

    if( currentScreenSize.x != mRenderTextureSize.x ||
        currentScreenSize.y != mRenderTextureSize.y)
    {
        mRenderTextureSize = Window::getCurrentWindow().getWindowSize();
        glBindTexture(GL_TEXTURE_2D, mRenderTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, mRenderTextureSize.x, mRenderTextureSize.y, 0, GL_RGBA, GL_FLOAT, NULL);
        glBindImageTexture(0, mRenderTexture, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    }

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mRenderTexture);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mOctreeSSBO);

    glUseProgram(mRenderProgramId);

    const glm::vec2 nearAndFarPlane = glm::vec2(camera->getZNear(), camera->getZFar());
    const glm::mat4x4& invViewMat = mOctreeMatrix * camera->getInverseViewMatrix();
    float screenHalfSize = 0.5f * glm::tan(glm::radians(camera->getFov())) * nearAndFarPlane.x;
    float screenHalfSizeAspectRatio = screenHalfSize * camera->getRatio();
    glm::vec2 nearPlaneHalfSize = glm::vec2(screenHalfSizeAspectRatio, screenHalfSize);

    // Set camera configuration
    glUniform2f(mPixelToViewLocation, 2.0f * nearPlaneHalfSize.x / static_cast<float>(mRenderTextureSize.x), 
                                      2.0f * nearPlaneHalfSize.y / static_cast<float>(mRenderTextureSize.y));
    glUniform2f(mNearPlaneHalfSizeLocation, nearPlaneHalfSize.x, nearPlaneHalfSize.y);
    glUniform2f(mNearAndFarLocation, nearAndFarPlane.x, nearAndFarPlane.y);
    glUniformMatrix4fv(mInvViewModelMatrixLocation, 1, GL_FALSE, glm::value_ptr(invViewMat));
    glUniform3f(mStartGridSizeLocation, mOctreeStartGridSize.x, mOctreeStartGridSize.y, mOctreeStartGridSize.z);
    glUniform1f(mDistanceScaleLocation, mOctreeDistanceScale);
    glUniform1f(mOctreeMinBorderValueLocation, mOctreeMinBorderValue);

    glDispatchCompute(mRenderTextureSize.x/16, mRenderTextureSize.y/16, 1);

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    mRenderMesh.draw(camera);
}

void RenderSdf::sendAlgorithmOptionsValues()
{
    //Options
    glUseProgram(mRenderProgramId);
    glUniform1i(mUseAOLocation, mUseAO);
    glUniform1i(mUseShadowsLocation, mUseShadows);
    glUniform1i(mUseSoftShadowsLocation, mUseSoftShadows);
    glUniform1i(mMaxIterationsLocation, mMaxIterations);
    glUniform1i(mMaxShadowIterationsLocation, mMaxShadowIterations);
    if (mEpsilon != mEpsilon10000/10000) mEpsilon = mEpsilon10000/10000;
    glUniform1f(mEpsilonLocation, mEpsilon);
}

void RenderSdf::sendLightingValues()
{
    //Lighting
    glUseProgram(mRenderProgramId);
    glUniform1i(mLightNumberLocation, mLightNumber);
    glUniform3fv(mLightPosLocation, 4, glm::value_ptr(mLightPosition[0]));
    glUniform3fv(mLightColorLocation, 4, glm::value_ptr(mLightColor[0]));
    glUniform1fv(mLightIntensityLocation, 4, &mLightIntensity[0]);
    glUniform1fv(mLightRadiusLocation, 4, &mLightRadius[0]);
}

void RenderSdf::sendMaterialValues()
{
    //Material
    glUseProgram(mRenderProgramId);
    glUniform1f(mMetallicLocation, mMetallic);
    glUniform1f(mRoughnessLocation, mRoughness);
    glUniform3f(mAlbedoLocation, mAlbedo.x, mAlbedo.y, mAlbedo.z);
    glUniform3f(mF0Location, mF0.x, mF0.y, mF0.z);
}

void RenderSdf::drawGui()
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

    if (mShowSceneGUI) 
    {
        ImGui::Begin("Scene");
        // Light settings
        {
            ImGui::Spacing();
            ImGui::Separator();
            
            ImGui::Text("Lighting settings");
            bool hasChg = false;

            hasChg = hasChg || ImGui::SliderInt("Lights", &mLightNumber, 1, 4);

            for (int i = 0; i < mLightNumber; ++i) {
                ImGui::Text("Light %d", i);
                std::string pos = "Position##"+std::to_string(i+48);
                std::string col = "Color##"+std::to_string(i+48);
                std::string intens = "Intensity##"+std::to_string(i+48);
                std::string radius = "Radius##"+std::to_string(i+48);
                hasChg = hasChg || ImGui::InputFloat3(pos.c_str(), reinterpret_cast<float*>(&mLightPosition[i]));
                hasChg = hasChg || ImGui::ColorEdit3(col.c_str(), reinterpret_cast<float*>(&mLightColor[i]));
                hasChg = hasChg || ImGui::SliderFloat(intens.c_str(), &mLightIntensity[i], 0.0f, 20.0f);
                hasChg = hasChg || ImGui::SliderFloat(radius.c_str(), &mLightRadius[i], 0.01f, 1.0f);
            }

            if(hasChg) sendLightingValues();
        }

        // Material settings
        {
            ImGui::Spacing();
            ImGui::Separator();

            //ImGui::Text("Transform");
            //ImGui::InputFloat3("Position", reinterpret_cast<float*>(&mPosition));
            //ImGui::InputFloat3("Rotation", reinterpret_cast<float*>(&mRotation));
            //ImGui::InputFloat3("Scale", reinterpret_cast<float*>(&mScale));
            //ImGui::Spacing();
            //ImGui::Separator();
            ImGui::Text("Material settings");
            bool hasChg = false;

            hasChg = hasChg || ImGui::SliderFloat("Metallic", &mMetallic, 0.0f, 1.0f);
            hasChg = hasChg || ImGui::SliderFloat("Roughness", &mRoughness, 0.0f, 1.0f);
            hasChg = hasChg || ImGui::ColorEdit3("Albedo", reinterpret_cast<float*>(&mAlbedo));
            hasChg = hasChg || ImGui::ColorEdit3("F0", reinterpret_cast<float*>(&mF0));

            if(hasChg) sendMaterialValues();
        }

        // Algorithm settings
        {
            ImGui::Spacing();
            ImGui::Separator();

            ImGui::Text("Algorithm Settings");
            if(ImGui::BeginCombo("Algorithm", algorithmsStr[static_cast<uint32_t>(mAlgorithm)]))
            {
                const Algorithm oldAlgoirthm = mAlgorithm;
                for (int n = 0; n < IM_ARRAYSIZE(algorithmsStr); n++)
                {
                    switch(static_cast<Algorithm>(n))
                    {
                        case SPHERE_TRACING:
                            if(mInputOctree->hasSdfOnlyAtSurface()) continue;
                            break;
                        case OCTREE_TRAVERSAL_SOLVER:
                            break;
                        case SPHERE_TRACING_SOLVER:
                            if(mInputOctree->hasSdfOnlyAtSurface()) continue;
                            break;
                    }

                    bool is_selected = mAlgorithm == static_cast<Algorithm>(n);
                    if (ImGui::Selectable(algorithmsStr[n], is_selected))
                        mAlgorithm = static_cast<Algorithm>(n);
                    if (is_selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();

                if(oldAlgoirthm != mAlgorithm)
                {
                    restart();
                }
            }

            bool hasChg = false;
            hasChg = hasChg || ImGui::InputInt("Max Iterations", &mMaxIterations);
            hasChg = hasChg || ImGui::Checkbox("Shadows", &mUseShadows);
            if (mUseShadows) hasChg = hasChg || ImGui::Checkbox("Soft Shadows", &mUseSoftShadows);
            if (mUseShadows) hasChg = hasChg || ImGui::InputInt("Max Shadow Iterations", &mMaxShadowIterations);
            hasChg = hasChg || ImGui::Checkbox("AO", &mUseAO);
            hasChg = hasChg || ImGui::SliderFloat("Epsilon * 10000", &mEpsilon10000, 0.001f, 21.0f);

            if(hasChg) sendAlgorithmOptionsValues();
        }

        ImGui::End();
    }
}