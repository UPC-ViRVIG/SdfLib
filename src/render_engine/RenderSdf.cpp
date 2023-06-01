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

using namespace sdflib;

RenderSdf::~RenderSdf()
{
    glDeleteProgram(mRenderProgramId);
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
        char* fileShader = loadShaderFromFile("./shaders/sdfOctreeRender.comp", &length);
        if (fileShader == nullptr) {
            std::filesystem::path p("sdfOctreeRender.comp");
            fileShader = loadShaderFromFile("../src/render_engine/shaders/" + p.filename().string(), &length);
            if (fileShader == nullptr)
                std::cout << "File " << "sdfOctreeRender.comp" << " not found" << std::endl;
        }

        glShaderSource(computeShaderId, 1, &fileShader, NULL);
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
        mTimeLocation = glGetUniformLocation(mRenderProgramId, "time");
        mTimer.start();

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
        // Set octree data
        glGenBuffers(1, &mOctreeSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, mOctreeSSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER, mInputOctree->getOctreeData().size() * sizeof(OctreeSdf::OctreeNode), mInputOctree->getOctreeData().data(), GL_STATIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, mOctreeSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

        mOctreeMatrix = glm::scale(glm::mat4(1.0f), 1.0f / mInputOctree->getGridBoundingBox().getSize()) * glm::translate(glm::mat4(1.0f), -mInputOctree->getGridBoundingBox().min);
        mOctreeDistanceScale = 1.0f / mInputOctree->getGridBoundingBox().getSize().x;
        mOctreeMinBorderValue = mInputOctree->getOctreeMinBorderValue();
    }

    // Set plane render
    std::shared_ptr<Mesh> planeMesh = PrimitivesFactory::getPlane();
    planeMesh->applyTransform(glm::scale(glm::mat4(1.0f), glm::vec3(2.0f)));

    mRenderMesh.start();
    mRenderMesh.setIndexData(planeMesh->getIndices());
    mRenderMesh.setVertexData(std::vector<RenderMesh::VertexParameterLayout> {
										RenderMesh::VertexParameterLayout(GL_FLOAT, 3)
								}, planeMesh->getVertices().data(), planeMesh->getVertices().size());

    screenPlaneShader.setInputTexture(mRenderTexture);
    mRenderMesh.setShader(&screenPlaneShader);

    mOctreeStartGridSize = mInputOctree->getStartGridSize();

    mInputOctree = nullptr; // We do not need the octree in the CPU any more
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
    glUniform1f(mTimeLocation, mTimer.getElapsedSeconds());

    glDispatchCompute(mRenderTextureSize.x/16, mRenderTextureSize.y/16, 1);

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    mRenderMesh.draw(camera);
}

void RenderSdf::drawGui()
{

}