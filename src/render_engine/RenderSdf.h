#ifndef RENDER_SDF_H
#define RENDER_SDF_H

#include <glad/glad.h>
#include <vector>

#include "SdfLib/OctreeSdf.h"
#include "SdfLib/utils/Timer.h"

#include "shaders/IShader.h"
#include "shaders/ScreenPlaneShader.h"
#include "System.h"
#include "RenderMesh.h"

class RenderSdf : public System
{
public:
    enum Algorithm
    {
        SPHERE_TRACING,
        OCTREE_TRAVERSAL_SH,
        OCTREE_TRAVERSAL_SOLVER,
        SPHERE_TRACING_SOLVER
    };

    const char* algorithmsStr[4] = {
        "Sphere tracing",
        "Octree traversal with sphere tracing",
        "Octree traversal with solver",
        "Sphere tracing with solver"
    };

    RenderSdf(std::shared_ptr<sdflib::IOctreeSdf> inputOctree, Algorithm algorithm, std::shared_ptr<sdflib::IOctreeSdf> inputTricubicOctree = nullptr)
    {
        mInputOctree = inputOctree;
        mInputTricubicOctree = inputTricubicOctree;
        mAlgorithm = algorithm;
    }

    ~RenderSdf();

    void restart();

    void setSdf(std::shared_ptr<sdflib::IOctreeSdf> inputOctree, Algorithm algorithm, std::shared_ptr<sdflib::IOctreeSdf> inputTricubicOctree = nullptr)
    {
        mInputOctree = inputOctree;
        mInputTricubicOctree = inputTricubicOctree;
        mAlgorithm = algorithm;
        restart();
    }

    void start() override;
    void draw(Camera* camera) override;
    void drawGui() override;

private:
    bool mFirstLoad = true;
    RenderMesh mRenderMesh;
    ScreenPlaneShader screenPlaneShader;
    unsigned int mRenderProgramId;
    unsigned int mRenderTexture;
    glm::ivec2 mRenderTextureSize;
    unsigned int mOctreeSSBO;
    unsigned int mOctreeTricubicSSBO;

    unsigned int mPixelToViewLocation;
    unsigned int mNearPlaneHalfSizeLocation;
    unsigned int mNearAndFarLocation;
    unsigned int mInvViewModelMatrixLocation;
    unsigned int mStartGridSizeLocation;
    unsigned int mDistanceScaleLocation;
    unsigned int mOctreeMinBorderValueLocation;
    
    //Options
    unsigned int mUseAOLocation;
    unsigned int mUseShadowsLocation;
    unsigned int mUseSoftShadowsLocation;
    unsigned int mMaxIterationsLocation;
    unsigned int mMaxShadowIterationsLocation;
    unsigned int mEpsilonLocation;

    float mEpsilon = 0.0001f;
    float mEpsilon10000 = 0.0001f * 10000;

    int mMaxIterations = 700;
    int mMaxShadowIterations = 64;

    bool mUseAO = false;
    bool mUseShadows = false;
    bool mUseSoftShadows = false;

    void sendAlgorithmOptionsValues();

    //Lighting
    unsigned int mLightNumberLocation;
    unsigned int mLightPosLocation;
    unsigned int mLightColorLocation;
    unsigned int mLightIntensityLocation;
    unsigned int mLightRadiusLocation;

    int mLightNumber = 1;
    glm::vec3 mLightPosition[4] =
    {
        glm::vec3 (1.0f, 2.0f, 1.0f),
        glm::vec3 (-1.0f, 2.0f, -1.0f),
        glm::vec3 (1.0f, -2.0f, 1.0f),
        glm::vec3 (-1.0f, -2.0f, -1.0f)
    };

    glm::vec3 mLightColor[4] =
    {
        glm::vec3(1.0f, 1.0f, 1.0f),
        glm::vec3(1.0f, 1.0f, 1.0f),
        glm::vec3(1.0f, 1.0f, 1.0f),
        glm::vec3(1.0f, 1.0f, 1.0f)
    };

    float mLightIntensity[4] = 
    {
        10.0f,
        10.0f,
        10.0f,
        10.0f
    };

    float mLightRadius[4] =
    {
        0.1f,
        0.1f,
        0.1f,
        0.1f
    };

    void sendLightingValues();

    //Material
    unsigned int mMetallicLocation;
    unsigned int mRoughnessLocation;
    unsigned int mAlbedoLocation;
    unsigned int mF0Location;

    float mMetallic = 0.0f;
    float mRoughness = 0.77f;
    glm::vec3 mAlbedo = glm::vec3(0.35f, 0.0f, 0.014f);
    glm::vec3 mF0 = glm::vec3(0.07f, 0.07f, 0.07f);

    void sendMaterialValues();

    //GUI
    bool mShowSceneGUI = false;

    glm::ivec3 mOctreeStartGridSize;
    glm::mat4x4 mOctreeMatrix;
    float mOctreeDistanceScale;
    float mOctreeMinBorderValue;

    std::shared_ptr<sdflib::IOctreeSdf> mInputOctree;
    std::shared_ptr<sdflib::IOctreeSdf> mInputTricubicOctree;
    Algorithm mAlgorithm;

};


#endif