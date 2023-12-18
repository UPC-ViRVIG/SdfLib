#ifndef RENDER_SDF_H
#define RENDER_SDF_H

#include <glad/glad.h>
#include <vector>

#include "System.h"
#include "shaders/IShader.h"
#include "shaders/ScreenPlaneShader.h"
#include "RenderMesh.h"

#include "SdfLib/OctreeSdf.h"
#include "SdfLib/utils/Timer.h"

class RenderSdf : public System
{
public:
    RenderSdf(std::shared_ptr<sdflib::OctreeSdf> inputOctree, std::shared_ptr<sdflib::OctreeSdf> inputTricubicOctree)
    {
        mInputOctree = inputOctree;
        mInputTricubicOctree = inputTricubicOctree;
    }

    ~RenderSdf();

    void restart();

    void setSdf(std::shared_ptr<sdflib::OctreeSdf> inputOctree, std::shared_ptr<sdflib::OctreeSdf> inputTricubicOctree)
    {
        mInputOctree = inputOctree;
        mInputTricubicOctree = inputTricubicOctree;
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
    
    unsigned int mEpsilonLocation;
    float mEpsilon = 0.0001f;
    float mEpsilon10000 = 0.0001f * 10000;
    
    //Options
    unsigned int mUseAOLocation;
    unsigned int mUseSoftShadowsLocation;
    unsigned int mUsePerlinNoiseLocation;
    unsigned int mOverRelaxationLocation;
    unsigned int mUseItColorModeLocation;
    unsigned int mDrawPlaneLocation;
    unsigned int mDrawLightsLocation;
    unsigned int mRaymarchVersionLocation;
    unsigned int mV1TriCubicLocation;
    unsigned int mUseTricubicNormalsLocation;
    unsigned int mMaxIterationsLocation;
    unsigned int mMaxColorIterationsLocation;
    unsigned int mMaxShadowIterationsLocation;

    int mMaxIterations = 1024;
    int mMaxColorIterations = 64;
    int mMaxShadowIterations = 512;


    bool mUseAO = false;
    bool mUseSoftShadows = false;
    bool mUsePerlinNoise = false;
    float mOverRelaxation = 1.47f;
    bool mUseItColorMode = false;
    bool mDrawPlane = false;
    bool mDrawLights = false;
    int mRaymarchVersion = 3;
    bool mV1TriCubic = true;
    bool mUseTricubicNormals = true;

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
        glm::vec3 (-1.0f, 2.0f, 1.0f),
        glm::vec3 (1.0f, 2.0f, -1.0f),
        glm::vec3 (-1.0f, 2.0f, -1.0f)
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

    //Material
    unsigned int mMetallicLocation;
    unsigned int mRoughnessLocation;
    unsigned int mAlbedoLocation;
    unsigned int mF0Location;

    float mMetallic = 0.0f;
    float mRoughness = 0.77f;
    glm::vec3 mAlbedo = glm::vec3(0.35f, 0.0f, 0.014f);
    glm::vec3 mF0 = glm::vec3(0.07f, 0.07f, 0.07f);

    //Geometric transformations
    unsigned int mPositionLocation;
    unsigned int mRotationLocation;
    unsigned int mScaleLocation;


    glm::vec3 mPosition = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 mRotation = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 mScale = glm::vec3(1.0f, 1.0f, 1.0f);

    //BoundingBox
    
    unsigned int mBBminLocation;
    unsigned int mBBmaxLocation;

    glm::vec3 mBBMin; 
    glm::vec3 mBBMax;

    //Plane
    unsigned int mPlanePosLocation;
    float mPlanePos = 0.0f;

    //GUI
    bool mShowSceneGUI = false;
    bool mShowLightingGUI = false;
    bool mShowAlgorithmGUI = false;
    bool mShowSdfModelGUI = false;

    sdflib::Timer mTimer;
    unsigned int mTimeLocation;

    glm::ivec3 mOctreeStartGridSize;
    glm::mat4x4 mOctreeMatrix;
    float mOctreeDistanceScale;
    float mOctreeMinBorderValue;

    std::shared_ptr<sdflib::OctreeSdf> mInputOctree;
    std::shared_ptr<sdflib::OctreeSdf> mInputTricubicOctree;

};


#endif