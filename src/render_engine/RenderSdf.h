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
    RenderSdf(std::shared_ptr<sdflib::OctreeSdf> inputOctree) 
    {
        mInputOctree = inputOctree;
    }
    ~RenderSdf();
    void start() override;
    void draw(Camera* camera) override;
    void drawGui() override;

private:
    RenderMesh mRenderMesh;
    ScreenPlaneShader screenPlaneShader;
    unsigned int mRenderProgramId;
    unsigned int mRenderTexture;
    glm::ivec2 mRenderTextureSize;
    unsigned int mOctreeSSBO;

    unsigned int mPixelToViewLocation;
    unsigned int mNearPlaneHalfSizeLocation;
    unsigned int mNearAndFarLocation;
    unsigned int mInvViewModelMatrixLocation;
    unsigned int mStartGridSizeLocation;
    unsigned int mDistanceScaleLocation;
    unsigned int mOctreeMinBorderValueLocation;
    
    unsigned int mEpsilonLocation;
    float mEpsilon = 0.0001f;
    
    //Options
    unsigned int mUseAOLocation;
    unsigned int mUseSoftShadowsLocation;
    unsigned int mUsePerlinNoiseLocation;
    unsigned int mOverRelaxationLocation;

    bool mUseAO = false;
    bool mUseSoftShadows = false;
    bool mUsePerlinNoise = false;
    float mOverRelaxation = 1.2f;

    //Lighting
    unsigned int mLightNumberLocation;
    unsigned int mLightPosLocation;
    unsigned int mLightColorLocation;
    unsigned int mLightIntensityLocation;

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
        glm::vec3(0.0f, 1.0f, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f),
        glm::vec3(1.0f, 0.0f, 1.0f)
    };

    float mLightIntensity[4] = 
    {
        10.0f,
        10.0f,
        10.0f,
        10.0f
    };

    //Material
    unsigned int mMetallicLocation;
    unsigned int mRoughnessLocation;
    unsigned int mAlbedoLocation;
    unsigned int mF0Location;

    float mMetallic = 0.0f;
    float mRoughness = 0.5f;
    glm::vec3 mAlbedo = glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 mF0 = glm::vec3(0.07f, 0.07f, 0.07f);

    //Plane
    unsigned int mPlanePosLocation;
    float mPlanePos = 0.0f;


    sdflib::Timer mTimer;
    unsigned int mTimeLocation;

    glm::ivec3 mOctreeStartGridSize;
    glm::mat4x4 mOctreeMatrix;
    float mOctreeDistanceScale;
    float mOctreeMinBorderValue;
    std::shared_ptr<sdflib::OctreeSdf> mInputOctree;

};


#endif