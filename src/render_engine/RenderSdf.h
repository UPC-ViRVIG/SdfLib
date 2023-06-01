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
    
    sdflib::Timer mTimer;
    unsigned int mTimeLocation;

    glm::ivec3 mOctreeStartGridSize;
    glm::mat4x4 mOctreeMatrix;
    float mOctreeDistanceScale;
    float mOctreeMinBorderValue;
    std::shared_ptr<sdflib::OctreeSdf> mInputOctree;

};


#endif