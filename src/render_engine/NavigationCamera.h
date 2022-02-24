#ifndef NAVEGATION_CAMERA_H
#define NAVEGATION_CAMERA_H

#include "Camera.h"

class NavigationCamera : public Camera
{
public:
    NavigationCamera() : 
        mBaseVelocity(0.07f),
        mMaxVelocity(0.8f),
        mCurrentVelocity(mBaseVelocity),
        mAcceleration(0.35f),
        mRotationVelocity(0.05f),
        mInRotationMode(false),
        mLastMousePosition(0.0f),
        mEulerAngles(0.0f)
    {}

    void update(float deltaTime) override;
    void drawGui() override;
private:
    float mBaseVelocity = 0.07f;
    float mMaxVelocity = 0.8f;
    float mCurrentVelocity = mBaseVelocity;
    float mAcceleration = 0.35f;

    float mRotationVelocity = 0.05f;
    bool mInRotationMode = false;
    glm::vec2 mLastMousePosition;
    glm::vec2 mEulerAngles;
};

#endif