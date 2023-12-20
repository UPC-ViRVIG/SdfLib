#ifndef NAVEGATION_CAMERA_H
#define NAVEGATION_CAMERA_H

#include "Camera.h"

class NavigationCamera : public Camera
{
public:
    NavigationCamera() : 
        mBaseVelocity(0.07f),
        mMaxVelocity(4.0f),
        mCurrentVelocity(mBaseVelocity),
        mAcceleration(1.0f),
        mRotationVelocity(0.002f),
        mInRotationMode(false),
        mLastMousePosition(0.0f),
        mEulerAngles(0.0f)
    {}

    void update(float deltaTime) override;
    void drawGui() override;
private:
    float mBaseVelocity = 0.07f;
    float mMaxVelocity = 4.0f;
    float mCurrentVelocity = mBaseVelocity;
    float mAcceleration = 1.0f;

    float mRotationVelocity = 0.002f;
    bool mInRotationMode = false;
    bool mShowGUI = false;
    glm::vec2 mLastMousePosition;
    glm::vec2 mEulerAngles;
};

#endif