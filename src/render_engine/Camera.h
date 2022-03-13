#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/matrix.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "System.h"

class Camera : public System
{
public:
    void start() override;

    void setFov(float fov) { mFov = fov; recalculateProjectionMatrix(); }
    void setZNear(float zNear) { mZNear = zNear; recalculateProjectionMatrix(); }
    void setZFar(float zFar) { mZFar = zFar; recalculateProjectionMatrix(); }
    void setPosition(glm::vec3 position) { mPosition = position; recalculateViewMatrix(); } 
    void setOrientation(glm::quat orientation) { mOrientation = orientation; recalculateViewMatrix(); }

    float getFov() const { return mFov; }
    float getRatio() const { return mAspectRatio; }
    float getZFar() const { return mZFar; }
	float getZNear() const { return mZNear; }
    glm::vec3 getPosition() const { return mPosition; } 
    glm::quat getOrientation() const { return mOrientation; }

    const glm::mat4x4& getProjectionMatrix() const { return mProjectionMatrix; }
    const glm::mat4x4& getViewMatrix() const { return mViewMatrix; }
    const glm::mat4x4& getInverseViewMatrix() const { return mInverseViewMatrix; }
    
    void resize(glm::ivec2 windowSize);
    void drawGui() override;

protected:
    // Camera Position
    glm::quat mOrientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	glm::vec3 mPosition =  glm::vec3(0.0f, 0.0f, 0.0f);

    // Camera properties
    float mFov = 60.0;
	float mZNear = 0.01f;
	float mZFar = 50.0f;
    float mAspectRatio = 1.0f;

    // Cached matrices
    glm::mat4x4 mProjectionMatrix;
    glm::mat4x4 mViewMatrix;
    glm::mat4x4 mInverseViewMatrix;
    void recalculateProjectionMatrix();
    void recalculateViewMatrix();


};

#endif