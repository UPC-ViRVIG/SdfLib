#include "Camera.h"
#include <imgui.h>

void Camera::start()
{
    recalculateProjectionMatrix();
    recalculateViewMatrix();
}

void Camera::resize(glm::ivec2 windowSize)
{
    mAspectRatio = static_cast<float>(windowSize.x) / static_cast<float>(windowSize.y);
    recalculateProjectionMatrix();
}

void Camera::drawGui()
{
    bool change = false;
    change |= ImGui::SliderFloat("Fov", &mFov, 10.0f, 170.0f);
    change |= ImGui::InputFloat("zNear", &mZNear);
    change |= ImGui::InputFloat("zFar", &mZFar);
    if (change) recalculateProjectionMatrix();
}

void Camera::recalculateProjectionMatrix()
{
    mProjectionMatrix = glm::perspective(glm::radians(mFov), mAspectRatio, mZNear, mZFar);
}

void Camera::recalculateViewMatrix()
{
    mInverseViewMatrix = glm::translate(glm::mat4x4(1.0f), mPosition);
    mInverseViewMatrix = mInverseViewMatrix * glm::mat4_cast(mOrientation);
    mViewMatrix = glm::inverse(mInverseViewMatrix);
}