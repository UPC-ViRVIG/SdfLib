#include "NavigationCamera.h"
#include "Window.h"
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <imgui.h>

void NavigationCamera::update(float deltaTime)
{

    // Set camera position
    glm::vec3 currentVelocity(0.0f);
    bool isMoving = false;

	if (Window::getCurrentWindow().isKeyPressed(GLFW_KEY_W)) 
    {
        currentVelocity += glm::vec3(0.0f, 0.0f, -1.0f) * mCurrentVelocity;
        isMoving = true;
	}
	else if (Window::getCurrentWindow().isKeyPressed(GLFW_KEY_S)) 
    {
        currentVelocity += glm::vec3(0.0f, 0.0f, 1.0f) * mCurrentVelocity;
        isMoving = true;
	}

	if (Window::getCurrentWindow().isKeyPressed(GLFW_KEY_A)) 
    {
        currentVelocity += glm::vec3(-1.0f, 0.0f, 0.0f) * mCurrentVelocity;
        isMoving = true;
	}
	else if (Window::getCurrentWindow().isKeyPressed(GLFW_KEY_D)) 
    {
        currentVelocity += glm::vec3(1.0f, 0.0f, 0.0f) * mCurrentVelocity;
        isMoving = true;
	}

	if (Window::getCurrentWindow().isKeyPressed(GLFW_KEY_SPACE)) {
        currentVelocity += glm::vec3(0.0f, 1.0f, 0.0f) * mCurrentVelocity;
        isMoving = true;
	}
	else if (Window::getCurrentWindow().isKeyPressed(GLFW_KEY_LEFT_SHIFT)) {
        currentVelocity += glm::vec3(0.0f, -1.0f, 0.0f) * mCurrentVelocity;
        isMoving = true;
	}

    setPosition(getPosition() + glm::mat3(getInverseViewMatrix()) * (currentVelocity * deltaTime));

    if(isMoving)
    {
        mCurrentVelocity = glm::min(mCurrentVelocity + mAcceleration * deltaTime, mMaxVelocity);
    }
    else
    {
        mCurrentVelocity = mBaseVelocity;
    }

    // Set camera rotation
    if (Window::getCurrentWindow().isMouseButtonPressed(GLFW_MOUSE_BUTTON_RIGHT)) 
    {
        if(!mInRotationMode) Window::getCurrentWindow().disableMouse();
        glm::vec2 mousePos = Window::getCurrentWindow().getMousePosition();
        if(mInRotationMode)
        {
            glm::vec2 offset = (mousePos - mLastMousePosition) * mRotationVelocity * deltaTime;
            mEulerAngles.x -= offset.y;
            mEulerAngles.y -= offset.x;
            setOrientation(
                glm::angleAxis(mEulerAngles.y, glm::vec3(0.0f, 1.0f, 0.0f)) *
                glm::angleAxis(mEulerAngles.x, glm::vec3(1.0f, 0.0f, 0.0f))
            );
        }

        mLastMousePosition = mousePos;
        mInRotationMode = true;
    }
    else
    {
        if(mInRotationMode) Window::getCurrentWindow().enableMouse();
        mInRotationMode = false;
    }
}

void NavigationCamera::drawGui()
{
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Text("Camera");
    Camera::drawGui();
    ImGui::InputFloat("Base velocity", &mBaseVelocity);
    ImGui::InputFloat("Max velocity", &mMaxVelocity);
    ImGui::InputFloat("Acceleration", &mAcceleration);
    ImGui::InputFloat("Rotation velocity", &mRotationVelocity);

    ImGui::InputFloat3("Camera position", reinterpret_cast<float*>(&mPosition));
    ImGui::InputFloat2("Camera rotation", reinterpret_cast<float*>(&mEulerAngles));
    setOrientation(
        glm::angleAxis(mEulerAngles.y, glm::vec3(0.0f, 1.0f, 0.0f)) *
        glm::angleAxis(mEulerAngles.x, glm::vec3(1.0f, 0.0f, 0.0f))
    );
}