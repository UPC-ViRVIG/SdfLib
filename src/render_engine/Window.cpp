#include "Window.h"
#include <iostream>
#include <set>
#include <algorithm>
#include <imgui.h>
#include <backends/imgui_impl_opengl3.h>
#include <backends/imgui_impl_glfw.h>

Window* Window::mCurrentWindow = nullptr;

Window::~Window() {
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}

void error_callback(int error, const char* description) {
	std::cout << "Error with glfw: " << description << std::endl;
}

bool Window::start() {
    if (!glfwInit()) {
		std::cout << "Error initializing GLFW" << std::endl;
		return false;
	}

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

	glfwSetErrorCallback(error_callback);

	// Start a glfw window
	mGlfwWindow = glfwCreateWindow(640, 640, "SharpBox", NULL, NULL);
	// glfwMaximizeWindow(glWindow);
	glfwMakeContextCurrent(mGlfwWindow);
	gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);

	// Enable the z buffer
	glEnable(GL_DEPTH_TEST);

	// Enable the back face culling process
	glDisable(GL_CULL_FACE);
	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);

	// Resize
	int width, height;
	glfwGetFramebufferSize(mGlfwWindow, &width, &height);
	mWindowSize.x = width;
	mWindowSize.y = height;
	glViewport(0, 0, width, height);

	// Start imgui
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();

	ImGui::StyleColorsDark();

	ImGui_ImplGlfw_InitForOpenGL(mGlfwWindow, true);
	ImGui_ImplOpenGL3_Init("#version 150");

	return mGlfwWindow != NULL;
}

bool Window::shouldClose() {
	return glfwWindowShouldClose(mGlfwWindow);
}

void Window::disableVerticalSync()
{
	glfwSwapInterval(0);
}

void Window::swapBuffers() {
	glfwSwapBuffers(mGlfwWindow);
}

bool Window::needResize() {
	int width, height;
	glfwGetFramebufferSize(mGlfwWindow, &width, &height);
	if (width != mWindowSize.x || height != mWindowSize.y) {
		glViewport(0, 0, width, height);
		mWindowSize.x = width; mWindowSize.y = height;
		return mWindowSize.x != 0 || mWindowSize.y != 0;
	}
	return false;
}

void Window::update() {
	// Glfw updates
	glClearColor(mBackgroundColor.r, mBackgroundColor.g, mBackgroundColor.b, mBackgroundColor.a);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
}

void Window::setBackgroudColor(glm::vec4 color) {
	mBackgroundColor = color;
}

glm::vec2 Window::getMousePosition() {
	double x; double y;
	glfwGetCursorPos(mGlfwWindow, &x, &y);
	return glm::vec2(x, y);
}

bool Window::isKeyPressed(int key) {
	return glfwGetKey(mGlfwWindow, key) == GLFW_PRESS;
}

bool Window::isKeyReleased(int key) {
	return glfwGetKey(mGlfwWindow, key) == GLFW_RELEASE;
}

bool Window::isMouseButtonPressed(int key) {
	return glfwGetMouseButton(mGlfwWindow, key) == GLFW_PRESS;
}

void Window::disableMouse() {
	glfwSetInputMode(mGlfwWindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
}

void Window::enableMouse() {
	glfwSetInputMode(mGlfwWindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
}