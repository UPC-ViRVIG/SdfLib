#ifndef WINDOW_H
#define WINDOW_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <assert.h>
#include <vector>

#ifndef NDEBUG
#define USE_VALIDATION_LAYERS
#endif

class Window {
public:
    ~Window();
    Window() {}
    static void setCurrentWindow(Window* win) {
        mCurrentWindow = win;
    }
    static Window& getCurrentWindow()
    {
        assert(mCurrentWindow != nullptr);
        return *mCurrentWindow;
    }

    bool start();
	bool shouldClose();
	void disableVerticalSync();
	void swapBuffers();
	void update();
	void setBackgroudColor(glm::vec4 color);
	bool needResize();
	bool isKeyPressed(int key);
	bool isKeyReleased(int key);
	bool isMouseButtonPressed(int key);
	void disableMouse();
	void enableMouse();
	glm::vec2 getMousePosition();
	glm::ivec2 getWindowSize() { return mWindowSize; }

private:
    static Window* mCurrentWindow;
    GLFWwindow* mGlfwWindow;
	glm::vec4 mBackgroundColor;
	glm::ivec2 mWindowSize;
};

#endif