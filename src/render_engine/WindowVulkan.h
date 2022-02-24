/*
#ifndef WINDOW_H
#define WINDOW_H

#include <vulkan/vulkan.h>
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

    // Vulkan structures
    void initVulkan();
    VkInstance mInstance;
    VkDevice mDevice;
    VkSurfaceKHR mSurface;

    VkQueue mGraphicsQueue;
    VkQueue mPresentQueue;

    VkSwapchainKHR mSwapChain;
    VkSurfaceFormatKHR mSwapChainFormat;
    VkPresentModeKHR mSwapChainPresentMode;
    VkExtent2D mSwapChainExtent;
    std::vector<VkImage> mSwapChainImages;
    std::vector<VkImageView> mSwapChainImageViews;

#ifdef USE_VALIDATION_LAYERS
    bool checkValidationLayerSupport();
    const std::vector<const char*> mValidationLayers = { "VK_LAYER_KHRONOS_validation" };
#endif

    bool checkDeviceExtensionSupport(VkPhysicalDevice device);
    const std::vector<const char*> mDeviceExtensions = {
        VK_KHR_SWAPCHAIN_EXTENSION_NAME
    };

    bool checkSwapChainSupport(VkPhysicalDevice device);
};

#endif
*/