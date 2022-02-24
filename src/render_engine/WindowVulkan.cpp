/*
#include "WindowVulkan.h"
#include <iostream>
#include <set>
#include <algorithm>

Window* Window::mCurrentWindow = nullptr;

Window::~Window() {
	// ImGui_ImplOpenGL3_Shutdown();
	// ImGui_ImplGlfw_Shutdown();
	// ImGui::DestroyContext();

	for(auto& imgView : mSwapChainImageViews)
	{
		vkDestroyImageView(mDevice, imgView, nullptr);
	}
	vkDestroySwapchainKHR(mDevice, mSwapChain, nullptr);
	vkDestroySurfaceKHR(mInstance, mSurface, nullptr);
	vkDestroyDevice(mDevice, nullptr);
	vkDestroyInstance(mInstance, nullptr);

	glfwDestroyWindow(mGlfwWindow);
    glfwTerminate();
}

void error_callback(int error, const char* description) {
	std::cout << "Error with glfw: " << description << std::endl;
}

#ifdef USE_VALIDATION_LAYERS
bool Window::checkValidationLayerSupport() {
    uint32_t layerCount;
    vkEnumerateInstanceLayerProperties(&layerCount, nullptr);

    std::vector<VkLayerProperties> availableLayers(layerCount);
    vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());

	for (const char* layerName : mValidationLayers) {
		bool layerFound = false;

		for (const auto& layerProperties : availableLayers) {
			if (strcmp(layerName, layerProperties.layerName) == 0) {
				layerFound = true;
				break;
			}
		}

		if (!layerFound) {
			return false;
		}
	}

	return true;
}
#endif

bool Window::checkDeviceExtensionSupport(VkPhysicalDevice device)
{
	uint32_t extensionCount;
    vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, nullptr);

    std::vector<VkExtensionProperties> availableExtensions(extensionCount);
    vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, availableExtensions.data());

    std::set<std::string> requiredExtensions(mDeviceExtensions.begin(), mDeviceExtensions.end());

    for (const auto& extension : availableExtensions) {
        requiredExtensions.erase(extension.extensionName);
    }

    return requiredExtensions.empty();
}

bool Window::checkSwapChainSupport(VkPhysicalDevice device)
{
	uint32_t formatCount;
	vkGetPhysicalDeviceSurfaceFormatsKHR(device, mSurface, &formatCount, nullptr);

	uint32_t presentModeCount;
	vkGetPhysicalDeviceSurfacePresentModesKHR(device, mSurface, &presentModeCount, nullptr);
	
	return formatCount != 0 && presentModeCount != 0;
}

void Window::initVulkan()
{
	// Create Instance
	{
		VkApplicationInfo appInfo{};
		appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
		appInfo.pApplicationName = "Hello Triangle";
		appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
		appInfo.pEngineName = "No Engine";
		appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
		appInfo.apiVersion = VK_API_VERSION_1_0;

		VkInstanceCreateInfo createInfo{};
		createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
		createInfo.pApplicationInfo = &appInfo;

		uint32_t glfwExtensionCount = 0;
		const char** glfwExtensions;

		glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);

		createInfo.enabledExtensionCount = glfwExtensionCount;
		createInfo.ppEnabledExtensionNames = glfwExtensions;

		createInfo.enabledLayerCount = 0;

#ifdef USE_VALIDATION_LAYERS
		createInfo.enabledLayerCount = static_cast<uint32_t>(mValidationLayers.size());
		createInfo.ppEnabledLayerNames = mValidationLayers.data();
#endif

		if(vkCreateInstance(&createInfo, nullptr, &mInstance) != VK_SUCCESS)
		{
			std::cout << "Error with Vulkan: failed to create an instance" << std::endl;
			assert(false);
		}
	}

	// Create surface
	if(glfwCreateWindowSurface(mInstance, mGlfwWindow, nullptr, &mSurface) != VK_SUCCESS)
	{
		std::cout << "Error with Vulkan: failed to create a window surface" << std::endl;
		assert(false);
	}

	// Load physical device
	VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
	{
		uint32_t deviceCount = 0;
		vkEnumeratePhysicalDevices(mInstance, &deviceCount, nullptr);
		if(deviceCount == 0)
		{
			std::cout << "Error with Vulkan: any avaliable GPU supports Vulkan" << std::endl;
			assert(false);
		}
		std::cout << "Num of avaliable devices: " << deviceCount << std::endl;

		std::vector<VkPhysicalDevice> devices(deviceCount);
		vkEnumeratePhysicalDevices(mInstance, &deviceCount, devices.data());
		
		for(VkPhysicalDevice& d : devices)
		{
			VkPhysicalDeviceProperties deviceProperties{};
			vkGetPhysicalDeviceProperties(d, &deviceProperties);

			VkPhysicalDeviceFeatures deviceFeatures{};
			vkGetPhysicalDeviceFeatures(d, &deviceFeatures);

			if(deviceProperties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU &&
			   checkDeviceExtensionSupport(d) && checkSwapChainSupport(d))
			{
				physicalDevice = d;
				std::cout << "Selected device name: " << deviceProperties.deviceName << std::endl;
				break;
			}
		}
	}

	uint32_t graphicsQueueIndex = 0;
	uint32_t presentQueueIndex = 0;

	// Create logical device
	{
		// Search avaliable queue families

		uint32_t queueFamilyCount = 0;
		vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount, nullptr);

		std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
		vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount, queueFamilies.data());

		for(auto& q : queueFamilies)
		{
			if(q.queueFlags & VK_QUEUE_GRAPHICS_BIT)
			{
				break;
			}
			graphicsQueueIndex++;
		}

		if(graphicsQueueIndex >= queueFamilies.size())
		{
			std::cout << "Error with Vulkan: Any queue graphics family avaliable" << std::endl;
			assert(false);
		}

		for(auto& q : queueFamilies)
		{
			VkBool32 presentSupport = false;
			vkGetPhysicalDeviceSurfaceSupportKHR(physicalDevice, presentQueueIndex, mSurface, &presentSupport);
			if(presentSupport)
			{
				break;
			}
			presentQueueIndex++;
		}

		if(presentQueueIndex >= queueFamilies.size())
		{
			std::cout << "Error with Vulkan: Any queue present family avaliable" << std::endl;
			assert(false);
		}

		std::set<uint32_t> uniqueQueueFamilies = { graphicsQueueIndex, presentQueueIndex };
		std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;

		for(uint32_t queueIndex : uniqueQueueFamilies)
		{
			VkDeviceQueueCreateInfo queueCreateInfo{};
			queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
			queueCreateInfo.queueFamilyIndex = queueIndex;
			queueCreateInfo.queueCount = 1;

			float queuePriority = 1.0f;
			queueCreateInfo.pQueuePriorities = &queuePriority;
			queueCreateInfos.push_back(queueCreateInfo);
		}

		VkPhysicalDeviceFeatures deviceFeatures{};

		VkDeviceCreateInfo createInfo{};
		createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;

		createInfo.pQueueCreateInfos = queueCreateInfos.data();
		createInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());

		createInfo.pEnabledFeatures = &deviceFeatures;

		createInfo.enabledLayerCount = 0;

#ifdef USE_VALIDATION_LAYERS
		createInfo.enabledLayerCount = static_cast<uint32_t>(mValidationLayers.size());
		createInfo.ppEnabledLayerNames = mValidationLayers.data();
#endif

		createInfo.enabledExtensionCount = static_cast<uint32_t>(mDeviceExtensions.size());
		createInfo.ppEnabledExtensionNames = mDeviceExtensions.data();

		if(vkCreateDevice(physicalDevice, &createInfo, nullptr, &mDevice) != VK_SUCCESS)
		{
			std::cout << "Error with Vulkan: failed to create logical device" << std::endl;
			assert(false);
		}
	}

	vkGetDeviceQueue(mDevice, graphicsQueueIndex, 0, &mGraphicsQueue);
	vkGetDeviceQueue(mDevice, presentQueueIndex, 0, &mPresentQueue);

	// Create swap chain
	{
		// Select format
		{
			uint32_t formatCount;
			vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, mSurface, &formatCount, nullptr);
			
			std::vector<VkSurfaceFormatKHR> formats;
			if (formatCount != 0) {
				formats.resize(formatCount);
				vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, mSurface, &formatCount, formats.data());
			}

			mSwapChainFormat = formats[0];
			for (const auto& f : formats) {
				if (f.format == VK_FORMAT_B8G8R8A8_SRGB && f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
					mSwapChainFormat = f;
				}
			}
		}

		// Select present mode
		{
			uint32_t presentModeCount;
			vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, mSurface, &presentModeCount, nullptr);
			
			std::vector<VkPresentModeKHR> presentModes;
			if (presentModeCount != 0) {
				presentModes.resize(presentModeCount);
				vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, mSurface, &presentModeCount, presentModes.data());
			}

			mSwapChainPresentMode = VK_PRESENT_MODE_FIFO_KHR;
			for (const auto& p : presentModes) {
				if (p == VK_PRESENT_MODE_MAILBOX_KHR) {
					mSwapChainPresentMode = p;
				}
			}
		}

		VkSurfaceCapabilitiesKHR capabilities;
		vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, mSurface, &capabilities);

		// Select extent
		{
			mSwapChainExtent = { static_cast<uint32_t>(mWindowSize.x), static_cast<uint32_t>(mWindowSize.y) };

			mSwapChainExtent.width = std::clamp(mSwapChainExtent.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
        	mSwapChainExtent.height = std::clamp(mSwapChainExtent.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);
		}

		uint32_t swapSize = capabilities.minImageCount + 1;

		if(capabilities.maxImageCount > 0 && swapSize > capabilities.maxImageCount)
		{
			swapSize = capabilities.maxImageCount;
		}

		VkSwapchainCreateInfoKHR createInfo{};
		createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
		createInfo.surface = mSurface;
		createInfo.minImageCount = swapSize;
		createInfo.imageFormat = mSwapChainFormat.format;
		createInfo.imageColorSpace = mSwapChainFormat.colorSpace;
		createInfo.imageExtent = mSwapChainExtent;
		createInfo.imageArrayLayers = 1;
		createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

		if(graphicsQueueIndex != presentQueueIndex)
		{
			uint32_t queueFamilyIndices[] = { graphicsQueueIndex, presentQueueIndex };
			createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
			createInfo.queueFamilyIndexCount = 2;
			createInfo.pQueueFamilyIndices = queueFamilyIndices;
		}
		else
		{
			createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
			createInfo.queueFamilyIndexCount = 0;
			createInfo.pQueueFamilyIndices = nullptr;
		}
		
		createInfo.preTransform = capabilities.currentTransform;
		createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
		createInfo.presentMode = mSwapChainPresentMode;
		createInfo.clipped = VK_TRUE;
		createInfo.oldSwapchain = VK_NULL_HANDLE;

		if (vkCreateSwapchainKHR(mDevice, &createInfo, nullptr, &mSwapChain) != VK_SUCCESS) 
		{
			std::cout << "Error with Vulkan: failed to create a swap chain" << std::endl;
			assert(false);
		}

		// Load swap chain images
		vkGetSwapchainImagesKHR(mDevice, mSwapChain, &swapSize, nullptr);
		mSwapChainImages.resize(swapSize);
		vkGetSwapchainImagesKHR(mDevice, mSwapChain, &swapSize, mSwapChainImages.data());

		mSwapChainImageViews.resize(swapSize);
		for (int i=0; i < swapSize; i++)
		{
			VkImageViewCreateInfo imgCreateInfo{};
			imgCreateInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
			imgCreateInfo.image = mSwapChainImages[i];

			imgCreateInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
			imgCreateInfo.format = mSwapChainFormat.format;

			imgCreateInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
			imgCreateInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
			imgCreateInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
			imgCreateInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;

			imgCreateInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
			imgCreateInfo.subresourceRange.baseMipLevel = 0;
			imgCreateInfo.subresourceRange.levelCount = 1;
			imgCreateInfo.subresourceRange.baseArrayLayer = 0;
			imgCreateInfo.subresourceRange.layerCount = 1;

			if(vkCreateImageView(mDevice, &imgCreateInfo, nullptr, &mSwapChainImageViews[i]) != VK_SUCCESS)
			{
				std::cout << "Error with Vulkan: failed to create image views" << std::endl;
				assert(false);
			}
		}
	}
}

bool Window::start() {
    if (!glfwInit()) {
		std::cout << "Error initializing GLFW" << std::endl;
		return false;
	}

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

	glfwSetErrorCallback(error_callback);

	// Start a glfw window
	mGlfwWindow = glfwCreateWindow(640, 640, "SharpBox", NULL, NULL);
	// glfwMaximizeWindow(glWindow);
	// glfwMakeContextCurrent(mGlfwWindow);
	// gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);

	if(glfwVulkanSupported() == GLFW_FALSE)
	{
		std::cout << "Error GLFW does not support vulkan" << std::endl;
		return false;
	}

	int width, height;
	glfwGetFramebufferSize(mGlfwWindow, &width, &height);
	mWindowSize.x = width;
	mWindowSize.y = height;

	initVulkan();

	//enable the z buffer
	// glEnable(GL_DEPTH_TEST);

	//enable the back face culling process
	
	// glEnable(GL_CULL_FACE);
	// glFrontFace(GL_CCW);
	// glCullFace(GL_BACK);

	//resize
	
	// glViewport(0, 0, width, height);

	//start migui
	// IMGUI_CHECKVERSION();
	// ImGui::CreateContext();
	// ImGuiIO& io = ImGui::GetIO();

	// ImGui::StyleColorsDark();

	// ImGui_ImplGlfw_InitForOpenGL(glWindow, true);
	// ImGui_ImplOpenGL3_Init("#version 150");

	return mGlfwWindow != NULL;
}

bool Window::shouldClose() {
	return glfwWindowShouldClose(mGlfwWindow);
}

void Window::swapBuffers() {
	//glfwSwapBuffers(mGlfwWindow);
}

bool Window::needResize() {
	int width, height;
	glfwGetFramebufferSize(mGlfwWindow, &width, &height);
	if (width != mWindowSize.x || height != mWindowSize.y) {
		// glViewport(0, 0, width, height);
		mWindowSize.x = width; mWindowSize.y = height;
		return mWindowSize.x != 0 || mWindowSize.y != 0;
	}
	return false;
}

void Window::update() {
	//glfw updates
	// glClearColor(mBackgroundColor.r, mBackgroundColor.g, mBackgroundColor.b, mBackgroundColor.a);
	//glClear(GL_COLOR_BUFFER_BIT || GL_DEPTH_BUFFER_BIT);
	// glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
*/