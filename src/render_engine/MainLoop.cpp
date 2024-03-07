#include "MainLoop.h"

#include <thread>
#include "Window.h"
#include "SdfLib/utils/Timer.h"
#include <imgui.h>
#include <backends/imgui_impl_opengl3.h>
#include <backends/imgui_impl_glfw.h>
#include <ImGuizmo.h>
#include <iostream>


void MainLoop::drawGui()
{
	if (ImGui::BeginMainMenuBar()) 
    {
        if (ImGui::BeginMenu("Performance")) 
        {
			ImGui::MenuItem("Show performance window", NULL, &mShowGUI);
			ImGui::EndMenu();		
        }
        ImGui::EndMenuBar();
    }

	if (mShowGUI) {
		ImGui::Begin("Performance");
		ImGui::Text("FPS: %f", 1.0f / ImGui::GetIO().DeltaTime);
		ImGui::Text("Frame time (ms): %f", ImGui::GetIO().DeltaTime * 1000.0f);
		ImGui::Text("Average FPS (Last 120 frames): %f", ImGui::GetIO().Framerate);
		ImGui::End();
	}

	if(Window::getCurrentWindow().isKeyPressed(GLFW_KEY_T))
	{
		lastFPSsIdx = 0;
	}

	return;
}

void MainLoop::start(Scene& scene, std::string name)
{
    Window window;
	window.setWindowName(name);
    window.start();

    sdflib::Timer deltaTimer;
	deltaTimer.start();

	sdflib::Timer fpsTimer;
	fpsTimer.start();

	Window::setCurrentWindow(&window);

	window.disableVerticalSync();

	scene.start();

	std::vector<float> lastFPSs(2000);
	lastFPSsIdx = lastFPSs.size();
	fpsTimer.start();

	while (!window.shouldClose()) {
		if (mFpsTarget > 0) glfwPollEvents();
		else glfwWaitEvents();

		if (window.needResize()) {
			scene.resize(window.getWindowSize());
		}
		window.update();

		// Imgui updates 
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();


		ImGuizmo::SetOrthographic(false);
		ImGuizmo::BeginFrame();

		// Scene update
		scene.update(deltaTimer.getElapsedSeconds());
		deltaTimer.start();

		// Scene draw
		scene.draw();
		
		// Imgui draw
		drawGui();

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		window.swapBuffers();
		
		if(lastFPSsIdx < lastFPSs.size())
		{
			lastFPSs[lastFPSsIdx++] = 1000.0f * fpsTimer.getElapsedSeconds();
			if(lastFPSsIdx >= lastFPSs.size())
			{
				// Calculate mean and variance
				float mean = 0.0f;
				for(float v : lastFPSs) mean += v;
				mean = mean / static_cast<float>(lastFPSs.size());
				
				float std = 0.0f;
				for(float v : lastFPSs) std += (v - mean) * (v - mean);
				std = glm::sqrt(std / static_cast<float>(lastFPSs.size()));

				// std::cout << mean << ", " << std << std::endl;
				std::cout << mean << std::endl;
			}
		}
		fpsTimer.start();


		/*
		int millisecondsPerFrame = 1000 / mFpsTarget;
		int aux = fpsTimer.getElapsedMilliseconds();
		if (millisecondsPerFrame > aux) {
			std::this_thread::sleep_for(std::chrono::milliseconds(millisecondsPerFrame - aux));
		}
		*/
		window.disableVerticalSync();
	}

	Window::setCurrentWindow(nullptr);
}