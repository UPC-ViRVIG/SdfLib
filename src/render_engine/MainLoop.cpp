#include "MainLoop.h"

#include <thread>
#include "Window.h"
#include "SdfLib/utils/Timer.h"
#include <imgui.h>
#include <backends/imgui_impl_opengl3.h>
#include <backends/imgui_impl_glfw.h>
#include <ImGuizmo.h>


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

	// window.disableVerticalSync();

	scene.start();

	while (!window.shouldClose()) {
		fpsTimer.start();

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

		int millisecondsPerFrame = 1000 / mFpsTarget;
		int aux = fpsTimer.getElapsedMilliseconds();
		if (millisecondsPerFrame > aux) {
			std::this_thread::sleep_for(std::chrono::milliseconds(millisecondsPerFrame - aux));
		}
		
	}

	Window::setCurrentWindow(nullptr);
}