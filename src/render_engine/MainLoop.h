#ifndef MAIN_LOOP_H
#define MAIN_LOOP_H

#include "Scene.h"

class MainLoop {
public:
	MainLoop() {}
    MainLoop(int fpsTarget) : mFpsTarget(fpsTarget) {}
	void start(Scene& scene, std::string name);
	void setFpsTarget(int fps) { mFpsTarget = fps; }
private:
	int mFpsTarget = 240;
	void drawGui();
	bool mShowGUI = false;
};

#endif