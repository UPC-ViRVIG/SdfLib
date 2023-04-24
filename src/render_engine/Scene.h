#ifndef SCENE_H
#define SCENE_H


#include <glm/glm.hpp>
#include <memory>
#include <vector>
#include "Camera.h"
#include "System.h"

class Scene 
{
public:
	virtual void start() = 0;
	virtual void update(float deltaTime);
	virtual void draw();
	virtual void resize(glm::ivec2 windowSize);

	// Camera set and getter
	void setMainCamera(std::shared_ptr<Camera> ptr ) {
		mainCamera = ptr;
	}

	Camera* getMainCamera() {
		return mainCamera.get();
	}

	void addSystem(std::shared_ptr<System> system)
	{
		system->systemId = nextSystemId++;
		systems.push_back(system);
	}

private:
	uint32_t nextSystemId = 0;
	std::shared_ptr<Camera> mainCamera;
	std::vector<std::shared_ptr<System>> systems;

};

#endif // SCENE_H