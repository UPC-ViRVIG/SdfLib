#include "Scene.h"
#include <imgui.h>

void Scene::update(float deltaTime)
{
    for(auto& s : systems)
    {
        if(s->callUpdate) s->update(deltaTime);
    }

    for(auto& s : systems)
    {
        if(s->callDrawGui)
        {
            ImGui::PushID(s->systemId);
            s->drawGui();
            ImGui::PopID();
        }
    }
}

void Scene::draw()
{
    if(mainCamera != nullptr)
    {
        for(auto& s : systems)
        {
            if(s->callDraw) s->draw(mainCamera.get());
        }
    }
}

void Scene::resize(glm::ivec2 windowSize)
{
    if(mainCamera != nullptr) mainCamera->resize(windowSize);
}