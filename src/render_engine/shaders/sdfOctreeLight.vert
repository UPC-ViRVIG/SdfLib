#version 430

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normals;

uniform mat4 projectionViewModelMatrix;
uniform mat4 worldToStartGridMatrix;
uniform mat3 normalWorldToStartGridMatrix;
uniform mat4 invViewModelMatrix;
uniform mat4 modelMatrix;
uniform mat3 normalModelMatrix;

out vec3 gridPosition;
out vec3 gridNormal;
out vec3 cameraPos;

void main() 
{
    cameraPos = vec3(worldToStartGridMatrix * invViewModelMatrix * vec4(vec3(0.0), 1.0));
    gridPosition = (worldToStartGridMatrix * modelMatrix * vec4(position, 1.0f)).xyz;
    gridNormal =  normalWorldToStartGridMatrix * normalModelMatrix * normals;
	gl_Position = projectionViewModelMatrix * vec4(position, 1.0f);

    // cameraPos = vec3(invViewModelMatrix * vec4(vec3(0.0), 1.0));
    // gridPosition = (modelMatrix * vec4(position, 1.0f)).xyz;
    // gridNormal = normalModelMatrix * normals;
	// gl_Position = projectionViewModelMatrix * vec4(position, 1.0f);
}
