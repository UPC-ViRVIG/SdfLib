#version 330 core
layout (location = 0) in vec3 position;

uniform mat4 projectionViewModelMatrix;
uniform mat4 modelMatrix;
uniform mat4 worldToGridMatrix;

out vec3 gridPosition;

void main() 
{
    gridPosition = (worldToGridMatrix * modelMatrix * vec4(position, 1.0)).xyz;
	gl_Position = projectionViewModelMatrix * vec4(position, 1.0);
}