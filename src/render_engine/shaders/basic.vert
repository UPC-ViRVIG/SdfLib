#version 330 core
layout (location = 0) in vec3 position;

uniform mat4 projectionViewModelMatrix;

void main() {
	gl_Position = projectionViewModelMatrix * vec4(position, 1.0f);
}
