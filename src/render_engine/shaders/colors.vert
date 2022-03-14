#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 color;

uniform mat4 projectionViewModelMatrix;

out vec3 outColor;

void main() {
	outColor = color;
	gl_Position = projectionViewModelMatrix * vec4(position, 1.0f);
}
