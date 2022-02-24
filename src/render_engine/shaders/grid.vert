#version 330 core
layout (location = 0) in vec3 position;

uniform mat4 projectionViewModelMatrix;
uniform mat4 projectionMatrix;
uniform mat4 viewModelMatrix;

const float normalOffset = 0.0001f;

void main() {
	vec4 pos = viewModelMatrix*vec4(position, 1.0f);
	pos.z += normalOffset;
	gl_Position = projectionMatrix * pos;
}