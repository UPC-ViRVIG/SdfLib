#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normals;

uniform mat4 projectionViewModelMatrix;
uniform mat3 normalModelMatrix;

out vec3 worldSpaceNormal;

void main() {
    worldSpaceNormal = normalModelMatrix * normals;
	gl_Position = projectionViewModelMatrix * vec4(position, 1.0f);
}
