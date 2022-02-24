#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normals;

uniform mat4 projectionViewModelMatrix;
uniform mat4 modelMatrix;
uniform mat3 normalModelMatrix;

out vec3 worldSpaceNormal;
out vec3 worldPosition;

void main() {
    worldSpaceNormal = normalModelMatrix * normals;
    worldPosition = (modelMatrix * vec4(position, 1.0f)).xyz;
	gl_Position = projectionViewModelMatrix * vec4(position, 1.0f);
}
