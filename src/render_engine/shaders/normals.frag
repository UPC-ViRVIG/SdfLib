#version 330 core

in vec3 worldSpaceNormal;
out vec4 fragColor;

const vec3 lightDir = normalize(vec3(0.5, 0.5, 0.0));

void main() {
	fragColor = vec4(vec3(0.7, 0.0, 0.0) * (0.2 + 0.8 * dot(normalize(worldSpaceNormal), lightDir)), 1.0);
}