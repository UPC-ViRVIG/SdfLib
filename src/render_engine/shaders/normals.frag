#version 330 core

uniform vec3 outColor = vec3(0.8, 0.0, 0.0);

in vec3 worldSpaceNormal;
out vec4 fragColor;

const vec3 lightDir = normalize(vec3(0.5, 0.5, 0.0));

void main() {
	fragColor = vec4(outColor * (0.5 + 0.5 * max(dot(normalize(worldSpaceNormal), lightDir), 0.0)), 1.0);
}