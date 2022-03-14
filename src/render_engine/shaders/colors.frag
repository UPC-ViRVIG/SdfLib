#version 330 core

in vec3 outColor;
out vec4 fragColor;

void main() {
	fragColor = vec4(outColor, 1.0);
}