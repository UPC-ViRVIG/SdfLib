#version 330 core

uniform vec4 outColor = vec4(0.8, 0.0, 0.0, 1.0);

out vec4 fragColor;


void main() {
	fragColor = outColor;
}