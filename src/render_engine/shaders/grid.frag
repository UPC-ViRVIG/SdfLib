#version 330 core

uniform vec4 color = vec4(0.0f, 0.0f, 0.0f, 1.0f);

out vec4 fragColor;  

void main() {
	fragColor = color;
}