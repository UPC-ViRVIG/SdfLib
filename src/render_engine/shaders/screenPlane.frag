#version 430

uniform sampler2D inputTexture;

out vec4 fragColor;

in vec2 uv;

void main() 
{
	fragColor = texture(inputTexture, uv);
	// fragColor = vec4(1.0, 0.0, 0.0, 1.0);
}