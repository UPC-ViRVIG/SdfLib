#version 430

layout (location = 0) in vec3 position;

out vec2 uv;

void main() 
{
	gl_Position = vec4(position, 1.0f);
    uv = 0.5 * (position.xy + vec2(1.0));
}
