#version 330 core

uniform vec4 cutPlane;

in vec3 worldSpaceNormal;
in vec3 worldPosition;
out vec4 fragColor;

const vec3 lightDir = normalize(vec3(0.5, 0.5, 0.0));

void main() {
    if(dot(cutPlane, vec4(worldPosition, 1.0)) > 0)
    {
        discard;
        return;
    }

	fragColor = vec4(vec3(0.7, 0.0, 0.0) * (0.2 + 0.8 * dot(normalize(worldSpaceNormal), lightDir)), 1.0);
}