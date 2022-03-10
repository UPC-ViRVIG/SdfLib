#version 330 core

out vec4 fragColor;

in vec3 gridPosition;
in float distToCamera;

uniform sampler3D sdfGrid;

uniform vec3 planeNormal; // normalized

uniform float gridValueRange = 1.0;
uniform vec3 gridSize;

uniform float surfaceThickness = 1.5;
uniform float gridThickness = 0.1;
uniform float linesThickness = 0.8;

uniform float linesSpace = 0.1f;

uniform bool printGrid = true;
uniform bool printIsolines = true;

const int paletteNumColors = 7;
const vec3 palette[7] = vec3[7](
	vec3(0.0f, 0.0f, 1.0f), 
	vec3(0.0f, 0.5f, 1.0f), 
	vec3(0.0f, 1.0f, 1.0f), 
	vec3(1.0f, 1.0f, 1.0f), 
	vec3(1.0f, 1.0f, 0.0f), 
	vec3(1.0f, 0.5f, 0.0f), 
	vec3(1.0f, 0.0f, 0.0f)
);

void main()
{
    vec3 distToBox = abs((gridPosition - 0.5/gridSize)/(1.0 - 1.0/gridSize) - vec3(0.5));
    if(max(max(distToBox.x, distToBox.y), distToBox.z) > 0.5)
    {
        discard;
        return;
    }

    // Calculate needed derivatives
    float dist = texture(sdfGrid, gridPosition).r;
    float dDist = max(length(vec2(dFdx(dist), dFdy(dist))), 0.0008);

    // Isosurface line
    float surfaceColorWeight = clamp(1.0 - pow(abs(dist) / (dDist * surfaceThickness), 8), 0.0, 1.0);
    
    // Grid lines
    vec3 distToGridAxis = abs(fract(gridPosition * gridSize) - vec3(0.5));
    float distToGrid = min(min((abs(planeNormal.x) < 0.95) ? distToGridAxis.x : 1.0, 
                               (abs(planeNormal.y) < 0.95) ? distToGridAxis.y : 1.0),
                               (abs(planeNormal.z) < 0.95) ? distToGridAxis.z : 1.0);
    float gridColorWeight = float(printGrid) * clamp(1.0 - pow(distToGrid / (sqrt(0.1 * distToCamera) * gridThickness), 8), 0.0, 1.0);

    // Isolevels lines
    float distToLevel = 0.5 - abs(fract(abs(dist) / linesSpace) - 0.5);
    float dDistToLevel = dDist / linesSpace;
    float linesColorWeight = float(printIsolines) * 0.5 * clamp(1.0 - pow(abs(distToLevel) / (dDistToLevel * linesThickness), 8), 0.0, 1.0);

    // Heat map color
    dist = 0.5 + 0.5 * dist / gridValueRange;
    float index = clamp(dist * (paletteNumColors-1), 0.0, float(paletteNumColors-1) - 0.01);
    vec3 finalColor = mix(palette[int(index)], palette[int(index)+1], fract(index));

    fragColor = vec4(mix(finalColor, vec3(0.0, 0.0, 0.0), max(max(surfaceColorWeight, gridColorWeight), linesColorWeight)), 1.0);
}