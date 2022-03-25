#version 430

out vec4 fragColor;

in vec3 gridPosition;
in float distToCamera;

layout(std430, binding = 3) buffer octree
{
    uint octreeData[];
};

uniform vec3 planeNormal; // normalized

uniform float octreeValueRange = 1.0;
uniform vec3 startGridSize;

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

const uint isLeafMask = 1 << 31;
const uint childrenIndexMask = ~(1 << 31);

uint roundFloat(float a)
{
    return (a > 0.5) ? 1 : 0;
}

float getDistance(vec3 point, out float distToGrid)
{
    vec3 fracPart = point * startGridSize;
    ivec3 arrayPos = ivec3(floor(fracPart));
    fracPart = fract(fracPart);
    int index = arrayPos.z * int(startGridSize.y * startGridSize.x) +
                arrayPos.y * int(startGridSize.x) +
                arrayPos.x;
    uint currentNode = octreeData[index];

    while(!bool(currentNode & isLeafMask))
    {
        uint childIdx = (roundFloat(fracPart.z) << 2) + 
                        (roundFloat(fracPart.y) << 1) + 
                         roundFloat(fracPart.x);

        currentNode = octreeData[(currentNode & childrenIndexMask) + childIdx];
        fracPart = fract(2.0 * fracPart);
    }

    vec3 distToGridAxis = vec3(0.5) - abs(fracPart - vec3(0.5));
    distToGrid = min(min((abs(planeNormal.x) < 0.95) ? distToGridAxis.x : 1.0, 
                         (abs(planeNormal.y) < 0.95) ? distToGridAxis.y : 1.0),
                         (abs(planeNormal.z) < 0.95) ? distToGridAxis.z : 1.0);

    uint vIndex = currentNode & childrenIndexMask;

    float d00 = uintBitsToFloat(octreeData[vIndex]) * (1.0f - fracPart.x) +
                uintBitsToFloat(octreeData[vIndex + 1]) * fracPart.x;
    float d01 = uintBitsToFloat(octreeData[vIndex + 2]) * (1.0f - fracPart.x) +
                uintBitsToFloat(octreeData[vIndex + 3]) * fracPart.x;
    float d10 = uintBitsToFloat(octreeData[vIndex + 4]) * (1.0f - fracPart.x) +
                uintBitsToFloat(octreeData[vIndex + 5]) * fracPart.x;
    float d11 = uintBitsToFloat(octreeData[vIndex + 6]) * (1.0f - fracPart.x) +
                uintBitsToFloat(octreeData[vIndex + 7]) * fracPart.x;

    float d0 = d00 * (1.0f - fracPart.y) + d01 * fracPart.y;
    float d1 = d10 * (1.0f - fracPart.y) + d11 * fracPart.y;

    return d0 * (1.0f - fracPart.z) + d1 * fracPart.z;
}

void main()
{
    vec3 distToBox = abs(gridPosition - vec3(0.5));
    if(max(max(distToBox.x, distToBox.y), distToBox.z) > 0.5)
    {
        discard;
        return;
    }

    uint firstElem = octreeData[0];

    // Calculate needed derivatives
    float distToGrid = 0.0;
    float dist = getDistance(gridPosition, distToGrid);
    float dDist = max(length(vec2(dFdx(dist), dFdy(dist))), 0.0008);

    // Isosurface line
    float surfaceColorWeight = clamp(1.0 - pow(abs(dist) / (dDist * surfaceThickness), 8), 0.0, 1.0);
    
    // Grid lines
    float gridColorWeight = float(printGrid) * clamp(1.0 - pow(distToGrid / (sqrt(0.1 * distToCamera) * gridThickness), 8), 0.0, 1.0);

    // Isolevels lines
    float distToLevel = 0.5 - abs(fract(abs(dist) / linesSpace) - 0.5);
    float dDistToLevel = dDist / linesSpace;
    float linesColorWeight = float(printIsolines) * 0.5 * clamp(1.0 - pow(abs(distToLevel) / (dDistToLevel * linesThickness), 8), 0.0, 1.0);

    // Heat map color
    dist = 0.5 + 0.5 * dist / octreeValueRange;
    float index = clamp(dist * (paletteNumColors-1), 0.0, float(paletteNumColors-1) - 0.01);
    vec3 finalColor = mix(palette[int(index)], palette[int(index)+1], fract(index));

    fragColor = vec4(mix(finalColor, vec3(0.0, 0.0, 0.0), max(max(surfaceColorWeight, gridColorWeight), linesColorWeight)), 1.0);
}