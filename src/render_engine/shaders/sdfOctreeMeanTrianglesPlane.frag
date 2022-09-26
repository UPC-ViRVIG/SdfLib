#version 430

out vec4 fragColor;

in vec3 gridPosition;
in float distToCamera;

layout(std430, binding = 3) buffer octree
{
    uint octreeData[];
};

uniform vec3 planeNormal; // normalized

uniform float minNumTriangles;
uniform float maxNumTriangles;
uniform vec3 startGridSize;

uniform float gridThickness = 0.0035;

// const int paletteNumColors = 7;
// const vec3 palette[7] = vec3[7](
// 	vec3(0.0f, 0.0f, 1.0f), 
// 	vec3(0.0f, 0.5f, 1.0f), 
// 	vec3(0.0f, 1.0f, 1.0f), 
// 	vec3(1.0f, 1.0f, 1.0f), 
// 	vec3(1.0f, 1.0f, 0.0f), 
// 	vec3(1.0f, 0.5f, 0.0f), 
// 	vec3(1.0f, 0.0f, 0.0f)
// );

// const int paletteNumColors = 7;
// const vec3 palette[7] = vec3[7](
// 	vec3(0.99f, 0.94f, 0.85f), 
// 	vec3(0.99f, 0.83f, 0.619f), 
// 	vec3(0.99f, 0.73f, 0.52f), 
// 	vec3(0.99f, 0.55f, 0.35f), 
// 	vec3(0.937f, 0.396f, 0.28f), 
// 	vec3(0.843f, 0.188f, 0.121f), 
// 	vec3(0.6f, 0.0f, 0.0f)
// );

const int paletteNumColors = 5;
const vec3 palette[5] = vec3[5](
    vec3(1.0f, 0.0f, 1.0f),
    vec3(0.0f, 0.0f, 1.0f),
    vec3(0.0f, 1.0f, 0.0f),
    vec3(1.0f, 1.0f, 0.0f),
    vec3(1.0f, 0.0f, 0.0f)
);

const uint isLeafMask = 1 << 31;
const uint childrenIndexMask = ~(1 << 31);

uint roundFloat(float a)
{
    return (a > 0.5) ? 1 : 0;
}

float getNumTriangles(vec3 point, out float distToGrid, out float nodeRelativeLength)
{
    vec3 fracPart = point * startGridSize;
    ivec3 arrayPos = ivec3(floor(fracPart));
    fracPart = fract(fracPart);
    int index = arrayPos.z * int(startGridSize.y * startGridSize.x) +
                arrayPos.y * int(startGridSize.x) +
                arrayPos.x;
    index = min(index, int(startGridSize.x * startGridSize.y * startGridSize.z));
    uint currentNode = octreeData[index];
    nodeRelativeLength = 1.0;

    while(!bool(currentNode & isLeafMask))
    {
        uint childIdx = (roundFloat(fracPart.z) << 2) + 
                        (roundFloat(fracPart.y) << 1) + 
                         roundFloat(fracPart.x);

        currentNode = octreeData[(currentNode & childrenIndexMask) + childIdx];
        fracPart = fract(2.0 * fracPart);
        nodeRelativeLength *= 0.5;
    }

    vec3 distToGridAxis = vec3(0.5) - abs(fracPart - vec3(0.5));
    distToGrid = min(min((abs(planeNormal.x) < 0.95) ? distToGridAxis.x : 1.0, 
                         (abs(planeNormal.y) < 0.95) ? distToGridAxis.y : 1.0),
                         (abs(planeNormal.z) < 0.95) ? distToGridAxis.z : 1.0);

    uint leafIndex = currentNode & childrenIndexMask;

    return childrenIndexMask & octreeData[leafIndex];
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
    float nodeRelativeLength;
    float numTriangles = getNumTriangles(gridPosition, distToGrid, nodeRelativeLength);

    // Grid lines
    // float gridColorWeight = clamp(1.0 - pow(distToGrid / (sqrt(0.1 * distToCamera) * gridThickness), 8), 0.0, 1.0);
    float gridColorWeight = clamp(1.0 - pow(distToGrid*nodeRelativeLength / gridThickness, 8), 0.0, 1.0);

    // Heat map color
    numTriangles = clamp((numTriangles - minNumTriangles) / (maxNumTriangles - minNumTriangles), 0.0, 1.0);
    float index = clamp(numTriangles * (paletteNumColors-1), 0.0, float(paletteNumColors-1) - 0.01);
    vec3 finalColor = mix(palette[int(index)], palette[int(index)+1], fract(index));

    fragColor = vec4(mix(finalColor, vec3(0.0, 0.0, 0.0), gridColorWeight), 1.0);
}