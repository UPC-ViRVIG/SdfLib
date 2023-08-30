#version 460 core

#define MAX_ITERATIONS 1024

uniform vec3 startGridSize;
layout(std430, binding = 3) buffer octree
{
    uint octreeData[];
};

const uint isLeafMask = 1 << 31;
const uint childrenIndexMask = ~(1 << 31);

uint roundFloat(float a)
{
    return (a >= 0.5) ? 1 : 0;
}

uniform vec3 materialAlbedoColor;
uniform float minBorderValue;
uniform float distanceScale;
uniform float time;

in vec3 gridPosition;
in vec3 gridNormal;
in vec3 cameraPos;

out vec4 fragColor;

// Light functions
vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

const float PI = 3.14159265359;

float DistributionGGX(vec3 N, vec3 H, float roughness)
{
    float a      = roughness*roughness;
    float a2     = a*a;
    float NdotH  = max(dot(N, H), 0.0);
    float NdotH2 = NdotH*NdotH;
	
    float num   = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;
	
    return num / denom;
}

float GeometrySchlickGGX(float NdotV, float roughness)
{
    float r = (roughness + 1.0);
    float k = (r*r) / 8.0;

    float num   = NdotV;
    float denom = NdotV * (1.0 - k) + k;
	
    return num / denom;
}
float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float ggx2  = GeometrySchlickGGX(NdotV, roughness);
    float ggx1  = GeometrySchlickGGX(NdotL, roughness);
	
    return ggx1 * ggx2;
}

// float getDistance(vec3 point, out float distToGrid, out float nodeRelativeLength, out float depth)
// {
//     vec3 fracPart = point * startGridSize;
//     ivec3 arrayPos = ivec3(floor(fracPart));
//     fracPart = fract(fracPart);

//     if(arrayPos.x < 0 || arrayPos.y < 0 || arrayPos.z < 0 ||
//        arrayPos.x >= startGridSize.x || arrayPos.y >= startGridSize.y || arrayPos.z >= startGridSize.z)
//     {
//         vec3 q = abs(point - vec3(0.5)) - 0.5;
//         return length(max(q, vec3(0.0))) + minBorderValue;
//     }

//     int index = arrayPos.z * int(startGridSize.y * startGridSize.x) +
//                 arrayPos.y * int(startGridSize.x) +
//                 arrayPos.x;
//     uint currentNode = octreeData[index];
//     nodeRelativeLength = 1.0;
//     depth = 0.0;

//     while(!bool(currentNode & isLeafMask))
//     {
//         uint childIdx = (roundFloat(fracPart.z) << 2) + 
//                         (roundFloat(fracPart.y) << 1) + 
//                          roundFloat(fracPart.x);

//         currentNode = octreeData[(currentNode & childrenIndexMask) + childIdx];
//         fracPart = fract(2.0 * fracPart);
//         nodeRelativeLength *= 0.5;
//         depth += 1.0;
//     }

//     vec3 distToGridAxis = vec3(0.5) - abs(fracPart - vec3(0.5));
//     distToGrid = min(min(distToGridAxis.x, distToGridAxis.y), distToGridAxis.z);

//     uint vIndex = currentNode & childrenIndexMask;

//     return 0.0f
//          + uintBitsToFloat(octreeData[vIndex + 0]) + uintBitsToFloat(octreeData[vIndex + 1]) * fracPart[0] + uintBitsToFloat(octreeData[vIndex + 2]) * fracPart[0] * fracPart[0] + uintBitsToFloat(octreeData[vIndex + 3]) * fracPart[0] * fracPart[0] * fracPart[0] + uintBitsToFloat(octreeData[vIndex + 4]) * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 5]) * fracPart[0] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 6]) * fracPart[0] * fracPart[0] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 7]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 8]) * fracPart[1] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 9]) * fracPart[0] * fracPart[1] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 10]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 11]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 12]) * fracPart[1] * fracPart[1] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 13]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 14]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 15]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1]
//          + uintBitsToFloat(octreeData[vIndex + 16]) * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 17]) * fracPart[0] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 18]) * fracPart[0] * fracPart[0] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 19]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 20]) * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 21]) * fracPart[0] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 22]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 23]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 24]) * fracPart[1] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 25]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 26]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 27]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 28]) * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 29]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 30]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 31]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2]
//          + uintBitsToFloat(octreeData[vIndex + 32]) * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 33]) * fracPart[0] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 34]) * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 35]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 36]) * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 37]) * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 38]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 39]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 40]) * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 41]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 42]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 43]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 44]) * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 45]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 46]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 47]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2]
//          + uintBitsToFloat(octreeData[vIndex + 48]) * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 49]) * fracPart[0] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 50]) * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 51]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 52]) * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 53]) * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 54]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 55]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 56]) * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 57]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 58]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 59]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 60]) * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 61]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 62]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 63]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2];
// }

float getDistance(vec3 point)
{
    vec3 fracPart = point * startGridSize;
    ivec3 arrayPos = ivec3(floor(fracPart));

    if(arrayPos.x < 0 || arrayPos.y < 0 || arrayPos.z < 0 ||
       arrayPos.x >= startGridSize.x || arrayPos.y >= startGridSize.y || arrayPos.z >= startGridSize.z)
    {
            vec3 q = abs(point - vec3(0.5)) - 0.5;
            point = clamp(point, vec3(1e4, 1e4, 1e4), vec3(1.0 - 1e4, 1.0 - 1e4, 1.0 - 1e4));
            return length(max(q, vec3(0.0)))/distanceScale + minBorderValue;
            //return length(max(q, vec3(0.0)))/distanceScale + getDistance(clampCoord);
    }

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

    uint vIndex = currentNode & childrenIndexMask;

    return 0.0
         + uintBitsToFloat(octreeData[vIndex + 0]) + uintBitsToFloat(octreeData[vIndex + 1]) * fracPart[0] + uintBitsToFloat(octreeData[vIndex + 2]) * fracPart[0] * fracPart[0] + uintBitsToFloat(octreeData[vIndex + 3]) * fracPart[0] * fracPart[0] * fracPart[0] + uintBitsToFloat(octreeData[vIndex + 4]) * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 5]) * fracPart[0] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 6]) * fracPart[0] * fracPart[0] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 7]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 8]) * fracPart[1] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 9]) * fracPart[0] * fracPart[1] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 10]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 11]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 12]) * fracPart[1] * fracPart[1] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 13]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 14]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] + uintBitsToFloat(octreeData[vIndex + 15]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1]
         + uintBitsToFloat(octreeData[vIndex + 16]) * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 17]) * fracPart[0] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 18]) * fracPart[0] * fracPart[0] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 19]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 20]) * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 21]) * fracPart[0] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 22]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 23]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 24]) * fracPart[1] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 25]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 26]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 27]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 28]) * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 29]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 30]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 31]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2]
         + uintBitsToFloat(octreeData[vIndex + 32]) * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 33]) * fracPart[0] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 34]) * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 35]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 36]) * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 37]) * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 38]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 39]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 40]) * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 41]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 42]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 43]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 44]) * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 45]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 46]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 47]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2]
         + uintBitsToFloat(octreeData[vIndex + 48]) * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 49]) * fracPart[0] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 50]) * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 51]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 52]) * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 53]) * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 54]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 55]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 56]) * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 57]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 58]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 59]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 60]) * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 61]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 62]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + uintBitsToFloat(octreeData[vIndex + 63]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2];
}

vec3 getGradient(vec3 point)
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

    uint vIndex = currentNode & childrenIndexMask;

    return normalize(vec3((1 * uintBitsToFloat(octreeData[vIndex + 1]) + 2 * uintBitsToFloat(octreeData[vIndex + 2]) * fracPart[0] + 3 * uintBitsToFloat(octreeData[vIndex + 3]) * fracPart[0] * fracPart[0] + 1 * uintBitsToFloat(octreeData[vIndex + 5]) * fracPart[1] + 2 * uintBitsToFloat(octreeData[vIndex + 6]) * fracPart[0] * fracPart[1] + 3 * uintBitsToFloat(octreeData[vIndex + 7]) * fracPart[0] * fracPart[0] * fracPart[1] + 1 * uintBitsToFloat(octreeData[vIndex + 9]) * fracPart[1] * fracPart[1] + 2 * uintBitsToFloat(octreeData[vIndex + 10]) * fracPart[0] * fracPart[1] * fracPart[1] + 3 * uintBitsToFloat(octreeData[vIndex + 11]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] + 1 * uintBitsToFloat(octreeData[vIndex + 13]) * fracPart[1] * fracPart[1] * fracPart[1] + 2 * uintBitsToFloat(octreeData[vIndex + 14]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] + 3 * uintBitsToFloat(octreeData[vIndex + 15]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1]
        + 1 * uintBitsToFloat(octreeData[vIndex + 17]) * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 18]) * fracPart[0] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 19]) * fracPart[0] * fracPart[0] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 21]) * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 22]) * fracPart[0] * fracPart[1] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 23]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 25]) * fracPart[1] * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 26]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 27]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 29]) * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 30]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 31]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2]
        + 1 * uintBitsToFloat(octreeData[vIndex + 33]) * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 34]) * fracPart[0] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 35]) * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 37]) * fracPart[1] * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 38]) * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 39]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 41]) * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 42]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 43]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 45]) * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 46]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 47]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2]
        + 1 * uintBitsToFloat(octreeData[vIndex + 49]) * fracPart[2] * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 50]) * fracPart[0] * fracPart[2] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 51]) * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 53]) * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 54]) * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 55]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 57]) * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 58]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 59]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 61]) * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 62]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 63]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2]),
        (1 * uintBitsToFloat(octreeData[vIndex + 4]) + 1 * uintBitsToFloat(octreeData[vIndex + 5]) * fracPart[0] + 1 * uintBitsToFloat(octreeData[vIndex + 6]) * fracPart[0] * fracPart[0] + 1 * uintBitsToFloat(octreeData[vIndex + 7]) * fracPart[0] * fracPart[0] * fracPart[0] + 2 * uintBitsToFloat(octreeData[vIndex + 8]) * fracPart[1] + 2 * uintBitsToFloat(octreeData[vIndex + 9]) * fracPart[0] * fracPart[1] + 2 * uintBitsToFloat(octreeData[vIndex + 10]) * fracPart[0] * fracPart[0] * fracPart[1] + 2 * uintBitsToFloat(octreeData[vIndex + 11]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] + 3 * uintBitsToFloat(octreeData[vIndex + 12]) * fracPart[1] * fracPart[1] + 3 * uintBitsToFloat(octreeData[vIndex + 13]) * fracPart[0] * fracPart[1] * fracPart[1] + 3 * uintBitsToFloat(octreeData[vIndex + 14]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] + 3 * uintBitsToFloat(octreeData[vIndex + 15]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1]
        + 1 * uintBitsToFloat(octreeData[vIndex + 20]) * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 21]) * fracPart[0] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 22]) * fracPart[0] * fracPart[0] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 23]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 24]) * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 25]) * fracPart[0] * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 26]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 27]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 28]) * fracPart[1] * fracPart[1] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 29]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 30]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 31]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2]
        + 1 * uintBitsToFloat(octreeData[vIndex + 36]) * fracPart[2] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 37]) * fracPart[0] * fracPart[2] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 38]) * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 39]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 40]) * fracPart[1] * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 41]) * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 42]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 43]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 44]) * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 45]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 46]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 47]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2]
        + 1 * uintBitsToFloat(octreeData[vIndex + 52]) * fracPart[2] * fracPart[2] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 53]) * fracPart[0] * fracPart[2] * fracPart[2] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 54]) * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] * fracPart[2] + 1 * uintBitsToFloat(octreeData[vIndex + 55]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 56]) * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 57]) * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 58]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 59]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 60]) * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 61]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 62]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 63]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] * fracPart[2]),
        (1 * uintBitsToFloat(octreeData[vIndex + 16]) + 1 * uintBitsToFloat(octreeData[vIndex + 17]) * fracPart[0] + 1 * uintBitsToFloat(octreeData[vIndex + 18]) * fracPart[0] * fracPart[0] + 1 * uintBitsToFloat(octreeData[vIndex + 19]) * fracPart[0] * fracPart[0] * fracPart[0] + 1 * uintBitsToFloat(octreeData[vIndex + 20]) * fracPart[1] + 1 * uintBitsToFloat(octreeData[vIndex + 21]) * fracPart[0] * fracPart[1] + 1 * uintBitsToFloat(octreeData[vIndex + 22]) * fracPart[0] * fracPart[0] * fracPart[1] + 1 * uintBitsToFloat(octreeData[vIndex + 23]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] + 1 * uintBitsToFloat(octreeData[vIndex + 24]) * fracPart[1] * fracPart[1] + 1 * uintBitsToFloat(octreeData[vIndex + 25]) * fracPart[0] * fracPart[1] * fracPart[1] + 1 * uintBitsToFloat(octreeData[vIndex + 26]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] + 1 * uintBitsToFloat(octreeData[vIndex + 27]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] + 1 * uintBitsToFloat(octreeData[vIndex + 28]) * fracPart[1] * fracPart[1] * fracPart[1] + 1 * uintBitsToFloat(octreeData[vIndex + 29]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] + 1 * uintBitsToFloat(octreeData[vIndex + 30]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] + 1 * uintBitsToFloat(octreeData[vIndex + 31]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1]
        + 2 * uintBitsToFloat(octreeData[vIndex + 32]) * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 33]) * fracPart[0] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 34]) * fracPart[0] * fracPart[0] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 35]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 36]) * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 37]) * fracPart[0] * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 38]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 39]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 40]) * fracPart[1] * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 41]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 42]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 43]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 44]) * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 45]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 46]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] + 2 * uintBitsToFloat(octreeData[vIndex + 47]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2]
        + 3 * uintBitsToFloat(octreeData[vIndex + 48]) * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 49]) * fracPart[0] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 50]) * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 51]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 52]) * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 53]) * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 54]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 55]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 56]) * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 57]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 58]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 59]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 60]) * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 61]) * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 62]) * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2] + 3 * uintBitsToFloat(octreeData[vIndex + 63]) * fracPart[0] * fracPart[0] * fracPart[0] * fracPart[1] * fracPart[1] * fracPart[1] * fracPart[2] * fracPart[2])
    ));
}

float map(vec3 pos)
{
    // vec3 aPos = pos + vec3(-0.5, -0.1, -0.5);
    // return min(distanceScale * getDistance(pos), max(length(aPos.xz) - 1.3, abs(aPos.y) - 0.07));
    return distanceScale * getDistance(pos);
}

float getAO(vec3 pos, vec3 n)
{
    float occ = 0.0;
    float decay = 1.0;
    for(int i=0; i < 8; i++)
    {
        float h = 0.005 + 0.05 * float(i)/8.0;
        float d = max(map(pos + n * h), 0.0);
        occ += max(h-d, 0.0);
        decay *= 0.85;
    }

    return max(1.0 - 1.7 * occ, 0.0);
}

float softshadow(vec3 ro, vec3 rd)
{
    float res = 1.0;
    float ph = 1e20;
    float t = 0.009;
    for( int i=0; i < 512 && t < 10.0; i++ )
    {
        float h = map(ro + rd*t);
        if( h < 1e-4 ) return 0.0;
        // float y = h * h / (2.0 * ph);
        // float d = sqrt(h * h - y * y);
        // res = min(res, d / max(0.0,t-y));
        res = min(res, h/t);
        ph = h;
        t += h;
    }
    return res;
}

float softshadowToPoint(vec3 ro, vec3 rd, float far)
{
    float res = 1.0;
    float ph = 1e20;
    float t = 0.005;
    for( int i=0; i < 512 && t < far; i++ )
    {
        float h = map(ro + rd*t);
        if( h < 1e-3 ) return 0.0;
        // float y = h * h / (2.0 * ph);
        // float d = sqrt(h * h - y * y);
        // res = min(res, d / max(0.0,t-y));
        res = min(res, h/t);
        ph = h;
        t += h;
    }
    return res;
}

vec3 mapGradient(vec3 pos)
{
    return getGradient(pos);
}

vec3 mapColor(vec3 pos, vec3 cameraPos)
{
    float metallic = 0.1;
    float roughness = 0.7;

    vec3 N = normalize(gridNormal);
    vec3 V = normalize(cameraPos - pos);

    vec3 aPos = pos + vec3(-0.5, -0.1, -0.5);
    float fd = max(length(aPos.xz) - 1.3, abs(aPos.y) - 0.07);

    // Object color
    // vec3 albedo = (fd < 1e-4) ? vec3(0.7, 0.7, 0.7) : vec3(0.72, 0.45, 0.20);
    // vec3 albedo = (pos.y < 0.163) ? vec3(0.7, 0.7, 0.7) : vec3(26.0 / 255.0, 1.0, 102.0 / 255.0);
    // vec3 albedo = vec3(0.867, 0.831, 0.773);
    // vec3 albedo = vec3(26.0 / 255.0, 1.0, 102.0 / 255.0);
    // vec3 albedo = (pos.y < 0.315) ? vec3(0.7, 0.7, 0.7) : vec3(0.4707, 0.173, 0.554);

    // Fresnel parameter
    vec3 F0 = vec3(0.17, 0.17, 0.17);
    F0 = mix(F0, materialAlbedoColor, metallic);

    vec3 Lo = vec3(0.0);
    // Directional light
    {
        // Light position
        vec3 lightPos = vec3(0.873572, 1.42857, 1.09321);
        // vec3 lightPos = vec3(0.609615, 0.517692, 0.501346);
        // float t = 0.5 * sin(time) + 0.5;
        // vec3 lightPos = vec3(0.56, 0.501346, 0.496346) * (1.0-t) + vec3(0.44, 0.501346, 0.496346) * t;

        float distToLight = length(lightPos - pos);
        vec3 L = normalize(lightPos - pos);
        vec3 H = normalize(V + L);
        vec3 sunColor = 20.0 * vec3(1.0, 0.8, 0.6); // Mix light color and intensity
        // float intensity = min(softshadowToPoint(pos + 0.0001 * N, L, distToLight)/0.0999, 1.0);
        // float intensity = min(softshadow(pos + 0.0001 * N, L)/0.0999, 1.0);
        float sphereLightRadius = 0.0961538;
        float coneAngle = atan(sphereLightRadius/distToLight); // Compute the cone angle if the light was totally visible
        float intensity = min(atan(softshadowToPoint(pos + 0.0003 * N, L, distToLight))/coneAngle, 1.0);
        // float intensity = 1.0;
        vec3 radiance = sunColor * intensity;
        
        // Cook-torrance brdf
        float NDF = DistributionGGX(N, H, roughness);        
        float G = GeometrySmith(N, V, L, roughness);      
        vec3 F = fresnelSchlick(max(dot(H, V), 0.0), F0);       
        
        vec3 kS = F;
        vec3 kD = vec3(1.0) - kS;
        kD *= 1.0 - metallic;	  
        
        vec3 numerator = NDF * G * F;
        float denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 0.0001;
        vec3 specular = numerator / denominator;  
            
        // Add to outgoing radiance to Lo
        float NdotL = max(dot(N, L), 0.0);                
        Lo += (kD * materialAlbedoColor / PI + specular) * radiance * NdotL;
    }

    vec3 ambient = vec3(0.5) * materialAlbedoColor * getAO(pos, N); // Ambient light estimation
    vec3 color = ambient + Lo;

    color = color / (color + vec3(1.0));
    color = pow(color, vec3(1.0/2.2));
   
    return color;
}

// bool raycast(vec3 startPos, vec3 dir, out vec3 resultPos, out float distToGrid, out float nodeRelativeLength, out float depth)
// bool raycast(vec3 startPos, vec3 dir, out vec3 resultPos)
// {
//     float accDistance = 0.0;
//     vec3 pos = startPos;
//     float lastDistance = 1e8;
//     uint it = 0;
//     while (lastDistance > 1e-5 && accDistance < nearAndFarPlane.y && it < MAX_ITERATIONS)
//     {
//         resultPos = pos;
//         // lastDistance = distanceScale * getDistance(pos, distToGrid, nodeRelativeLength, depth);
//         lastDistance = map(pos);
//         float dist = max(lastDistance, 0.0);
//         accDistance += dist;
//         pos += dir * dist;
//         it += 1;
//     }
//     return lastDistance < 1e-5;
// }

// std::array<glm::vec3, 5> colorsPalette = 
// {
//     glm::vec3(1.0f, 0.0f, 1.0f), 
//     glm::vec3(0.0f, 0.0f, 1.0f), 
//     glm::vec3(0.0f, 1.0f, 0.0f), 
//     glm::vec3(1.0f, 1.0f, 0.0f), 
//     glm::vec3(1.0f, 0.0f, 0.0f),
// };

const int paletteNumColors = 5;
const vec3 palette[5] = vec3[5](
    vec3(1.0f, 0.0f, 1.0f), 
    vec3(0.0f, 0.0f, 1.0f), 
    vec3(0.0f, 1.0f, 0.0f), 
    vec3(1.0f, 1.0f, 0.0f), 
    vec3(1.0f, 0.0f, 0.0f)
);

void main()
{
    vec3 outColor = vec3(0.9);
    
    vec3 hitPoint = gridPosition;
    outColor = mapColor(hitPoint, cameraPos);

    fragColor = vec4(outColor, 1.0);
}