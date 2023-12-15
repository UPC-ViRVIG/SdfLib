#version 460 core

#define MAX_AO_ITERATIONS 8

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

uniform float epsilon;

//Options 
uniform int maxIterations;
uniform int maxShadowIterations;

uniform float overRelaxation;
uniform bool useAO;

uniform bool useSoftShadows;

//Lighting
uniform int lightNumber;
uniform vec3 lightPos[4];
uniform float lightIntensity[4];
uniform vec3 lightColor[4];
uniform float lightRadius[4];

//Material
uniform float matMetallic;
uniform float matRoughness;
uniform vec3 matAlbedo;
uniform vec3 matF0;

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

/*
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
*/

//SCENE
float map(vec3 pos)
{
    // vec3 aPos = pos + vec3(-0.5, -0.1, -0.5);
    // return min(distanceScale * getDistance(pos), max(length(aPos.xz) - 1.3, abs(aPos.y) - 0.07));
    return distanceScale * getDistance(pos);
}

//Gradient of the scene
vec3 mapGradient(vec3 pos)
{
    // Get the exact gradient
    return getGradient(pos);

    // Get an approximation of the gradient
    // float dist = map(pos);
    // return normalize(vec3(
    //     map(pos + vec3(epsilon, 0, 0)) - dist,
    //     map(pos + vec3(0, epsilon, 0)) - dist,
    //     map(pos + vec3(0, 0, epsilon)) - dist
    // ));
}

//LIGHTING
float getAO(vec3 pos, vec3 n)
{
    float occ = 0.0;
    float decay = 1.0;
    for(int i=0; i < MAX_AO_ITERATIONS; i++)
    {
        float h = 0.002 + 0.1 * float(i)/8.0;
        float d = map(pos + n * h);
        occ += max(h-d, 0.0);
        decay *= 0.8;
    }

    return min(1.0 - 1.5 * occ, 1.0);
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

//Attempt to apply over relaxation to soft shadows too
float softshadowOR(vec3 ro, vec3 rd, float far, float omega)
{
    float res = 1.0;
    float ph = 0.0;
    float t = 0.005;
    float stepLength = 0.0;
    for( int i=0; i < maxShadowIterations && t < far; i++ )
    {
        float h = map(ro + rd * t);
        bool fail = omega > 1.0f && (h + ph) < stepLength;
        if (fail)
        {
            stepLength -= omega * stepLength;
            omega = 1.0f;
        }
        else
        {
            stepLength = h * omega;
        }

        ph = h;

        if(!fail && h < epsilon ) return 0.0;

        if (!fail) res = min(res, h/t);

        t += stepLength;
    }
    return res;
}

//Inigo Quilez improved soft shadow
float softshadow( in vec3 ro, in vec3 rd, float mint, float maxt, float w )
{
    float res = 1.0;
    float t = mint;
    for( int i=0; i<maxShadowIterations && t<maxt; i++ )
    {
        float h = map(ro + t*rd);
        res = min( res, h/(w*t) );
        t += clamp(h, 0.005, 0.50);
        if( res<-1.0 || t>maxt ) break;
    }
    res = max(res,-1.0);
    return 0.25*(1.0+res)*(1.0+res)*(2.0-res);
}


vec3 mapColor(vec3 pos, vec3 cameraPos)
{
    //Normal
    vec3 N = normalize(gridNormal);
    //View vector
    vec3 V = normalize(cameraPos - pos);

    //Plane vs model
    vec3 aPos = pos + vec3(-0.5, -0.1, -0.5);
    float fd = max(length(aPos.xz) - 1.3, abs(aPos.y) - 0.07);


    // Fresnel parameter
    vec3 F0 = mix(matF0, matAlbedo, matMetallic);

    vec3 Lo = vec3(0.0);

    // Directional lights
    for (int i = 0; i < lightNumber; i++) 
    {
        float distToLight = length(lightPos[i] - pos);
        vec3 L = normalize(lightPos[i] - pos);
        vec3 H = normalize(V + L);

        vec3 sunColor = lightIntensity[i] * lightColor[i];

        float coneAngle = atan(lightRadius[i]/distToLight);
        float solidAngle = PI * sin(coneAngle) * pow((lightRadius[i]/distToLight), 2.0);
        //float intensity = useSoftShadows ? min(atan(softshadowToPoint(pos + epsilon * N, L, distToLight)) / coneAngle, 1.0) : 1.0f;
        //float intensity = useSoftShadows ? min(atan(softshadowOR(pos + epsilon * N, L, distToLight, overRelaxation)) / coneAngle, 1.0) : 1.0f;
        float intensity = useSoftShadows ? softshadow(pos + epsilon * N, L, 0.005, distToLight, solidAngle) : 1.0f;
        vec3 radiance = sunColor * intensity;
        
        // Cook-torrance brdf
        float NDF = DistributionGGX(N, H, matRoughness);        
        float G = GeometrySmith(N, V, L, matRoughness);      
        vec3 F = fresnelSchlick(max(dot(H, V), 0.0), F0);       
        
        vec3 kS = F;
        vec3 kD = vec3(1.0) - kS;
        kD *= 1.0 - matMetallic;	  
        
        vec3 numerator = NDF * G * F;
        float denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + epsilon;
        vec3 specular = numerator / denominator;  
            
        // Add to outgoing radiance Lo
        float NdotL = max(dot(N, L), 0.0);                
        Lo += (kD * matAlbedo / PI + specular) * radiance * NdotL;
    }

    vec3 ambient = useAO ? vec3(0.5) * matAlbedo * getAO(pos, N) : vec3(0.5) * matAlbedo; // Ambient light estimation
    vec3 color = ambient + Lo;

    color = color / (color + vec3(1.0));
    color = pow(color, vec3(1.0/2.2));
   
    return color;
}

void main()
{
    vec3 outColor = vec3(0.9);
    
    vec3 hitPoint = gridPosition;
    outColor = mapColor(hitPoint, cameraPos);

    fragColor = vec4(outColor, 1.0);
}