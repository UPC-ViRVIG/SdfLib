#ifndef SDF_EXPORT_FUNC_H
#define SDF_EXPORT_FUNC_H

#include "utils/Mesh.h"
#include "sdf/ExactOctreeSdf.h"
#include <vector>

#define EXPORT __declspec(dllexport)


extern "C" EXPORT void saveExactOctreeSdf(SdfFunction* sdfPointer, char* path);

extern "C" EXPORT SdfFunction* loadExactOctreeSdf(char* path);

extern "C" EXPORT SdfFunction* createExactOctreeSdf(glm::vec3* vertices, uint32_t numVertices, 
                                              uint32_t* indices, uint32_t numIndices,
                                              float bbMinX, float bbMinY, float bbMinZ,
                                              float bbMaxX, float bbMaxY, float bbMaxZ,
                                              uint32_t startOctreeDepth,
                                              uint32_t maxOctreeDepth,
                                              uint32_t minTrianglesPerNode);

// extern "C" EXPORT SdfFunction* createExactOctreeSdf(float bbMinX, float bbMinY, float bbMinZ,
//                                                     float bbMaxX, float bbMaxY, float bbMaxZ,
//                                                     uint32_t startOctreeDepth,
//                                                     uint32_t maxOctreeDepth,
//                                                     uint32_t minTrianglesPerNode);

extern "C" EXPORT float getDistance(SdfFunction* sdfPointer, float pointX, float pointY, float pointZ);

extern "C" EXPORT float getDistanceAndGradient(SdfFunction* sdfPointer, float pointX, float pointY, float pointZ, glm::vec3* outGradient);

extern "C" EXPORT void deleteSdf(SdfFunction* sdfPointer);

#endif