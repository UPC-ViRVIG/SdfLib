#ifndef SDF_EXPORT_FUNC_H
#define SDF_EXPORT_FUNC_H

#include "utils/Mesh.h"
#include "sdf/ExactOctreeSdf.h"
#include "sdf/OctreeSdf.h"
#include <vector>

#define EXPORT __declspec(dllexport)


extern "C" EXPORT void saveSdf(SdfFunction* sdfPointer, char* path);

extern "C" EXPORT SdfFunction* loadSdf(char* path);

extern "C" EXPORT SdfFunction* createExactOctreeSdf(glm::vec3* vertices, uint32_t numVertices, 
                                              uint32_t* indices, uint32_t numIndices,
                                              float bbMinX, float bbMinY, float bbMinZ,
                                              float bbMaxX, float bbMaxY, float bbMaxZ,
                                              uint32_t startOctreeDepth,
                                              uint32_t maxOctreeDepth,
                                              uint32_t minTrianglesPerNode,
                                              uint32_t numThreads);

extern "C" EXPORT SdfFunction* createOctreeSdf(glm::vec3* vertices, uint32_t numVertices,
                                    uint32_t* indices, uint32_t numIndices,
                                    float bbMinX, float bbMinY, float bbMinZ,
                                    float bbMaxX, float bbMaxY, float bbMaxZ,
                                    uint32_t startOctreeDepth,
                                    uint32_t maxOctreeDepth,
                                    float maxError,
                                    uint32_t numThreads);

// extern "C" EXPORT SdfFunction* createExactOctreeSdf(float bbMinX, float bbMinY, float bbMinZ,
//                                                     float bbMaxX, float bbMaxY, float bbMaxZ,
//                                                     uint32_t startOctreeDepth,
//                                                     uint32_t maxOctreeDepth,
//                                                     uint32_t minTrianglesPerNode);

extern "C" EXPORT float getDistance(SdfFunction* sdfPointer, float pointX, float pointY, float pointZ);

extern "C" EXPORT float getDistanceAndGradient(SdfFunction* sdfPointer, float pointX, float pointY, float pointZ, glm::vec3* outGradient);

extern "C" EXPORT glm::vec3 getBBMinPoint(SdfFunction* sdfPointer);

extern "C" EXPORT glm::vec3 getBBSize(SdfFunction* sdfPointer);

extern "C" EXPORT uint32_t getStartGridSize(SdfFunction* sdfPointer);

extern "C" EXPORT uint32_t getOctreeDataSize(SdfFunction* sdfPointer);

extern "C" EXPORT void getOctreeData(SdfFunction* sdfPointer, uint32_t* data);

extern "C" EXPORT void deleteSdf(SdfFunction* sdfPointer);

#endif