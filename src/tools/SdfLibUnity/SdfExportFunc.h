#ifndef SDF_EXPORT_FUNC_H
#define SDF_EXPORT_FUNC_H

#include "SdfLib/utils/Mesh.h"
#include "SdfLib/ExactOctreeSdf.h"
#include "SdfLib/OctreeSdf.h"
#include <vector>

#if _WIN32
    #define EXPORT __declspec(dllexport)
#else
    #define EXPORT  __attribute__((dllexport))
#endif


extern "C" EXPORT void saveSdf(sdflib::SdfFunction* sdfPointer, char* path);

extern "C" EXPORT sdflib::SdfFunction* loadSdf(char* path);

extern "C" EXPORT sdflib::SdfFunction* createExactOctreeSdf(glm::vec3* vertices, uint32_t numVertices, 
                                              uint32_t* indices, uint32_t numIndices,
                                              float bbMinX, float bbMinY, float bbMinZ,
                                              float bbMaxX, float bbMaxY, float bbMaxZ,
                                              uint32_t startOctreeDepth,
                                              uint32_t maxOctreeDepth,
                                              uint32_t minTrianglesPerNode,
                                              uint32_t numThreads);

extern "C" EXPORT sdflib::SdfFunction* createOctreeSdf(glm::vec3* vertices, uint32_t numVertices,
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

extern "C" EXPORT float getDistance(sdflib::SdfFunction* sdfPointer, float pointX, float pointY, float pointZ);

extern "C" EXPORT float getDistanceAndGradient(sdflib::SdfFunction* sdfPointer, float pointX, float pointY, float pointZ, glm::vec3* outGradient);

extern "C" EXPORT glm::vec3 getBBMinPoint(sdflib::SdfFunction* sdfPointer);

extern "C" EXPORT glm::vec3 getBBSize(sdflib::SdfFunction* sdfPointer);

extern "C" EXPORT uint32_t getStartGridSize(sdflib::SdfFunction* sdfPointer);

extern "C" EXPORT uint32_t getOctreeDataSize(sdflib::SdfFunction* sdfPointer);

extern "C" EXPORT void getOctreeData(sdflib::SdfFunction* sdfPointer, uint32_t* data);

extern "C" EXPORT void deleteSdf(sdflib::SdfFunction* sdfPointer);

#endif