// #include "OctreeSdf.h"
// #include "utils/Timer.h"
// #include "sdf/InterpolationMethods.h"
// #include "sdf/TrianglesInfluence.h"

// #include <string>
// #include <cstring>
// #include <filesystem>
// #include <glm/glm.hpp>
// #include <glad/glad.h>
// #include <GLFW/glfw3.h>

// //#define PRINT_TRIANGLES_STATS

// namespace sdflib
// {
// void compareTrees(uint32_t tIndex1, uint32_t tIndex2, OctreeSdf::OctreeNode* tree1, OctreeSdf::OctreeNode* tree2, float& accError, uint32_t& numSamples, float& maxError)
// {
//     if(tree1[tIndex1].isLeaf() ^ tree2[tIndex2].isLeaf())
//     {
//         std::cout << "Trees are not the same" << std::endl;
//         return;
//     }

//     if(tree1[tIndex1].isLeaf())
//     {
//         uint32_t nodeInfo1 = tree1[tIndex1].getChildrenIndex();
//         uint32_t nodeInfo2 = tree2[tIndex2].getChildrenIndex();
//         for(uint32_t i=0; i < 8; i++)
//         {
//             accError += glm::abs(tree1[nodeInfo1 + i].value - tree2[nodeInfo2 + i].value);
//             numSamples++;

//             maxError = glm::max(maxError, glm::abs(tree1[nodeInfo1 + i].value - tree2[nodeInfo2 + i].value));
//         }
//     }
//     else
//     {
//         uint32_t nodeInfo1 = tree1[tIndex1].getChildrenIndex();
//         uint32_t nodeInfo2 = tree2[tIndex2].getChildrenIndex();
//         for(uint32_t i=0; i < 8; i++)
//         {
//             compareTrees(
//                 nodeInfo1 + i,
//                 nodeInfo2 + i,
//                 tree1, tree2,
//                 accError, numSamples, maxError
//             );
//         }
//     }
// }


// void countNodes(uint32_t tIndex, OctreeSdf::OctreeNode* tree, uint32_t& numEndNodes, uint32_t depth)
// {
//     if(tree[tIndex].isLeaf())
//     {
//         numEndNodes += (1 << (3 * (8 - depth)));
//     }
//     else
//     {
//         uint32_t nodeInfo1 = tree[tIndex].getChildrenIndex();
//         for(uint32_t i=0; i < 8; i++)
//         {
//             countNodes(
//                 nodeInfo1 + i,
//                 tree,
//                 numEndNodes, depth+1
//             );
//         }
//     }
// }

// char* loadShaderFromFile(std::string path, unsigned long* length)
// {
//     std::ifstream file;
// 	file.open(path, std::ios_base::in | std::ios_base::binary);
// 	if (!file.good()) return nullptr;
// 	file.seekg(0, std::ios::end);
// 	*length = file.tellg();
// 	(*length)++;
// 	char* ret = new char[*length];
// 	file.seekg(0, std::ios::beg);
// 	file.read(ret, *length);
// 	file.close();
// 	ret[(*length) - 1] = 0;
// 	return ret;
// }

// // #define CHECK_DEBUG_DIRECTORY

// template<typename T> 
// void setShaderStorage(uint32_t binding, unsigned int& bufferId, std::vector<T> array)
// {
//     glGenBuffers(1, &bufferId);
//     glBindBuffer(GL_SHADER_STORAGE_BUFFER, bufferId);
//     glBufferData(GL_SHADER_STORAGE_BUFFER, array.size() * sizeof(T), array.data(), GL_STREAM_DRAW);
//     if(binding != std::numeric_limits<uint32_t>::max()) glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, bufferId);
//     glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
// }

// template<typename T> 
// void setDispatchIndirectBuffer(unsigned int& bufferId, std::vector<T> array)
// {
//     glGenBuffers(1, &bufferId);
//     glBindBuffer(GL_DISPATCH_INDIRECT_BUFFER, bufferId);
//     glBufferData(GL_DISPATCH_INDIRECT_BUFFER, array.size() * sizeof(T), array.data(), GL_STREAM_DRAW);
//     glBindBuffer(GL_DISPATCH_INDIRECT_BUFFER, 0);
// }

// template<typename T>
// void setShaderStorage(uint32_t binding, unsigned int& bufferId, uint32_t size)
// {
//     glGenBuffers(1, &bufferId);
//     glBindBuffer(GL_SHADER_STORAGE_BUFFER, bufferId);
//     glBufferData(GL_SHADER_STORAGE_BUFFER, size * sizeof(T), NULL, GL_STREAM_DRAW);
//     if(binding != std::numeric_limits<uint32_t>::max()) glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, bufferId);
//     glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
// }

// template<typename T>
// void setShaderStorageData(unsigned int bufferId, uint32_t startIndex, std::vector<T>& array)
// {
//     glBindBuffer(GL_SHADER_STORAGE_BUFFER, bufferId);
//     glBufferSubData(GL_SHADER_STORAGE_BUFFER, startIndex * sizeof(T), array.size() * sizeof(T), array.data());
//     glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
// }

// void setAtomicCounter(uint32_t binding, unsigned int& counterId, uint32_t startValue=0)
// {
//     glGenBuffers(1, &counterId);
//     glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, counterId);
//     glBufferData(GL_ATOMIC_COUNTER_BUFFER, sizeof(uint32_t), &startValue, GL_DYNAMIC_DRAW);
//     glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, binding, counterId);
//     glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, 0);
// }

// void setAtomicCounters(uint32_t binding, unsigned int& counterId, uint32_t numCounters, uint32_t startValue=0)
// {
//     glGenBuffers(1, &counterId);
//     glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, counterId);
//     std::vector<uint32_t> counters(numCounters, startValue);
//     glBufferData(GL_ATOMIC_COUNTER_BUFFER, counters.size() * sizeof(uint32_t), counters.data(), GL_DYNAMIC_DRAW);
//     glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, binding, counterId);
//     glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, 0);
// }

// void setAtomicCounterData(unsigned int counterId, uint32_t startValue=0)
// {
//     glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, counterId);
//     glBufferData(GL_ATOMIC_COUNTER_BUFFER, sizeof(uint32_t), &startValue, GL_DYNAMIC_DRAW);
// }

// void setAtomicCountersData(unsigned int counterId, uint32_t numCounters, uint32_t startValue=0)
// {
//     glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, counterId);
//     std::vector<uint32_t> counters(numCounters, startValue);
//     glBufferData(GL_ATOMIC_COUNTER_BUFFER, counters.size() * sizeof(uint32_t), counters.data(), GL_DYNAMIC_DRAW);
// }

// struct NodeTask
// {
//     glm::vec3 center;
//     float halfSize;
//     uint32_t indexParentTriangles;
//     uint32_t numParentTriangles;
//     uint32_t octreeNodeIndex;
//     uint32_t depth;
//     std::array<uint32_t, 8> verticesIndex;
//     std::array<float, 8> verticesDist;
// };

// struct TriangleDataPadding
// {
//     TriangleDataPadding() {}
//     TriangleDataPadding(TriangleUtils::TriangleData& tData)
//     {
//         origin = tData.origin;
//         glm::mat3x4 m;
//         m[0] = glm::mat4x4::col_type(0.0f, tData.transform[0].x, tData.transform[0].y, tData.transform[0].z);
//         m[1] = glm::mat4x4::col_type(0.0f, tData.transform[1].x, tData.transform[1].y, tData.transform[1].z);
//         m[2] = glm::mat4x4::col_type(0.0f, tData.transform[2].x, tData.transform[2].y, tData.transform[2].z);

//         transform = m;
//         b = tData.b;
//         c = tData.c;
//         v2 = tData.v2;
//         v3 = tData.v3;
//         for(uint32_t i=0; i < 3; i++)
//             edgesNormal[i] = glm::vec4(tData.edgesNormal[i], 0.0f);

//         for(uint32_t i=0; i < 3; i++)
//             verticesNormal[i] = glm::vec4(tData.verticesNormal[i], 0.0f);
//     }

//     glm::vec3 origin;
//     glm::mat3x4 transform;

//     // Edge direction normalized in the triangle space
//     // The a vector is always the x-axis
//     float paddingB;
//     glm::vec2 b;
//     glm::vec2 c;

//     // Vertices position in triangle space
//     // v1 is always at the origin
//     float v2; // In x-axis
//     float paddingV3;
//     glm::vec2 v3;

//     // Triangle normals transformed
//     std::array<glm::vec4, 3> edgesNormal;
//     std::array<glm::vec4, 3> verticesNormal;
// };

// struct DispatchIndirectCommand 
// {
//     uint32_t num_groups_x;
//     uint32_t num_groups_y;
//     uint32_t num_groups_z;
// };

// GLenum checkForOpenGLErrors()
// {
//     GLenum errorCode;
//     while((errorCode = glGetError()) != GL_NO_ERROR)
//     {
//         SPDLOG_ERROR("OpenGL error with code {}", errorCode);
//         return errorCode;
//     }

//     return GL_NO_ERROR;
// }

// constexpr uint32_t WORK_GROUP_SIZE = 64;

// void OctreeSdf::initOctreeInGPU(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
//                                 float terminationThreshold, OctreeSdf::TerminationRule terminationRule)
// {
//     Timer timer;
//     Timer totalTimer;
//     timer.start();

//     // Setup OpenGL
//     if (!glfwInit()) {
// 		std::cout << "Error initializing GLFW" << std::endl;
// 		return;
// 	}

//     glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
// 	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);

// 	GLFWwindow* mGlfwWindow = glfwCreateWindow(640, 640, "SharpBox", NULL, NULL);
// 	// glfwMaximizeWindow(glWindow);
// 	glfwMakeContextCurrent(mGlfwWindow);

//     gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);

//     // Load shader
//     unsigned int programId;
//     {
//         unsigned int computeShaderId = glCreateShader(GL_COMPUTE_SHADER);

//         unsigned long length;
//         char* fileShader = loadShaderFromFile("./shaders/octreeSdf.comp", &length);
//         if (fileShader == nullptr) {
//     #ifdef CHECK_DEBUG_DIRECTORY
//             std::filesystem::path p("octreeSdf.comp");
//             fileShader = loadShaderFromFile("../src/sdf/" + p.filename().string(), &length);
//             if (fileShader == nullptr)
//                 std::cout << "File " << "octreeSdf.comp" << " not found" << std::endl;
//     #else
//             std::cout << "File " << "octreeSdf.comp" << " not found" << std::endl;
//     #endif
//         }

//         std::string resFileShader;
//         resFileShader.append("#version 460 core\n\n");
//         resFileShader.append("#define LOCAL_SIZE " + std::to_string(WORK_GROUP_SIZE) + "\n");
//         resFileShader.append("#define MAX_DEPTH " + std::to_string(maxDepth) + "\n");
//         resFileShader.append("#define SQ_TERMINATION_THRESHOLD " + std::to_string(terminationThreshold*terminationThreshold) + "\n\n");
//         resFileShader.append("layout (local_size_x = " + std::to_string(WORK_GROUP_SIZE) + ") in;\n\n");
//         resFileShader.append(fileShader);

//         const char* resFileShaderPtr = resFileShader.c_str();
//         glShaderSource(computeShaderId, 1, &resFileShaderPtr, NULL);
//         glCompileShader(computeShaderId);

//         int success;
//         glGetShaderiv(computeShaderId, GL_COMPILE_STATUS, &success);
//         if (!success) {
//             char infoLog[512];
//             glGetShaderInfoLog(computeShaderId, 512, NULL, infoLog);
//             std::cout << "-> Vertex Shader error ( " << "octreeSdf.comp" << " ):" << std::endl;
//             std::cout << infoLog << std::endl;
//             return;
//         }

//         checkForOpenGLErrors();

//         delete[] fileShader;

//         programId = glCreateProgram();
//         glAttachShader(programId, computeShaderId);
//         glLinkProgram(programId);
//     }

//     std::cout << "Shaders compiled" << std::endl;

//     glUseProgram(programId);

//     const std::array<glm::vec3, 8> childrens = 
//     {
//         glm::vec3(-1.0f, -1.0f, -1.0f),
//         glm::vec3(1.0f, -1.0f, -1.0f),
//         glm::vec3(-1.0f, 1.0f, -1.0f),
//         glm::vec3(1.0f, 1.0f, -1.0f),

//         glm::vec3(-1.0f, -1.0f, 1.0f),
//         glm::vec3(1.0f, -1.0f, 1.0f),
//         glm::vec3(-1.0f, 1.0f, 1.0f),
//         glm::vec3(1.0f, 1.0f, 1.0f)
//     };

//     const std::array<glm::vec3, 19> nodeSamplePoints =
//     {
//         glm::vec3(0.0f, -1.0f, -1.0f),
//         glm::vec3(-1.0f, 0.0f, -1.0f),
//         glm::vec3(0.0f, 0.0f, -1.0f),
//         glm::vec3(1.0f, 0.0f, -1.0f),
//         glm::vec3(0.0f, 1.0f, -1.0f),

//         glm::vec3(-1.0f, -1.0f, 0.0f),
//         glm::vec3(0.0f, -1.0f, 0.0f),
//         glm::vec3(1.0f, -1.0f, 0.0f),
//         glm::vec3(-1.0f, 0.0f, 0.0f),
//         glm::vec3(0.0f),
//         glm::vec3(1.0f, 0.0f, 0.0f),
//         glm::vec3(-1.0f, 1.0f, 0.0f),
//         glm::vec3(0.0f, 1.0f, 0.0f),
//         glm::vec3(1.0f, 1.0f, 0.0f),

//         glm::vec3(0.0f, -1.0f, 1.0f),
//         glm::vec3(-1.0f, 0.0f, 1.0f),
//         glm::vec3(0.0f, 0.0f, 1.0f),
//         glm::vec3(1.0f, 0.0f, 1.0f),
//         glm::vec3(0.0f, 1.0f, 1.0f),
//     };

//     //ShaderParameters
//     unsigned int counters;
//     unsigned int outputOctreeSizeCounter;

//     unsigned int trianglesDataBuffer;
//     unsigned int inputTasksBuffer;
//     unsigned int outputTasksBuffer;
//     unsigned int inputTrianglesBuffer;
//     unsigned int outputTrianglesBuffer;
//     unsigned int outputOctreeBuffer;
//     unsigned int meshIndicesBuffer;
//     unsigned int meshVerticesBuffer;


//     // Load and reserve data
//     /*
//         The atomic counter counters stores:
//         -> Counter of tasks
//         -> Counter of next number of work groups
//         -> Counter of tasks triangles
//     */
//     setAtomicCounters(0, counters, 3);
//     setAtomicCounter(2, outputOctreeSizeCounter);

//     std::vector<TriangleUtils::TriangleData> trianglesData(TriangleUtils::calculateMeshTriangleData(mesh));
//     std::vector<TriangleDataPadding> trianglesDataPad(trianglesData.size());
//     uint32_t idx = 0;
//     for(TriangleUtils::TriangleData& elem : trianglesData)
//     {
//         trianglesDataPad[idx++] = TriangleDataPadding(elem);
//     }
//     setShaderStorage(3, trianglesDataBuffer, trianglesDataPad);

//     uint32_t maxNumTasks = 1 << (3 * maxDepth);
//     unsigned int inputTasksBufferBinding = 4;
//     setShaderStorage<NodeTask>(4, inputTasksBuffer, maxNumTasks);

//     if(checkForOpenGLErrors() == GL_OUT_OF_MEMORY)
//     {
//         SPDLOG_ERROR("Cannot allocate the task array in the GPU");
//         return;
//     }

//     std::vector<uint32_t> inputTriangles(trianglesData.size());
//     for(uint32_t i=0; i < inputTriangles.size(); i++) inputTriangles[i] = i;
//     std::vector<NodeTask> tasks;
//     {
//         const uint32_t startOctreeDepth = START_OCTREE_DEPTH;
//         PerNodeRegionTrianglesInfluence<TriLinearInterpolation> perNodeRegionTrianglesInfluence;
//         perNodeRegionTrianglesInfluence.initCaches(mBox, startOctreeDepth + 1);
//         float newSize = 0.5f * mBox.getSize().x * glm::pow(0.5f, startOctreeDepth);
//         const glm::vec3 startCenter = mBox.min + newSize;
//         const uint32_t voxelsPerAxis = 1 << startOctreeDepth;
//         tasks.resize(voxelsPerAxis * voxelsPerAxis * voxelsPerAxis);
//         for(uint32_t k=0; k < voxelsPerAxis; k++)
//         {
//             for(uint32_t j=0; j < voxelsPerAxis; j++)
//             {
//                 for(uint32_t i=0; i < voxelsPerAxis; i++)
//                 {
//                     tasks[k * voxelsPerAxis * voxelsPerAxis + j * voxelsPerAxis + i] = {
//                         startCenter + glm::vec3(i, j, k) * 2.0f * newSize,
//                         newSize,
//                         0,
//                         static_cast<uint32_t>(trianglesData.size()),
//                         k * voxelsPerAxis * voxelsPerAxis + j * voxelsPerAxis + i,
//                         startOctreeDepth
//                     };
//                     std::array<float, TriLinearInterpolation::NUM_COEFFICIENTS> nullArray;
//                     perNodeRegionTrianglesInfluence.calculateVerticesInfo(startCenter + glm::vec3(i, j, k) * 2.0f * newSize, newSize, inputTriangles,
//                                                                             childrens, 0u, nullArray,
//                                                                             *reinterpret_cast<std::array<std::array<float, 1>, 8>*>(&tasks[k * voxelsPerAxis * voxelsPerAxis + j * voxelsPerAxis + i].verticesDist),
//                                                                             tasks[k * voxelsPerAxis * voxelsPerAxis + j * voxelsPerAxis + i].verticesIndex,
//                                                                             mesh, trianglesData);
//                 }
//             }
//         }

//         setAtomicCounterData(outputOctreeSizeCounter, voxelsPerAxis * voxelsPerAxis * voxelsPerAxis);
//     }
//     setShaderStorageData(inputTasksBuffer, 0, tasks);

//     unsigned int outputTasksBufferBinding = 5;
//     setShaderStorage<NodeTask>(5, outputTasksBuffer, maxNumTasks);

//     uint32_t maxNumTriangles = 32 * maxNumTasks;
//     // {
//     //     float meanTrianglesPerLevel = static_cast<float>(trianglesData.size());
//     //     for(uint32_t d=2; d < maxDepth; d++) meanTrianglesPerLevel = 0.22f * meanTrianglesPerLevel;

//     //     maxNumTriangles = maxNumTasks * static_cast<uint32_t>(glm::ceil(meanTrianglesPerLevel));
//     // }
//     unsigned int inputTrianglesBufferBinding = 6;
//     setShaderStorage<uint32_t>(6, inputTrianglesBuffer, maxNumTriangles);

//     if(checkForOpenGLErrors() == GL_OUT_OF_MEMORY)
//     {
//         SPDLOG_ERROR("Cannot allocate the triangles cache array in the GPU");
//         return;
//     }

//     setShaderStorageData(inputTrianglesBuffer, 0, inputTriangles);

//     unsigned int outputTrianglesBufferBinding = 7;
//     setShaderStorage<uint32_t>(7, outputTrianglesBuffer, maxNumTriangles);

//     unsigned int octreeBufferSize = 0;
//     for(uint32_t d=startDepth; d <= (maxDepth+1); d++) octreeBufferSize += 1 << (3 * d);
//     octreeBufferSize = static_cast<uint32_t>(glm::ceil(1.0f * static_cast<float>(octreeBufferSize)));
//     setShaderStorage<uint32_t>(8, outputOctreeBuffer, octreeBufferSize);

//     if(checkForOpenGLErrors() == GL_OUT_OF_MEMORY)
//     {
//         SPDLOG_ERROR("Cannot allocate the octree array in the GPU");
//         return;
//     }

//     setShaderStorage(9, meshIndicesBuffer, mesh.getIndices());
//     std::vector<glm::vec4> gpuVertices(mesh.getVertices().size());
//     for(uint32_t i=0; i < mesh.getVertices().size(); i++)
//     {
//         gpuVertices[i] = glm::vec4(mesh.getVertices()[i], 0.0f);
//     }
//     setShaderStorage(10, meshVerticesBuffer, gpuVertices);

//     unsigned int currentNumTasksBuffer;
//     std::vector<uint32_t> currentNumTasks(1, tasks.size());
//     setShaderStorage(11, currentNumTasksBuffer, currentNumTasks);

//     unsigned int dispatchIndirectBuffer;
//     std::vector<DispatchIndirectCommand> dispatchIndirectData(1);
//     dispatchIndirectData[0].num_groups_x = tasks.size();
//     dispatchIndirectData[0].num_groups_y = 1;
//     dispatchIndirectData[0].num_groups_z = 1;
//     setDispatchIndirectBuffer(dispatchIndirectBuffer, dispatchIndirectData);

//     std::vector<uint32_t> countersCacheData(3 * (maxDepth+1), 0);
//     unsigned int countersCacheBuffer;
//     setShaderStorage(std::numeric_limits<uint32_t>::max(), countersCacheBuffer, countersCacheData);

//     checkForOpenGLErrors();

//     std::cout << "Data allocated" << std::endl;
//     SPDLOG_INFO("Data allocation: {}s", timer.getElapsedSeconds());

//     std::array<unsigned int, 2> gpuQueries;
//     glGenQueries(2, gpuQueries.data());
    
//     timer.start();
//     totalTimer.start();
//     float totalTime = 0.0f;

//     #ifdef SDFLIB_PRINT_STATISTICS
//     glBeginQuery(GL_TIME_ELAPSED, gpuQueries[1]);
//     #endif

//     // glDispatchCompute((tasks.size() + WORK_GROUP_SIZE - 1) / WORK_GROUP_SIZE, 1, 1);
    
//     glBindBuffer(GL_DISPATCH_INDIRECT_BUFFER, dispatchIndirectBuffer);
//     glDispatchComputeIndirect(0);

//     #ifdef SDFLIB_PRINT_STATISTICS
//     glEndQuery(GL_TIME_ELAPSED);
//     {
//         int done = false;
//         while(!done)
//         {
//             glGetQueryObjectiv(gpuQueries[1], GL_QUERY_RESULT_AVAILABLE, &done);
//         }

//         uint64_t elapsedTime2;
//         glGetQueryObjectui64v(gpuQueries[1], GL_QUERY_RESULT, &elapsedTime2);
//         SPDLOG_INFO("Depth {}, gpu computation time {}ms", 1, static_cast<float>(elapsedTime2) / 1000000.0f);
//         totalTime += static_cast<float>(elapsedTime2) / 1000000000.0f;

//         SPDLOG_INFO("Depth {}, computation time {}s", 1, timer.getElapsedSeconds());
//     }
//     #endif

//     uint32_t lastNotEndedNodes = 8;
//     for(uint32_t d=2; d <= maxDepth; d++)
//     {
//         #ifdef SDFLIB_PRINT_STATISTICS
//         timer.start();
//         #endif

//         #ifdef PRINT_TRIANGLES_STATS
//         // uint32_t numTasks;
//         // glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, counters);
//         // glGetBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 0, sizeof(uint32_t), &numTasks);
//         // SPDLOG_INFO("Depth {}, num tasks {}", d-1, numTasks);
//         // SPDLOG_INFO("Depth {}, ended nodes {}", d-1, 8*lastNotEndedNodes - numTasks/8);
//         // lastNotEndedNodes = numTasks/8;

//         // glUnmapBuffer(GL_ATOMIC_COUNTER_BUFFER);

//         // uint32_t numTriangles;
//         // glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, counters);
//         // glGetBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 8, sizeof(uint32_t), &numTriangles);

//         // if(numTriangles > maxNumTriangles)
//         // {
//         //     SPDLOG_ERROR("Max num of triangles reach");
//         // }

//         // glBindBuffer(GL_SHADER_STORAGE_BUFFER, outputTasksBuffer);
//         // void* dataPtr = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);
//         // {
//         //     NodeTask* outTasksArray = reinterpret_cast<NodeTask*>(dataPtr);
//         //     uint32_t sumTriangles = 0;
//         //     float sumHalfSize = 0.0f;
//         //     float maxHalfSize = 0.0f;
//         //     float minHalfSize = INFINITY;
//         //     std::vector<std::pair<uint32_t, uint32_t>> tasksTriIntervals;
//         //     for(uint32_t t=0; t < numTasks; t+=8)
//         //     {
//         //         maxHalfSize = glm::max(maxHalfSize, outTasksArray[t].halfSize);
//         //         minHalfSize = glm::min(minHalfSize, outTasksArray[t].halfSize);
//         //         sumTriangles += outTasksArray[t].numParentTriangles;
//         //         tasksTriIntervals.push_back(std::make_pair(outTasksArray[t].indexParentTriangles, outTasksArray[t].numParentTriangles));
//         //     }

//         //     std::sort(tasksTriIntervals.begin(), tasksTriIntervals.end(), 
//         //         [] (const std::pair<uint32_t, uint32_t>& p1, const std::pair<uint32_t, uint32_t>& p2) {
//         //             return p1.first < p2.first;
//         //         });
            

//         //     for(uint32_t i=1; i < tasksTriIntervals.size(); i++)
//         //     {
//         //         if(tasksTriIntervals[i-1].first + tasksTriIntervals[i-1].second > tasksTriIntervals[i].first)
//         //         {
//         //             std::cout << "triangle interval error" << std::endl;
//         //         }
//         //     }

//         //     SPDLOG_INFO("Depth {}, half size interval [{}, {}]", d-1, minHalfSize, maxHalfSize);
//         //     SPDLOG_INFO("Depth {}, mean triangles per node {}", d-1, static_cast<float>(sumTriangles) / static_cast<float>(numTasks/8));
//         //     SPDLOG_INFO("Depth {}, depth {}", d-1, outTasksArray[0].depth);
//         // }
//         // glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
//         #endif

//         #ifdef SDFLIB_PRINT_STATISTICS
//         glBeginQuery(GL_TIME_ELAPSED, gpuQueries[0]);
//         #endif

//         glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, counters);
//         glBindBuffer(GL_SHADER_STORAGE_BUFFER, currentNumTasksBuffer);
//         glCopyBufferSubData(GL_ATOMIC_COUNTER_BUFFER, GL_SHADER_STORAGE_BUFFER, 0, 0, sizeof(uint32_t));

//         glBindBuffer(GL_DISPATCH_INDIRECT_BUFFER, dispatchIndirectBuffer);
//         glCopyBufferSubData(GL_ATOMIC_COUNTER_BUFFER, GL_DISPATCH_INDIRECT_BUFFER, 4, 0, sizeof(uint32_t));

//         glBindBuffer(GL_SHADER_STORAGE_BUFFER, countersCacheBuffer);
//         glCopyBufferSubData(GL_ATOMIC_COUNTER_BUFFER, GL_SHADER_STORAGE_BUFFER, 0, (d-1) * 3 * sizeof(uint32_t), 3 * sizeof(uint32_t));

//         std::swap(inputTasksBuffer, outputTasksBuffer);
//         std::swap(inputTrianglesBuffer, outputTrianglesBuffer);

//         glBindBufferBase(GL_SHADER_STORAGE_BUFFER, inputTasksBufferBinding, inputTasksBuffer);
//         glBindBufferBase(GL_SHADER_STORAGE_BUFFER, outputTasksBufferBinding, outputTasksBuffer);

//         glBindBufferBase(GL_SHADER_STORAGE_BUFFER, inputTrianglesBufferBinding, inputTrianglesBuffer);
//         glBindBufferBase(GL_SHADER_STORAGE_BUFFER, outputTrianglesBufferBinding, outputTrianglesBuffer);

//         // Reset counter
//         setAtomicCountersData(counters, 3, 0);

//         #ifdef SDFLIB_PRINT_STATISTICS
//         glEndQuery(GL_TIME_ELAPSED);
//         glBeginQuery(GL_TIME_ELAPSED, gpuQueries[1]);
//         #endif

//         // glDispatchCompute((numTasks + WORK_GROUP_SIZE - 1) / WORK_GROUP_SIZE, 1, 1);
//         glBindBuffer(GL_DISPATCH_INDIRECT_BUFFER, dispatchIndirectBuffer);
//         glDispatchComputeIndirect(0);

//         #ifdef SDFLIB_PRINT_STATISTICS
//         glEndQuery(GL_TIME_ELAPSED);

//         int done = false;
//         while(!done)
//         {
//             glGetQueryObjectiv(gpuQueries[1], GL_QUERY_RESULT_AVAILABLE, &done);
//         }
//         {
//             uint64_t elapsedTime1;
//             glGetQueryObjectui64v(gpuQueries[0], GL_QUERY_RESULT, &elapsedTime1);

//             uint64_t elapsedTime2;
//             glGetQueryObjectui64v(gpuQueries[1], GL_QUERY_RESULT, &elapsedTime2);

//             SPDLOG_INFO("Depth {}, gpu initialization time {}ms", d, static_cast<float>(elapsedTime1) / 1000000.0f);
//             SPDLOG_INFO("Depth {}, gpu computation time {}ms", d, static_cast<float>(elapsedTime2) / 1000000.0f);
//             totalTime += (static_cast<float>(elapsedTime2) + static_cast<float>(elapsedTime1)) / 1000000000.0f;
//         }
//         SPDLOG_INFO("Depth {}, computation time {}s", d, timer.getElapsedSeconds());
//         #endif
//     }

//     glFinish(); glEnd();

//     SPDLOG_INFO("Total computation time {}s", totalTimer.getElapsedSeconds());
//     SPDLOG_INFO("Total gpu time {}s", totalTime);
//     timer.start();

//     glBindBuffer(GL_SHADER_STORAGE_BUFFER, countersCacheBuffer);
//     glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 3 * maxDepth * sizeof(uint32_t), countersCacheData.data());

//     uint32_t octreeSize;
//     glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, outputOctreeSizeCounter);
//     glGetBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 0, sizeof(uint32_t), &octreeSize);

//     std::vector<OctreeNode> octreeData(octreeSize);
//     glBindBuffer(GL_SHADER_STORAGE_BUFFER, outputOctreeBuffer);
//     glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, octreeData.size() * sizeof(OctreeNode), octreeData.data());

//     SPDLOG_INFO("Get octree data {}s", timer.getElapsedSeconds());

//     if(octreeSize > octreeBufferSize)
//     {
//         SPDLOG_ERROR("The final octree is bigger than the allocated memory");
//         return;
//     }

//     for(uint32_t d=1; d < maxDepth; d++)
//     {
//         uint32_t numTasks = countersCacheData[3 * d];
//         uint32_t nextNumWorkGroups = countersCacheData[3 * d + 1];
//         uint32_t sumTriangles = countersCacheData[3 * d + 2];
//         SPDLOG_INFO("Depth {}, not ended nodes {}", d, numTasks);
//         SPDLOG_INFO("Depth {}, next number of work groups {}", d, nextNumWorkGroups);
//         SPDLOG_INFO("Depth {}, number of triangles {}", d, sumTriangles);
//         if(sumTriangles > maxNumTriangles)
//         {
//             SPDLOG_ERROR("At depth {}, the number of triangles saved was bigger than the triangles array", d);
//             return;
//         }
//     }

//     if(mOctreeData.size() != 0)
//     {
//         std::cout << mOctreeData.size() << " // " << octreeSize << std::endl;
//         {
//             float accError = 0.0f;
//             uint32_t numSamples = 0;
//             float maxError = 0.0f;
//             uint32_t numEndNodesCPU = 0;
//             uint32_t numEndNodesGPU = 0;
//             const uint32_t startOctreeDepth = START_OCTREE_DEPTH;
//             const uint32_t voxelsPerAxis = 1 << startOctreeDepth;
//             for(uint32_t k=0; k < voxelsPerAxis; k++)
//             {
//                 for(uint32_t j=0; j < voxelsPerAxis; j++)
//                 {
//                     for(uint32_t i=0; i < voxelsPerAxis; i++)
//                     {
//                         compareTrees(k * voxelsPerAxis * voxelsPerAxis + j * voxelsPerAxis + i, k * voxelsPerAxis * voxelsPerAxis + j * voxelsPerAxis + i,
//                                     mOctreeData.data(), octreeData.data(),
//                                     accError, numSamples, maxError);


//                         countNodes(k * voxelsPerAxis * voxelsPerAxis + j * voxelsPerAxis + i,
//                                     mOctreeData.data(), numEndNodesCPU, 0);

//                         countNodes(k * voxelsPerAxis * voxelsPerAxis + j * voxelsPerAxis + i,
//                                     octreeData.data(), numEndNodesGPU, 0);
//                     }
//                 }
//             }

//             std::cout << numEndNodesCPU << " // " << numEndNodesGPU << std::endl;

//             SPDLOG_INFO("Mean error {}", accError / static_cast<float>(numSamples));
//             SPDLOG_INFO("Max error {}", maxError);
//         }
//     }
    
//     mOctreeData = std::move(octreeData);
// }
// }
