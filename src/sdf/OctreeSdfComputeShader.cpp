#include "OctreeSdf.h"
#include "utils/Timer.h"
#include "sdf/InterpolationMethods.h"
#include "sdf/TrianglesInfluence.h"

#include <string>
#include <cstring>
#include <filesystem>
#include <glm/glm.hpp>
#include <glad/glad.h>
#include <GLFW/glfw3.h>


void compareTrees(uint32_t tIndex1, uint32_t tIndex2, OctreeSdf::OctreeNode* tree1, OctreeSdf::OctreeNode* tree2, float& accError, uint32_t& numSamples, float& maxError)
{
    if(tree1[tIndex1].isLeaf() ^ tree2[tIndex2].isLeaf())
    {
        std::cout << "Trees are not the same" << std::endl;
    }

    if(tree1[tIndex1].isLeaf())
    {
        uint32_t nodeInfo1 = tree1[tIndex1].getChildrenIndex();
        uint32_t nodeInfo2 = tree2[tIndex2].getChildrenIndex();
        for(uint32_t i=0; i < 8; i++)
        {
            accError += glm::abs(tree1[nodeInfo1 + i].value - tree2[nodeInfo2 + i].value);
            numSamples++;

            maxError = glm::max(maxError, glm::abs(tree1[nodeInfo1 + i].value - tree2[nodeInfo2 + i].value));
        }
    }
    else
    {
        compareTrees(
            tree1[tIndex1].getChildrenIndex(),
            tree2[tIndex2].getChildrenIndex(),
            tree1, tree2,
            accError, numSamples, maxError
        );
    }
}

char* loadShaderFromFile(std::string path, unsigned long* length)
{
    std::ifstream file;
	file.open(path, std::ios_base::in | std::ios_base::binary);
	if (!file.good()) return nullptr;
	file.seekg(0, std::ios::end);
	*length = file.tellg();
	(*length)++;
	char* ret = new char[*length];
	file.seekg(0, std::ios::beg);
	file.read(ret, *length);
	file.close();
	ret[(*length) - 1] = 0;
	return ret;
}

#define CHECK_DEBUG_DIRECTORY

template<typename T> 
void setShaderStorage(uint32_t binding, unsigned int& bufferId, std::vector<T> array)
{
    glGenBuffers(1, &bufferId);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, bufferId);
    glBufferData(GL_SHADER_STORAGE_BUFFER, array.size() * sizeof(T), array.data(), GL_STREAM_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, bufferId);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

template<typename T>
void setShaderStorage(uint32_t binding, unsigned int& bufferId, uint32_t size)
{
    glGenBuffers(1, &bufferId);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, bufferId);
    glBufferData(GL_SHADER_STORAGE_BUFFER, size * sizeof(T), NULL, GL_STREAM_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, bufferId);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

template<typename T>
void setShaderStorageData(unsigned int bufferId, uint32_t startIndex, std::vector<T>& array)
{
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, bufferId);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, startIndex * sizeof(T), array.size() * sizeof(T), array.data());
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void setAtomicCounter(uint32_t binding, unsigned int& counterId, uint32_t startValue=0)
{
    glGenBuffers(1, &counterId);
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, counterId);
    glBufferData(GL_ATOMIC_COUNTER_BUFFER, sizeof(uint32_t), &startValue, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, binding, counterId);
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, 0);
}

void setAtomicCounterData(unsigned int counterId, uint32_t startValue=0)
{
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, counterId);
    glBufferData(GL_ATOMIC_COUNTER_BUFFER, sizeof(uint32_t), &startValue, GL_DYNAMIC_DRAW);
}

struct NodeTask
{
    glm::vec3 center;
    float halfSize;
    uint32_t indexParentTriangles;
    uint32_t numParentTriangles;
    uint32_t octreeNodeIndex;
    uint32_t depth;
    // std::array<uint32_t, 8> verticesIndex;
    // std::array<float, 8> verticesDist;
};

struct TriangleDataPadding
{
    TriangleDataPadding() {}
    TriangleDataPadding(TriangleUtils::TriangleData& tData)
    {
        origin = tData.origin;
        glm::mat3x4 m;
        m[0] = glm::mat4x4::col_type(0.0f, tData.transform[0].x, tData.transform[0].y, tData.transform[0].z);
        m[1] = glm::mat4x4::col_type(0.0f, tData.transform[1].x, tData.transform[1].y, tData.transform[1].z);
        m[2] = glm::mat4x4::col_type(0.0f, tData.transform[2].x, tData.transform[2].y, tData.transform[2].z);

        transform = m;
        b = tData.b;
        c = tData.c;
        v2 = tData.v2;
        v3 = tData.v3;
        for(uint32_t i=0; i < 3; i++)
            edgesNormal[i] = glm::vec4(tData.edgesNormal[i], 0.0f);

        for(uint32_t i=0; i < 3; i++)
            verticesNormal[i] = glm::vec4(tData.verticesNormal[i], 0.0f);
    }

    glm::vec3 origin;
    glm::mat3x4 transform;

    // Edge direction normalized in the triangle space
    // The a vector is always the x-axis
    float paddingB;
    glm::vec2 b;
    glm::vec2 c;

    // Vertices position in triangle space
    // v1 is always at the origin
    float v2; // In x-axis
    float paddingV3;
    glm::vec2 v3;

    // Triangle normals transformed
    std::array<glm::vec4, 3> edgesNormal;
    std::array<glm::vec4, 3> verticesNormal;
};

void OctreeSdf::initOctreeInGPU(const Mesh& mesh, uint32_t startDepth, uint32_t maxDepth,
                                float terminationThreshold, OctreeSdf::TerminationRule terminationRule)
{
    Timer timer;
    timer.start();

    // Setup OpenGL
    if (!glfwInit()) {
		std::cout << "Error initializing GLFW" << std::endl;
		return;
	}

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);

	GLFWwindow* mGlfwWindow = glfwCreateWindow(640, 640, "SharpBox", NULL, NULL);
	// glfwMaximizeWindow(glWindow);
	glfwMakeContextCurrent(mGlfwWindow);

    gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);

    // Load shader
    unsigned int computeShaderId = glCreateShader(GL_COMPUTE_SHADER);

    unsigned long length;
    char* fileShader = loadShaderFromFile("./shaders/octreeSdf.comp", &length);
	if (fileShader == nullptr) {
#ifdef CHECK_DEBUG_DIRECTORY
		std::filesystem::path p("octreeSdf.comp");
		fileShader = loadShaderFromFile("../src/sdf/" + p.filename().string(), &length);
		if (fileShader == nullptr)
			std::cout << "File " << "octreeSdf.comp" << " not found" << std::endl;
#else
		std::cout << "File " << "octreeSdf.comp" << " not found" << std::endl;
#endif
	}

    glShaderSource(computeShaderId, 1, &fileShader, NULL);
	glCompileShader(computeShaderId);

    int success;
	glGetShaderiv(computeShaderId, GL_COMPILE_STATUS, &success);
	if (!success) {
		char infoLog[512];
		glGetShaderInfoLog(computeShaderId, 512, NULL, infoLog);
		std::cout << "-> Vertex Shader error ( " << "octreeSdf.comp" << " ):" << std::endl;
		std::cout << infoLog << std::endl;
	}

	delete[] fileShader;

    std::cout << "Shader compiled" << std::endl;

    unsigned int programId = glCreateProgram();
    glAttachShader(programId, computeShaderId);
    glLinkProgram(programId);

    glUseProgram(programId);

    const std::array<glm::vec3, 8> childrens = 
    {
        glm::vec3(-1.0f, -1.0f, -1.0f),
        glm::vec3(1.0f, -1.0f, -1.0f),
        glm::vec3(-1.0f, 1.0f, -1.0f),
        glm::vec3(1.0f, 1.0f, -1.0f),

        glm::vec3(-1.0f, -1.0f, 1.0f),
        glm::vec3(1.0f, -1.0f, 1.0f),
        glm::vec3(-1.0f, 1.0f, 1.0f),
        glm::vec3(1.0f, 1.0f, 1.0f)
    };

    //ShaderParameters
    unsigned int taskCounter;
    unsigned int trianglesCounter;
    unsigned int outputOctreeSizeCounter;

    unsigned int trianglesDataBuffer;
    unsigned int inputTasksBuffer;
    unsigned int outputTasksBuffer;
    unsigned int inputTrianglesBuffer;
    unsigned int outputTrianglesBuffer;
    unsigned int outputOctreeBuffer;
    unsigned int meshIndicesBuffer;
    unsigned int meshVerticesBuffer;


    // Load and reserve data
    setAtomicCounter(0, taskCounter);
    setAtomicCounter(1, trianglesCounter);
    setAtomicCounter(2, outputOctreeSizeCounter);

    std::vector<TriangleUtils::TriangleData> trianglesData(TriangleUtils::calculateMeshTriangleData(mesh));
    std::vector<TriangleDataPadding> trianglesDataPad(trianglesData.size());
    uint32_t idx = 0;
    for(TriangleUtils::TriangleData& elem : trianglesData)
    {
        trianglesDataPad[idx++] = TriangleDataPadding(elem);
    }
    setShaderStorage(3, trianglesDataBuffer, trianglesDataPad);

    uint32_t maxNumTasks = 1 << (3 * maxDepth);
    unsigned int inputTasksBufferBinding = 4;
    setShaderStorage<NodeTask>(4, inputTasksBuffer, maxNumTasks);

    std::vector<uint32_t> inputTriangles(trianglesData.size());
    for(uint32_t i=0; i < inputTriangles.size(); i++) inputTriangles[i] = i;
    std::vector<NodeTask> tasks;
    {
        const uint32_t startOctreeDepth = START_OCTREE_DEPTH;
        PerNodeRegionTrianglesInfluence<TriLinearInterpolation> perNodeRegionTrianglesInfluence;
        perNodeRegionTrianglesInfluence.initCaches(mBox, startOctreeDepth + 1);
        float newSize = 0.5f * mBox.getSize().x * glm::pow(0.5f, startOctreeDepth);
        const glm::vec3 startCenter = mBox.min + newSize;
        const uint32_t voxelsPerAxis = 1 << startOctreeDepth;
        tasks.resize(voxelsPerAxis * voxelsPerAxis * voxelsPerAxis);
        for(uint32_t k=0; k < voxelsPerAxis; k++)
        {
            for(uint32_t j=0; j < voxelsPerAxis; j++)
            {
                for(uint32_t i=0; i < voxelsPerAxis; i++)
                {
                    tasks[k * voxelsPerAxis * voxelsPerAxis + j * voxelsPerAxis + i] = {
                        startCenter + glm::vec3(i, j, k) * 2.0f * newSize,
                        newSize,
                        0,
                        static_cast<uint32_t>(trianglesData.size()),
                        k * voxelsPerAxis * voxelsPerAxis + j * voxelsPerAxis + i,
                        startOctreeDepth
                    };
                    // std::array<float, TriLinearInterpolation::NUM_COEFFICIENTS> nullArray;
                    // perNodeRegionTrianglesInfluence.calculateVerticesInfo(startCenter + glm::vec3(i, j, k) * 2.0f * newSize, newSize, inputTriangles,
                    //                                                         childrens, 0u, nullArray,
                    //                                                         *reinterpret_cast<std::array<std::array<float, 1>, 8>*>(&tasks[k * voxelsPerAxis * voxelsPerAxis + j * voxelsPerAxis + i].verticesDist),
                    //                                                         tasks[k * voxelsPerAxis * voxelsPerAxis + j * voxelsPerAxis + i].verticesIndex,
                    //                                                         mesh, trianglesData);
                }
            }
        }

        setAtomicCounterData(outputOctreeSizeCounter, voxelsPerAxis * voxelsPerAxis * voxelsPerAxis);
    }
    setShaderStorageData(inputTasksBuffer, 0, tasks);

    unsigned int outputTasksBufferBinding = 5;
    setShaderStorage<NodeTask>(5, outputTasksBuffer, maxNumTasks);

    uint32_t maxNumTriangles = 1000 * maxNumTasks;
    unsigned int inputTrianglesBufferBinding = 6;
    setShaderStorage<uint32_t>(6, inputTrianglesBuffer, maxNumTriangles);
    setShaderStorageData(inputTrianglesBuffer, 0, inputTriangles);

    unsigned int outputTrianglesBufferBinding = 7;
    setShaderStorage<uint32_t>(7, outputTrianglesBuffer, maxNumTriangles);
    unsigned int octreeBufferSize = 16 * maxNumTasks;
    setShaderStorage<uint32_t>(8, outputOctreeBuffer, octreeBufferSize);

    setShaderStorage(9, meshIndicesBuffer, mesh.getIndices());
    std::vector<glm::vec4> gpuVertices(mesh.getVertices().size());
    for(uint32_t i=0; i < mesh.getVertices().size(); i++)
    {
        gpuVertices[i] = glm::vec4(mesh.getVertices()[i], 0.0f);
    }
    setShaderStorage(10, meshVerticesBuffer, gpuVertices);

    unsigned int currentNumTasksBuffer;
    std::vector<uint32_t> currentNumTasks(1, tasks.size());
    setShaderStorage(11, currentNumTasksBuffer, currentNumTasks);

    std::cout << "Data allocated" << std::endl;
    SPDLOG_INFO("Data allocation: {}s", timer.getElapsedSeconds());
    timer.start();

    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, taskCounter);
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, trianglesCounter);
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, outputOctreeSizeCounter);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, trianglesDataBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, inputTasksBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, outputTasksBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, inputTrianglesBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, outputTrianglesBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, outputOctreeBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, meshIndicesBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, meshVerticesBuffer);

    std::cout << "First dispatch" << std::endl;
    glDispatchCompute(tasks.size(), 1, 1);

    glFinish();

    std::cout << "end first dispatch" << std::endl;
    glMemoryBarrier(GL_ALL_BARRIER_BITS);

    SPDLOG_INFO("Depth 0, computation time: {}s", timer.getElapsedSeconds());

    for(uint32_t d=2; d <= maxDepth; d++)
    {
        timer.start();
        glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, taskCounter);
        void* counterPtr = glMapBuffer(GL_ATOMIC_COUNTER_BUFFER, GL_READ_ONLY);

        uint32_t numTasks = reinterpret_cast<unsigned int*>(counterPtr)[0];
        SPDLOG_INFO("Depth {}, num tasks {}", d-1, numTasks);

        glUnmapBuffer(GL_ATOMIC_COUNTER_BUFFER);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, outputTasksBuffer);
        void* dataPtr = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);
        {
            NodeTask* outTasksArray = reinterpret_cast<NodeTask*>(dataPtr);
            uint32_t sumTriangles = 0;
            float sumHalfSize = 0.0f;
            for(uint32_t t=0; t < numTasks; t+=8)
            {
                sumTriangles += outTasksArray[t].numParentTriangles;
            }

            SPDLOG_INFO("Depth {}, mean triangles per node {}", d-1, static_cast<float>(sumTriangles) / static_cast<float>(numTasks/8));
            SPDLOG_INFO("Depth {}, depth {}", d-1, outTasksArray[0].depth);
        }
        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

        currentNumTasks[0] = numTasks;
        setShaderStorageData(currentNumTasksBuffer, 0, currentNumTasks);

        std::swap(inputTasksBuffer, outputTasksBuffer);
        std::swap(inputTrianglesBuffer, outputTrianglesBuffer);

        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, inputTasksBufferBinding, inputTasksBuffer);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, outputTasksBufferBinding, outputTasksBuffer);

        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, inputTrianglesBufferBinding, inputTrianglesBuffer);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, outputTrianglesBufferBinding, outputTrianglesBuffer);

        // Reset counter
        setAtomicCounterData(taskCounter, 0);
        setAtomicCounterData(trianglesCounter, 0);

        SPDLOG_INFO("Depth {}, set / get data time: {}s", d, timer.getElapsedSeconds());
        timer.start();
        glDispatchCompute(numTasks, 1, 1);

        glFinish();

        glMemoryBarrier(GL_ALL_BARRIER_BITS);

        SPDLOG_INFO("Depth {}, computation time {}s", d, timer.getElapsedSeconds());
    }

    timer.start();

    std::cout << "end call" << std::endl;

    // glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, outputOctreeSizeCounter);
    // void* counterPtr = glMapBuffer(GL_ATOMIC_COUNTER_BUFFER, GL_READ_ONLY);

    // uint32_t octreeSize = reinterpret_cast<uint32_t*>(counterPtr)[0];

    // glUnmapBuffer(GL_ATOMIC_COUNTER_BUFFER);

    // glBindBuffer(GL_SHADER_STORAGE_BUFFER, outputOctreeBuffer);
    // void* dataPtr = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);
    // OctreeNode* octreeData = reinterpret_cast<OctreeNode*>(dataPtr);
    // std::cout << maxNumTasks << std::endl;
    // std::cout << mOctreeData.size() << " // " << octreeSize << std::endl;
    // {
    //     float accError = 0.0f;
    //     uint32_t numSamples = 0;
    //     float maxError = 0.0f;
    //     const uint32_t startOctreeDepth = START_OCTREE_DEPTH;
    //     const uint32_t voxelsPerAxis = 1 << startOctreeDepth;
    //     for(uint32_t k=0; k < voxelsPerAxis; k++)
    //     {
    //         for(uint32_t j=0; j < voxelsPerAxis; j++)
    //         {
    //             for(uint32_t i=0; i < voxelsPerAxis; i++)
    //             {
    //                 compareTrees(k * voxelsPerAxis * voxelsPerAxis + j * voxelsPerAxis + i, k * voxelsPerAxis * voxelsPerAxis + j * voxelsPerAxis + i,
    //                             mOctreeData.data(), octreeData,
    //                             accError, numSamples, maxError);
    //             }
    //         }
    //     }

    //     SPDLOG_INFO("Mean error {}", accError / static_cast<float>(numSamples));
    //     SPDLOG_INFO("Max error {}", maxError);
    // }
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

    SPDLOG_INFO("Get octree data {}s", timer.getElapsedSeconds());

    // glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, taskCounter);
    // void* counterPtr = glMapBuffer(GL_ATOMIC_COUNTER_BUFFER, GL_READ_ONLY);

    // uint32_t numTasks = reinterpret_cast<unsigned int*>(counterPtr)[0];
    // std::cout << "NumTasks: " << reinterpret_cast<unsigned int*>(counterPtr)[0] << std::endl;

    // glUnmapBuffer(GL_ATOMIC_COUNTER_BUFFER);

    // glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, trianglesCounter);
    // counterPtr = glMapBuffer(GL_ATOMIC_COUNTER_BUFFER, GL_READ_ONLY);
    
    // std::cout << "NumTriangles: " << reinterpret_cast<unsigned int*>(counterPtr)[0] << std::endl;

    // glUnmapBuffer(GL_ATOMIC_COUNTER_BUFFER);
}
