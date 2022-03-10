cmake_minimum_required(VERSION 3.20)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/CMakeConfig.txt")
    include("${CMAKE_CURRENT_SOURCE_DIR}/CMakeConfig.txt")
endif()

# Set the project name and version
project(SDFoctree VERSION 0.1)

# Specify the c++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Get all project files
file(GLOB SOURCE_FILES src/*.cpp)
file(GLOB HEADER_FILES src/*.h)

file(GLOB RENDER_ENGINE_SOURCE_FILES src/render_engine/*.cpp)
file(GLOB RENDER_ENGINE_HEADER_FILES src/render_engine/*.h)

file(GLOB RENDER_ENGINE_SHADERS_SOURCE_FILES src/render_engine/shaders/*.cpp)
file(GLOB RENDER_ENGINE_SHADERS_HEADER_FILES src/render_engine/shaders/*.h)

file(GLOB SDF_SOURCE_FILES src/sdf/*.cpp)
file(GLOB SDF_HEADER_FILES src/sdf/*.h)

file(GLOB UTILS_SOURCE_FILES src/utils/*.cpp)
file(GLOB UTILS_HEADER_FILES src/utils/*.h)

# Add libraries
add_library(${PROJECT_NAME}    ${SOURCE_FILES} ${HEADER_FILES}
                               ${RENDER_ENGINE_SOURCE_FILES} ${RENDER_ENGINE_HEADER_FILES}
                               ${RENDER_ENGINE_SHADERS_SOURCE_FILES} ${RENDER_ENGINE_SHADERS_HEADER_FILES}
                               ${SDF_SOURCE_FILES} ${SDF_HEADER_FILES}
                               ${UTILS_SOURCE_FILES} ${UTILS_HEADER_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC src/)

# Add shaders
file(GLOB SHADER_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/src/render_engine/shaders 
                       src/render_engine/shaders/*.frag 
                       src/render_engine/shaders/*.vert)

foreach(SHADER IN LISTS SHADER_FILES)
    add_custom_command(OUTPUT ${SHADER}
            COMMAND cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/src/render_engine/shaders/${SHADER} $<TARGET_FILE_DIR:${PROJECT_NAME}>/shaders/${SHADER}
            DEPENDS src/render_engine/shaders/${SHADER}
        )
endforeach()

add_custom_target(copyShaders ALL SOURCES ${SHADER_FILES})

add_dependencies(${PROJECT_NAME} copyShaders)

# Add dependencies
add_subdirectory(libs)

target_link_libraries(${PROJECT_NAME} PUBLIC glm)
target_link_libraries(${PROJECT_NAME} PUBLIC eigen)
target_link_libraries(${PROJECT_NAME} PUBLIC glfw)
target_link_libraries(${PROJECT_NAME} PUBLIC imgui)
target_link_libraries(${PROJECT_NAME} PUBLIC imguizmo)
target_link_libraries(${PROJECT_NAME} PUBLIC glad)
target_link_libraries(${PROJECT_NAME} PUBLIC assimp)
target_link_libraries(${PROJECT_NAME} PUBLIC args)
target_link_libraries(${PROJECT_NAME} PUBLIC spdlog)
target_link_libraries(${PROJECT_NAME} PUBLIC cereal)
#target_link_libraries(${PROJECT_NAME} PUBLIC vulkan)

# Add executable
add_executable(TestScene src/tools/TestScene/main.cpp)
target_link_libraries(TestScene PUBLIC ${PROJECT_NAME})

add_executable(TriangleDistanceTest src/tools/TriangleDistanceTest/main.cpp)
target_link_libraries(TestScene PUBLIC ${PROJECT_NAME})

add_executable(UniformGridSdfTest src/tools/UniformGridSdfTest/main.cpp)
target_link_libraries(UniformGridSdfTest PUBLIC ${PROJECT_NAME})

add_executable(UniformGridSdfOctreeTest src/tools/UniformGridSdfOctreeTest/main.cpp)
target_link_libraries(UniformGridSdfOctreeTest PUBLIC ${PROJECT_NAME})

add_executable(GJKtest src/tools/GJKtest/main.cpp)
target_link_libraries(GJKtest PUBLIC ${PROJECT_NAME})

add_executable(UniformGridViewer src/tools/UniformGridViewer/main.cpp)
target_link_libraries(UniformGridViewer PUBLIC ${PROJECT_NAME})

add_executable(SdfExporter src/tools/SdfExporter/main.cpp)
target_link_libraries(SdfExporter PUBLIC ${PROJECT_NAME})
