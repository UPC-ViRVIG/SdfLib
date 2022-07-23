cmake_minimum_required(VERSION 3.20)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/CMakeConfig.txt")
    include("${CMAKE_CURRENT_SOURCE_DIR}/CMakeConfig.txt")
endif()

# Set the project name and version
project(SDFoctree VERSION 0.1)

add_compile_definitions(PRINT_STATISTICS)

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

include_directories(include)

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
target_link_libraries(TriangleDistanceTest PUBLIC ${PROJECT_NAME})

add_executable(UniformGridSdfTest src/tools/UniformGridSdfTest/main.cpp)
target_link_libraries(UniformGridSdfTest PUBLIC ${PROJECT_NAME})

add_executable(UniformGridSdfOctreeTest src/tools/UniformGridSdfOctreeTest/main.cpp)
target_link_libraries(UniformGridSdfOctreeTest PUBLIC ${PROJECT_NAME})

add_executable(OctreeExactMeanOfTrianglesViewer src/tools/OctreeExactMeanOfTrianglesViewer/main.cpp)
target_link_libraries(OctreeExactMeanOfTrianglesViewer PUBLIC ${PROJECT_NAME})

add_executable(SdfOctreeTest src/tools/SdfOctreeTest/main.cpp)
target_link_libraries(SdfOctreeTest PUBLIC ${PROJECT_NAME})

add_executable(GJKtest src/tools/GJKtest/main.cpp)
target_link_libraries(GJKtest PUBLIC ${PROJECT_NAME})

add_executable(CalculateInterpolationParameters src/tools/CalculateInterpolationParameters/main.cpp)
target_link_libraries(CalculateInterpolationParameters PUBLIC ${PROJECT_NAME})

add_executable(SdfViewer src/tools/SdfViewer/main.cpp src/tools/SdfViewer/InfluenceRegionCreator.h)
target_link_libraries(SdfViewer PUBLIC ${PROJECT_NAME})

add_executable(SdfExporter src/tools/SdfExporter/main.cpp)
target_link_libraries(SdfExporter PUBLIC ${PROJECT_NAME})

add_executable(SdfError src/tools/SdfError/main.cpp)
target_link_libraries(SdfError PUBLIC ${PROJECT_NAME})

# Uncomment to test with the other methods
# list(APPEND CMAKE_MODULE_PATH C:/Users/eduard/vcpkg/installed/x86-windows/share)
# set(GMP_INCLUDE_DIR C:/Users/eduard/vcpkg/installed/x86-windows/include)
# set(GMP_LIBRARIES C:/Users/eduard/vcpkg/installed/x86-windows/lib/gmp.lib)
# set(MPFR_INCLUDE_DIR C:/Users/eduard/vcpkg/installed/x86-windows/include)
# set(MPFR_LIBRARIES C:/Users/eduard/vcpkg/installed/x86-windows/lib/mpfr.lib)
# set(Boost_INCLUDE_DIR C:/Users/eduard/vcpkg/installed/x86-windows/include)
# find_package(CGAL CONFIG REQUIRED)
# add_executable(SdfErrorCompare src/tools/SdfErrorCompare/main.cpp)
# target_link_libraries(SdfErrorCompare PUBLIC ${PROJECT_NAME} CGAL::CGAL)