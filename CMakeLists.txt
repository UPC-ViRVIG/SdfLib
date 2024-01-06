cmake_minimum_required(VERSION 3.16)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/CMakeConfig.txt")
    include("${CMAKE_CURRENT_SOURCE_DIR}/CMakeConfig.txt")
endif()

# Set the project name and version
project(SdfLib VERSION 0.1)

if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
    add_compile_options(-fPIC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lstdc++fs -std=c++17")
endif()


# Project options
option(SDFLIB_DEBUG_INFO "Print debug information" OFF)
option(SDFLIB_BUILD_APPS "Build executables for using the library" OFF)
option(SDFLIB_BUILD_DEBUG_APPS "Build executables for debugging purposes" OFF)

# Libraries options
option(SDFLIB_USE_ASSIMP "Use assimp library for importing models" ON)
option(SDFLIB_USE_OPENMP "Use OpenMP for accelerate the structures construction" ON)
option(SDFLIB_USE_ENOKI "Use Enoki for some optimizations" ON)

option(SDFLIB_USE_SYSTEM_GLM "Use glm library via find_package instead of downloading it" OFF)
option(SDFLIB_USE_SYSTEM_SPDLOG "Use spdlog library via find_package instead of downloading it" OFF)
option(SDFLIB_USE_SYSTEM_CEREAL "Use cereal library via find_package instead of downloading it" OFF)
option(SDFLIB_USE_SYSTEM_ASSIMP "Use assimp library via find_package instead of downloading it" OFF)


if(SDFLIB_DEBUG_INFO)
    add_compile_definitions(SDFLIB_PRINT_STATISTICS)
endif()

# Specify the c++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Get all project files
file(GLOB SOURCE_FILES src/*.cpp)
file(GLOB HEADER_FILES src/*.h)
file(GLOB PUBLIC_HEADER_FILES include/SdfLib/*.h)

file(GLOB RENDER_ENGINE_SOURCE_FILES src/render_engine/*.cpp)
file(GLOB RENDER_ENGINE_HEADER_FILES src/render_engine/*.h)

file(GLOB RENDER_ENGINE_SHADERS_SOURCE_FILES src/render_engine/shaders/*.cpp)
file(GLOB RENDER_ENGINE_SHADERS_HEADER_FILES src/render_engine/shaders/*.h)

file(GLOB SDF_SOURCE_FILES src/sdf/*.cpp)
file(GLOB SDF_HEADER_FILES src/sdf/*.h)

file(GLOB UTILS_SOURCE_FILES src/utils/*.cpp)
file(GLOB UTILS_HEADER_FILES src/utils/*.h)

# Add libraries
add_library(${PROJECT_NAME} STATIC  ${SOURCE_FILES} ${HEADER_FILES} ${PUBLIC_HEADER_FILES}
                                    ${SDF_SOURCE_FILES} ${SDF_HEADER_FILES}
                                    ${UTILS_SOURCE_FILES} ${UTILS_HEADER_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC include/)
target_include_directories(${PROJECT_NAME} PRIVATE src/)

# Add shaders
file(GLOB SHADER_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/src/render_engine/shaders 
                       src/render_engine/shaders/*.frag 
                       src/render_engine/shaders/*.vert
                       src/render_engine/shaders/*.comp)

foreach(SHADER IN LISTS SHADER_FILES)
    add_custom_command(OUTPUT ${SHADER}
            COMMAND cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/src/render_engine/shaders/${SHADER} $<TARGET_FILE_DIR:${PROJECT_NAME}>/shaders/${SHADER}
            DEPENDS src/render_engine/shaders/${SHADER}
        )
endforeach()

add_custom_target(copyShaders ALL SOURCES ${SHADER_FILES})

# Add dependencies
if(SDFLIB_USE_SYSTEM_ASSIMP)
    find_package(assimp REQUIRED)
endif()
if(SDFLIB_USE_SYSTEM_SPDLOG)
    find_package(spdlog REQUIRED)
endif()
if(SDFLIB_USE_SYSTEM_CEREAL)
    find_package(cereal REQUIRED)
endif()
if(SDFLIB_USE_SYSTEM_GLM)
    find_package(glm REQUIRED)
    set(SDFLIB_GLM_TARGET glm::glm)
endif()
add_subdirectory(libs)

if(SDFLIB_USE_ENOKI)
    target_link_libraries(${PROJECT_NAME} PUBLIC enoki)
    target_link_libraries(${PROJECT_NAME} PUBLIC eigen) # FCPW requirement
    target_link_libraries(${PROJECT_NAME} PUBLIC fcpw)
    target_compile_definitions(${PROJECT_NAME} PUBLIC -DENOKI_AVAILABLE)
endif()

if(SDFLIB_USE_ASSIMP)
    target_link_libraries(${PROJECT_NAME} PUBLIC assimp::assimp)
    target_compile_definitions(${PROJECT_NAME} PUBLIC -DSDFLIB_ASSIMP_AVAILABLE)
endif()

if(SDFLIB_BUILD_APPS OR SDFLIB_BUILD_DEBUG_APPS)
    target_link_libraries(${PROJECT_NAME} PUBLIC args)
    target_link_libraries(${PROJECT_NAME} PUBLIC stb_image)
endif()
    
target_link_libraries(${PROJECT_NAME} PUBLIC ${SDFLIB_GLM_TARGET})
target_link_libraries(${PROJECT_NAME} PUBLIC spdlog::spdlog)
target_link_libraries(${PROJECT_NAME} PUBLIC cereal::cereal)
target_link_libraries(${PROJECT_NAME} PUBLIC icg)

if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
    target_link_libraries(${PROJECT_NAME} PUBLIC -lstdc++fs)
endif()

# Add openMP
if(SDFLIB_USE_OPENMP)
    if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        message("Enabling openmp llvm extension")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /openmp:llvm")
        target_compile_definitions(${PROJECT_NAME} PUBLIC -DOPENMP_AVAILABLE)
    else()
        find_package(OpenMP)
        if(OpenMP_CXX_FOUND)
        	message("OpenMP version ${OpenMP_CXX_VERSION}")
            target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
            target_compile_definitions(${PROJECT_NAME} PUBLIC -DOPENMP_AVAILABLE)
        else()
            message("Disabling openmp")
        endif()
    endif()    
endif()

# Add executable
if (NOT UNIX)
    add_library(SdfLibUnity SHARED src/tools/SdfLibUnity/SdfExportFunc.h
                                src/tools/SdfLibUnity/SdfExportFunc.cpp)
    target_link_libraries(SdfLibUnity PRIVATE ${PROJECT_NAME})
endif()

if(SDFLIB_BUILD_APPS)
    add_executable(SdfViewer src/tools/SdfViewer/main.cpp src/tools/SdfViewer/InfluenceRegionCreator.h
                    ${RENDER_ENGINE_SOURCE_FILES} ${RENDER_ENGINE_HEADER_FILES}
                    ${RENDER_ENGINE_SHADERS_SOURCE_FILES} ${RENDER_ENGINE_SHADERS_HEADER_FILES})
    target_link_libraries(SdfViewer PUBLIC ${PROJECT_NAME} glfw imgui imguizmo glad)
    target_include_directories(SdfViewer PRIVATE src/)
    add_dependencies(SdfViewer copyShaders)

    add_executable(SdfRender src/tools/SdfRender/main.cpp
                    ${RENDER_ENGINE_SOURCE_FILES} ${RENDER_ENGINE_HEADER_FILES}
                    ${RENDER_ENGINE_SHADERS_SOURCE_FILES} ${RENDER_ENGINE_SHADERS_HEADER_FILES})
    target_link_libraries(SdfRender PUBLIC ${PROJECT_NAME} glfw imgui imguizmo glad)
    target_include_directories(SdfRender PRIVATE src/)
    add_dependencies(SdfRender copyShaders)

    add_executable(SdfLight src/tools/SdfLight/main.cpp
                    ${RENDER_ENGINE_SOURCE_FILES} ${RENDER_ENGINE_HEADER_FILES}
                    ${RENDER_ENGINE_SHADERS_SOURCE_FILES} ${RENDER_ENGINE_SHADERS_HEADER_FILES})
    target_link_libraries(SdfLight PUBLIC ${PROJECT_NAME} glfw imgui imguizmo glad)
    target_include_directories(SdfLight PRIVATE src/)
    add_dependencies(SdfLight copyShaders)

    add_executable(SdfExporter src/tools/SdfExporter/main.cpp)
    target_link_libraries(SdfExporter PUBLIC ${PROJECT_NAME})

    add_executable(SdfError src/tools/SdfError/main.cpp)
    target_link_libraries(SdfError PUBLIC ${PROJECT_NAME})
endif()

if(SDFLIB_BUILD_DEBUG_APPS)
    add_executable(TestScene src/tools/TestScene/main.cpp)
    target_link_libraries(TestScene PUBLIC ${PROJECT_NAME} glfw imgui imguizmo glad)

    add_executable(TriangleDistanceTest src/tools/TriangleDistanceTest/main.cpp)
    target_link_libraries(TriangleDistanceTest PUBLIC ${PROJECT_NAME})

    add_executable(UniformGridSdfTest src/tools/UniformGridSdfTest/main.cpp)
    target_link_libraries(UniformGridSdfTest PUBLIC ${PROJECT_NAME})

    add_executable(UniformGridSdfOctreeTest src/tools/UniformGridSdfOctreeTest/main.cpp)
    target_link_libraries(UniformGridSdfOctreeTest PUBLIC ${PROJECT_NAME})

    add_executable(OctreeExactMeanOfTrianglesViewer src/tools/OctreeExactMeanOfTrianglesViewer/main.cpp
                    ${RENDER_ENGINE_SOURCE_FILES} ${RENDER_ENGINE_HEADER_FILES}
                    ${RENDER_ENGINE_SHADERS_SOURCE_FILES} ${RENDER_ENGINE_SHADERS_HEADER_FILES})
    target_link_libraries(OctreeExactMeanOfTrianglesViewer PUBLIC ${PROJECT_NAME} glfw imgui imguizmo glad)
    target_include_directories(OctreeExactMeanOfTrianglesViewer PRIVATE src/)
    add_dependencies(OctreeExactMeanOfTrianglesViewer copyShaders)

    add_executable(SdfOctreeTest src/tools/SdfOctreeTest/main.cpp)
    target_link_libraries(SdfOctreeTest PUBLIC ${PROJECT_NAME})

    add_executable(GJKtest src/tools/GJKtest/main.cpp)
    target_link_libraries(GJKtest PUBLIC ${PROJECT_NAME})

    target_link_libraries(${PROJECT_NAME} PUBLIC eigen)
    add_executable(CalculateInterpolationParameters src/tools/CalculateInterpolationParameters/main.cpp)
    target_link_libraries(CalculateInterpolationParameters PUBLIC ${PROJECT_NAME})

    ## Uncomment to test with the other methods
    # set(CGAL_DIR C:/Users/eduard/vcpkg/installed/x64-windows/share/cgal)
    # set(GMP_INCLUDE_DIR C:/Users/eduard/vcpkg/installed/x64-windows/include)
    # set(GMP_LIBRARIES C:/Users/eduard/vcpkg/installed/x64-windows/lib/gmp.lib)
    # set(MPFR_INCLUDE_DIR C:/Users/eduard/vcpkg/installed/x64-windows/include)
    # set(MPFR_LIBRARIES C:/Users/eduard/vcpkg/installed/x64-windows/lib/mpfr.lib)
    # set(Boost_INCLUDE_DIR C:/Users/eduard/vcpkg/installed/x64-windows/include)
    # find_package(CGAL CONFIG REQUIRED)

    # add_executable(SdfOffsets src/tools/SdfOffsets/main.cpp)
    # target_link_libraries(SdfOffsets PUBLIC ${PROJECT_NAME} CGAL::CGAL)

    # list(APPEND CMAKE_MODULE_PATH "C:/Program Files/OpenVDB/lib/cmake/OpenVDB")
    # set(BOOST_ROOT C:/Users/eduard/Documents/boost_1_76_0/stage/lib)
    # set(Boost_INCLUDE_DIR C:/Users/eduard/Documents/boost_1_76_0)
    # find_package(OpenVDB REQUIRED)

    # list(APPEND CMAKE_MODULE_PATH "C:/Program Files/OpenVDB/lib/cmake/OpenVDB")
    # #set(Tbb_INCLUDE_DIR C:/dev/vcpkg/installed/x64-windows/include/tbb)
    # #set(Tbb_LIB_COMPONENTS C:/dev/vcpkg/installed/x64-windows/bin)
    # include(C:/Users/user/vcpkg/scripts/buildsystems/vcpkg.cmake)
    # find_package(TBB CONFIG REQUIRED)
    # find_package(OpenVDB REQUIRED)

    add_executable(SdfErrorCompare src/tools/SdfErrorCompare/main.cpp)
    #target_link_libraries(SdfErrorCompare PUBLIC ${PROJECT_NAME} CGAL::CGAL OpenVDB::openvdb)
    # target_link_libraries(SdfErrorCompare PUBLIC ${PROJECT_NAME} OpenVDB::openvdb)
    target_link_libraries(SdfErrorCompare PUBLIC ${PROJECT_NAME})

    add_executable(ImageQueryTime src/tools/ImageQueryTime/main.cpp)
    # target_link_libraries(ImageQueryTime PUBLIC ${PROJECT_NAME} CGAL::CGAL OpenVDB::openvdb)
    target_link_libraries(ImageQueryTime PUBLIC ${PROJECT_NAME})
endif()
