include(FetchContent)
set(CMAKE_POLICY_DEFAULT_CMP0048 NEW)

# glm
if(NOT SDFLIB_USE_SYSTEM_GLM)
	FetchContent_Declare(glm_lib
	  GIT_REPOSITORY https://github.com/g-truc/glm.git
	  GIT_TAG 89e52e327d7a3ae61eb402850ba36ac4dd111987 # 0.9.8
	)

	FetchContent_GetProperties(glm_lib)
	if(NOT glm_lib_POPULATED)
		FetchContent_Populate(glm_lib)
		add_subdirectory(${glm_lib_SOURCE_DIR} ${glm_lib_BINARY_DIR})
	endif()
	set(SDFLIB_GLM_TARGET glm PARENT_SCOPE)
endif()

# assimp
if(SDFLIB_USE_ASSIMP)
	if(NOT SDFLIB_USE_SYSTEM_ASSIMP)
		FetchContent_Declare(assimp_lib
		  GIT_REPOSITORY https://github.com/assimp/assimp.git
		  GIT_TAG 9519a62dd20799c5493c638d1ef5a6f484e5faf1 # 5.2.5
		)
	
		if(NOT assimp_lib)
			FetchContent_Populate(assimp_lib)
		
			set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
			set(BUILD_SHARED_LIBS OFF)
			set(ASSIMP_BUILD_ASSIMP_TOOLS OFF)
			set(ASSIMP_BUILD_TESTS OFF)
			set(ASSIMP_INSTALL OFF)
			set(ASSIMP_INJECT_DEBUG_POSTFIX OFF)
			set(ASSIMP_BUILD_ASSIMP_VIEW OFF)
		
			add_subdirectory(${assimp_lib_SOURCE_DIR} ${assimp_lib_BINARY_DIR})
		endif()
	endif()
endif()

# args
if(SDFLIB_BUILD_APPS OR SDFLIB_BUILD_DEBUG_APPS)
	FetchContent_Declare(args_lib
	  GIT_REPOSITORY https://github.com/Taywee/args.git
	  GIT_TAG a48e1f880813b367d2354963a58dedbf2b708584 # 6.3.0
	)
	
	FetchContent_GetProperties(args_lib)
	if(NOT args_lib_POPULATED)
		FetchContent_Populate(args_lib)
		add_library(args INTERFACE)
		target_include_directories(args INTERFACE ${args_lib_SOURCE_DIR})
	endif()
 endif()

# spdlog
if(NOT SDFLIB_USE_SYSTEM_SPDLOG)
	FetchContent_Declare(spdlog_lib
	  GIT_REPOSITORY https://github.com/gabime/spdlog.git
	  GIT_TAG eb3220622e73a4889eee355ffa37972b3cac3df5 # 1.9.2
	)

	FetchContent_GetProperties(spdlog_lib)
	if(NOT spdlog_lib_POPULATED)
	   FetchContent_Populate(spdlog_lib)
	   add_subdirectory(${spdlog_lib_SOURCE_DIR} ${spdlog_lib_BINARY_DIR})
	endif()
endif()

# cereal
if(NOT SDFLIB_USE_SYSTEM_CEREAL)
	FetchContent_Declare(cereal_lib
	  GIT_REPOSITORY https://github.com/USCiLab/cereal.git
	  GIT_TAG ebef1e929807629befafbb2918ea1a08c7194554 # 1.3.2
	)

	FetchContent_GetProperties(cereal_lib)
	if(NOT cereal_lib_POPULATED)
		FetchContent_Populate(cereal_lib)
		set(BUILD_DOC OFF)
		set(BUILD_SANDBOX  OFF)
		set(BUILD_TESTS OFF)
		set(CEREAL_INSTALL OFF)
		set(SKIP_PERFORMANCE_COMPARISON ON)
		add_subdirectory(${cereal_lib_SOURCE_DIR} ${cereal_lib_BINARY_DIR})
	endif()
endif()

# Enoki
if(SDFLIB_USE_ENOKI)
	FetchContent_Declare(enoki_lib
		GIT_REPOSITORY https://github.com/mitsuba-renderer/enoki.git
		GIT_TAG 2a18afa
	)
	FetchContent_GetProperties(enoki_lib)
	if(NOT enoki_lib_POPULATED)
		FetchContent_Populate(enoki_lib)
		add_library(enoki INTERFACE)
		add_subdirectory(${enoki_lib_SOURCE_DIR} ${enoki_lib_BINARY_DIR})
		target_include_directories(enoki INTERFACE ${enoki_lib_SOURCE_DIR}/include)
	endif()

 	# FCPW
	FetchContent_Declare(fcpw_lib
		GIT_REPOSITORY https://github.com/rohan-sawhney/fcpw.git
		GIT_TAG dd65ec2
	)
	
	FetchContent_GetProperties(fcpw_lib)
	if(NOT fcpw_lib_POPULATED)
		FetchContent_Populate(fcpw_lib)
		add_subdirectory(${fcpw_lib_SOURCE_DIR} ${fcpw_lib_BINARY_DIR})
		target_include_directories(fcpw INTERFACE ${fcpw_lib_SOURCE_DIR})
	endif()
endif()

# eigen
if(SDFLIB_BUILD_DEBUG_APPS OR SDFLIB_USE_ENOKI)
	FetchContent_Declare(eigen_lib
	GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
	GIT_TAG 46126273552afe13692929523d34006f54c19719 # 3.4
	)
	
	FetchContent_GetProperties(eigen_lib)
	if(NOT eigen_lib_POPULATED)
		FetchContent_Populate(eigen_lib)
		add_library(eigen INTERFACE)
		target_include_directories(eigen INTERFACE ${eigen_lib_SOURCE_DIR})
	endif()
 endif()

# stb
if(SDFLIB_BUILD_APPS OR SDFLIB_BUILD_DEBUG_APPS)
	add_library(stb_image INTERFACE)
	target_include_directories(stb_image INTERFACE stb)
 endif()

# icg
add_library(icg INTERFACE)
target_include_directories(icg INTERFACE InteractiveComputerGraphics)

if(SDFLIB_BUILD_APPS OR SDFLIB_BUILD_DEBUG_APPS)
	# glfw
	FetchContent_Declare(glfw_lib
	GIT_REPOSITORY https://github.com/glfw/glfw.git
	GIT_TAG 7d5a16ce714f0b5f4efa3262de22e4d948851525 # 3.3.6
	)

	FetchContent_GetProperties(glfw_lib)
	if(NOT glfw_lib_POPULATED)
		FetchContent_Populate(glfw_lib)
		add_subdirectory(${glfw_lib_SOURCE_DIR} ${glfw_lib_BINARY_DIR})
	endif()

	# glad
	add_library(glad STATIC glad/src/glad.c)
	target_include_directories(glad PUBLIC glad/include)

	# imgui
	FetchContent_Declare(imgui_lib
	GIT_REPOSITORY https://github.com/ocornut/imgui.git
	GIT_TAG c71a50deb5ddf1ea386b91e60fa2e4a26d080074 # 1.87
	)

	FetchContent_GetProperties(imgui_lib)
	if(NOT imgui_lib_POPULATED)
		FetchContent_Populate(imgui_lib)
		file(GLOB imgui_SOURCE_FILES ${imgui_lib_SOURCE_DIR}/*.cpp)
		file(GLOB imgui_HEADER_FILES ${imgui_lib_SOURCE_DIR}/*.h)
		add_library(imgui STATIC ${imgui_SOURCE_FILES} 
								${imgui_lib_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
								${imgui_lib_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
								${imgui_HEADER_FILES}
								${imgui_lib_SOURCE_DIR}/backends/imgui_impl_glfw.h
								${imgui_lib_SOURCE_DIR}/backends/imgui_impl_opengl3.h)
		
		target_include_directories(imgui PUBLIC ${imgui_lib_SOURCE_DIR}
										PUBLIC ${imgui_lib_SOURCE_DIR}/backends)
		target_link_libraries(imgui PRIVATE glfw
									PRIVATE glad)
	endif()

	# imguizmo
	FetchContent_Declare(imguizmo_lib
	GIT_REPOSITORY https://github.com/CedricGuillemet/ImGuizmo.git
	GIT_TAG 664cf2d73864a36b2a8b5091d33fc4578c885eca # 1.83
	)

	FetchContent_GetProperties(imguizmo_lib)
	if(NOT imguizmo_lib_POPULATED)
		FetchContent_Populate(imguizmo_lib)
		file(GLOB imguizmo_SOURCE_FILES ${imguizmo_lib_SOURCE_DIR}/*.cpp)
		file(GLOB imguizmo_HEADER_FILES ${imguizmo_lib_SOURCE_DIR}/*.h)
		add_library(imguizmo STATIC ${imguizmo_SOURCE_FILES} 
								${imguizmo_HEADER_FILES})
		
		target_include_directories(imguizmo PUBLIC ${imguizmo_lib_SOURCE_DIR})
		target_link_libraries(imguizmo  PRIVATE glfw
										PRIVATE glad
										PRIVATE imgui)
	endif()
endif()
