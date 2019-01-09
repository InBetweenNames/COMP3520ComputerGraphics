cmake_minimum_required(VERSION 3.7)

project(SDL2TemplateCMake)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(SDL2 REQUIRED)
find_package(SDL2_ttf REQUIRED)

add_executable(main main.cpp)

target_include_directories(main
		PRIVATE		
		${CMAKE_CURRENT_SOURCE_DIR}
		PUBLIC
		${SDL2_INCLUDE_DIR}
		${SDL2_TTF_INCLUDE_DIR}
)

target_compile_options(main
		PRIVATE
		-Wall
		-Werror
)

target_compile_features(main PRIVATE cxx_std_17)

target_link_libraries(main
		PUBLIC
		${SDL2_LIBRARY}
		${SDL2_TTF_LIBRARY}
		)
