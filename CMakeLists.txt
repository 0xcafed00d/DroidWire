cmake_minimum_required(VERSION 3.13)

project(cpp_project CXX)

set(CMAKE_CXX_STANDARD 20)

add_executable(
	${CMAKE_PROJECT_NAME}
	src/main.cpp
)

target_link_libraries(${CMAKE_PROJECT_NAME})