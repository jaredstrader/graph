cmake_minimum_required(VERSION 3.0)
project(graph VERSION 1.0.0 LANGUAGES CXX)

SET(CMAKE_CXX_FLAGS "-O3 -std=c++11")

########################
#######DIRECTORY########
########################
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

#define variable for binary directory 
add_definitions(-DCMAKE_RUNTIME_OUTPUT_DIRECTORY="${CMAKE_BINARY_DIR}/bin/")

########################
######Dependencies######
########################

INCLUDE_DIRECTORIES(
	include
	src
	${PROJECT_SOURCE_DIR}
)

#######################
########Library########
#######################
add_library(graph
	include/Graph.cpp
	include/Edge.cpp)

#####################
########Tests########
#####################
add_executable(testBellmanFord tests/testBellmanFord.cpp)
target_link_libraries(testBellmanFord 
	graph
 )

add_executable(testDijkstras tests/testDijkstras.cpp)
target_link_libraries(testDijkstras 
	graph
 )

add_executable(testFloydWarshall tests/testFloydWarshall.cpp)
target_link_libraries(testFloydWarshall 
	graph
 )

# add_subdirectory(tests)
# enable_testing()