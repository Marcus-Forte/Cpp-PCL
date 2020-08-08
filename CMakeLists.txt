cmake_minimum_required(VERSION 2.8)
project(PclTest)

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set (CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set (EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
find_package(PCL 1.7 REQUIRED)


#message(STATUS "PATH = ${CMAKE_SOURCE_DIR}")

file(GLOB SOURCES "src/*.cpp")
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
add_definitions(${PCL_DEFINITIONS})
link_libraries(${PCL_LIBRARIES} -pthread)

#add_executable (PclTest ${SOURCES})
#target_include_directories(PclTest PUBLIC "include/")

add_executable(normals src/normals.cpp)
add_executable(volume src/volume.cpp)
add_executable(grid src/grid.cpp)
add_executable(plane src/plane.cpp)



