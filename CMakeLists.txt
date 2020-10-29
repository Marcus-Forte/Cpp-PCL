cmake_minimum_required(VERSION 2.8)
project(MyPCL)

# My Arch pcl-11 folder
set (CMAKE_PREFIX_PATH /home/marcus/PCL/install/share)


set (CMAKE_BUILD_TYPE Release)
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
set (EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin) #make binary folder
find_package(PCL 1.7 REQUIRED)
find_package(OpenMP)


#message(STATUS "PATH = ${CMAKE_SOURCE_DIR}")

file(GLOB SOURCES "src/*.cpp")
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
# add_definitions(${PCL_DEFINITIONS})
link_libraries(${PCL_LIBRARIES} -pthread OpenMP::OpenMP_CXX)



add_library(Utils STATIC src/PCUtils.cpp)

add_executable(pcl_view_example src/pcl_view_example.cpp)
add_executable(filters src/filters.cpp)
add_executable(plane src/plane.cpp)
add_executable(volume src/volume.cpp)
add_executable(normals src/normals.cpp)
add_executable(volume_studies src/volume.cpp)
add_executable(visualize src/visualize.cpp)
add_executable(geocrop src/geocrop.cpp)
add_executable(subsample src/subsample.cpp)
add_executable(segmentation src/segmentation.cpp)
add_executable(registration src/registration.cpp)
add_executable(registration2d src/registration2d.cpp)

target_link_libraries(volume Utils)
target_link_libraries(volume_studies Utils)
target_link_libraries(geocrop Utils)
target_link_libraries(visualize Utils)


# converter
add_executable(extconverter src/extconverter.cpp)


# tools
add_executable(pcl_transform_point_cloud src/pcl_transform_point_cloud.cpp)






