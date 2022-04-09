cmake_minimum_required(VERSION 3.8)
project(marcusPCL)

# set (CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install) #avoid installing in default path
	

set (CMAKE_BUILD_TYPE Release)
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set (EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin) #make binary folder
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/lib)
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/lib)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/lib)

find_package(PCL 1.10 REQUIRED)

find_package(OpenMP)


# find_package(PCL 1.12 PATHS /home/marcus/Software/pcl/build/install/share/pcl-1.12 REQUIRED)

set (CMAKE_SKIP_INSTALL_ALL_DEPENDENCY ON) #install only target

message(STATUS "PCL VERSION -> " ${PCL_VERSION})

find_package(OpenCV)

get_filename_component(datasetPath ${CMAKE_SOURCE_DIR}/dataset REALPATH)
add_definitions (-DDATASET_PATH="${datasetPath}/")

# file(GLOB SOURCES "src/*.cpp")
# link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(include)
link_libraries(${PCL_LIBRARIES} -pthread)



add_subdirectory(utilities)
add_subdirectory(volume)
add_subdirectory(cuda)
add_subdirectory(registration)
add_subdirectory(tests)
# add_subdirectory(qt)





add_executable(filters src/filters.cpp)
add_executable(plane src/plane.cpp)
add_executable(normals src/normals.cpp)

add_executable(visualize src/visualize.cpp)
add_executable(laser_camera_calibration src/laser_camera_calibration.cpp)


add_executable(geocrop src/geocrop.cpp)
add_executable(subsample src/subsample.cpp)
add_executable(segmentation src/segmentation.cpp)

add_executable(convert2xyz src/convert2xyz.cpp)


# converter
add_executable(extconverter src/extconverter.cpp)


# linkage
target_link_libraries(filters Utils)
target_link_libraries(geocrop Utils)
target_link_libraries(visualize Utils)
target_link_libraries(laser_camera_calibration ${OpenCV_LIBRARIES})
target_link_libraries(convert2xyz OpenMP::OpenMP_CXX) 






