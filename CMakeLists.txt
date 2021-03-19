cmake_minimum_required(VERSION 3.1)
project(marcusPCL)


#delanator
add_subdirectory(submodules/delaunator-cpp)

set (CMAKE_BUILD_TYPE Debug)
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
set (EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin) #make binary folder
find_package(PCL 1.7 REQUIRED)
find_package(OpenMP)


message(STATUS "PCL VERSION -> " ${PCL_VERSION})


#message(STATUS "PATH = ${CMAKE_SOURCE_DIR}")

file(GLOB SOURCES "src/*.cpp")
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
# add_definitions(${PCL_DEFINITIONS})
link_libraries(${PCL_LIBRARIES} -pthread OpenMP::OpenMP_CXX)



add_library(Utils STATIC src/PCUtils.cpp)
# add_library(Features STATIC src/Features.cpp) # header only

add_executable(filters src/filters.cpp)
add_executable(plane src/plane.cpp)
add_executable(volume src/volume.cpp)
add_executable(normals src/normals.cpp)
add_executable(volume_studies src/volume.cpp)
add_executable(visualize src/visualize.cpp)
add_executable(duna_visualize src/duna_visualize.cpp)
add_executable(geocrop src/geocrop.cpp)
add_executable(subsample src/subsample.cpp)
add_executable(segmentation src/segmentation.cpp)
add_executable(registration src/registration.cpp)
add_executable(features_registration src/registration2d_features.cpp)
add_executable(features src/features_app.cpp)

target_include_directories(volume PUBLIC submodules/delaunator-cpp/include)


# test zone
file(GLOB cpp_list RELATIVE ${CMAKE_SOURCE_DIR} "test_src/*.cpp")

foreach(file_path ${cpp_list})
	message(STATUS ${file_path})
	string(REPLACE "test_src/" "" name_src ${file_path})
	string(REPLACE ".cpp" "" name_final ${name_src})
	message(STATUS ${name_final})
	add_executable(${name_final} ${file_path})
endforeach()





# linkage
target_link_libraries(volume Utils)
target_link_libraries(volume_studies Utils)
target_link_libraries(geocrop Utils)
target_link_libraries(visualize Utils)
 #target_link_libraries(features_registration Utils) #header only
 

# converter
add_executable(extconverter src/extconverter.cpp)



# this is independent of PCL GPU support

find_package(CUDA)
if(${CUDA_FOUND})
	message(STATUS "CUDA OK!")
	enable_language(CUDA)
	add_executable(gputransform cuda_src/transform_test.cpp cuda_src/transform_kernel.cu)
	target_include_directories(gputransform PUBLIC ${CUDA_INCLUDE_DIRS})
	target_link_libraries(gputransform ${CUDA_LIBRARIES})
	set_property(TARGET gputransform PROPERTY CUDA_ARCHITECTURES 30) # IMPORTANT !!!!

endif()


