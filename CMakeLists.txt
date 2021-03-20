cmake_minimum_required(VERSION 3.8)
project(marcusPCL)

set (CMAKE_BUILD_TYPE Debug)
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
set (EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin) #make binary folder
find_package(PCL 1.8 REQUIRED)


message(STATUS "PCL VERSION -> " ${PCL_VERSION})




# file(GLOB SOURCES "src/*.cpp")
# link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
link_libraries(${PCL_LIBRARIES} -pthread)



add_subdirectory(utilities)
add_subdirectory(volume)
add_subdirectory(cuda)
add_subdirectory(slam)
add_subdirectory(tests)





add_executable(filters src/filters.cpp)
add_executable(plane src/plane.cpp)
add_executable(normals src/normals.cpp)

add_executable(visualize src/visualize.cpp)
add_executable(duna_visualize src/duna_visualize.cpp)

add_executable(geocrop src/geocrop.cpp)
add_executable(subsample src/subsample.cpp)
add_executable(segmentation src/segmentation.cpp)


# converter
add_executable(extconverter src/extconverter.cpp)


# linkage
target_link_libraries(filters Utils)
target_link_libraries(geocrop Utils)
target_link_libraries(visualize Utils)
target_link_libraries(duna_visualize Utils)
 






