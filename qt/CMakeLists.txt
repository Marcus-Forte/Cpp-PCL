list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# Requirements
# 1) Install Qt5
# 2) Compile VTK and set QtGroup ON
# 3) Compile PCL and set VTK dir

set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt5 REQUIRED Widgets)
find_package(VTK 9.0 PATHS /home/marcus/Software/VTK-9.1.0.rc2/build/install  REQUIRED)
find_package(PCL 1.12 PATHS /home/marcus/Software/pcl/build/install/share/pcl-1.12 REQUIRED)

message(STATUS "PCL: ${PCL_VERSION}")
message(STATUS "VTK: ${VTK_VERSION}")
message(STATUS "QT: ${Qt5_VERSION}")



add_executable(qt_test 
src/main.cpp
src/pcd_view_qt.cpp
src/pcd_view_qt.ui)

target_link_libraries(qt_test ${PCL_LIBRARIES}  Qt5::Widgets )
target_include_directories(qt_test PUBLIC src)

