set (CMAKE_BUILD_TYPE RelWithDebInfo)

# set (CMAKE_BUILD_TYPE Debug)

find_package(OpenMP)

# include_directories(${OpenMP_INCLUDE_DIRS})
# 


add_library (my_icp src/my/my_icp.cpp src/my/my_correspondence_estimator.cpp)


add_executable(features src/features_app.cpp)
add_executable(registration src/registration.cpp)
add_executable(my_registration src/my_registration.cpp)

target_link_libraries(my_registration my_icp)