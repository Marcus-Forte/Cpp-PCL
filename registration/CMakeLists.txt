set (CMAKE_BUILD_TYPE Release)

find_package(OpenMP)

# include_directories(${OpenMP_INCLUDE_DIRS})
# 
add_executable(features src/features_app.cpp)
add_executable(registration src/registration.cpp)
add_executable(registration_normals src/registration_normals.cpp)
add_executable(my_registration src/my_registration.cpp)

target_link_libraries(my_registration PUBLIC OpenMP::OpenMP_CXX)