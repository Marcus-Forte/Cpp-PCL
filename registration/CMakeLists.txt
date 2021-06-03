set (CMAKE_BUILD_TYPE RelWithDebInfo)

# set (CMAKE_BUILD_TYPE Debug)

find_package(OpenMP)

# include_directories(${OpenMP_INCLUDE_DIRS})
# 

set (LibraryName "duna_matching")

add_library (${LibraryName} SHARED
 src/duna/icp.cpp
 src/duna/correspondence_estimator.cpp
 src/duna/transform_normals.cpp)


add_executable(features src/features_app.cpp)
# add_executable(registration src/registration.cpp)
add_executable(my_registration src/my_registration.cpp)

target_link_libraries(my_registration ${LibraryName})

# export my icp

# message(WARNING ${CMAKE_INSTALL_INCLUDEDIR})
# message(WARNING ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS ${LibraryName} 
        DESTINATION lib)

file(GLOB header_files "${CMAKE_CURRENT_SOURCE_DIR}/src/duna/*.h")
file(GLOB header_files_ "${CMAKE_CURRENT_SOURCE_DIR}/src/duna/*.hpp")

install(FILES ${header_files_} ${header_files}
        DESTINATION include/${LibraryName} 
)

# install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src/my"
#         DESTINATION include/${LibraryName} 
#         FILES_MATCHING
#         PATTERN "*.h"
#         PATTERN "*.hpp")
