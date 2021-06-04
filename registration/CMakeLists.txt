# set (CMAKE_BUILD_TYPE RelWithDebInfo)

set (CMAKE_BUILD_TYPE Debug)

find_package(OpenCV )


# include_directories(${OpenMP_INCLUDE_DIRS})
# 

set (LibraryName "duna_matching")
set (Library2Name "marcus")

add_library (${Library2Name} SHARED
 src/marcus/icp.cpp
 src/marcus/correspondence_estimator.cpp
 src/marcus/transform_normals.cpp)

 add_library (${LibraryName} SHARED
 src/duna/feature_registration.cpp
#  src/duna/transform_normals.cpp)
 )

 target_include_directories(${LibraryName} PUBLIC ${OpenCV_INCLUDE_DIRS})
 target_link_libraries(${LibraryName} PUBLIC ${OpenCV_LIBRARIES})       
# add_executable(registration src/registration.cpp)


add_executable(my_registration src/my_registration.cpp)
add_executable(duna_registration_test src/duna_registration_test.cpp)


target_link_libraries(duna_registration_test ${LibraryName})
target_link_libraries(my_registration ${Library2Name})

# export my icp

# message(WARNING ${CMAKE_INSTALL_INCLUDEDIR})
# message(WARNING ${CMAKE_CURRENT_SOURCE_DIR})



file(GLOB header_files "${CMAKE_CURRENT_SOURCE_DIR}/src/duna/*.h")
file(GLOB header_files_ "${CMAKE_CURRENT_SOURCE_DIR}/src/duna/*.hpp")



install(FILES ${header_files_} ${header_files}
        DESTINATION include/${LibraryName} 
)

install(TARGETS ${LibraryName} 
        DESTINATION lib)

file(GLOB header_files2 "${CMAKE_CURRENT_SOURCE_DIR}/src/marcus/*.h")
file(GLOB header_files2_ "${CMAKE_CURRENT_SOURCE_DIR}/src/marcus/*.hpp")

install(FILES ${header_files2_} ${header_files2}
        DESTINATION include/${Library2Name} 
)

install(TARGETS ${Library2Name} 
        DESTINATION lib)

# install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src/my"
#         DESTINATION include/${LibraryName} 
#         FILES_MATCHING
#         PATTERN "*.h"
#         PATTERN "*.hpp")
