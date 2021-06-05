# set (CMAKE_BUILD_TYPE RelWithDebInfo)

set (CMAKE_BUILD_TYPE Release)

set (LibraryName "marcus")

add_library (${LibraryName} SHARED
 src/marcus/icp.cpp
 src/marcus/correspondence_estimator.cpp
 src/marcus/transform_normals.cpp)


 # test
add_executable(my_registration src/my_registration.cpp)


target_link_libraries(my_registration ${LibraryName})

# installation

file(GLOB header_files "${CMAKE_CURRENT_SOURCE_DIR}/src/marcus/*.h")
file(GLOB header_files_ "${CMAKE_CURRENT_SOURCE_DIR}/src/marcus/*.hpp")

install(FILES ${header_files_} ${header_files}
        DESTINATION include/${LibraryName} 
)

install(TARGETS ${LibraryName} 
        DESTINATION lib)

# install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src/my"
#         DESTINATION include/${LibraryName} 
#         FILES_MATCHING
#         PATTERN "*.h"
#         PATTERN "*.hpp")
