# set (CMAKE_BUILD_TYPE RelWithDebInfo)

set (CMAKE_BUILD_TYPE Release)



 # test
add_executable(feature_registration src/feature_registration.cpp)
target_link_libraries(feature_registration ${PCL_LIBRARIES})

# installation

# file(GLOB header_files "${CMAKE_CURRENT_SOURCE_DIR}/src/marcus/*.h")
# file(GLOB header_files_ "${CMAKE_CURRENT_SOURCE_DIR}/src/marcus/*.hpp")

# install(FILES ${header_files_} ${header_files}
#         DESTINATION include/${LibraryName} 
# )

# install(TARGETS ${LibraryName} 
#         DESTINATION lib)

# install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src/my"
#         DESTINATION include/${LibraryName} 
#         FILES_MATCHING
#         PATTERN "*.h"
#         PATTERN "*.hpp")
