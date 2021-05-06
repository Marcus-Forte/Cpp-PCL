
# add_library(delabella src/delabella.cpp)

add_executable(volume src/volume.cpp)
add_executable(volume_studies src/volume_studies.cpp)
add_executable(surface src/surface.cpp)

target_link_libraries(volume Utils)
target_link_libraries(volume_studies Utils)
target_link_libraries(surface Utils)
