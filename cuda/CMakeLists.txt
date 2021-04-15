

find_package(CUDA)
if(${CUDA_FOUND})
        message(STATUS "CUDA OK!")
        enable_language(CUDA)
        add_executable(gputransform src/gputransform.cpp src/transform_kernel.cu)
        target_include_directories(gputransform PUBLIC ${CUDA_INCLUDE_DIRS})
        target_link_libraries(gputransform ${CUDA_LIBRARIES})
        #set_property(TARGET gputransform PROPERTY CUDA_ARCHITECTURES 30) # I    MPORTANT !!!!
        

endif()

