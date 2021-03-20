# test zone
file(GLOB cpp_list RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/*.cpp")

foreach(file_path ${cpp_list})
        message(STATUS ${file_path})
        string(REPLACE "src/" "" name_src ${file_path})
        string(REPLACE ".cpp" "" name_final ${name_src})
        #message(STATUS ${name_final})
        add_executable(${name_final} ${file_path})
endforeach()

