cmake_minimum_required(VERSION 3.12)

include("${CMAKE_CURRENT_SOURCE_DIR}/Configure.cmake")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(PREDEFINED_TARGETS_FOLDER "Cmake")
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER)

project(LinGeo)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/googletest")
include_directories("${CMAKE_SOURCE_DIR}/3rdparty/googletest/googletest/include")

add_subdirectory(main)
add_subdirectory(src)
add_subdirectory(tests)
