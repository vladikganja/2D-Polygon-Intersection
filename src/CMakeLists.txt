project(src)

file(GLOB ALL_SOURCE_FILES *.cpp *.h *.hpp)

add_library(src STATIC ${ALL_SOURCE_FILES})
target_include_directories(src PUBLIC .)
