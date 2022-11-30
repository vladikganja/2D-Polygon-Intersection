project(main)

file(GLOB ALL_SOURCE_FILES *.cpp *.h *.hpp)

add_executable(main ${ALL_SOURCE_FILES})
target_include_directories(main PUBLIC .)
target_link_libraries(main src)
