project(tests)

file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.h *.hpp)

add_executable(tests ${ALL_SOURCE_FILES})
target_link_libraries(tests src gtest gtest_main)
