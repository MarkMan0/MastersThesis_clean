cmake_minimum_required(VERSION 3.18.0)


add_library(vec2 INTERFACE)
target_include_directories(vec2 INTERFACE include/)

add_executable(vec2_test test/test_vec2.cpp)
target_link_libraries(vec2_test PUBLIC GTest::gtest_main vec2)

include(GoogleTest)
gtest_discover_tests(vec2_test)
