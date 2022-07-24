cmake_minimum_required(VERSION 3.18.0)


add_executable(encoder_tests encoder_tests.cpp)
target_link_libraries(encoder_tests GTest::gtest_main Motion)

add_executable(utility_tests utility_tests.cpp)
target_link_libraries(utility_tests Motion GTest::gtest_main)

add_executable(geometry_tests geometry_tests.cpp)
target_link_libraries(geometry_tests Motion GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(encoder_tests)
gtest_discover_tests(utility_tests)
gtest_discover_tests(geometry_tests)
