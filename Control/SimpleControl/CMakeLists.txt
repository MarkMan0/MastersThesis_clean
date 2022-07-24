cmake_minimum_required(VERSION 3.18.0)

add_executable(SimpleControl src/main.cpp)
target_link_libraries(SimpleControl Motion)
