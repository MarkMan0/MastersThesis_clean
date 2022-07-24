cmake_minimum_required(VERSION 3.18.0)


add_compile_definitions(NOMINMAX)
add_compile_definitions(_USE_MATH_DEFINES)
add_compile_definitions(LOGS_DIR="${CMAKE_SOURCE_DIR}")

add_subdirectory(libs/vec2)
add_subdirectory(libs/fuzzylite)
add_subdirectory(libs/SerialPortWrapper)
add_subdirectory(RobotControl)
add_subdirectory(SimpleControl)
