cmake_minimum_required(VERSION 3.18.0)


add_library(fuzzylite_release STATIC IMPORTED GLOBAL)
set_target_properties(fuzzylite_release PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/bin_release/fuzzylite-static.lib")
target_include_directories(fuzzylite_release INTERFACE "include/")

add_library(fuzzylite_debug STATIC IMPORTED GLOBAL)
set_target_properties(fuzzylite_debug PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/bin_debug/fuzzylite-static-debug.lib")
target_link_libraries(fuzzylite_debug INTERFACE winmm.lib dbghelp.lib shlwapi.lib)
target_include_directories(fuzzylite_debug INTERFACE "include/")
target_link_options(fuzzylite_debug INTERFACE "/ignore:4099")
