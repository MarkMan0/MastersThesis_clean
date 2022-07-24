cmake_minimum_required(VERSION 3.18.0)


set(INCLUDE_DIR "include/")
set(SRC
  src/Controller.cpp
  src/Servo.cpp
  src/VirtualServo.cpp
  src/Logger.cpp
  src/SimpleMotion.cpp
  src/Position.cpp
  src/Navigation.cpp
  src/MotionQueue.cpp
  src/PlanExecutor.cpp
  src/Path.cpp
  src/Plan.cpp
)

add_library(Motion ${SRC})
target_link_libraries(Motion vec2 Eigen3::Eigen SerialPortWrapper)
target_link_libraries(Motion debug fuzzylite_debug optimized fuzzylite_release)
target_include_directories(Motion PUBLIC ${INCLUDE_DIR})

add_subdirectory(test)
