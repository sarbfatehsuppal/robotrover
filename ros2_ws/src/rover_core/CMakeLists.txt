cmake_minimum_required(VERSION 3.16)
project(rover_core LANGUAGES CXX)

# Header-only C++17, no ROS dependencies on purpose: the same headers are
# compiled into the STM32 and ESP32 firmware (see firmware/*/platformio.ini).

find_package(ament_cmake REQUIRED)

add_library(rover_core INTERFACE)
target_compile_features(rover_core INTERFACE cxx_std_17)
target_include_directories(rover_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rover_core EXPORT export_rover_core)

ament_export_targets(export_rover_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
