cmake_minimum_required(VERSION 3.5)
project(aviata)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include) 
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

find_package(ament_cmake)
IF( ament_cmake_FOUND )
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
ENDIF()

find_package(MAVSDK REQUIRED)

file(GLOB_RECURSE SOURCES
    src/*.hpp
    src/*.cpp
)
add_executable(aviata ${SOURCES})
IF( ament_cmake_FOUND )
  ament_target_dependencies(aviata rclcpp std_msgs)
ENDIF()

target_link_libraries(aviata
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk
)

IF( ament_cmake_FOUND )
  ament_package()
ENDIF()
