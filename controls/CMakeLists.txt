cmake_minimum_required(VERSION 3.5)
project(aviata)

# On MacOS, need to run `export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:"."` to get project to execute

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
  find_package(rosidl_default_generators REQUIRED)
ENDIF()

find_package(MAVSDK REQUIRED)

file(GLOB_RECURSE SOURCES
    src/lib/*.hpp
    src/lib/*.cpp
)
add_library(aviata_lib ${SOURCES})
IF( ament_cmake_FOUND )
  ament_target_dependencies(aviata_lib rclcpp std_msgs)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "src/msg/DroneStatus.msg"
  )
  rosidl_target_interfaces(aviata_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
ENDIF()

target_link_libraries(aviata_lib
    MAVSDK::mavsdk
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_mavlink_passthrough
)

add_executable(drone src/aviata_drone.cpp)
add_executable(ground src/ground.cpp)
add_executable(aviata_test src/test.cpp)
target_link_libraries(drone aviata_lib)
target_link_libraries(ground aviata_lib)
target_link_libraries(aviata_test aviata_lib)

# install(TARGETS 
#   aviata
#   DESTINATION lib/${PROJECT_NAME})

IF( ament_cmake_FOUND )
  ament_package()
ENDIF()
