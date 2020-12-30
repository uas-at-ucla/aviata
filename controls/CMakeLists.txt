cmake_minimum_required(VERSION 3.5)
project(aviata)

# On Linux, need to run `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"."` to get project to execute
# On MacOS, use DYLD_LIBRARY_PATH instead

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include) 
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(MAVSDK REQUIRED)

file(GLOB_RECURSE ROS_MESSAGES RELATIVE ${CMAKE_SOURCE_DIR}
    src/msg/*.msg
)
rosidl_generate_interfaces(${PROJECT_NAME} ${ROS_MESSAGES})

file(GLOB_RECURSE LIB_SOURCES RELATIVE ${CMAKE_SOURCE_DIR}
    src/lib/*.hpp
    src/lib/*.cpp
)
add_library(aviata_lib ${LIB_SOURCES})
ament_target_dependencies(aviata_lib rclcpp std_msgs)
rosidl_target_interfaces(aviata_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(aviata_lib
    MAVSDK::mavsdk
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_mavlink_passthrough
)

add_executable(aviata_drone src/aviata_drone.cpp)
add_executable(aviata_ground src/aviata_ground.cpp)
add_executable(aviata_test src/aviata_test.cpp)
target_link_libraries(aviata_drone aviata_lib)
target_link_libraries(aviata_ground aviata_lib)
target_link_libraries(aviata_test aviata_lib)

# install(TARGETS 
#     aviata
#     DESTINATION lib/${PROJECT_NAME})

ament_package()
