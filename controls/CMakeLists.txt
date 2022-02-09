cmake_minimum_required(VERSION 3.5)
project(aviata)

# On Linux, need to run `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"."` to get project to execute
# On MacOS, use `export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:"."` instead

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic
        # Disable warnings that occur in ROS generated source files
        -Wno-deprecated-declarations
    )
endif()

set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include) 
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)
link_directories(/usr/local/lib)

find_package(Qt5Widgets CONFIG)

if(Qt5Widgets_FOUND)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    if (POLICY CMP0071)
        cmake_policy(SET CMP0071 NEW)
    endif()
endif()

list(APPEND CMAKE_MODULE_PATH "/pigpio-master/util") # For Docker. TODO for RPi.
find_package(pigpio)
if(pigpio_FOUND)
    add_compile_definitions(USE_PIGPIO)
endif()

file(GLOB_RECURSE ROS_MESSAGES RELATIVE ${CMAKE_SOURCE_DIR}
    src/msg/*.msg
)

# rosidl_generate_interfaces(${PROJECT_NAME} ${ROS_MESSAGES})

file(GLOB_RECURSE ROS_SERVICES RELATIVE ${CMAKE_SOURCE_DIR}
    src/srv/*.srv
)

rosidl_generate_interfaces(${PROJECT_NAME} ${ROS_MESSAGES} ${ROS_SERVICES})

file(GLOB_RECURSE LIB_SOURCES RELATIVE ${CMAKE_SOURCE_DIR}
    src/lib/*.hpp
    src/lib/*.cpp
)
add_library(aviata_lib ${LIB_SOURCES})
ament_target_dependencies(aviata_lib rclcpp std_msgs)
rosidl_target_interfaces(aviata_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")

function(configure_mavsdk)
    set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
    set(CMAKE_BUILD_TYPE Release)
    set(BUILD_SHARED_LIBS ON)
    set(BUILD_TESTS OFF)
    set(SUPERBUILD OFF)
    add_subdirectory(src/MAVSDK EXCLUDE_FROM_ALL)
endfunction()
configure_mavsdk()
target_link_libraries(aviata_lib
    mavsdk
    mavsdk_telemetry
    mavsdk_action
    mavsdk_mavlink_passthrough
    mavsdk_offboard
    ${OpenCV_LIBS}
    apriltag
)
target_include_directories(aviata_lib SYSTEM PUBLIC ${CMAKE_SOURCE_DIR}/src/MAVSDK/src/third_party/mavlink/include)

if(pigpio_FOUND)
    target_link_libraries(aviata_lib pigpio)
endif()

if(Qt5Widgets_FOUND)
    file(GLOB_RECURSE LIB_GROUND_SOURCES RELATIVE ${CMAKE_SOURCE_DIR}
        src/lib_ground/*.hpp
        src/lib_ground/*.cpp
    )
    add_library(aviata_lib_ground ${LIB_GROUND_SOURCES})
    target_link_libraries(aviata_lib_ground aviata_lib Qt5::Widgets)
endif()

include_directories(${CMAKE_SOURCE_DIR}/src ${CMAKE_BINARY_DIR}/aviata_lib_ground_autogen/include)

add_executable(aviata_drone src/aviata_drone.cpp)
add_executable(aviata_test src/aviata_test.cpp)
add_executable(demo_listener src/demo_listener.cpp)
add_executable(demo_talker src/demo_talker.cpp)
target_link_libraries(aviata_drone aviata_lib)
target_link_libraries(aviata_test aviata_lib)
target_link_libraries(demo_listener aviata_lib)
target_link_libraries(demo_talker aviata_lib)

# Build ground station if Qt is installed
if(Qt5Widgets_FOUND)
    add_executable(aviata_ground src/aviata_ground.cpp)
    target_link_libraries(aviata_ground aviata_lib_ground)
endif()

# install(TARGETS 
#     aviata
#     DESTINATION lib/${PROJECT_NAME})

ament_package()
