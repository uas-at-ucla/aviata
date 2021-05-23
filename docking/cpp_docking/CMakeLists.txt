
cmake_minimum_required(VERSION 3.10.2)
project(aviata_docking)

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -std=c++2a)
endif()

# Set release build type by default
# invoke with cmake .. -DCMAKE_BUILD_TYPE=Debug to enable debugging
# set optimized build with -DCMAKE_BUILD_TYPE=Release (CMake will cache last used value, so delete build/ and reinvoke cmake .. with desired option to change)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# set(raspicam_DIR "/usr/local/lib/cmake")
# find_package(raspicam REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)

include_directories(./src)

add_executable(docking_simulation src/main.cpp src/drone.cpp src/camera_simulator.cpp src/image_analyzer.cpp src/pid_controller.cpp)
target_compile_definitions(docking_simulation PUBLIC -DUSE_RASPI_CAMERA=0)
target_link_libraries(docking_simulation
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${OpenCV_LIBS}
    apriltag
)

add_executable(docking_physical EXCLUDE_FROM_ALL src/main.cpp src/drone.cpp src/raspi_camera.cpp src/image_analyzer.cpp src/pid_controller.cpp src/docking_detector.cpp)
target_compile_definitions(docking_physical PUBLIC -DUSE_RASPI_CAMERA=1)
target_link_libraries(docking_physical
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${OpenCV_LIBS}
    pigpio
    apriltag
)