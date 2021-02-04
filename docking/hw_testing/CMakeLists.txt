
cmake_minimum_required(VERSION 3.10.2)
project(aviata_docking)

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set release build type by default
# invoke with cmake .. -DCMAKE_BUILD_TYPE=Debug to enable debugging
# revert to optimization with -DCMAKE_BUILD_TYPE=Release (CMake will cache last used value)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# set(raspicam_DIR "/usr/local/lib/cmake")
# find_package(raspicam REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)

include_directories(./src)

add_executable(docking src/docking.cpp src/drone.cpp src/raspi_camera.cpp src/image_analyzer.cpp src/pid_controller.cpp)
target_link_libraries(docking
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${OpenCV_LIBS}
    # ${raspicam_CV_LIBS}
    apriltag
)
