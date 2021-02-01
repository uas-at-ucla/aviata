
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

add_compile_options(-g)

set(raspicam_DIR "/usr/local/lib/cmake")
find_package(raspicam REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(./src)

add_executable(docking src/docking.cpp src/drone.cpp src/raspi_camera.cpp)
target_link_libraries(docking
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${OpenCV_LIBS}
    ${raspicam_CV_LIBS}
)