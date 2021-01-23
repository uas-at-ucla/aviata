
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

find_package(MAVSDK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(./src)

add_executable(docking src/docking.cpp src/drone.cpp src/camera_simulator.cpp src/image_analyzer.cpp apriltags/src/TagDetector.cc apriltags/src/TagDetection.cc apriltags/src/TagFamily.cc apriltags/src/FloatImage.cc apriltags/src/Edge.cc apriltags/src/Gaussian.cc apriltags/src/GLine2D.cc apriltags/src/GLineSegment2D.cc apriltags/src/GrayModel.cc apriltags/src/Homography33.cc apriltags/src/MathUtil.cc apriltags/src/Quad.cc apriltags/src/Segment.cc apriltags/src/UnionFindSimple.cc)
target_link_libraries(docking
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    Eigen3::Eigen
    ${OpenCV_LIBS}
)