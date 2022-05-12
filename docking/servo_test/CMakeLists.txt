
cmake_minimum_required(VERSION 3.10.2)
project(servo_test)

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

include_directories(./src)

add_executable(servo_test src/main.cpp)
target_link_libraries(servo_test
    pigpio
)
