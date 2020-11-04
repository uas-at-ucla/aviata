cmake_minimum_required(VERSION 2.8.12)

project(aviata)

if(NOT MSVC)
    add_definitions("-std=c++11 -Wall -Wextra")
else()
    add_definitions("-std=c++11 -WX -W2")
endif()

find_package(MAVSDK REQUIRED)

file(GLOB_RECURSE SOURCES
    src/*.hpp
    src/*.cpp
)

add_executable(aviata ${SOURCES})

target_link_libraries(aviata
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk
)
