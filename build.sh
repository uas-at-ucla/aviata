#!/bin/bash
mkdir -p controls/build
cd controls/build
cmake -D CMAKE_CXX_FLAGS=-DUSE_ROS ..
make
