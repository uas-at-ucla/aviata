#!/bin/bash

# USAGE: ./generate_makefile.sh [release]
# if "release" is set as an argument, cmake will
# generate the Makefile with compiler optimizations
# otherwise, the compiler will be set in debug mode

OPTS="-DCMAKE_BUILD_TYPE=Debug"
if [ "$1" == "release" ]; then
	echo "building in release mode"
	OPTS="-DCMAKE_BUILD_TYPE=Release"
fi

mkdir build
cd build && cmake $OPTS ..
