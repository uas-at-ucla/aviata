#!/bin/bash
git submodule update --init --recursive
mkdir -p controls/build
cd controls/build
cmake ..
make
