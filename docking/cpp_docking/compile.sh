#!/bin/bash

# USAGE: ./compile.sh
# if you want to build this project so it runs on the Raspberry Pi,
# specify `docking_physical` as an argument

cd build && make $1
