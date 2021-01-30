#!/bin/bash

#This step is to init and clone OONF to current repo
git submodule update --init -- ./mesh/OONF

echo "Now OONF is cloned to current repo, go to folder ./mesh/OONF for instructions how to build and run the framework"
