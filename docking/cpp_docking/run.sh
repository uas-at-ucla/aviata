#!/bin/bash

TIMESTAMP=`date +%Y-%m-%d_%H-%M-%S`
cd build && ./docking_simulation | tee $TIMESTAMP.log
