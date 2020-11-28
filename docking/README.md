# AVIATA Docking Subsystem

This subsystem focuses on the docking portion of AVIATA - from the time a drone reaches the system, to navigating to its assigned docking location, to physically attaching itself to the rest of the system.

## Directory organization:
* The main Python project is found under `docking/`
  * `camera_simulator`: this module outputs an image of what the drone "sees" given the drone's real-time position and a starting position for the Apriltag.
  * `image_analyzer`: this module consists of a single function, which analyzes a picture taken by the drone to identify the Apriltag and determine how far away we are in the x (east), y (north), z (down) directions, and how far off we are rotationally (yaw).
  * `drone`: main module that defines a Drone class, to handle communicating with the drone through MAVSDK and with the other modules.
  * `docking`: startup script
* The rest of the folders are old code related to research during the proposal stage and preliminary designs

## Installation/Prerequisites
* [Python3](https://www.python.org/downloads/)
* PX4 and Gazebo
  * See [this UAS@UCLA guide](https://uasatucla.org/docs/software/tutorials/new-page)
  * If you are on Linux, no need for the VM
  * If you are on macOS and don't want to use a VM, you may be able to install Gazebo 9 natively
  * There is no alternative for Windows (that I know of)

## Running
1. Start Gazebo (from the directory in which you installed PX4)
```
$ make px4_sitl gazebo
```
2. Run the startup script (from this directory)
```
$ python3 docking/docking.py
```