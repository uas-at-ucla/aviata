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
2. Wait for PX4 to finish initializing - you'll see a log message in the terminal that states `INFO home_set`. This should take roughly 10 seconds from the moment the Gazebo window shows up. Otherwise, you may get strange errors such as the drone not taking off when it says it has.
3. Run the startup script (from this directory)
```
$ python3 docking/docking.py
```

## Extra Python libraries needed
mavsdk
```
$ pip3 install mavsdk
```
tkinter
```
$ sudo apt-get install python3-tk
```

# C++ Version

### Basic info:
* Currently isolated to `docking/cpp_docking`
* Build using CMake
  * You should only have to rerun `cmake ..` if `CMakeLists.txt` changes
  * To run changes to the code itself, just run `make` followed by the executable (`./docking`)
* Right now, if you run the code, the drone is programmed to take off, then rotate in place at 36 degrees/second. The camera simulator shows this in real time. 

### Progress
- Translation is being done line-by-line
- Attempt to keep it as close in structure as possible to Python version to prevent introducing new bugs during translation
- Some APIs are slightly different, mainly in OpenCV (e.g. most Mats are passed and manipulated by reference instead of being returned, such as for resizing)
  - OpenCV docs include Python and C++ API together, making it easy
  - MAVSDK API is mostly the same in Python and C++, but very well documented
- Currently translated:
  - Drone class
    - connection to Gazebo
    - takeoff
    - initiating docking
  - Camera Simulator class
- Need to work on
  - Image Analyzer class
  - Remainder of drone class
    - stage 1
    - stage 2
    - landing
    

```
$ cd cpp_docking
$ mkdir build
$ cd build
$ cmake .. # generate Makefile
$ make # compile
$ ./docking # run the code
```

### Setup
* Install C++ bindings for MAVSDK
  * Download the `.deb` file from [their repository](https://github.com/mavlink/MAVSDK/releases) corresponding to your Ubuntu version (20.04 for me)
  * Install: `sudo dpkg -i mavsdk_0.24.0_ubuntu18.04_amd64.deb`
  * See "Installation" section on MAVSDK's QuickStart guide: https://mavsdk.mavlink.io/develop/en/cpp/quickstart.html
* Install OpenCV for C++
  * For some reason, I didn't have to do this step - I think it's because I already had it from installing OpenCV for the Python version of docking,
  and CMake was able to find it. I think it should build without you messing with OpenCV, but if that fails, instructions here: https://docs.opencv.org/4.5.1/d0/d3d/tutorial_general_install.html
  * FYI I have v4.2 installed
* AprilTag not yet implemented, so no library setup necessary (yet)

Install any other missing libraries using `pip`.