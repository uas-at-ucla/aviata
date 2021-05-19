# AVIATA Docking Subsystem

This subsystem focuses on the docking portion of AVIATA - from the time a drone reaches the system, to navigating to its assigned docking location, to physically attaching itself to the rest of the system.

## Directory organization:
* The main C++ project is found under `cpp_docking/`
  * `camera_simulator`: outputs an image of what the drone "sees" given the drone's real-time position and a starting position for the Apriltag
  * `image_analyzer`: tanalyzes a picture taken by the drone to identify the Apriltag and determine how far away we are in the x (east), y (north), z (down) directions, and how far off we are rotationally (yaw)
  * `drone`: handles communicating with the drone through MAVSDK and with the other modules
  * `pid_controller`: turns calculated errors from the latest camera frame into velocity commands to send to the drone
  * `raspi_camera`: interface for the camera on-board the drone. Replaces `camera_simulator` when run in physical testing
  * `docking_detector`: controls GPIO pins for the circuit that enables the drone to know whether it is docked. This module is ignored if compiling for simulation.
  * `docking`: entrypoint
* The rest of the folders are old code related to research during the proposal stage and preliminary designs

## Installation/Prerequisites
* CMake
* PX4 and Gazebo (requires a UNIX-like environment - likely through a virtual machine, depending on your OS)
  * See [this UAS@UCLA guide](https://uasatucla.org/docs/software/tutorials/new-page)
  * If you are on Linux, no need for the VM
  * If you are on macOS and don't want to use a VM, you may be able to install Gazebo 9 natively
  * There is no alternative for Windows (that I know of)
* C++ bindings for MAVSDK
  * Download the `.deb` file from [their repository](https://github.com/mavlink/MAVSDK/releases) corresponding to your Ubuntu version (20.04 for me)
  * Install: `sudo dpkg -i mavsdk_0.35.1_ubuntu20.04_amd64.deb`
  * See "Installation" section on MAVSDK's QuickStart guide: https://mavsdk.mavlink.io/develop/en/cpp/quickstart.html
* OpenCV for C++
  * Instructions here: https://docs.opencv.org/4.5.1/d0/d3d/tutorial_general_install.html. You may need to compile from source.
  * Install at least v4.2
  * Check if you already have it: `ls /usr/include | grep opencv`
  * To install on Raspberry Pi: https://qengineering.eu/install-opencv-4.5-on-raspberry-pi-4.html
* AprilTag: https://github.com/AprilRobotics/apriltag
  * Clone the repository, then follow the README
* `pigpio`, if running on Raspberry Pi. Ignore if running simulation only
  * http://abyz.me.uk/rpi/pigpio/download.html
  * Perform these steps in a directory where the files can safely live permanently (e.g. `~/Documents`)
  * Using GPIO through this library unfortunately requires `sudo` - i.e., you'll have to run the docking code with `sudo ./docking-physical`
```
$ wget https://github.com/joan2937/pigpio/archive/master.zip
$ unzip master.zip
$ cd pigpio-master
$ make
$ sudo make install
```

## Preparing and compiling the code

Use CMake as follows

To build for simulation,
```
$ cd cpp_docking
$ mkdir build
$ cd build
$ cmake .. # generate Makefile
$ make # compile
```

To build for the Raspberry Pi,

```
$ cd cpp_docking
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release .. # generate Makefile (with optimization)
$ make docking_physical # compile
```
General notes
  * You should only have to rerun `cmake ..` if `CMakeLists.txt` changes
  * To run changes to the code itself, just run `make` followed by the executable (e.g. `docking_simulation`)

## Running the simulation
1. Start Gazebo (from the directory in which you installed PX4)
```
$ make px4_sitl gazebo
```
You can also run Gazebo in headless mode - i.e., without a GUI showing the 3D world. You'll still be able to see what the drone's doing through the camera simulator (in 1st person), but you won't have a 3rd person view. The benefit is that it requires much less CPU power to run.
```
$ HEADLESS=1 make px4_sitl gazebo
```

2. Wait for PX4 to finish initializing - you'll see a log message in the terminal that states `INFO home_set`. This should take roughly 10 seconds from the moment the Gazebo window shows up. Otherwise, you may get strange errors such as the drone not taking off when it says it has.
3. Run the program script
```
$ cd build/
$ ./docking_simulation
```

## Running on the Raspberry Pi

Superuser privileges are required due to use of GPIO pins and the way that `pigpio` is installed (using `sudo make install`, which adds the relevant files to `/usr/...`). I haven't explored this deeply enough yet to find a workaround, but one should be possible given that GPIO permissions are generally available to any normal user within the `gpio` group.
```
$ sudo ./docking-physical
```

Optional: save logs for later debrief with
```
$ sudo ./docking-physical | tee log.txt
```
