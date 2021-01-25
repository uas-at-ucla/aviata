# Autonomous Vehicle Infinite Air Time Apparatus (AVIATA)
Our research project for the 2020 NASA University Student Research Challenge (USRC) is an autonomous drone swarm that is capabable of collaboratively carrying a payload and swapping in and out mid-flight, effectively giving the swarm an infinite flight time. Possible applications include long-distance transportation, emergency relief, and any other instances that require heavy payloads and/or long flight times.

This repository focuses on the software aspects of AVIATA, which include inter-drone communication through a mesh network, ground-swarm communication, and a GPS and vision-based docking system for attaching to the payload.

Structure:
* `docking/` - autonomous docking implementation, currently includes code developed for research during the proposal stage 
* `controls/` - all the non-docking controls code for the Raspberry Pi
* `mesh/` - mesh networking between drones
* `ground/` - everything related to the ground station, including communication with drones and coordination/monitoring for swapping

# Controls Code
## Building Controls Code
Installation Requirements:
* cmake
* https://mavsdk.mavlink.io/develop/en/getting_started/installation.html
* ROS2

Build:
```bash
# source your ROS2 setup script
cd controls
mkdir build
cd build
cmake ..
make
```

## Running Controls Code
### Single Drone PX4 Simulation
Run in the PX4-Autopilot folder:
```bash
make px4_sitl gazebo_typhoon_h480
```

### Multi-Vehicle PX4 Simulation
Run in the PX4-Autopilot folder:
```bash
Tools/gazebo_sitl_multiple_run.sh -m typhoon_h480 -n 2
```

### Run AVIATA Controls Code (temporary)
First, run this:
* On Linux: `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"."`
* On MacOS: `export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:"."`

Next, run the `aviata_drone` executable from within the controls/build folder. For example, you can run these two commands in separate terminals if you are using multi-vehicle simulation:
```bash
./aviata_drone follower follow udp://:14541
```
```bash
./aviata_drone leader lead udp://:14540
```

## Build PX4-Autopilot for Pixhawk
```bash
make px4_fmu-v3_default
```
