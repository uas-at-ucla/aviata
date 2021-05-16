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

Next, run the `aviata_drone` executable from within the controls/build folder. The usage is `./aviata_drone <drone_id> <initial_state> [docking_slot] [connection_url]` (the docking slot number does not matter for simulation). For example, you can run these two commands in separate terminals if you are using multi-vehicle simulation:
```bash
./aviata_drone drone1 leader 0 udp://:14540
```
```bash
./aviata_drone drone2 follower 1 udp://:14541
```

The drone that was initiated as the leader can be controlled from QGroundControl, and the follower should roughly copy it, although expect some deviations since the program expects the drones to be docked.

The `aviata_ground` executable currently allows you to specify the leader drone name, and will tell the drone to hand off the leader role to another drone. The new leader can then be controlled from QGroundControl. The ground station is in progress, so more features to come.

## Build PX4-Autopilot for Pixhawk
```bash
make px4_fmu-v3_default
```

# Comms Code
## Init and build
First run ./init_mesh.sh to initiate and clone the submodule code. Then go to ./mesh/OONF to buiuld the framework. use command:
```bash
cd build
cmake ..
make
```

## Disable Other Network Services
If other network services are running on the machine and using the network interface for the mesh network, they may interfere with it. For example, if you have wpa_supplicant.conf configured to connect to a WiFi network, run `wpa_cli terminate`, or alternatively comment out the contents of wpa_supplicant.conf and reboot.

## Run the code
First go to build folder (mesh/OONF/build) and create a file with unique id (0 - 255), note that NO node in your network should have the same ID.
Then, go to OONF root folder (mesh/OONF) and run
```bash
run.sh <INTERFACE_NAME>  
```
<INTERFACE_NAME> should be the network interace you want to use for the mesh network, check it with command ifconfig.

For more instructions and troubleshooting, see: https://docs.google.com/document/d/18OtGG7QGEEyhy4e1Phuv2IOWkYh8ndprgu_6wa8lDLM/edit?usp=sharing
