# Autonomous Vehicle Infinite Air Time Apparatus (AVIATA)
Our research project for the 2020 NASA University Student Research Challenge (USRC) is an autonomous drone swarm that is capabable of collaboratively carrying a payload and swapping in and out mid-flight, effectively giving the swarm an infinite flight time. Possible applications include long-distance transportation, emergency relief, and any other instances that require heavy payloads and/or long flight times.

This repository focuses on the software aspects of AVIATA, which include inter-drone communication through a mesh network, ground-swarm communication, and a GPS and vision-based docking system for attaching to the payload.

Structure:
* `docking/` - autonomous docking implementation, currently includes code developed for research during the proposal stage 
* `controls/` - all the non-docking controls code for the Raspberry Pi
* `mesh/` - mesh networking between drones
* `ground/` - everything related to the ground station, including communication with drones and coordination/monitoring for swapping

## Acknowledgement
These results are based upon work supported by the NASA Aeronautics Research Mission Directorate under award number 80NSSC20K1452. This material is based upon a proposal tentatively selected by NASA for a grant award of $10,811, subject to successful crowdfunding. Any opinions, findings, and conclusions or recommendations expressed in this material are those of the authors and do not necessarily reflect the views of NASA.

# Python Simulation
Documentation for the Python simulation is coming soon!

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

### Run AVIATA Controls Code
Source your ROS2 setup script, then run this:
* On Linux: `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"."`
* On MacOS: `export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:"."`

A shortcut for these things on RPi is `./rpi_tools/env_setup.sh`.

Next, run the `aviata_drone` executable from within the controls/build folder. The usage is `./aviata_drone <drone_id> <initial_state> [docking_slot] [connection_url]` (the docking slot number does not matter for simulation). For example, you can run these two commands in separate terminals if you are using multi-vehicle simulation:
```bash
./aviata_drone drone1 leader 0 udp://:14540
```
```bash
./aviata_drone drone2 follower 1 udp://:14541
```

The drone that was initiated as the leader can be controlled from QGroundControl, and the follower should roughly copy it, although expect some deviations since the program expects the drones to be docked.

Drones can also be started in `standby` mode, meaning they are idle (presumably at the ground station). Currently, this mode immediately transitions to docking, so the drone will immediately take off, fly to the swarm, and begin the docking process, but eventually this transition will be triggered by an external command.
```bash
./aviata_drone drone2 standby 1 udp://:14541
```

The `aviata_ground` executable currently allows you to specify the leader drone name, and will tell the drone to hand off the leader role to another drone. The new leader can then be controlled from QGroundControl. The ground station is in progress, so more features to come.

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

# Initial Software/Electronics Setup
## Raspberry Pi Setup
1. Flash the SD Card for the Pi with Raspberry Pi OS Lite.
2. SSH into the Pi and give it internet access either directly or via sharing on your computer.
3. Install git: `sudo apt-get update && sudo apt-get install -y git`
3. Clone this repository: `git clone https://github.com/uas-at-ucla/aviata.git`
4. Run `./aviata/rpi_tools/install.sh`. Make sure you can let it do its thing for a while, since ROS2 takes a long time to install.
5. Enable the serial port on the Pi. See 23:40-25:51 of [this video](https://youtu.be/kB9YyG2V-nA?t=1420).
6. Enable the camera on the Pi (Interfacing Options in `sudo raspi-config`).

## Setting PX4 Params
1. Connect a drone via any means to QGroundControl.
2. Change, add, or remove parameters in **set_px4_params.sh** as necessary (in the root folder of this project).
3. Either in **set_px4_params.sh** or QGroundControl itself, set MAV_SYS_ID to an appropriate unique ID (only needed for cooperative controls tests).
4. Copy all or part of the contents of **set_px4_params.sh** and paste in QGroundControl -> Analyze Tools -> MAVLink Console.
5. The parameters have been set! Now remove power from the Pixhawk. Once you power it back on, you're all set!
6. Next time you want to update some parameters, you can skip step 3, as well as step 5 depending on which parameters are changed.

## Building and Flashing PX4 Firmware (only needed for cooperative controls tests)
1. Retrieve the code from https://github.com/uas-at-ucla-dependencies/PX4-Autopilot/tree/release/1.11-aviata (use the **release/1.11-aviata** branch).
2. If needed, set the contents of **src/lib/mixer_module/aviata_mixers.h** (in PX4-Autopilot) with the appropriate set of mixers generated by **simulation/generate_matrices_<config>.py** (in this repository), e.g. `python3 generate_matrices_aviata.py > aviata_mixers.h`.
3. Run `make px4_fmu-v3_default` in the PX4-Autopilot folder. You may need to install some things to build the code. See https://docs.px4.io/master/en/dev_setup/dev_env.html for more details, but note that you do not need any simulation tools for this step.
4. Using QGroundControl, flash the custom firmware file **PX4-Autopilot/build/px4_fmu-v3_default/px4_fmu-v3_default.px4** to the Pixhawk. If this fails, follow the instructions to update the FMUv2 bootloader [here](https://docs.px4.io/master/en/config/firmware.html#fmuv2-bootloader-update) and try again.

## Pixhawk Wiring
Most of the Pixhawk wiring involved is pretty standard. However, if conducting cooperative controls tests, you must use an airframe that uses the AUX outputs instead of the main outputs (details in **set_px4_params.sh**). Therefore, when wiring the ESCs, use the corresponding AUX outputs instead of the main ouputs.

For reference, here is the motor numbering for a hexacopter: https://docs.px4.io/master/en/airframes/airframe_reference.html#hexarotor-x

Raspberry Pi wiring can be found here: https://community.sixfab.com/uploads/default/original/1X/7b9bf4fdc3cc82072622c1eb84d6b76fc9ea94f6.png. You may need to do some soldering to make the proper wire for this.

## Flashing the Telemetry Radio
To connect multiple drones to one laptop, we configure the telemetry radios with firmware that supports multipoint communication. There is a ground station radio with ID 0, and a radio for each drone with its own unique ID.
1. Download **radio~hm_trp.ihx** from https://drive.google.com/drive/folders/1yd3WoalLU3xQads8v3oJmjZb5dtD86ct?usp=sharing
2. Retrieve the code from https://github.com/RFDesign/SiK. In particular, you only need the file **Firmware/tools/uploader.py**.
3. Follow the instructions [here](https://risc.readthedocs.io/2-multi-point-telemetry.html#upload-firmware-to-the-radio) (starting with "Upload Firmware to the radio") to configure each radio.
