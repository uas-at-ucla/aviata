#ifndef PX4_IO_HPP
#define PX4_IO_HPP

#include <string>
#include <vector>
#include <mavsdk/mavsdk.h>

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

#define MAV_CMD_AVIATA_FINALIZE_DOCKING 43001
#define MAV_CMD_AVIATA_SET_CONFIGURATION 43002
#define MAV_CMD_AVIATA_SET_STANDALONE 43003

int takeoff_and_land_test(int argc, char** argv);

std::shared_ptr<mavsdk::System> connect_to_pixhawk(std::string drone_id, std::string connection_url);

int arm_system();

int disarm_system();

int takeoff_system();

int land_system();

void goto_gps_position(float lat, float lon); // for DOCKED_LEADER (send attitude and thrust to followers)

void get_attitude_and_thrust(float q[4], float* thrust);

void set_attitude_and_thrust(float q[4], float thrust);

#endif
