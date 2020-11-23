#include <string>

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

#define MAV_CMD_AVIATA_FINALIZE_DOCKING 43001
#define MAV_CMD_AVIATA_SET_CONFIGURATION 43002
#define MAV_CMD_AVIATA_SET_STANDALONE 43003

int takeoff_and_land_test(int argc, char** argv);

int connect_to_pixhawk(std::string connection_url);

void arm_drone(); // for drones in STANDBY / DOCKED_FOLLOWER

void arm_frame(); // for DOCKED_LEADER (send arm_drone() to followers)

void disarm_drone(); // for drones in STANDBY / DOCKED_FOLLOWER

void disarm_frame(); // for DOCKED_LEADER (send disarm_drone() to followers)

void takeoff_drone(); // for drones in STANDBY

void takeoff_frame(); // for DOCKED_LEADER (send attitude and thrust to followers)

void land_drone(); // for any undocked drone

void land_frame(); // for DOCKED_LEADER (send attitude and thrust to followers)

void goto_gps_position(float lat, float lon); // for DOCKED_LEADER (send attitude and thrust to followers)

void get_attitude_and_thrust(float q[4], float* thrust);

void set_attitude_and_thrust(float q[4], float thrust);
