#ifndef PX4_IO_HPP
#define PX4_IO_HPP

#include <string>
#include <vector>
#include <mavsdk/mavsdk.h>

#include "../mavlink/v2.0/common/mavlink.h"

#include "dronetelemetry.hpp"

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

// mavlink message ID for attitude targets
// mavlink message id's https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml
// mavlink message typedefs https://github.com/mavlink/c_library_v2/tree/master/common
// AVIATA mavlink fork: https://github.com/uas-at-ucla-dependencies/mavlink

int takeoff_and_land_test(int argc, char** argv);

std::shared_ptr<mavsdk::System> connect_to_pixhawk(std::string drone_id, std::string connection_url);

int arm_system();

int disarm_system();

int takeoff_system();

int land_system();

// void goto_gps_position(double lat, double lon); // for DOCKED_LEADER (send attitude and thrust to followers)

int goto_gps_position(double lat, double lon, float alt, float yaw);

void subscribe_attitude_and_thrust(float q[4], float* thrust); //async
void subscribe_attitude_and_thrust(mavlink_attitude_target_t *att_target_struct);

void unsubscribe_attitude_and_thrust();

int set_attitude_and_thrust(float q[4], float* thrust);
int set_attitude_and_thrust(mavlink_attitude_target_t *att_target_struct);

#endif
