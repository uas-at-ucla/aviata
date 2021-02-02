#ifndef PX4_IO_HPP
#define PX4_IO_HPP

#include <string>
#include <vector>

#include "../mavlink/v2.0/common/mavlink.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include "mavsdk_callback_manager.hpp"

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

// Copied from PX4 source code
#define VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED 1
#define PX4_CUSTOM_MAIN_MODE_OFFBOARD 6

using namespace mavsdk;

// mavlink message id's https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml
// mavlink message typedefs https://github.com/mavlink/c_library_v2/tree/master/common
// AVIATA mavlink fork: https://github.com/uas-at-ucla-dependencies/mavlink

class PX4IO
{
public:
    PX4IO(std::string drone_id);
    
    bool connect_to_pixhawk(std::string connection_url, int timeout_seconds);
    
    void call_queued_mavsdk_callbacks();

    int arm_system();
    int disarm_system();

    int arm();
    int disarm();

    int set_offboard_mode();

    int takeoff_system();

    int land_system();

    // void goto_gps_position(double lat, double lon); // for DOCKED_LEADER (send attitude and thrust to followers)

    int goto_gps_position(double lat, double lon, float alt, float yaw);

    void subscribe_attitude_target(std::function<void(const mavlink_attitude_target_t&)> user_callback);
    void unsubscribe_attitude_target();
    int set_attitude_target(mavlink_set_attitude_target_t& att_target_struct);

    void subscribe_flight_mode(std::function<void(Telemetry::FlightMode)> user_callback);
    void unsubscribe_flight_mode();

    void subscribe_armed(std::function<void(bool)> user_callback);
    void unsubscribe_armed();

    int dock(uint8_t docking_slot, uint8_t* missing_drones, uint8_t n_missing);
    int undock();

    int takeoff_and_land_test(int argc, char** argv);

private:
    const std::string drone_id;

    Mavsdk mav;
    std::shared_ptr<System> sys;
    std::shared_ptr<Telemetry> telemetry;
    std::shared_ptr<Action> action;
    std::shared_ptr<MavlinkPassthrough> mavlink_passthrough;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t our_system_id;
    uint8_t our_component_id;

    MavsdkCallbackManager mavsdk_callback_manager;

    void usage(std::string bin_name);
    void component_discovered(ComponentType component_type);
};

#endif
