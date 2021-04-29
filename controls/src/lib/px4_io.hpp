#ifndef PX4_IO_HPP
#define PX4_IO_HPP

#include <string>
#include <vector>

#include "../mavlink/v2.0/common/mavlink.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include "mavsdk_callback_manager.hpp"
#include "dronestatus.hpp"

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

// Copied from PX4 source code
typedef enum VEHICLE_MODE_FLAG {
	VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, /* 0b00000001 Reserved for future use. | */
	VEHICLE_MODE_FLAG_TEST_ENABLED = 2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
	VEHICLE_MODE_FLAG_AUTO_ENABLED = 4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
	VEHICLE_MODE_FLAG_GUIDED_ENABLED = 8, /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
	VEHICLE_MODE_FLAG_STABILIZE_ENABLED = 16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
	VEHICLE_MODE_FLAG_HIL_ENABLED = 32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
	VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED = 64, /* 0b01000000 remote control input is enabled. | */
	VEHICLE_MODE_FLAG_SAFETY_ARMED = 128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | */
	VEHICLE_MODE_FLAG_ENUM_END = 129, /*  | */
} VEHICLE_MODE_FLAG;
enum PX4_CUSTOM_MAIN_MODE {
	PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
	PX4_CUSTOM_MAIN_MODE_ALTCTL,
	PX4_CUSTOM_MAIN_MODE_POSCTL,
	PX4_CUSTOM_MAIN_MODE_AUTO,
	PX4_CUSTOM_MAIN_MODE_ACRO,
	PX4_CUSTOM_MAIN_MODE_OFFBOARD,
	PX4_CUSTOM_MAIN_MODE_STABILIZED,
	PX4_CUSTOM_MAIN_MODE_RATTITUDE,
	PX4_CUSTOM_MAIN_MODE_SIMPLE /* unused, but reserved for future use */
};
enum PX4_CUSTOM_SUB_MODE_AUTO {
	PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
	PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
	PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
	PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
	PX4_CUSTOM_SUB_MODE_AUTO_RTL,
	PX4_CUSTOM_SUB_MODE_AUTO_LAND,
	PX4_CUSTOM_SUB_MODE_AUTO_RESERVED_DO_NOT_USE, // was PX4_CUSTOM_SUB_MODE_AUTO_RTGS, deleted 2020-03-05
	PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET,
	PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND
};

using namespace mavsdk;

// mavlink message id's https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml
// mavlink message typedefs https://github.com/mavlink/c_library_v2/tree/master/common
// AVIATA mavlink fork: https://github.com/uas-at-ucla-dependencies/mavlink

class PX4IO
{
public:
    PX4IO(std::string drone_id, DroneSettings drone_settings);
    
    bool connect_to_pixhawk(std::string connection_url, int timeout_seconds);
    std::shared_ptr<Telemetry> telemetry_ptr();
    std::shared_ptr<Offboard> offboard_ptr();

    void call_queued_mavsdk_callbacks();

    int arm_system();
    int disarm_system();

    int arm();
    int disarm();

    int set_offboard_mode();
    int set_hold_mode();

    int takeoff_system();

    int land_system();

    // void goto_gps_position(double lat, double lon); // for DOCKED_LEADER (send attitude and thrust to followers)

    int goto_gps_position(double lat, double lon, float alt, float yaw);

    void subscribe_attitude_target(std::function<void(const mavlink_attitude_target_t&)> user_callback);
    void unsubscribe_attitude_target();
    int set_attitude_target(mavlink_set_attitude_target_t& att_target_struct);

    void subscribe_flight_mode(std::function<void(Telemetry::FlightMode)> user_callback);
    void unsubscribe_flight_mode();
    void subscribe_status_text(std::function<void(Telemetry::StatusText)> user_callback);
    void unsubscribe_status_text();

    void subscribe_armed(std::function<void(bool)> user_callback);
    void unsubscribe_armed();

    int dock(uint8_t docking_slot, uint8_t* missing_drones, uint8_t n_missing);
    int undock();

    int takeoff_and_land_test(int argc, char** argv);

private:
    const std::string drone_id;
    const DroneSettings drone_settings;

    Mavsdk mav;
    std::shared_ptr<System> sys;
    std::shared_ptr<Telemetry> telemetry;
    std::shared_ptr<Offboard> offboard;
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
