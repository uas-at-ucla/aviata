/*
 * Based on the MAVSDK example here: https://github.com/mavlink/MAVSDK/tree/develop/examples/takeoff_land
 * This file will become a helper for interfacing with PX4.
 */

#include "px4_io.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <thread>

using namespace std::this_thread;
using namespace std::chrono;

PX4IO::PX4IO(std::string drone_id, DroneSettings drone_settings): drone_id(drone_id), drone_settings(drone_settings) {}

//Discovering systems (the new way): https://mavsdk.mavlink.io/develop/en/cpp/api_changes.html
//returns true if connection successful, false otherwise
bool PX4IO::connect_to_pixhawk(std::string connection_url, int timeout_seconds)
{
    ConnectionResult connection_result;
    connection_result = mav.add_any_connection(connection_url);
    // connect
    if (connection_result != ConnectionResult::Success) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to connect: " << connection_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return false;
    }

    std::cout << "Waiting to discover system..." << std::endl;
    // discover system and return - method sourced from link above
    // auto new_system_promise = std::promise<std::shared_ptr<System>>{};
    // auto new_system_future = new_system_promise.get_future();
    // mav.subscribe_on_new_system([&]() {
    //     std::cout << "Discovered system " << drone_id << std::endl;
    //     new_system_promise.set_value(mav.systems().at(0));
    //     mav.subscribe_on_new_system(nullptr);
    // });

    // // We usually receive heartbeats at 1Hz, therefore we should find a system after around 2 seconds.
    // sleep_for(seconds(2));

    // auto system = new_system_future.get();
    // if (system == nullptr) {
    //     std::cout << ERROR_CONSOLE_TEXT << drone_id << " was not found, exiting." 
    //               << NORMAL_CONSOLE_TEXT << std::endl;
    //     return false;
    // }

    while (mav.systems().size() == 0) {
        if (timeout_seconds == 0) {
            std::cout << ERROR_CONSOLE_TEXT << drone_id << " was not found, exiting." 
                      << NORMAL_CONSOLE_TEXT << std::endl;
            return false;
        }
        sleep_for(seconds(1));
        timeout_seconds--;
    }
    auto system = mav.systems().at(0);

    sys = system;
    telemetry = std::make_shared<Telemetry>(sys);
    offboard = std::make_shared<Offboard>(sys);
    action = std::make_shared<Action>(sys);
    mavlink_passthrough = std::make_shared<MavlinkPassthrough>(sys);
    target_system = mavlink_passthrough->get_target_sysid();
    target_component = mavlink_passthrough->get_target_compid();
    our_system_id = mavlink_passthrough->get_our_sysid();
    our_component_id = mavlink_passthrough->get_our_compid();

    return true;
}

uint8_t PX4IO::drone_system_id() {
    return target_system;
}

std::shared_ptr<Telemetry> PX4IO::telemetry_ptr() {
    return telemetry;
}

std::shared_ptr<Offboard> PX4IO::offboard_ptr() {
    return offboard;
}

void PX4IO::call_queued_mavsdk_callbacks() {
    mavsdk_callback_manager.call_queued_mavsdk_callbacks();
}

// @return 1 if successful, 0 otherwise
int PX4IO::wait_for_arm()
{
    if (telemetry->armed()) // if already armed
        return 1;
    while (!telemetry->health_all_ok())
    {
        std::cout << drone_id << " is getting ready to arm." << std::endl;
        sleep_for(milliseconds(100));
    }
    // Arm vehicle
    std::cout << "Arming " << drone_id << "..." << std::endl;
    const Action::Result arm_result = action->arm();
    if (arm_result != Action::Result::Success)
    {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to arm: " << arm_result << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 0;
    }
    while (!telemetry->armed())
    {
        sleep_for(milliseconds(100));
    }
    return 1;
}

// @return 1 if successful, 0 otherwise
int PX4IO::wait_for_disarm()
{
    if (!telemetry->armed()) // if already disarmed
        return 1;
    // Verify drone is on ground
    while (telemetry->in_air() != true)
    {
        std::cout << "Verifying " << drone_id << " is not in the air..." << std::endl;
        sleep_for(milliseconds(100));
    }
    // Disarm vehicle
    std::cout << "Disarming " << drone_id << "..." << std::endl;
    const Action::Result disarm_result = action->disarm();
    if (disarm_result != Action::Result::Success)
    {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to disarm: " << disarm_result << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 0;
    }
    return 1;
}

// @return 1 if successful, 0 otherwise
int PX4IO::arm()
{
    std::cout << "Arming " << drone_id << std::endl;
    const Action::Result result = action->arm();
    if (result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to arm: " << result << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 0;
    }
    return 1;
}

// @return 1 if successful, 0 otherwise
int PX4IO::disarm()
{
    std::cout << "Disarming " << drone_id << std::endl;
    const Action::Result result = action->disarm();
    if (result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to disarm: " << result << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 0;
    }
    return 1;
}

// @return 1 if successful, 0 otherwise
int PX4IO::set_offboard_mode() {
    MavlinkPassthrough::CommandLong offboard_command;
    offboard_command.target_sysid = target_system;
    offboard_command.target_compid = target_component;
    offboard_command.command = MAV_CMD_DO_SET_MODE;
    offboard_command.param1 = VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED;
    offboard_command.param2 = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
    MavlinkPassthrough::Result result = mavlink_passthrough->send_command_long(offboard_command);
    if (result != MavlinkPassthrough::Result::Success) {
        return 0;
    }
    return 1;
}

// @return 1 if successful, 0 otherwise
int PX4IO::set_hold_mode() {
    MavlinkPassthrough::CommandLong command;
    command.target_sysid = target_system;
    command.target_compid = target_component;
    command.command = MAV_CMD_DO_SET_MODE;
    command.param1 = VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED;
    command.param2 = PX4_CUSTOM_MAIN_MODE_AUTO;
    command.param3 = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
    MavlinkPassthrough::Result result = mavlink_passthrough->send_command_long(command);
    if (result != MavlinkPassthrough::Result::Success) {
        return 0;
    }
    return 1;
}

// @return 1 if successful, 0 otherwise
int PX4IO::takeoff_system()
{
    if (!telemetry->armed()){
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " is not armed. Arm first before takeoff." 
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 0;
    }

    std::cout << drone_id << " taking off..." << std::endl;
    const Action::Result takeoff_result = action->takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to takeoff:" << takeoff_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 0;
    }
    return 1;
}

// @return 1 if successful, 0 otherwise
int PX4IO::land_system()
{
    std::cout << drone_id << " landing..." << std::endl;
    const Action::Result land_result = action->land();
    if (land_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to land:" << land_result 
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 0;
    }
    return 1;
}

// @return 1 if successful, 0 otherwise
int PX4IO::goto_gps_position(double lat, double lon, float alt, float yaw)
// for DOCKED_LEADER (send attitude and thrust to followers)
{
    const Action::Result goto_result = action->goto_location(lat, lon, alt, yaw);
    if (goto_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to go to:" << lat << " " << lon
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 0;
    }
    return 1;
}

void PX4IO::subscribe_attitude_target(const std::function<void(const mavlink_attitude_target_t&)>& user_callback)
{
    mavsdk_callback_manager.subscribe_mavsdk_callback<const mavlink_message_t&>(
        [this](std::function<void(const mavlink_message_t&)> callback) {
            mavlink_passthrough->subscribe_message_async(MAVLINK_MSG_ID_ATTITUDE_TARGET, callback);
        },
        [user_callback](const mavlink_message_t& attitude_target_message) {
            mavlink_attitude_target_t att_target_struct;
            mavlink_msg_attitude_target_decode(&attitude_target_message, &att_target_struct);
            user_callback(att_target_struct);
        }
    );
}

void PX4IO::unsubscribe_attitude_target()
{
    mavlink_passthrough->subscribe_message_async(MAVLINK_MSG_ID_ATTITUDE_TARGET, nullptr);
}

// @brief Ignores the attitude rates
// @return 1 if successful, 0 otherwise
// int PX4IO::set_attitude_and_thrust(float q[4], float* thrust)
// {
//     mavlink_set_attitude_target_t set_att_struct;
//     std::copy(q, q + 4, set_att_struct.q);
//     set_att_struct.thrust = *thrust;
//     set_att_struct.type_mask =  ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE |
//                                 ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE |
//                                 ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE;

//     mavlink_message_t set_attitude_target_message;
//     uint16_t encode_result; //encode function returns a uint16_t, not sure what it represents
//     encode_result = mavlink_msg_set_attitude_target_encode(our_system_id, our_component_id, &set_attitude_target_message, &set_att_struct);
    
//     MavlinkPassthrough::Result set_att_result = mavlink_passthrough->send_message(set_attitude_target_message);
//     if ( set_att_result != MavlinkPassthrough::Result::Success){
//         std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to set attitude and thrust." 
//                   << NORMAL_CONSOLE_TEXT << std::endl;
//         return 0;
//     }
//     return 1;    
// }

// @brief Copies the attitude rates
// @return 1 if successful, 0 otherwise
int PX4IO::set_attitude_target(mavlink_set_attitude_target_t& att_target_struct)
{
    // att_target_struct.body_roll_rate = 0;
    // att_target_struct.body_pitch_rate = 0;
    att_target_struct.body_yaw_rate = 0;
    att_target_struct.type_mask = ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE | 
                                  ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE |
                                  ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE;
    att_target_struct.target_system = target_system;
    att_target_struct.target_component = target_component;

    mavlink_message_t set_attitude_target_message;
    uint16_t encode_result; //encode function returns a uint16_t, not sure what it represents
    encode_result = mavlink_msg_set_attitude_target_encode(our_system_id, our_component_id, &set_attitude_target_message, &att_target_struct);

    MavlinkPassthrough::Result result = mavlink_passthrough->send_message(set_attitude_target_message);
    if (result != MavlinkPassthrough::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to set attitude and thrust." 
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 0;
    }
    return 1;    
}

int PX4IO::set_mixer_docked(uint8_t docking_slot, uint8_t* missing_drones, uint8_t n_missing)
{
    if (!drone_settings.modify_px4_mixers) {
        return 1;
    }

    if (n_missing > 6) { // too many missing
        return 0;
    }

    MavlinkPassthrough::CommandLong cmd;
    cmd.target_sysid = sys->get_system_id();
    cmd.target_compid = 0;
    cmd.command = MAV_CMD_AVIATA_FINALIZE_DOCKING;
    cmd.param1 = docking_slot;
    float* missing_drone_ptrs[] = {&cmd.param2, &cmd.param3, &cmd.param4, &cmd.param5, &cmd.param6, &cmd.param7};
    uint8_t i = 0;
    for (; i < n_missing && i < 6; i++) {
        *missing_drone_ptrs[i] = missing_drones[i];
    }
    
    for (; i < 6; i++) {
        *missing_drone_ptrs[i] = NAN;
    }
    
    MavlinkPassthrough::Result result = mavlink_passthrough->send_command_long(cmd);
    if (result != MavlinkPassthrough::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to send docking MAVLink command." 
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 0;
    }
    return 1;
}

int PX4IO::set_mixer_configuration(uint8_t* missing_drones, uint8_t n_missing)
{
    if (!drone_settings.modify_px4_mixers) {
        return 1;
    }

    if (n_missing > 6) { // too many missing
        return 0;
    }

    MavlinkPassthrough::CommandLong cmd;
    cmd.target_sysid = sys->get_system_id();
    cmd.target_compid = 0;
    cmd.command = MAV_CMD_AVIATA_SET_CONFIGURATION;
    float* missing_drone_ptrs[] = {&cmd.param2, &cmd.param3, &cmd.param4, &cmd.param5, &cmd.param6, &cmd.param7};
    uint8_t i = 0;
    for (; i < n_missing && i < 6; i++) {
        *missing_drone_ptrs[i] = missing_drones[i];
    }
    
    for (; i < 6; i++) {
        *missing_drone_ptrs[i] = NAN;
    }
    
    MavlinkPassthrough::Result result = mavlink_passthrough->send_command_long(cmd);
    if (result != MavlinkPassthrough::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to send docking MAVLink command." 
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 0;
    }
    return 1;
}

int PX4IO::set_mixer_undocked()
{
    if (!drone_settings.modify_px4_mixers) {
        return 1;
    }

    MavlinkPassthrough::CommandLong cmd;
    cmd.target_sysid = sys->get_system_id();
    cmd.target_compid = 0;
    cmd.command = MAV_CMD_AVIATA_SET_STANDALONE;
    
    MavlinkPassthrough::Result result = mavlink_passthrough->send_command_long(cmd);
    if (result != MavlinkPassthrough::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to send undocking MAVLink command." 
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 0;
    }
    return 1;
}

int PX4IO::set_attitude_offset(float (&att_offset)[4]) {

    // This could send a MAVLINK Message or a MAVLINK Command
    // A MAVLINK Command is probably fine

    MavlinkPassthrough::CommandLong cmd;
    cmd.target_sysid = sys->get_system_id();
    cmd.target_compid = 0;

    cmd.command = MAV_CMD_AVIATA_SET_ATT_OFFSET; // TODO make new command ID

    cmd.param1 = att_offset[0];
    cmd.param2 = att_offset[1];
    cmd.param3 = att_offset[2];
    cmd.param4 = att_offset[3];
    
    MavlinkPassthrough::Result result = mavlink_passthrough->send_command_long(cmd);
    if (result != MavlinkPassthrough::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to send attitude offset MAVLink command." 
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 0;
    }
    return 1;
}

////////////////////////////////////////////
// example code below
////////////////////////////////////////////

void PX4IO::usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

void PX4IO::component_discovered(ComponentType component_type)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Discovered a component with type "
              << unsigned(component_type) << std::endl;
}

int PX4IO::takeoff_and_land_test(int argc, char** argv)
{
    //Mavsdk mavsdk;
    std::string connection_url;
    ConnectionResult connection_result;

    bool discovered_system = false;
    if (argc == 2) {
        connection_url = argv[1];
        connection_result = mav.add_any_connection(connection_url);
    } else {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Connection failed: " << connection_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    std::cout << "Waiting to discover system..." << std::endl;
    mav.subscribe_on_new_system([this, &discovered_system]() {
        const auto system = mav.systems().at(0);

        if (system->is_connected()) {
            std::cout << "Discovered system" << std::endl;
            discovered_system = true;
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a system after around 2
    // seconds.
    sleep_for(seconds(2));

    if (!discovered_system) {
        std::cout << ERROR_CONSOLE_TEXT << "No system found, exiting." << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 1;
    }

    auto system = mav.systems().at(0);

    MavlinkPassthrough mavlink_passthrough(system);
    MavlinkPassthrough::CommandLong cmd;

    // Register a callback so we get told when components (camera, gimbal) etc
    // are found.
    system->register_component_discovered_callback([this](ComponentType component_type){ component_discovered(component_type); });

    auto telemetry = std::make_shared<Telemetry>(system);
    auto action = std::make_shared<Action>(system);

    // We want to listen to the altitude of the drone at 1 Hz.
    const Telemetry::Result set_rate_result = telemetry->set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Setting rate failed:" << set_rate_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry->subscribe_position([](Telemetry::Position position) {
        std::cout << TELEMETRY_CONSOLE_TEXT // set to blue
                  << "Altitude: " << position.relative_altitude_m << " m"
                  << NORMAL_CONSOLE_TEXT // set to default color again
                  << std::endl;
    });

    // Check if vehicle is ready to arm
    while (telemetry->health_all_ok() != true) {
        std::cout << "Vehicle is getting ready to arm" << std::endl;
        sleep_for(seconds(1));
    }

    // Arm vehicle
    std::cout << "Arming..." << std::endl;
    const Action::Result arm_result = action->arm();

    if (arm_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Arming failed:" << arm_result << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 1;
    }

    // AVIATA Test - Finalize docking in position 0, with drone 1 missing
    cmd.target_sysid = system->get_system_id();
    cmd.target_compid = 0;
    cmd.command = MAV_CMD_AVIATA_FINALIZE_DOCKING;
    cmd.param1 = 0; // docking slot
    cmd.param2 = 1; // missing_drones[0]
    cmd.param3 = NAN;
    cmd.param4 = NAN;
    cmd.param5 = NAN;
    cmd.param6 = NAN;
    cmd.param7 = NAN;
    mavlink_passthrough.send_command_long(cmd);

    // Take off
    std::cout << "Taking off..." << std::endl;
    const Action::Result takeoff_result = action->takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Takeoff failed:" << takeoff_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Let it hover for a bit before landing again.
    sleep_for(seconds(10));

    // AVIATA Test - Update configuration to be drones 1 and 2 missing
    cmd.target_sysid = system->get_system_id();
    cmd.target_compid = 0;
    cmd.command = MAV_CMD_AVIATA_SET_CONFIGURATION;
    cmd.param2 = 1; // missing_drones[0]
    cmd.param3 = 2; // missing_drones[1]
    cmd.param4 = NAN;
    cmd.param5 = NAN;
    cmd.param6 = NAN;
    cmd.param7 = NAN;
    mavlink_passthrough.send_command_long(cmd);

    std::cout << "Landing..." << std::endl;
    const Action::Result land_result = action->land();
    if (land_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Land failed:" << land_result << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 1;
    }

    // Check if vehicle is still in air
    while (telemetry->in_air()) {
        std::cout << "Vehicle is landing..." << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "Landed!" << std::endl;

    // AVIATA Test - Set to undocked
    cmd.target_sysid = system->get_system_id();
    cmd.target_compid = 0;
    cmd.command = MAV_CMD_AVIATA_SET_STANDALONE;
    mavlink_passthrough.send_command_long(cmd);

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished..." << std::endl;

    return 0;
}
