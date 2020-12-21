/*
 * Based on the MAVSDK example here: https://github.com/mavlink/MAVSDK/tree/develop/examples/takeoff_land
 * This file will become a helper for interfacing with PX4.
 */

#include "px4_io.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <thread>

using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;

// mavsdk
Mavsdk mav; // name change to avoid namespace conflict
std::shared_ptr<Telemetry> telemetry;
std::shared_ptr<Action> action;
std::shared_ptr<MavlinkPassthrough> mavlink_passthrough;
std::string drone_id;

// mavlink
mavlink_attitude_target_t  att_struct;
uint8_t target_system;
uint8_t target_component;



std::shared_ptr<System> connect_to_pixhawk(std::string drone_ID, std::string connection_url)
//returns System pointer if connected, nullptr otherwise
//Discovering systems (the new way): https://mavsdk.mavlink.io/develop/en/cpp/api_changes.html
{
    drone_id = drone_ID;    
    ConnectionResult connection_result;
    connection_result = mav.add_any_connection(connection_url);
    // connect
    if (connection_result != ConnectionResult::Success) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to connect: " << connection_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return nullptr;
    }

    std::cout << "Waiting to discover system..." << std::endl;
    // discover system and return - method sourced from link above
    auto new_system_promise = std::promise<std::shared_ptr<System>>{};
    auto new_system_future = new_system_promise.get_future();
    mav.subscribe_on_new_system([&]() {
        std::cout << "Discovered system " << drone_id << std::endl;
        new_system_promise.set_value(mav.systems().at(0));
        mav.subscribe_on_new_system(nullptr);
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a system after around 2 seconds.
    sleep_for(seconds(2));

    auto sys = new_system_future.get();
    if (sys == nullptr) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " was not found, exiting." 
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return nullptr;
    }
    telemetry = std::make_shared<Telemetry>(sys);
    action = std::make_shared<Action>(sys);
    mavlink_passthrough = std::make_shared<MavlinkPassthrough>(sys);
    target_system = mavlink_passthrough->get_target_sysid();
    target_component = mavlink_passthrough->get_target_compid();

    return sys;
}

int arm_system()
//return 0 if fail, 1 if successs
{
    while (telemetry->health_all_ok() != true) {
        std::cout << drone_id << " is getting ready to arm." << std::endl;
        sleep_for(seconds(1));
    }

    // Arm vehicle
    std::cout << "Arming " << drone_id << "..." << std::endl;
    const Action::Result arm_result = action->arm();
    if (arm_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to arm: " << arm_result << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 0;
    }
    return 1;
}

int disarm_system()
{
    // Verify drone is on ground
    while (telemetry->in_air() != true) {
        std::cout << "Verifying " << drone_id << " is not in the air..." << std::endl;
        sleep_for(seconds(1));
    }

    // Disarm vehicle
    std::cout << "Disarming " << drone_id << "..." << std::endl;
    const Action::Result disarm_result = action->disarm();
    if (disarm_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to disarm: " << disarm_result << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 0;
    }
    return 1;
}

int takeoff_system()
{
    if (telemetry->armed() != true){
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

int land_system()
{
    std::cout << drone_id << " landing..." << std::endl;
    const Action::Result land_result = action->land();
    if (land_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to land:" << land_result 
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 0;
    }

    // Check if vehicle is still in air
    while (telemetry->in_air()) {
        std::cout << drone_id << " is landing..." << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << drone_id << " landed!" << std::endl;
    return 1;
}

void goto_gps_position(float lat, float lon)
// for DOCKED_LEADER (send attitude and thrust to followers)
{
    
}

void init_get_attitude_and_thrust(float q[4], float* thrust)
{
    mavlink_passthrough->subscribe_message_async(ATTITUDE_TARGET_ID, [&](const mavlink_message_t &attitude_target_message){
        mavlink_msg_attitude_target_decode(&attitude_target_message, &att_struct);
        q = att_struct.q;
        thrust = &att_struct.thrust;
        });

}

int set_attitude_and_thrust(float q[4], float* thrust)
{
    mavlink_set_attitude_target_t set_att_struct;
    std::copy(q, q + 4, set_att_struct.q);
    set_att_struct.thrust = *thrust;
    set_att_struct.body_roll_rate = att_struct.body_roll_rate;
    set_att_struct.body_pitch_rate = att_struct.body_pitch_rate;
    set_att_struct.body_yaw_rate = att_struct.body_yaw_rate;

    mavlink_message_t set_attitude_target_message;
    uint16_t encode_result; //encode function returns a uint16_t, not sure what it represents
    encode_result = mavlink_msg_set_attitude_target_encode(target_system, target_component, &set_attitude_target_message, &set_att_struct);


    MavlinkPassthrough::Result set_att_result = mavlink_passthrough->send_message(set_attitude_target_message);
    if ( set_att_result != MavlinkPassthrough::Result::Success){
        std::cout << ERROR_CONSOLE_TEXT << drone_id << " failed to set attitude and thrust." 
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 0;
    }
    return 1;    
}

////////////////////////////////////////////
// example code below
////////////////////////////////////////////

void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

void component_discovered(ComponentType component_type)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Discovered a component with type "
              << unsigned(component_type) << std::endl;
}

int takeoff_and_land_test(int argc, char** argv)
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
    mav.subscribe_on_new_system([&discovered_system]() {
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
    system->register_component_discovered_callback(component_discovered);

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
