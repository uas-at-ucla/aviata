#ifndef DRONETELEMETRY_HPP
#define DRONETELEMETRY_HPP

#include <string>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "px4_io.hpp"

class DroneTelemetry
{
public:
    //dronestatus fields
    mavsdk::Telemetry::Position dronePosition; //https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_position.html
    mavsdk::Telemetry::Battery droneBattery; //https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_quaternion.html
    mavsdk::Telemetry::Quaternion droneQuarternion; //https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_battery.html
    
    DroneTelemetry(PX4IO& px4_io);
    ~DroneTelemetry();
    void init_telem(); // initializes telemetry values

private:
    PX4IO& px4_io;
};

#endif
