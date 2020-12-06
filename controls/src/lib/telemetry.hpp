#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

#include <string>
#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

class DroneTelemetry
{
    public:
        mavsdk::Telemetry::Position dronePosition; //https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_position.html
        mavsdk::Telemetry::Battery droneBattery; //https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_quaternion.html
        mavsdk::Telemetry::Quaternion droneQuarternion; //https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_battery.html
        
        // MAVSDK
        std::string connection_url;
	    std::shared_ptr<mavsdk::System> system;
        mavsdk::Telemetry telem = mavsdk::Telemetry(system); // mavsdk Telemetry object
        
        DroneTelemetry();   
        void init_telem(); // initializes telemetry values
};

#endif