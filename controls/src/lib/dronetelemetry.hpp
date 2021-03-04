#ifndef DRONETELEMETRY_HPP
#define DRONETELEMETRY_HPP

#include <string>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "px4_io.hpp"

using namespace mavsdk;
class DroneTelemetry
{
public:
    //dronestatus fields
    // mavsdk::Telemetry::Position dronePosition; //https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_position.html
    float dronePosition[4];
    Telemetry::Battery droneBattery; //https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_quaternion.html
    Telemetry::Quaternion droneQuarternion; //https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_battery.html
    
    //attitude targets
    float q_target[4];
    float thrust_target;
    mavlink_attitude_target_t attitude_target;
    
    DroneTelemetry(PX4IO& px4_io);
    ~DroneTelemetry();
    void init_telem(); // initializes telemetry values

private:
    PX4IO& px4_io;
    std::shared_ptr<Telemetry> telemetry;
};

#endif
