#include "telemetry.hpp"

DroneTelemetry::DroneTelemetry()
{
    system = connect_to_pixhawk(connection_url);
}

void DroneTelemetry::init_telem()
{
    telem.subscribe_position([&](mavsdk::Telemetry::Position position) {
        dronePosition = position;
    });
    telem.subscribe_attitude_quaternion([&](mavsdk::Telemetry::Quaternion quaternion){
        droneQuarternion = quaternion;
    });
    telem.subscribe_battery([&](mavsdk::Telemetry::Battery battery){
        droneBattery = battery;
    });
}