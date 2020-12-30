#include "dronetelemetry.hpp"

DroneTelemetry::DroneTelemetry(PX4IO& px4_io): px4_io(px4_io) {}

DroneTelemetry::~DroneTelemetry()
{
    // cancel telem subscriptions
    // px4_io.subscribe_position(nullptr);
    // px4_io.subscribe_attitude_quaternion(nullptr);
    // px4_io.subscribe_battery(nullptr);
}

void DroneTelemetry::init_telem()
{
    // TODO add these subscription functions to px4_io
    // px4_io.subscribe_position([&](mavsdk::Telemetry::Position position) {
    //     dronePosition = position;
    // });
    // px4_io.subscribe_attitude_quaternion([&](mavsdk::Telemetry::Quaternion quaternion){
    //     droneQuarternion = quaternion;
    // });
    // px4_io.subscribe_battery([&](mavsdk::Telemetry::Battery battery){
    //     droneBattery = battery;
    // });
}
