#include "dronetelemetry.hpp"

DroneTelemetry::DroneTelemetry(PX4IO& px4_io): px4_io(px4_io) {}

DroneTelemetry::~DroneTelemetry()
{
    // cancel telem subscriptions
    // px4_io.subscribe_position(nullptr);
    // px4_io.subscribe_attitude_quaternion(nullptr);
    // px4_io.subscribe_battery(nullptr);
    // px4_io.unsubscribe_attitude_target();
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

    //drone always subscribing to its own target setpoint
    // px4_io.subscribe_attitude_target([](const mavlink_attitude_target_t& attitude_target) {
    //      std::copy(attitude_target.q, attitude_target.q + 4, q_target);
    //      thrust_target = attitude_target.thrust;
    // });
}
