#include "dronetelemetry.hpp"

DroneTelemetry::DroneTelemetry(PX4IO& px4_io): px4_io(px4_io) {}

DroneTelemetry::~DroneTelemetry()
{
    // cancel telem subscriptions
    px4_io.telemetry_ptr()->subscribe_position(nullptr);
    px4_io.telemetry_ptr()->subscribe_attitude_quaternion(nullptr);
    px4_io.telemetry_ptr()->subscribe_battery(nullptr);
    px4_io.telemetry_ptr()->subscribe_attitude_euler(nullptr);
}

void DroneTelemetry::init_telem()
{    
    px4_io.telemetry_ptr()->subscribe_position([&](Telemetry::Position position) {
        dronePosition[0] = position.latitude_deg;
        dronePosition[1] = position.longitude_deg;
        dronePosition[2] = position.absolute_altitude_m;
        dronePosition[3] = position.relative_altitude_m;
    });
    px4_io.telemetry_ptr()->subscribe_attitude_quaternion([&](Telemetry::Quaternion quaternion) {
        droneQuarternion = quaternion;
    });
    px4_io.telemetry_ptr()->subscribe_battery([&](Telemetry::Battery battery){
        droneBattery = battery;
    });

    px4_io.telemetry_ptr()->subscribe_attitude_euler([&](Telemetry::EulerAngle euler_angle) {
        droneEulerAngle = euler_angle;
    });

    // most likely unused
    // px4_io.subscribe_attitude_target([this](const mavlink_attitude_target_t& attitude_target) {
    //     std::copy(std::begin(attitude_target.q), std::end(attitude_target.q), std::begin(q_target));
    //     thrust_target = attitude_target.thrust;
    //     this->attitude_target = attitude_target;
    // });
}
