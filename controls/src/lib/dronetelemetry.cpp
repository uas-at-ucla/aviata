#include "dronetelemetry.hpp"

DroneTelemetry::DroneTelemetry(PX4IO& px4_io): px4_io(px4_io) 
{
    px4_io.get_telemetry_ptr(telemetry);
}

DroneTelemetry::~DroneTelemetry()
{
    // cancel telem subscriptions
    telemetry->subscribe_position(nullptr);
    telemetry->subscribe_attitude_quaternion(nullptr);
    telemetry->subscribe_battery(nullptr);
    px4_io.unsubscribe_attitude_target();
}

void DroneTelemetry::init_telem()
{
    telemetry->subscribe_position([&](Telemetry::Position position) {
        dronePosition[0] = position.latitude_deg;
        dronePosition[1] = position.longitude_deg;
        dronePosition[2] = position.absolute_altitude_m;
        dronePosition[3] = position.relative_altitude_m;
    });
    telemetry->subscribe_attitude_quaternion([&](Telemetry::Quaternion quaternion){
        droneQuarternion = quaternion;
    });
    telemetry->subscribe_battery([&](Telemetry::Battery battery){
        droneBattery = battery;
    });

    //drone always subscribing to its own target setpoint
    px4_io.subscribe_attitude_target([this](const mavlink_attitude_target_t& attitude_target) {
        std::copy(std::begin(attitude_target.q), std::end(attitude_target.q), std::begin(q_target));
        thrust_target = attitude_target.thrust;
        this->attitude_target = attitude_target;
    });
}
