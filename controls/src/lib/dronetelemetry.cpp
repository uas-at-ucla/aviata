#include "dronetelemetry.hpp"

DroneTelemetry::DroneTelemetry(std::shared_ptr<mavsdk::System> system)
{
    telem = new mavsdk::Telemetry(system);
}

DroneTelemetry::~DroneTelemetry()
{
    // cancel telem subscriptions
    telem->subscribe_position(nullptr);
    telem->subscribe_attitude_quaternion(nullptr);
    telem->subscribe_battery(nullptr);
    delete telem;
}

void DroneTelemetry::init_telem()
{
    telem->subscribe_position([&](mavsdk::Telemetry::Position position) {
        dronePosition = position;
    });
    telem->subscribe_attitude_quaternion([&](mavsdk::Telemetry::Quaternion quaternion){
        droneQuarternion = quaternion;
    });
    telem->subscribe_battery([&](mavsdk::Telemetry::Battery battery){
        droneBattery = battery;
    });
}