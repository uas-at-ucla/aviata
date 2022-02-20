#include "px4_telemetry.hpp"

PX4Telemetry::PX4Telemetry(PX4IO& px4_io): px4_io(px4_io) {}

PX4Telemetry::~PX4Telemetry()
{
    // cancel telem subscriptions
    px4_io.unsubscribe_telemetry(&Telemetry::subscribe_position);
    px4_io.unsubscribe_telemetry(&Telemetry::subscribe_battery);
    px4_io.unsubscribe_telemetry(&Telemetry::subscribe_attitude_quaternion);
    px4_io.unsubscribe_telemetry(&Telemetry::subscribe_attitude_euler);
}

template<typename T>
void PX4Telemetry::subscribe_variable(void (Telemetry::*subscribe_function)(std::function<void(T)>), T& var) {
    px4_io.subscribe_telemetry(subscribe_function, std::function([&var](T val) {
        var = val;
    }));
}

void PX4Telemetry::init()
{
    subscribe_variable(&Telemetry::subscribe_position,            this->position);
    subscribe_variable(&Telemetry::subscribe_battery,             this->battery);
    subscribe_variable(&Telemetry::subscribe_attitude_quaternion, this->att_q);
    subscribe_variable(&Telemetry::subscribe_attitude_euler,      this->att_euler);
    subscribe_variable(&Telemetry::subscribe_velocity_ned,        this->velocity);
}
