#ifndef PX4_TELEMETRY_HPP
#define PX4_TELEMETRY_HPP

#include <string>
#include "MAVSDK/src/core/mavsdk.h"
#include "MAVSDK/src/plugins/telemetry/include/plugins/telemetry/telemetry.h"
#include "px4_io.hpp"

using namespace mavsdk;

class PX4Telemetry
{
public:
    Telemetry::Position   position;  // https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_telemetry_1_1_position.html
    Telemetry::Battery    battery;   // https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_telemetry_1_1_battery.html
    Telemetry::Quaternion att_q;     // https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_telemetry_1_1_quaternion.html
    Telemetry::EulerAngle att_euler; // https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_telemetry_1_1_euler_angle.html

    PX4Telemetry(PX4IO& px4_io);
    ~PX4Telemetry();
    void init();

private:
    PX4IO& px4_io;

    template<typename T>
    void subscribe_variable(void (Telemetry::*subscribe_function)(std::function<void(T)>), T& var);
};

#endif
