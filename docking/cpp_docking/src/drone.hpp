#ifndef DRONE_H_
#define DRONE_H_

#define MAX_ATTEMPTS 3
#define MAX_HEIGHT 10
#define MAX_HEIGHT_STAGE_2 3
#define STAGE_1_TOLERANCE 0.10
#define STAGE_2_TOLERANCE 0.05

#if USE_RASPI_CAMERA == 1
    #include "raspi_camera.hpp"
    typedef RaspiCamera Camera;
#else
    #include "camera_simulator.hpp"
    typedef CameraSimulator Camera;
#endif

#include "image_analyzer.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
const float ALTITUDE_DISP = BOOM_LENGTH / 2 / tan(to_radians(CAMERA_FOV_VERTICAL / 2)) * 2;
class Drone
{

public:
    Drone(Target t);
    bool connect_gazebo();
    bool arm();
    bool takeoff(int takeoff_alt);
    void initiate_docking(int target_id);
    bool land();

    // testing functions
    void test0();
    void test1();
    void test2();
    void simulation_test_moving_target();

private:
    Mavsdk mavsdk;
    Camera camera;
    ImageAnalyzer image_analyzer;
    Target m_target_info;

    float m_north;
    float m_east;
    float m_down;
    float m_yaw;
    float m_dt; // loop cycle time, seconds

    std::shared_ptr<mavsdk::System> m_system; // pointer to mavsdk connection to drone

    void set_position(float n, float e, float d); // set position obtained from telemetry
    void set_yaw(float yaw);                      // set yaw angle obtained from telemetry

    bool stage1(int target_id);
    void stage2(int target_id);
    void offset_errors(Errors &errs, int target_id); // offset for stg 1->2 transition
    void safe_land();

    inline std::string get_landed_state_string(mavsdk::Telemetry::LandedState ls)
    {
        switch (ls)
        {
            case Telemetry::LandedState::OnGround: return "On ground";
            case Telemetry::LandedState::TakingOff: return "Taking off";
            case Telemetry::LandedState::Landing: return "Landing";
            case Telemetry::LandedState::InAir: return "In air";
            default: return "UNKNOWN";
        }
    }
};

#endif // DRONE_H_