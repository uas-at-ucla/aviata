#include "drone.hpp"
#include "util.hpp"
#include "pid_controller.hpp"

#include <iostream>
#include <future>
#include <string>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using namespace std::chrono;
using std::this_thread::sleep_for;

Drone::Drone(Target t)
    : camera(t), image_analyzer(), m_target_info(t), m_north(0), m_east(0), m_down(-5), m_yaw(0)
{
}

/**
 * Connect to Gazebo and run pre-flight checks
 * This function should be called before attempting to take any other action with the drone.
 * 
 * @return true if connection succeeded, false otherwise
 * */
bool Drone::connect_px4()
{
    std::string tag = "Connecting";

    std::string connection_url;
#if PLATFORM == RASPBERRY_PI
    connection_url = "serial:///dev/ttyAMA0:921600";
#else
    connection_url = "udp://:14540";
#endif

    ConnectionResult connection_result = mavsdk.add_any_connection(connection_url);

    if (connection_result != ConnectionResult::Success)
    {
        log(tag, "Connection failed, see next line", true);
        std::cout << connection_result << std::endl;
        return false;
    }

    log(tag, "Waiting to discover system");
    std::promise<void> discover_promise;
    auto discover_future = discover_promise.get_future();

    mavsdk.subscribe_on_new_system([&mavsdk = mavsdk, &discover_promise, &tag]()
                                   {
                                       const auto system = mavsdk.systems().at(0);

                                       if (system->is_connected())
                                       {
                                           log(tag, "Discovered system");
                                           discover_promise.set_value();
                                       }
                                   });

    discover_future.wait();

    // System got discovered.
    m_system = mavsdk.systems().at(0);
    telemetry = std::make_shared<Telemetry>(m_system);
    action = std::make_shared<Action>(m_system);
    offboard = std::make_shared<Offboard>(m_system);

    while (!telemetry->health_all_ok())
    {
        log(tag, "Waiting for system to be ready");
        // log(tag, "is_home_position_ok " + std::to_string(telemetry->health().is_home_position_ok));
        // log(tag, "is_global_position_ok " + std::to_string(telemetry->health().is_global_position_ok));
        // log(tag, "is_local_position_ok " + std::to_string(telemetry->health().is_local_position_ok));
        // log(tag, "is_level_calibration_ok " + std::to_string(telemetry->health().is_level_calibration_ok));
        // log(tag, "is_magnetometer_calibration_ok " + std::to_string(telemetry->health().is_magnetometer_calibration_ok));
        // log(tag, "is_accelerometer_calibration_ok " + std::to_string(telemetry->health().is_accelerometer_calibration_ok));
        // log(tag, "is_gyrometer_calibration_ok " + std::to_string(telemetry->health().is_gyrometer_calibration_ok));
        sleep_for(seconds(1));
    }

    log(tag, "System is ready");
    return true;
}

/**
 * Attempt to arm the drone.
 * 
 * @return true if successful, false otherwise.
 * */
bool Drone::arm()
{
    std::string tag = "Arming";

    // Arm the drone
    Action::Result arm_result = action->arm();
    if (arm_result != Action::Result::Success)
    {
        log(tag, "Arming failed", true);
        return false;
    }
    log(tag, "Armed");
    return true;
}

/**
 * Attempt to take off and start offboard mode. Drone must be armed first.
 * 
 * @return true if successful, false otherwise
 * */
bool Drone::takeoff(int takeoff_alt)
{
    std::string tag = "Takeoff";
    std::promise<void> in_air_promise;
    auto in_air_future = in_air_promise.get_future();

    // Attempt to take off
    action->set_takeoff_altitude(takeoff_alt);
    Action::Result takeoff_result = action->takeoff();
    if (takeoff_result != Action::Result::Success)
    {
        log(tag, "Takeoff failed, error code on next line", true);
        std::cout << "Code: " << takeoff_result << std::endl;
        return false;
    }

    log(tag, "Taking off");

    // Wait for the take off to be complete
    telemetry->subscribe_landed_state([&in_air_promise, this, &tag](Telemetry::LandedState landed)
                                      {
                                          std::cout << "LandedState: " << landed << std::endl;
                                          if (landed == Telemetry::LandedState::InAir)
                                          {
                                              telemetry->subscribe_landed_state(nullptr);
                                              in_air_promise.set_value();
                                              log(tag, "Take off successful");
                                          }
                                      });

    in_air_future.wait(); // wait for the asynchronous call to be finished (when in_air_promise resolves)

    // Start offboard mode
    Offboard::VelocityBodyYawspeed stay{};
    offboard->set_velocity_body(stay); // set initial setpoint

    Offboard::Result offboard_result = offboard->start();
    if (offboard_result != Offboard::Result::Success)
    {
        log(tag, "Offboard start failed, error code on next line", true);
        std::cout << "Code: " << offboard_result << std::endl;
        return false;
    }

    log(tag, "Offboard started");

    return true;
}

void Drone::set_position(float north, float east, float down)
{
    m_north = north;
    m_east = east;
    m_down = down;
}

void Drone::set_yaw(float yaw)
{
    m_yaw = yaw;
}

void Drone::warm_camera()
{
    log("Drone Pre-flight", "Warming camera up...");
    for (int count = 0; count < 60; count++)
    {
        camera.update_current_image(0, 0, 5, 0, 0);
    }
}

void Drone::initiate_docking(int target_id, bool full_docking)
{
    // set up telemetry updates; happens in background through callbacks
    const Telemetry::Result set_rate_result_p = telemetry->set_rate_position_velocity_ned(20);
    const Telemetry::Result set_rate_result_a = telemetry->set_rate_attitude(20);
    if (set_rate_result_a != Telemetry::Result::Success || set_rate_result_p != Telemetry::Result::Success)
    {
        std::cout << "Setting attitude rate possibly failed:" << set_rate_result_a << '\n';
        std::cout << "Setting position rate possibly failed:" << set_rate_result_p << '\n';
    }

    // Subscribe to getting periodic telemetry updates with lambda function
    telemetry->subscribe_position_velocity_ned([this](Telemetry::PositionVelocityNed p)
                                               { set_position(p.position.north_m, p.position.east_m, p.position.down_m); });
    telemetry->subscribe_attitude_euler([this](Telemetry::EulerAngle e)
                                        { set_yaw(e.yaw_deg); });

    //Disengage servo if not already
    #if PLATFORM == RASPBERRY_PI
        docking_detector.disengage_servo();
#endif

    // begin docking
    if (full_docking)
    { // do complete 2-stage docking
        bool stage1_success = dock(target_id, STAGE_1);
        if (stage1_success)
        {
            dock(target_id, STAGE_2);
        }
    }
    else
    {                             // do 1-stage docking (used for testing)
        dock(target_id, STAGE_2); // tag id = 0, central target
    }

    telemetry->subscribe_position_velocity_ned(nullptr);
    telemetry->subscribe_attitude_euler(nullptr);

    land();
}

/**
 * Attempts a particular stage of the docking process
 * Previously used to have 1 function per stage, but code got too long/duplicated
 * Easier to have 1 function with conditional separation of logic for small parts that are unique to a stage
 * 
 * @param target_id is the target to dock to
 * @param stage is which stage we're in
 * @return true if successful
 * */
bool Drone::dock(int target_id, int stage)
{
    std::string log_tag = "Stage " + std::to_string(stage);
    log(log_tag, "Docking Beginning");

    // Initialize all persistent variables
    PIDController pid;
    Mat img;
    int failed_frames = 0;
    int successful_frames = 0;
    int time_span = 0;
    int total_frames = 0;
    Velocities velocities;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    bool success = false;

    while (time_span < 60 * 1000)
    {
        Errors errs;
        Offboard::VelocityBodyYawspeed change{};
        std::string tags_detected = "";

        img = camera.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, 0);
        bool is_tag_detected = image_analyzer.processImage(img, stage == STAGE_1 ? 0 : target_id, tags_detected, errs); // central target has id = 0

        if (is_tag_detected)
        {
            failed_frames = 0;
            if (stage == STAGE_1)
                offset_errors(errs, target_id); // adjusts errors for stage 1 to stage 2 transition
            else
                errs.alt -= 0.35; //0.30;
            velocities = pid.getVelocities(errs.x, errs.y, errs.alt, errs.yaw, 1.5);
            log(log_tag, "Apriltag found! Timestamp: " + std::to_string(time_span) +
                             " errors: x=" + std::to_string(errs.x) + " y=" + std::to_string(errs.y) + " z=" + std::to_string(errs.alt) + " yaw=" + std::to_string(errs.yaw) + " velocities: x=" + std::to_string(velocities.x) + " y=" + std::to_string(velocities.y) + " z=" + std::to_string(velocities.alt) +
                             " yaw_speed=" + std::to_string(velocities.yaw) + " successful frames: " + std::to_string(successful_frames));

            // Check if we're centered enough to consider transitioning
            float tolerance = stage == STAGE_1 ? STAGE_1_TOLERANCE : STAGE_2_TOLERANCE;
            if (errs.x < tolerance && errs.x > -1.0 * tolerance &&
                errs.y < tolerance && errs.y > -1.0 * tolerance &&
                errs.alt < tolerance && errs.alt > -1.0 * tolerance) // &&
            // errs.yaw < 5.0 && errs.yaw > -5.0)
            {
                successful_frames++;
            }
            else
            {
                successful_frames = 0;
            }

            // Transition if we've been centered for over 1 second
            if (successful_frames > 10)
            {
                success = true;
                break; // TODO: calculate FPS to determine real 1 second duration
            }

            // Dynamically adjust z velocity to only descend when close enough to being centered
            // double safe_view = 2 * (errs.alt - velocities.alt * 0.1) * tan(to_radians(CAMERA_FOV_VERTICAL / 2));
            // if (abs(errs.x) >= safe_view || abs(errs.y) >= safe_view) {
            //     velocities.alt = -0.1;
            //     velocities.yaw = 0.0;
            // }

            // Dynamic adjustment wasn't working great, but the following seemed to work better
            // Prevent the drone from descending if the target is farther than 50% of the way to the edge of the frame from the center in x or y
            float frame_size_meters_x = 640 * errs.tag_pixel_ratio;
            float frame_size_meters_y = 480 * errs.tag_pixel_ratio;
            if (abs(errs.x) >= 0.5 * (frame_size_meters_x / 2) || abs(errs.y) >= 0.5 * (frame_size_meters_y / 2))
            {
                velocities.alt = -0.1;
                velocities.yaw = 0.0;
            }

            change.forward_m_s = velocities.y;
            change.right_m_s = velocities.x;
            change.down_m_s = velocities.alt;
            change.yawspeed_deg_s = velocities.yaw;
            failed_frames = 0;
        }
        else
        {
            log(log_tag, "Failed to find Apriltag, number missed frames: " + std::to_string(failed_frames));

            if (failed_frames > 10)
            {
                change.forward_m_s = 0.0;
                change.right_m_s = 0.0;
                change.down_m_s = 0.0;
                change.yawspeed_deg_s = 0.0;
            }
            else
            {
                change.forward_m_s = velocities.y * 0.8;
                change.right_m_s = velocities.x * 0.8;
                change.down_m_s = velocities.alt * 0.8;
                change.yawspeed_deg_s = 0.0;
            }
            failed_frames++;
        }

        if (m_down > 5.0)
        {
            log(log_tag, "Emergency land, too high! Altitude: " + std::to_string(m_down), true);
            land(); // emergency land, too high
            return false;
        }

        offboard->set_velocity_body(change);
        // cv::imwrite("test_stage" + std::to_string(stage) + "_"+ std::to_string(time_span) + ".png", img); // save frame as png for debug

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        duration<double, std::milli> d = (t2 - t1);
        time_span = d.count();
        total_frames++;
    }

    if (stage == STAGE_1)
    {
        Offboard::VelocityBodyYawspeed hold{};
        offboard->set_velocity_body(hold);
    }
    else
    {
        // attempt to engage docking mechanism
        // Todo (spin servo)

        // land slowly for now instead of docking mechanism
        Offboard::VelocityBodyYawspeed hold{};
        hold.down_m_s = 0.2;
        offboard->set_velocity_body(hold);

#if PLATFORM == RASPBERRY_PI
        while (true)
        {
            if (docking_detector.is_docked())
            {
                log(log_tag, "Successfully docked!");
                for(int i = 0; i < SERVO_ATTEMPTS; i++){
                    if(docking_detector.engage_servo()){
                        break;
                        sleep_for(milliseconds(300));
                    }
                }
                break;
            }
            else
            {
                sleep_for(milliseconds(500));
            }
        }
#endif
    }

    log(log_tag, "Number of frames processed in " + std::to_string(time_span) + " ms: " + std::to_string(total_frames));
    log(log_tag, "Average FPS: " + std::to_string(total_frames / (time_span / 1000)));
    return success;
}

// Lands and disarms drone
bool Drone::land()
{
    log("Landing", "Landing");
    Offboard::Result offboard_result = offboard->stop();
    if (offboard_result != Offboard::Result::Success)
    {
        log("Landing", "Offboard stop failed, error code on next line", true);
        std::cout << "Code: " << offboard_result << std::endl;
    }

    Action::Result land_result = action->land();
    while (land_result != Action::Result::Success)
    {
        log("Landing", "Land failed, error code on next line", true);
        std::cout << "Code: " << land_result << std::endl;
        land_result = action->land();
    }

    // subscribe to telemetry updates to monitor landing progress
    const Telemetry::Result set_rate_result = telemetry->set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success)
    {
        log("Landing", "Telemetry Error", true);
    }
    telemetry->subscribe_position([](Telemetry::Position position)
                                  { log("Landing", "Current altitude: " + std::to_string(position.relative_altitude_m)); });
    Telemetry::LandedState curr_state = telemetry->landed_state();
    log("Landing", "Current landed state: " + get_landed_state_string(curr_state));
    telemetry->subscribe_landed_state([this, &curr_state](Telemetry::LandedState landed_state)
                                      {
                                          if (curr_state != landed_state)
                                          {
                                              curr_state = landed_state;
                                              log("Landing", "Current landed state: " + get_landed_state_string(curr_state));
                                          }
                                      });

    // wait for landing to be complete
    while (curr_state != Telemetry::LandedState::OnGround)
    {
        sleep_for(seconds(1));
    }

    // cleanup and disarm
    log("Landing", "Landed successfully " + std::to_string(telemetry->in_air()));
    Action::Result disarm_result = action->disarm();
    if (disarm_result != Action::Result::Success)
    {
        log("Landing", "Disarming failed", true);
        std::cout << "Code: " << disarm_result << std::endl;
        return false;
        ;
    }
    log("Landing", "Disarmed successfully");
    return true;
}

// Utility method to adjust errors for stage 1
void Drone::offset_errors(Errors &errs, int id)
{
    float target_offset = id <= 3 ? abs(id - 3) * 45 : (11 - id) * 45;
    float x = errs.x + BOOM_LENGTH / 2 * cos(to_radians(target_offset - errs.yaw));
    float y = errs.y + BOOM_LENGTH / 2 * sin(to_radians(target_offset - errs.yaw));
    errs.alt -= ALTITUDE_DISP;
    errs.x = x;
    errs.y = y;
    float yaw = errs.yaw + 90 - ((id - 1) * 45);
    errs.yaw = yaw;
}