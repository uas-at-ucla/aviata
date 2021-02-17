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
    : camera(t), image_analyzer(), m_target_info(t), m_north(0), m_east(0), m_down(-5), m_yaw(0), m_dt(0.05)
{
}

/**
 * Connect to Gazebo and run pre-flight checks
 * This function should be called before attempting to take any other action with the drone.
 * 
 * @return true if connection succeeded, false otherwise
 * */
bool Drone::connect_gazebo()
{
    std::string tag = "Connecting";

    std::string connection_url;
    #if USE_RASPI_CAMERA == 1
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

    mavsdk.subscribe_on_new_system([&mavsdk = mavsdk, &discover_promise, &tag]() {
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
    auto telemetry = Telemetry{m_system};

    while (!telemetry.health_all_ok())
    {
        log(tag, "Waiting for system to be ready");
        // log(tag, "is_home_position_ok " + std::to_string(telemetry.health().is_home_position_ok));
        // log(tag, "is_global_position_ok " + std::to_string(telemetry.health().is_global_position_ok));
        // log(tag, "is_local_position_ok " + std::to_string(telemetry.health().is_local_position_ok));
        // log(tag, "is_level_calibration_ok " + std::to_string(telemetry.health().is_level_calibration_ok));
        // log(tag, "is_magnetometer_calibration_ok " + std::to_string(telemetry.health().is_magnetometer_calibration_ok));
        // log(tag, "is_accelerometer_calibration_ok " + std::to_string(telemetry.health().is_accelerometer_calibration_ok));
        // log(tag, "is_gyrometer_calibration_ok " + std::to_string(telemetry.health().is_gyrometer_calibration_ok));
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
    auto action = Action{m_system};
    Action::Result arm_result = action.arm();
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
    auto action = Action{m_system};
    action.set_takeoff_altitude(takeoff_alt);
    Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success)
    {
        log(tag, "Takeoff failed, error code on next line", true);
        std::cout << "Code: " << takeoff_result << std::endl;
        return false;
    }

    log(tag, "Taking off");

    // Wait for the take off to be complete
    auto telemetry = Telemetry{m_system};
    telemetry.subscribe_landed_state([&in_air_promise, &telemetry, &tag](Telemetry::LandedState landed) {
        if (landed == Telemetry::LandedState::InAir)
        {
            telemetry.subscribe_landed_state(nullptr);
            in_air_promise.set_value();
            log(tag, "Take off successful");
        }
    });

    in_air_future.wait(); // wait for the asynchronous call to be finished (when in_air_promise resolves)

    // Start offboard mode
    auto offboard = Offboard{m_system};
    Offboard::VelocityNedYaw stay{};
    offboard.set_velocity_ned(stay); // set initial setpoint

    Offboard::Result offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success)
    {
        log(tag, "Offboard start failed, error code on next line", true);
        std::cout << "Code: " << offboard_result << std::endl;
        return false;
    }

    log(tag, "Offboard started");

    return true;
}

//Starts docking process
void Drone::initiate_docking(int target_id)
{
    // set up telemetry updates; happens in background through callbacks
    auto telemetry = Telemetry{m_system}; // NOTE: telemetry updates are only received as long as the telemetry pointer is alive
    const Telemetry::Result set_rate_result_p = telemetry.set_rate_position_velocity_ned(40);
    const Telemetry::Result set_rate_result_a = telemetry.set_rate_attitude(40);
    if (set_rate_result_a != Telemetry::Result::Success || set_rate_result_p != Telemetry::Result::Success)
    {
        std::cout << "Setting attitude rate possibly failed:" << set_rate_result_a << '\n';
        std::cout << "Setting position rate possibly failed:" << set_rate_result_p << '\n';
    }

    // Subscribe to getting periodic telemetry updates with lambda function
    telemetry.subscribe_position_velocity_ned([this](Telemetry::PositionVelocityNed p) {
        set_position(p.position.north_m, p.position.east_m, p.position.down_m);
    });
    telemetry.subscribe_attitude_euler([this](Telemetry::EulerAngle e) {
        set_yaw(e.yaw_deg);
    });

    // begin docking
    bool stage1_outcome = stage1(target_id);
    if (stage1_outcome)
    {
        log("Docking", "Stage 1 Success");
        stage2(target_id);
    }
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

/*
 * Attempts stage 1 of docking process
 * @return true if successful
 * */
bool Drone::stage1(int target_id)
{
    log("Stage 1", "Docking Beginning");

    auto offboard = Offboard{m_system};
    PIDController pid(m_dt);
    std::string tags = "";
    Mat img;

    int failed_frames = 0;
    int successful_frames = 0;

    while (true)
    {
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        img = camera.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, 0);
        std::array<float, 4> errs = {0, 0, 0, 0};
        bool is_tag_detected = image_analyzer.processImage(img, 0, m_yaw, tags, errs); //Detects apriltags and calculates errors
        log("Tags Detected", tags);

        if (!is_tag_detected) //Central target not found
        {
            log("No Tag Detected, Failed Frames", std::to_string(failed_frames) + "");
            if (m_down * -1.0 - m_target_info.alt > MAX_HEIGHT) //Above maximum height, docking fails
            {
                log("Docking", "Error: above maximum height", true);
                return false;
            }
            if (failed_frames > 1 / m_dt) //Ascends to try to find central target
            {
                log("Docking", "Ascending");
                Offboard::VelocityBodyYawspeed change{};
                change.down_m_s = -0.2f;
                offboard.set_velocity_body(change);
            }
            sleep_for(milliseconds((int)(1000 * m_dt)));
            continue;
        }

        failed_frames = 0;
        log("Docking", "Errors: x_err: " + std::to_string(errs[0]) + " y_err: " + std::to_string(errs[1]) + " alt_err: " + std::to_string(errs[2]) + " rot_err: " + std::to_string(errs[3]));
        offset_errors(errs, target_id); //Adjusts errors for stage 1 to stage 2 transition
        log("Docking", "Offset Errors: x_err: " + std::to_string(errs[0]) + " y_err: " + std::to_string(errs[1]) + " alt_err: " + std::to_string(errs[2]) + " rot_err: " + std::to_string(errs[3]));
        std::array<float, 3> velocities = pid.getVelocities(errs[0], errs[1], errs[2], 0.4); //Gets velocities for errors
        log("Docking", "Velocities: east: " + std::to_string(velocities[0]) + " north: " + std::to_string(velocities[1]) + " down: " + std::to_string(velocities[2]));

        //Checks if drone within correct tolerance
        if (errs[0] < STAGE_1_TOLERANCE && errs[0] > -1.0 * STAGE_1_TOLERANCE && errs[1] < STAGE_1_TOLERANCE && errs[1] > -1.0 * STAGE_1_TOLERANCE &&
            errs[2] < STAGE_1_TOLERANCE && errs[2] > -1.0 * STAGE_1_TOLERANCE && errs[3] < 2.0 && errs[3] > -2.0)
        {
            successful_frames++;
        }
        else
        {
            successful_frames = 0;
        }

        if (successful_frames >= 1 / m_dt)
            break; //Stage 1 succesful

        //Updates drone velocities with calculated values
        Offboard::VelocityNedYaw change{};
        change.yaw_deg = errs[3];
        change.north_m_s = velocities[1];
        change.east_m_s = velocities[0];
        change.down_m_s = velocities[2];
        offboard.set_velocity_ned(change);

        //Calculates how long to wait to keep refresh rate constant
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        duration<double, std::milli> d = (t2 - t1);
        int time_span = d.count();
        log("Docking", "Iteration Time: " + std::to_string(time_span));
        if (time_span < m_dt * 1000)
        {
            sleep_for(milliseconds((int)(m_dt * 1000 - time_span)));
        }
    }

    return true;
}

/**
 * Attempts stage 2 of docking
 * Will execute completely if successful, drone will safely land if failure
 */
void Drone::stage2(int target_id)
{
    log("Stage 2", "Docking Beginning");
    auto offboard = Offboard{m_system};
    std::string tags = "";
    PIDController pid(m_dt);
    Mat img;

    int successful_frames = 0;

    while (true)
    {
        //Update image and process for errors
        img = camera.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, target_id);
        std::array<float, 4> errs = {0, 0, 0, 0};
        bool is_tag_detected = image_analyzer.processImage(img, target_id, m_yaw, tags, errs);
        log("Tags Detected", tags);
        int checked_frames = 0;
        int docking_attempts = 0;

        while (!is_tag_detected) //Target tag not detected
        {
            checked_frames++;
            log("No Tag Detected", "Checked Frames: " + std::to_string(checked_frames) + "");

            Offboard::VelocityBodyYawspeed change{}; //Ascends to try to find peripheral target
            change.down_m_s = -0.1f;
            offboard.set_velocity_body(change);

            if (checked_frames > 1 / m_dt) //If not detected for one second,
            {
                docking_attempts++;
                if (docking_attempts > MAX_ATTEMPTS) //Maximum attempts exceeded, docking failure
                {
                    log("DOCKING FAILED", "Maximum Attempts Exceeded", true);
                    safe_land();

                    return;
                }
                img = camera.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, target_id);
                is_tag_detected = image_analyzer.processImage(img, target_id, m_yaw, tags, errs);

                //Ascends until max height or until peripheral target detected
                while (!is_tag_detected && m_down * -1.0 - m_target_info.alt < MAX_HEIGHT_STAGE_2)
                {
                    log("Docking", "Ascending for peripheral target");
                    Offboard::VelocityBodyYawspeed change{};
                    change.down_m_s = -0.2f;
                    offboard.set_velocity_body(change);
                    img = camera.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, target_id);
                    is_tag_detected = image_analyzer.processImage(img, target_id, m_yaw, tags, errs);
                }
                if (is_tag_detected) //Peripheral target detected, continues with docking normally
                    break;

                log("Docking", "Peripheral target not detected, ascending for central target");
                //Peripheral target not detected, ascend until max height reached or central target detected
                while (!is_tag_detected && m_down * -1.0 - m_target_info.alt < MAX_HEIGHT)
                {
                    Offboard::VelocityBodyYawspeed change{};
                    change.down_m_s = -0.2f;
                    offboard.set_velocity_body(change);
                    img = camera.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, 0);
                    is_tag_detected = image_analyzer.processImage(img, 0, m_yaw, tags, errs);
                }

                //Central target detected, re-attempt stage 1
                if (is_tag_detected)
                {
                    log("Docking", "Central target detected, re-attempting stage 1");
                    stage1(target_id);
                    img = camera.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, target_id);
                    is_tag_detected = image_analyzer.processImage(img, target_id, m_yaw, tags, errs);
                    checked_frames = 0;
                }
                else //Maximum height exceeded, docking failure
                {
                    log("DOCKING FAILED", "Maximum Height Exceeded", true);
                    safe_land();
                    return;
                }
            }
            sleep_for(milliseconds((int)m_dt * 1000));
            img = camera.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, target_id);
            is_tag_detected = image_analyzer.processImage(img, target_id, m_yaw, tags, errs);
            log("Tags Detected", tags);
        }

        errs[2] -= 0.05; //Adjusts altitude error for desired location

        //Checks if within docking range
        if (errs[2] < STAGE_2_TOLERANCE * 2 && errs[2] > -1.0 * STAGE_2_TOLERANCE && errs[3] < 2.0 && errs[3] > -2.0 && errs[0] < STAGE_2_TOLERANCE && errs[0] > -1.0 * STAGE_2_TOLERANCE && errs[1] < STAGE_2_TOLERANCE && errs[1] > -1.0 * STAGE_2_TOLERANCE)
        {
            successful_frames++;
        }
        else
        {
            successful_frames = 0;
        }
        if (successful_frames > 1 / m_dt) //Docking success, within range for 1 second
            break;

        //Adjusts errors to prevent rotating out of frame
        if (errs[0] < STAGE_1_TOLERANCE && errs[0] > -1.0 * STAGE_1_TOLERANCE && errs[1] < STAGE_1_TOLERANCE && errs[1] > -1.0 * STAGE_1_TOLERANCE)
        {
            errs[3] *= tanh(OVERSHOOT_CONSTANT * errs[2] * errs[2]);
        }
        else
        {
            errs[3] -= 90;
            errs[3] *= tanh(OVERSHOOT_CONSTANT * errs[2] * errs[2]);
        }

        log("Docking", "Errors: x_err: " + std::to_string(errs[0]) + " y_err: " + std::to_string(errs[1]) + " alt_err: " + std::to_string(errs[2]) + " rot_err: " + std::to_string(errs[3]));

        //Adjusts errors to decrease overshooting
        errs[0] *= tanh(OVERSHOOT_CONSTANT * errs[2] * errs[2]);
        errs[1] *= tanh(OVERSHOOT_CONSTANT * errs[2] * errs[2]);
        errs[2] *= tanh(OVERSHOOT_CONSTANT * errs[2] * errs[2]);

        std::array<float, 3> velocities = pid.getVelocities(errs[0], errs[1], errs[2], 0.1); //Gets velocities for errors
        log("Docking", "Velocities: east: " + std::to_string(velocities[0]) + " north: " + std::to_string(velocities[1]) + " down: " + std::to_string(velocities[2]));

        //Updates drone velocities
        Offboard::VelocityNedYaw change{};
        change.yaw_deg = errs[3];
        change.north_m_s = velocities[1];
        change.east_m_s = velocities[0];
        change.down_m_s = velocities[2];
        offboard.set_velocity_ned(change);

        sleep_for(milliseconds((int)m_dt * 1000));
    }
}

//Method to move drone away from frame by ascending and moving to the north
void Drone::safe_land()
{
    log("Landing", "Safe landing begun");
    auto offboard = Offboard{m_system};
    for (int i = 0; i < 5 / m_dt; i++)
    {
        Offboard::VelocityNedYaw change{};
        change.north_m_s = 0.4f;
        change.down_m_s = -0.4f;
        offboard.set_velocity_ned(change);
    }
    land();
}

//Lands and disarms drone
void Drone::land()
{
    log("Landing", "Landing");
    auto telemetry = Telemetry{m_system};
    auto action = Action{m_system};
    auto offboard = Offboard{m_system};
    Offboard::Result offboard_result = offboard.stop();
    if (offboard_result != Offboard::Result::Success)
    {
        log("Landing", "Offboard stop failed, error code on next line", true);
        std::cout << "Code: " << offboard_result << std::endl;
        return;
    }

    const Action::Result land_result = action.land();
    if (land_result != Action::Result::Success)
    {
        log("Landing", "Land failed, error code on next line", true);
        std::cout << "Code: " << land_result << std::endl;
        return;
    }

    const Telemetry::Result set_rate_result = telemetry.set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success)
    {
        log("Landing", "Telemetry Error", true);
        return;
    }
    telemetry.subscribe_position([](Telemetry::Position position) {
        log("Landing", "Current altitude: " + std::to_string(position.relative_altitude_m));
    });

    while (telemetry.in_air())
    {
        sleep_for(seconds(1));
    }

    log("Landing", "Landed successfully " + std::to_string(telemetry.in_air()));
    Action::Result disarm_result = action.disarm();
    if (disarm_result != Action::Result::Success)
    {
        log("Landing", "Disarming failed", true);
        std::cout << "Code: " << disarm_result << std::endl;
        return;
    }
    log("Landing", "Disarmed successfully");
}

//Utility method to adjust errors for stage 1
void Drone::offset_errors(std::array<float, 4> &errs, int id)
{
    float target_offset = id <= 3 ? abs(id - 3) * 45 : (11 - id) * 45;
    float x = errs[0] + BOOM_LENGTH / 2 * cos(to_radians(target_offset - errs[3]));
    float y = errs[1] + BOOM_LENGTH / 2 * sin(to_radians(target_offset - errs[3]));
    errs[2] -= ALTITUDE_DISP;
    errs[0] = x;
    errs[1] = y;
}

/////////////// Testing functions

void Drone::test0()
{
    bool arm_code = arm();
    if (!arm_code)
    {
        log("Test 0", "Failed to arm");
        return;
    }
    bool takeoff_code = takeoff(2);
    if (!takeoff_code)
    {
        log("Test 0", "Failed to take off");
        return;
    }

    log("Test 0", "Holding...");

    int time_span = 0;
    auto offboard = Offboard{m_system};
    high_resolution_clock::time_point start = high_resolution_clock::now();
    while (time_span < 1000 * 5)
    {
        Offboard::VelocityNedYaw stay{};
        offboard.set_velocity_ned(stay);
        high_resolution_clock::time_point now = high_resolution_clock::now();
        duration<double, std::milli> d = (now - start);
        time_span = d.count();
    }
    log("Test 0", "Done holding");
    land();
}

void Drone::test1()
{
    bool arm_code = arm();
    if (!arm_code)
    {
        log("Test 1", "Failed to arm");
        return;
    }
    bool takeoff_code = takeoff(3);
    if (!takeoff_code)
    {
        log("Test 1", "Failed to take off");
        return;
    }

    log("Test 1", "Holding...");

    Mat img;
    std::string tag = "Test 1";
    std::string tags_detected = "";

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    int time_span = 0;
    int count_frames = 0;

    auto offboard = Offboard{m_system};
    auto telemetry = Telemetry{m_system}; // 2 and 3
    telemetry.subscribe_attitude_euler([this](Telemetry::EulerAngle e) {
        set_yaw(e.yaw_deg);
    });

    PIDController pid(m_dt);

    std::array<float, 4> errs = {0, 0, 0, 0};
    while (time_span < 10 * 1000 /* 10 seconds */)
    {
        high_resolution_clock::time_point f1 = high_resolution_clock::now();
        img = camera.update_current_image(0, 0, 5, 0, 0); // this is blocking
        high_resolution_clock::time_point f2 = high_resolution_clock::now();
        duration<double, std::milli> c1 = (f2 - f1);

        high_resolution_clock::time_point f3 = high_resolution_clock::now();
        bool is_tag_detected = image_analyzer.processImage(img, 0, m_yaw, tags_detected, errs); //Detects apriltags and calculates errors
        high_resolution_clock::time_point f4 = high_resolution_clock::now();
        duration<double, std::milli> c2 = (f4 - f3);

        Offboard::VelocityNedYaw change{};
        change.down_m_s = -0.2;
        if (is_tag_detected)
        {
            log(tag, "Apriltag found! camera: " + std::to_string(c1.count()) + " detector: " + std::to_string(c2.count()) +
                         " errors: " + std::to_string(errs[0]) + " " + std::to_string(errs[1]) + " " + std::to_string(errs[2]) + " " + std::to_string(errs[3]));
            std::array<float, 3> velocities = pid.getVelocities(errs[0], errs[1], errs[2], 0.1); // 3
            change.north_m_s = velocities[1]; // 3
            change.east_m_s = velocities[0]; // 3
            change.yaw_deg = errs[3]; // 2
        } else {
            log(tag, "Failed to find Apriltag");
            change.yaw_deg = m_yaw; // 2
            change.north_m_s = 0.0; // 3
            change.east_m_s = 0.0; // 3
        }

        offboard.set_velocity_ned(change);
        // cv::imwrite("test"+ std::to_string(time_span) + ".png", img); // any (debug)

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        duration<double, std::milli> d = (t2 - t1);
        time_span = d.count();
        count_frames++;
    }
    cv::imwrite("test.png", img);
    log(tag, "Number of frames processed in " + std::to_string(time_span) + " ms: " + std::to_string(count_frames));
    log(tag, "Average FPS: " + std::to_string(count_frames / (time_span / 1000)));

    log("Test 1", "Done holding");
    land();
}