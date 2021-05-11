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
        std::cout << "LandedState: " << landed << std::endl;
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
    Offboard::VelocityBodyYawspeed stay{};
    offboard.set_velocity_body(stay); // set initial setpoint

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
        land();
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
        Errors errs;
        bool is_tag_detected = image_analyzer.processImage(img, 0, tags, errs); //Detects apriltags and calculates errors
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
        log("Docking", "Errors: x_err: " + std::to_string(errs.x) + " y_err: " + std::to_string(errs.y) + " alt_err: " + std::to_string(errs.alt) + " rot_err: " + std::to_string(errs.yaw));
        offset_errors(errs, target_id); //Adjusts errors for stage 1 to stage 2 transition
        log("Docking", "Offset Errors: x_err: " + std::to_string(errs.x) + " y_err: " + std::to_string(errs.y) + " alt_err: " + std::to_string(errs.alt) + " rot_err: " + std::to_string(errs.yaw));
        Velocities velocities = pid.getVelocities(errs.x, errs.y, errs.alt, errs.yaw,0.4); //Gets velocities for errors
        log("Docking", "Velocities: east: " + std::to_string(velocities.x) + " north: " + std::to_string(velocities.y) + " down: " + std::to_string(velocities.alt) + " yaw: " + std::to_string(velocities.yaw));

        //Checks if drone within correct tolerance
        if (errs.x < STAGE_1_TOLERANCE && errs.x > -1.0 * STAGE_1_TOLERANCE && errs.y < STAGE_1_TOLERANCE && errs.y > -1.0 * STAGE_1_TOLERANCE &&
            errs.alt < STAGE_1_TOLERANCE && errs.alt > -1.0 * STAGE_1_TOLERANCE && errs.yaw < 2.0 && errs.yaw > -2.0)
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
        Offboard::VelocityBodyYawspeed change{};
        change.yawspeed_deg_s = velocities.yaw;
        change.forward_m_s = velocities.y;
        change.right_m_s = velocities.x;
        change.down_m_s = velocities.alt;
        offboard.set_velocity_body(change);

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
    PIDController pid(m_dt,true);
    Mat img;
    bool has_centered = false;

    int successful_frames = 0;

    while (true)
    {
        //Update image and process for errors
        img = camera.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, target_id);
        Errors errs;
        bool is_tag_detected = image_analyzer.processImage(img, target_id, tags, errs);
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
                is_tag_detected = image_analyzer.processImage(img, target_id, tags, errs);

                //Ascends until max height or until peripheral target detected
                while (!is_tag_detected && m_down * -1.0 - m_target_info.alt < MAX_HEIGHT_STAGE_2)
                {
                    log("Docking", "Ascending for peripheral target");
                    Offboard::VelocityBodyYawspeed change{};
                    change.down_m_s = -0.2f;
                    offboard.set_velocity_body(change);
                    img = camera.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, target_id);
                    is_tag_detected = image_analyzer.processImage(img, target_id, tags, errs);
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
                    is_tag_detected = image_analyzer.processImage(img, 0, tags, errs);
                }

                //Central target detected, re-attempt stage 1
                if (is_tag_detected)
                {
                    log("Docking", "Central target detected, re-attempting stage 1");
                    stage1(target_id);
                    img = camera.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, target_id);
                    is_tag_detected = image_analyzer.processImage(img, target_id, tags, errs);
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
            is_tag_detected = image_analyzer.processImage(img, target_id, tags, errs);
            log("Tags Detected", tags);
        }

        errs.alt -= 0.05; //Adjusts altitude error for desired location

        //Checks if within docking range
        if (errs.alt < STAGE_2_TOLERANCE * 4 && errs.alt > -4 * STAGE_2_TOLERANCE && /*errs.yaw < 2.0 && errs.yaw > -2.0 &&*/ errs.x < STAGE_2_TOLERANCE && errs.x > -1.0 * STAGE_2_TOLERANCE && errs.y < STAGE_2_TOLERANCE && errs.y > -1.0 * STAGE_2_TOLERANCE)
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
        bool is_centered = errs.x < STAGE_1_TOLERANCE && errs.x > -1.0 * STAGE_1_TOLERANCE && errs.y < STAGE_1_TOLERANCE && errs.y > -1.0 * STAGE_1_TOLERANCE;
        if (!is_centered && !has_centered) {
            errs.yaw -= 90;
            has_centered = true; // prevent rerotating away once we've done it once
        }

        log("Docking", "Errors: x_err: " + std::to_string(errs.x) + " y_err: " + std::to_string(errs.y) + " alt_err: " + std::to_string(errs.alt) + " rot_err: " + std::to_string(errs.yaw));

        Velocities velocities = pid.getVelocities(errs.x, errs.y, errs.alt, errs.yaw,0.4); //Gets velocities for errors
        log("Docking", "Velocities: east: " + std::to_string(velocities.x) + " north: " + std::to_string(velocities.y) + " down: " + std::to_string(velocities.alt));

        //Predicts where drone's minimum horizontal FOV after descending, and checks to make sure the tag will remain in frame
        //Uses smaller vertical FOV instead of more specific FOV to guard against possible rotation
        double safe_view=2*(errs.alt-velocities.alt*m_dt)*tan(to_radians(CAMERA_FOV_VERTICAL/2));
        if(abs(errs.x)>=safe_view||abs(errs.y)>=safe_view){
            velocities.alt=0;
        }

        //Updates drone velocities
        Offboard::VelocityNedYaw change{};
        change.yaw_deg = 0;//errs.yaw;
        change.north_m_s = velocities.y;
        change.east_m_s = velocities.x;
        change.down_m_s = velocities.alt;
        offboard.set_velocity_ned(change);
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
bool Drone::land()
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
    }

    const Action::Result land_result = action.land();
    if (land_result != Action::Result::Success)
    {
        log("Landing", "Land failed, error code on next line", true);
        std::cout << "Code: " << land_result << std::endl;
        return false; // should reattempt landing
    }

    // subscribe to telemetry updates to monitor
    const Telemetry::Result set_rate_result = telemetry.set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success) {
        log("Landing", "Telemetry Error", true);
    }
    telemetry.subscribe_position([](Telemetry::Position position) {
        log("Landing", "Current altitude: " + std::to_string(position.relative_altitude_m));
    });
    Telemetry::LandedState curr_state = telemetry.landed_state();
    log("Landing", "Current landed state: " + get_landed_state_string(curr_state));
    telemetry.subscribe_landed_state([this, &curr_state](Telemetry::LandedState landed_state) {
        if (curr_state != landed_state) {
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
    log("Landing", "Landed successfully " + std::to_string(telemetry.in_air()));
    Action::Result disarm_result = action.disarm();
    if (disarm_result != Action::Result::Success)
    {
        log("Landing", "Disarming failed", true);
        std::cout << "Code: " << disarm_result << std::endl;
        return false;;
    }
    log("Landing", "Disarmed successfully");
    return true;
}

//Utility method to adjust errors for stage 1
void Drone::offset_errors(Errors &errs, int id)
{
    float target_offset = id <= 3 ? abs(id - 3) * 45 : (11 - id) * 45;
    float x = errs.x + BOOM_LENGTH / 2 * cos(to_radians(target_offset - errs.yaw));
    float y = errs.y + BOOM_LENGTH / 2 * sin(to_radians(target_offset - errs.yaw));
    errs.alt -= ALTITUDE_DISP;
    errs.x = x;
    errs.y = y;
    float yaw=errs.yaw+90-((id-1)*45);
    errs.yaw=yaw;
}

/////////////// Testing functions

void Drone::test0()
{
}

void Drone::test1()
{
    log("Test 1", "Warming camera up...");
    for (int count = 0; count < 60; count++) {
        camera.update_current_image(0, 0, 5, 0, 0);
    }
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
    auto telemetry = Telemetry{m_system};

    const Telemetry::Result set_rate_result_p = telemetry.set_rate_position_velocity_ned(20);
    const Telemetry::Result set_rate_result_a = telemetry.set_rate_attitude(20);
    if (set_rate_result_a != Telemetry::Result::Success || set_rate_result_p != Telemetry::Result::Success)
    {
        std::cout << "Setting attitude rate possibly failed:" << set_rate_result_a << '\n';
        std::cout << "Setting position rate possibly failed:" << set_rate_result_p << '\n';
    }

    // Subscribe to getting periodic telemetry updates with lambda function
    telemetry.subscribe_position_velocity_ned([this](Telemetry::PositionVelocityNed p) {
        // set_position(p.position.north_m, p.position.east_m, p.position.down_m);
        if (p.position.down_m > 5.0) {
            log("TEST 1", "ALTITUDE TOO HIGH, MUST LAND. Altitude: " + std::to_string(p.position.down_m), true);
            this->land();
        }
    });
    telemetry.subscribe_attitude_euler([this](Telemetry::EulerAngle e) {
        set_yaw(e.yaw_deg);
    });

    PIDController pid(m_dt);

    Errors errs;
    int n_missed_frames = 0;
    while (time_span < 30 * 1000 /* 30 seconds */)
    {
        high_resolution_clock::time_point f1 = high_resolution_clock::now();
        img = camera.update_current_image(0, 0, 5, 0, 0); // this is blocking
        high_resolution_clock::time_point f2 = high_resolution_clock::now();
        duration<double, std::milli> c1 = (f2 - f1);

        high_resolution_clock::time_point f3 = high_resolution_clock::now();
        bool is_tag_detected = image_analyzer.processImage(img, 0, tags_detected, errs);
        high_resolution_clock::time_point f4 = high_resolution_clock::now();
        duration<double, std::milli> c2 = (f4 - f3);

        Offboard::VelocityBodyYawspeed change{};
        // change.down_m_s = -0.2;
        Velocities velocities; 
        if (is_tag_detected)
        {
            errs.alt -= 0.35;
            velocities = pid.getVelocities(errs.x, errs.y, errs.alt, errs.yaw, 1.5);
            log(tag, "Apriltag found! " + std::to_string(time_span) + " camera: " + std::to_string(c1.count()) + " detector: " + std::to_string(c2.count()) +
                         " errors: x=" + std::to_string(errs.x) + " y=" + std::to_string(errs.y) + " z=" + std::to_string(errs.alt) + " yaw=" + std::to_string(errs.yaw)
                         + " velocities: x=" + std::to_string(velocities.x) + " y=" + std::to_string(velocities.y) + " z=" + std::to_string(velocities.alt) + " yaw_speed=" + std::to_string(velocities.yaw));
            
            double safe_view = 2 * (errs.alt - velocities.alt * 0.1) * tan(to_radians(CAMERA_FOV_VERTICAL / 2));
            if (abs(errs.x) >= safe_view || abs(errs.y) >= safe_view) {
                velocities.alt = -0.1;
            }
            
            change.forward_m_s = velocities.y;
            change.right_m_s = velocities.x;
            change.down_m_s = velocities.alt;
            change.yawspeed_deg_s = velocities.yaw;
            n_missed_frames = 0;
        } else {
            log(tag, "Failed to find Apriltag, number missed frames: " + std::to_string(n_missed_frames)); 

            if (n_missed_frames > 10) {
                change.forward_m_s = 0.0;
                change.right_m_s = 0.0;
                change.down_m_s = 0.0;
                change.yawspeed_deg_s = 0.0;
            } else {
                change.forward_m_s = velocities.y * 0.9;
                change.right_m_s = velocities.x * 0.9;
                change.down_m_s = velocities.alt * 0.9;
                change.yawspeed_deg_s = 0.0;
            }
            n_missed_frames++;
        }

        offboard.set_velocity_body(change); 
        // cv::imwrite("test"+ std::to_string(time_span) + ".png", img); // any (debug)

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        duration<double, std::milli> d = (t2 - t1);
        time_span = d.count();
        count_frames++;
    }
    Offboard::VelocityNedYaw hold{};
    offboard.set_velocity_ned(hold);
    if (!img.empty()) cv::imwrite("test.png", img);
    log(tag, "Number of frames processed in " + std::to_string(time_span) + " ms: " + std::to_string(count_frames));
    log(tag, "Average FPS: " + std::to_string(count_frames / (time_span / 1000)));
    bool landed = false;
    do {
        landed = land();
    } while (landed == false);
}

void Drone::simulation_test_moving_target() {
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

    auto offboard = Offboard{m_system};
    auto telemetry = Telemetry{m_system}; // 2 and 3
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
    PIDController pid(m_dt);
    std::string tags = "";
    Mat img;

    int time_span = 0; // time per iteration

    int sign_switch = 0;
    int lat_sign = 1;
    int lon_sign = 1;
    while (true)
    {
        high_resolution_clock::time_point t1 = high_resolution_clock::now();

        sign_switch += time_span;
        if (sign_switch > 2000) {
            lat_sign = ((rand() % 2 == 0) ? 1 : -1);
            lon_sign = ((rand() % 2 == 0) ? 1 : -1);
            sign_switch = 0;
        }
        m_target_info.lat += .2 * time_span / 1000 * lat_sign;
        m_target_info.lon += .2 * time_span / 1000 * lon_sign;
        // camera.update_target(m_target_info);
        img = camera.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, 0);
        Errors errs;
        image_analyzer.processImage(img, 0, tags, errs); //Detects apriltags and calculates errors
        log("Tags Detected", tags);

        log("Docking", "Errors: x_err: " + std::to_string(errs.x) + " y_err: " + std::to_string(errs.y) + " alt_err: " + std::to_string(errs.alt) + " rot_err: " + std::to_string(errs.yaw));
        errs.alt -= 1;
        log("Docking", "Offset Errors: x_err: " + std::to_string(errs.x) + " y_err: " + std::to_string(errs.y) + " alt_err: " + std::to_string(errs.alt) + " rot_err: " + std::to_string(errs.yaw));
        Velocities velocities = pid.getVelocities(errs.x, errs.y, errs.alt, errs.yaw,0.4); //Gets velocities for errors
        log("Docking", "Velocities: east: " + std::to_string(velocities.x) + " north: " + std::to_string(velocities.y) + " down: " + std::to_string(velocities.alt));

        //Updates drone velocities with calculated values
        Offboard::VelocityNedYaw change{};
        change.yaw_deg = errs.yaw;
        change.north_m_s = velocities.y;
        change.east_m_s = velocities.x;
        change.down_m_s = velocities.alt;
        offboard.set_velocity_ned(change);

        //Calculates how long to wait to keep refresh rate constant
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        duration<double, std::milli> d = (t2 - t1);
        time_span = d.count();
        log("Docking", "Iteration Time: " + std::to_string(time_span));
    }
}

void Drone::test_telemetry() {
    auto telemetry = Telemetry{m_system}; // 2 and 3
    telemetry.set_rate_attitude(20);
    telemetry.subscribe_attitude_euler([this](Telemetry::EulerAngle e) {
        set_yaw(e.yaw_deg);
    });

    Mat img;
    std::string tag = "Test telem";
    std::string tags_detected = "";

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    // int time_span = 0;
    int count_frames = 0;
    Errors errs;
    while (true)
    {
        high_resolution_clock::time_point f1 = high_resolution_clock::now();
        img = camera.update_current_image(0, 0, 5, 0, 0); // this is blocking
        high_resolution_clock::time_point f2 = high_resolution_clock::now();
        duration<double, std::milli> c1 = (f2 - f1);

        high_resolution_clock::time_point f3 = high_resolution_clock::now();
        bool is_tag_detected = image_analyzer.processImage(img, 0, tags_detected, errs); //Detects apriltags and calculates errors
        high_resolution_clock::time_point f4 = high_resolution_clock::now();
        duration<double, std::milli> c2 = (f4 - f3);

        if (is_tag_detected)
        {
            log(tag, "Apriltag found! camera: " + std::to_string(c1.count()) + " detector: " + std::to_string(c2.count()) +
                         " errors: " + std::to_string(errs.x) + " " + std::to_string(errs.y) + " " + std::to_string(errs.alt) + " " + std::to_string(errs.yaw));
            // std::array<float, 3> velocities = pid.getVelocities(errs.x, errs.y, errs.alt, 0.2);
        } else {
            log(tag, "Failed to find Apriltag");
        }

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        duration<double, std::milli> d = (t2 - t1);
        // time_span = d.count();
        count_frames++;
    }
}