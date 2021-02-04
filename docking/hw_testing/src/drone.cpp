#include "drone.hpp"
#include "util.hpp"
#include "pid_controller.hpp"

#include <iostream>
#include <future>
#include <string>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using namespace std::chrono;
using std::this_thread::sleep_for;

Drone::Drone(Target t)
    : raspi_camera(), image_analyzer(), m_north(0), m_east(0), m_down(-5), m_yaw(0), m_target_info(t), m_dt(0.05)
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
    std::string connection_url = "serial:///dev/ttyAMA0:921600"; // change as needed, or add command-line parsing
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
        sleep_for(seconds(1));
    }

    log(tag, "System is ready");
    return true;
}

bool Drone::arm() {
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
 * Attempt to take off.
 * 
 * @return true if successful, false otherwise
 * */
bool Drone::takeoff()
{

    // Attempt to take off
    // std::string tag = "Takeoff";
    // std::promise<void> in_air_promise;
    // auto in_air_future = in_air_promise.get_future();
    // auto action = Action{m_system};
    // action.set_takeoff_altitude(4);
    // Action::Result takeoff_result = action.takeoff();
    // if (takeoff_result != Action::Result::Success)
    // {
    //     log(tag, "Takeoff failed, error code on next line", true);
    //     std::cout << "Code: " << takeoff_result << std::endl;
    //     return false;
    // }

    // log(tag, "Taking off");

    // // Wait for the take off to be complete
    // auto telemetry = Telemetry{m_system};
    // telemetry.subscribe_landed_state([&in_air_promise, &telemetry, &tag](Telemetry::LandedState landed) {
    //     if (landed == Telemetry::LandedState::InAir)
    //     {
    //         telemetry.subscribe_landed_state(nullptr);
    //         in_air_promise.set_value();
    //         log(tag, "Take off successful");
    //     }
    // });

    // in_air_future.wait(); // wait for the asynchronous call to be finished (when in_air_promise resolves)

    // // Start offboard mode
    // auto offboard = Offboard{m_system};
    // Offboard::VelocityNedYaw stay{};
    // offboard.set_velocity_ned(stay); // set initial setpoint

    // Offboard::Result offboard_result = offboard.start();
    // if (offboard_result != Offboard::Result::Success)
    // {
    //     log(tag, "Offboard start failed, error code on next line", true);
    //     std::cout << "Code: " << offboard_result << std::endl;
    //     return false;
    // }

    // log(tag, "Offboard started");

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// TESTING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Drone::test1() {
    Mat img;
    std::string tag = "Test 1";
    std::string tags_detected = "";
    raspi_camera.start();

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    int time_span = 0;
    int count_frames = 0;
    while (time_span < 5 * 1000 /* 5 seconds */) {
        high_resolution_clock::time_point f1 = high_resolution_clock::now();
        img = raspi_camera.update_current_image(); // this is blocking
        high_resolution_clock::time_point f2 = high_resolution_clock::now();
        duration<double, std::milli> c1 = (f2 - f1);

        high_resolution_clock::time_point f3 = high_resolution_clock::now();
        float *errs = image_analyzer.processImage(img, 0, m_yaw, tags_detected); //Detects apriltags and calculates errors
        high_resolution_clock::time_point f4 = high_resolution_clock::now();
        duration<double, std::milli> c2 = (f4 - f3);

        if (errs != nullptr) {
            log(tag, "Apriltag found! camera: " + std::to_string(c1.count()) + " detector: " + std::to_string(c2.count()));
        } else {
            log(tag, "Failed to find Apriltag");
        }

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        duration<double, std::milli> d = (t2 - t1);
        time_span = d.count();
        count_frames++;
    }
    cv::imwrite("out/test.png", img);
    raspi_camera.end();
    log(tag, "Number of frames processed in " + std::to_string(time_span) + " ms: " + std::to_string(count_frames));
    log(tag, "Average FPS: " + std::to_string(count_frames / (time_span / 1000)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// DOCKING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Starts docking process
void Drone::initiate_docking(int target_id)
{

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
 */
bool Drone::stage1(int target_id)
{
    log("Stage 1", "Docking Beginning");
    auto telemetry = Telemetry{m_system}; // NOTE: telemetry updates are only received as long as the telemetry pointer is alive
                                          // will need to either resubscribe in stage 2 or move this to initiate_docking to keep it alive
                                          // and pass as argument to stage1(), stage2()
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

    auto offboard = Offboard{m_system};

    int failed_frames = 0;
    int successful_frames = 0;
    PIDController *pid = new PIDController(m_dt);
    Mat img;
    std::string tags = "";

    while (true)
    {
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        img = raspi_camera.update_current_image();
        float *errs = image_analyzer.processImage(img, 0, m_yaw, tags); //Detects apriltags and calculates errors
        log("Tags Detected", tags);

        if (errs == nullptr) //Central target not found
        {
            log("No Tag Detected, Failed Frames", std::to_string(failed_frames) + "");
            if (m_down * -1.0 - m_target_info.alt > MAX_HEIGHT) //Above maximum height, docking fails
            {
                log("Docking", "Error: above maximum height", true);
                delete pid;
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
        float *velocities = pid->getVelocities(errs[0], errs[1], errs[2], 0.4); //Gets velocities for errors
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

        delete errs; //Cleanup
        delete velocities;

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

    delete pid;
    return true;
}

/**
 * Attempts stage 2 of docking
 * Will execute completely if successful, drone will safely land if failure
 */
void Drone::stage2(int target_id)
{
    log("Stage 2", "Docking Beginning");
    PIDController *pid = new PIDController(m_dt);
    int successful_frames = 0;
    std::string tags = "";
    auto offboard = Offboard{m_system};
    // Register for attitude updates
    auto telemetry = Telemetry{m_system}; // NOTE: telemetry updates are only received as long as the telemetry pointer is alive
                                          // will need to either resubscribe in stage 2 or move this to initiate_docking to keep it alive
                                          // and pass as argument to stage1(), stage2()
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

    Mat img;
    while (true)
    {
        //Update image and process for errors
        img = raspi_camera.update_current_image();
        float *errs = image_analyzer.processImage(img, target_id, m_yaw, tags);
        log("Tags Detected", tags);
        int checked_frames = 0;
        int docking_attempts = 0;

        while (errs == nullptr) //Target tag not detected
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

                    delete pid;
                    return;
                }
                img = raspi_camera.update_current_image();
                errs = image_analyzer.processImage(img, target_id, m_yaw, tags);

                //Ascends until max height or until peripheral target detected
                while (errs == nullptr && m_down * -1.0 - m_target_info.alt < MAX_HEIGHT_STAGE_2)
                {
                    log("Docking", "Ascending for peripheral target");
                    Offboard::VelocityBodyYawspeed change{};
                    change.down_m_s = -0.2f;
                    offboard.set_velocity_body(change);
                    img = raspi_camera.update_current_image();
                    errs = image_analyzer.processImage(img, target_id, m_yaw, tags);
                }
                if (errs != nullptr) //Peripheral target detected, continues with docking normally
                    break;

                log("Docking", "Peripheral target not detected, ascending for central target");
                //Peripheral target not detected, ascend until max height reached or central target detected
                while (errs == nullptr && m_down * -1.0 - m_target_info.alt < MAX_HEIGHT)
                {
                    Offboard::VelocityBodyYawspeed change{};
                    change.down_m_s = -0.2f;
                    offboard.set_velocity_body(change);
                    img = raspi_camera.update_current_image();
                    errs = image_analyzer.processImage(img, 0, m_yaw, tags);
                }

                //Central target detected, re-attempt stage 1
                if (errs != nullptr)
                {
                    log("Docking", "Central target detected, re-attempting stage 1");
                    stage1(target_id);
                    img = raspi_camera.update_current_image();
                    errs = image_analyzer.processImage(img, target_id, m_yaw, tags);
                    checked_frames = 0;
                }
                else //Maximum height exceeded, docking failure
                {
                    log("DOCKING FAILED", "Maximum Height Exceeded", true);
                    safe_land();

                    delete pid;
                    return;
                }
            }
            sleep_for(milliseconds((int)m_dt * 1000));
            img = raspi_camera.update_current_image();
            float *errs = image_analyzer.processImage(img, target_id, m_yaw, tags);
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

        log("Docking", "Errors: x_err: " + std::to_string(errs[0]) + " y_err: " + std::to_string(errs[1]) + " alt_err: " + std::to_string(errs[2]) + " rot_err: " + std::to_string(errs[3]));
        float *velocities = pid->getVelocities(errs[0], errs[1], errs[2], 0.1); //Gets velocities for errors
        log("Docking", "Velocities: east: " + std::to_string(velocities[0]) + " north: " + std::to_string(velocities[1]) + " down: " + std::to_string(velocities[2]));

        //Updates drone velocities
        Offboard::VelocityNedYaw change{};
        change.yaw_deg = errs[3];
        change.north_m_s = velocities[1];
        change.east_m_s = velocities[0];
        change.down_m_s = velocities[2];
        offboard.set_velocity_ned(change);

        //Cleanup
        delete errs;
        delete velocities;
        sleep_for(milliseconds((int)m_dt * 1000));
    }
    delete pid;
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
    offboard.stop();
    action.land();
    const Telemetry::Result set_rate_result = telemetry.set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success)
    {
        log("Landing", "Telemetry Error", true);
        return;
    }
    telemetry.subscribe_position([](Telemetry::Position position) {
        log("Landing", std::to_string(position.relative_altitude_m));
    });
    while (telemetry.in_air())
    {
        sleep_for(seconds(1));
    }
    log("Landing", "Landed");
    action.disarm();
    log("Landing", "Disarmed");
}

//Utility method to adjust errors for stage 1
void Drone::offset_errors(float *errs, int id)
{
    float target_offset = id <= 3 ? abs(id - 3) * 45 : (11 - id) * 45;
    float x = errs[0] + BOOM_LENGTH / 2 * cos(to_radians(target_offset - errs[3]));
    float y = errs[1] + BOOM_LENGTH / 2 * sin(to_radians(target_offset - errs[3]));
    errs[2] -= ALTITUDE_DISP;
    errs[0] = x;
    errs[1] = y;
}