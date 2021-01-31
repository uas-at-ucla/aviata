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
    : m_north(0), m_east(0), m_down(-5), m_yaw(0), m_dt(0.05), camera_simulator(t), m_target_info(t),image_analyzer()
{}

/**
 * Connect to Gazebo and run pre-flight checks
 * This function should be called before attempting to take any other action with the drone.
 * 
 * @return true if connection succeeded, false otherwise
 * */
bool Drone::connect_gazebo() {
    std::string tag = "Connecting";
    std::string connection_url = "udp://:14540"; // change as needed, or add command-line parsing
    ConnectionResult connection_result = mavsdk.add_any_connection(connection_url);

    if (connection_result != ConnectionResult::Success) {
        log(tag, "Connection failed, see next line", true);
        std::cout << connection_result << std::endl;
        return false;
    }

    log(tag, "Waiting to discover system");
    std::promise<void> discover_promise;
    auto discover_future = discover_promise.get_future();

    mavsdk.subscribe_on_new_system([&mavsdk=mavsdk, &discover_promise, &tag]() {
        const auto system = mavsdk.systems().at(0);

        if (system->is_connected()) {
            log(tag, "Discovered system");
            discover_promise.set_value();
        }
    });

    discover_future.wait();

    // System got discovered.
    m_system = mavsdk.systems().at(0);
    auto telemetry = Telemetry{m_system};

    while (!telemetry.health_all_ok()) {
        log(tag, "Waiting for system to be ready");
        sleep_for(seconds(1));
    }

    log(tag, "System is ready");
    return true;
}

/**
 * Attempt to take off.
 * 
 * @return true if successful, false otherwise
 * */
bool Drone::takeoff() {
    std::string tag = "Takeoff";
    std::promise<void> in_air_promise;
    auto in_air_future = in_air_promise.get_future();

    // Arm the drone
    auto action = Action{m_system};
    Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        log(tag, "Arming failed", true);
        return false;
    }
    log(tag, "Armed");

    // Attempt to take off
    action.set_takeoff_altitude(8);
    Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        log(tag, "Takeoff failed, error code on next line", true);
        std::cout << "Code: " << takeoff_result << std::endl;
        return false;
    }

    log(tag, "Taking off");

    // Wait for the take off to be complete
    auto telemetry = Telemetry{m_system};
    telemetry.subscribe_landed_state([&in_air_promise, &telemetry, &tag](Telemetry::LandedState landed) {
        if (landed == Telemetry::LandedState::InAir) {
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
    if (offboard_result != Offboard::Result::Success) {
        log(tag, "Offboard start failed, error code on next line", true);
        std::cout << "Code: " << offboard_result << std::endl;
        return false;
    }

    log(tag, "Offboard started");

    return true;
}

void Drone::initiate_docking(int target_id) {

    bool stage1_outcome = stage1(target_id);
    if (stage1_outcome) {
        stage2(target_id);

    }   
}

void Drone::set_position(float north, float east, float down) {
    m_north = north;
    m_east = east;
    m_down = down;
}

void Drone::set_yaw(float yaw) {
    m_yaw = yaw;
}

bool Drone::stage1(int target_id) {


    // Register for attitude updates
    auto telemetry = Telemetry{m_system}; // NOTE: telemetry updates are only received as long as the telemetry pointer is alive
                                          // will need to either resubscribe in stage 2 or move this to initiate_docking to keep it alive
                                          // and pass as argument to stage1(), stage2()
    const Telemetry::Result set_rate_result_p = telemetry.set_rate_position_velocity_ned(40);
    const Telemetry::Result set_rate_result_a = telemetry.set_rate_attitude(40);
    if (set_rate_result_a != Telemetry::Result::Success || set_rate_result_p != Telemetry::Result::Success) {
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

    while (true) {
        high_resolution_clock::time_point t1 = high_resolution_clock::now();

        Offboard::VelocityBodyYawspeed spin{};
        spin.yawspeed_deg_s = 36.0f;
        offboard.set_velocity_body(spin);
        log("Yaw", std::to_string(m_yaw));
        
        Mat currImg=camera_simulator.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, 0); // looks a bit choppy only cause it's at 20fps, remove sleep_for and it's smooth
        std::string tags="";
        float* errs=image_analyzer.processImage(currImg,0,m_yaw,tags);
        log("Tags",tags);

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        duration<double, std::milli> d = (t2 - t1);
        int time_span = d.count();
        if (time_span < m_dt * 1000) {
            sleep_for(milliseconds((int) (m_dt * 1000 - time_span)));
        }
    }
    return true;
}

void Drone::stage2(int target_id) {
    log("Stage 2","Docking Beginning");
    PIDController* pid=new PIDController(m_dt);
    int successful_frames=0;
    std::string tags="";

    while (true){
        Mat img=camera_simulator.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, 0);
        float* errs=image_analyzer.processImage(currImg,id,m_yaw,tags);
        int checked_frames=0;
        int docking_attempts=0;

        while(errs==nullptr){
            checked_frames++;
            log("No Tag Detected, Checked Frames",checked_frames+"");
            offboard.set_velocity_body({0,0,-0/1,0});

            if(checked_frames>1/m_dt){
                docking_attempts++;
                if(docking_attempts>MAX_ATTEMPTS){
                    log("DOCKING FAILED","Maximum Attempts Exceeded",true);
                    safe_land();

                    delete pid;
                    return;
                }
            }
            camera_simulator.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, 0);
            errs=image_analyzer.processImage(currImg,id,m_yaw,tags);
            while(errs==nullptr&&m_down*-1.0-m_target.alt<MAX_HEIGHT_STAGE_2){
                offboard.set_velocity_body({0,0,-0.2,0});
                camera_simulator.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, 0);
                errs=image_analyzer.processImage(currImg,id,m_yaw,tags);
            }
            if(errs!=nullptr) break;
            while(errs==nullptr&&m_down*-1.0-m_target_alt<MAX_HEIGHT){
                offboard.set_velocity_body({0,0,-0.2,0});
                camera_simulator.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, 0);
                errs=image_analyzer.processImage(currImg,0,m_yaw,tags);
            }
            if(errs!=nullptr){
                stage1(id);
                camera_simulator.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, 0);
                errs=image_analyzer.processImage(currImg,0,m_yaw,tags);
                checked_frames=0;
            }
            else{
                log("DOCKING FAILED","Maximum Attempts Exceeded",true);
                safe_land();

                delete pid;
                return;
            }
            sleep_for(m_dt*1000);
        }
        errs[2]-=0.05;
        if(errs[2]<STAGE_2_TOLERANCE*2&&errs[2]>-1.0*STAGE_2_TOLERANCE&&errs[3]<2.0&&errs[3]>-2.0 &&errs[0]<STAGE_2_TOLERANCE
        &&errs[0]>-1.0* STAGE_2_TOLERANCE&&errs[1]<STAGE_2_TOLERANCE&&errs[1]>-1.0* STAGE_2_TOLERANCE){
            successful_frames++;
        }
        else{
            successful_frames=0;
        }
        if(successful_frames>1/m_dt)break;

        float* velocities=pid.getVelocities(errs[0],errs[1],errs[3],0.1)
        offboard.set_velocity_ned({velocities[1],velocities[0],velocities[2],errs[3]});
        delete errs;
        delete velocities;
        sleep_for(m_dt*1000);
    }
    delete pid;
}

void Drone::offset_errors(double x, double y, double alt, double rot, int target_id) {

}

void Drone::safe_land() {
    land();
}

void Drone::land() {

}