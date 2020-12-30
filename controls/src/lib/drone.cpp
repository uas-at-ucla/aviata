#include "drone.hpp"

using namespace std::chrono;

void Drone::init() {
    Network::init();
}

Drone::Drone(std::string drone_id): drone_id(drone_id), network(), px4_io(drone_id), telemValues(px4_io)
{
    telemValues.init_telem();
    drone_state = STANDBY;

    drone_status.drone_id = drone_id;
    drone_status.drone_state = drone_state;
    drone_status.docking_slot = docking_slot;
}

int Drone::run(std::string connection_url) {
    // Do all the things
}

int Drone::test_get_att_target(std::string connection_url) {
    if (px4_io.connect_to_pixhawk(connection_url, 5) == false) {
        return 1;
    }

    px4_io.subscribe_attitude_target([](const mavlink_attitude_target_t& attitude_target) {
        std::cout << "thrust: " << attitude_target.thrust << std::endl;
    });

    px4_io.arm_system();

    bool began_descent = false;
    std::cout << "Taking off!" << std::endl;
    int64_t takeoff_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    px4_io.takeoff_system();
    
    while (true) {
        if (!began_descent && duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - takeoff_time >= 10000) {
            std::cout << "Landing!" << std::endl;
            px4_io.land_system();
            began_descent = true;
        }
        px4_io.call_queued_mavsdk_callbacks();
    }

    return 0;
}

void Drone::update_drone_status()
{
    drone_status.drone_state = drone_state;
    drone_status.docking_slot = docking_slot;
    drone_status.gps_position = telemValues.dronePosition;
    drone_status.yaw = telemValues.droneQuarternion.z; //TODO: verify 
    drone_status.battery_percent = telemValues.droneBattery.remaining_percent;
}

void Drone::arm_drone() // for drones in STANDBY / DOCKED_FOLLOWER
{
    if(drone_state == STANDBY || drone_state == DOCKED_FOLLOWER)
        px4_io.arm_system();
}   

void Drone::arm_frame() // for DOCKED_LEADER (send arm_drone() to followers)
{

}

void Drone::disarm_drone() // for drones in STANDBY / DOCKED_FOLLOWER
{
    if(drone_state == STANDBY || drone_state == DOCKED_FOLLOWER)
        px4_io.disarm_system();
}    

void Drone::disarm_frame() // for DOCKED_LEADER (send disarm_drone() to followers)
{

}
    
void Drone::takeoff_drone() // for drones in STANDBY
{
    if(drone_state == STANDBY)
        px4_io.takeoff_system();   
}    
    
void Drone::takeoff_frame() // for DOCKED_LEADER (send attitude and thrust to followers)
{

}
    
void Drone::land_drone() // for any undocked drone
{
    if(drone_state == STANDBY || drone_state == NEEDS_SERVICE)
        px4_io.land_system();  
}
    
void Drone::land_frame() // for DOCKED_LEADER (send attitude and thrust to followers)
{

}

void Drone::undock()
{
    if (drone_state == DOCKED_FOLLOWER)
        drone_state = UNDOCKING;
    // TODO implment control
}

// @param n frame position
void Drone::dock(int n)
{
    // TODO possibly integrate controls stuff through ROS here?
}

void Drone::become_leader()
{
    if (drone_state == DOCKED_FOLLOWER)
    {
        drone_state = DOCKED_LEADER;
        // TODO implement controls
    }
}

void Drone::become_follower() //for successful sender of request_new_leader
{
    if (drone_state == DOCKED_LEADER)
    {
        drone_state = DOCKED_FOLLOWER;
    }

    if (1) //undock request
    {
        drone_state = UNDOCKING;
    }
}

void Drone::get_leader_setpoint(float q[4], float *thrust)
{
    q = telemValues.q_target;
    thrust = &(telemValues.thrust_target);
}

void Drone::set_follower_setpoint(float q[4], float *thrust)
{
    if (drone_state == DOCKED_FOLLOWER) //is this check necessary?
        // offset calculations
        px4_io.set_attitude_and_thrust(q, thrust);
}
