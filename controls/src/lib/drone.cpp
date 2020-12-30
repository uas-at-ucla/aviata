#include "drone.hpp"
#include "px4_io.hpp"

Drone::Drone(std::string drone_id, PX4IO& px4_io): drone_id(drone_id), telemValues(px4_io), px4_io(px4_io)
{
    telemValues.init_telem();
    drone_state = STANDBY;

    drone_status.drone_id = drone_id;
    drone_status.drone_state = drone_state;
    drone_status.docking_slot = docking_slot;
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