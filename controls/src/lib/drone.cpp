#include "drone.hpp"
#include "px4_io.hpp"

Drone::Drone()
{
	//TODO: default constructor?
    std::cout<<"\n\n\nbonkers.\n\n\n"<<std::endl;
}

Drone::Drone(std::string drone_id, std::string connection_url)
{
    this->drone_id = drone_id;
    system = connect_to_pixhawk(drone_id,connection_url);
    telemValues = new DroneTelemetry(system);
    telemValues->init_telem();
    drone_state = STANDBY;

    drone_status.drone_id = this->drone_id;
    drone_status.drone_state = this->drone_state;
    drone_status.docking_slot = this->docking_slot;
}

Drone::~Drone()
{
    delete telemValues;
}

void Drone::update_drone_status()
{
    drone_status.drone_state = drone_state;
    drone_status.docking_slot = docking_slot;
    drone_status.gps_position = telemValues->dronePosition;
    drone_status.yaw = telemValues->droneQuarternion.z; //TODO: verify 
    drone_status.battery_percent = telemValues->droneBattery.remaining_percent;
}

void Drone::arm_drone() // for drones in STANDBY / DOCKED_FOLLOWER
{
    if(drone_state == STANDBY || drone_state == DOCKED_FOLLOWER)
        arm_system();
}   

void Drone::arm_frame() // for DOCKED_LEADER (send arm_drone() to followers)
{

}    

void Drone::disarm_drone() // for drones in STANDBY / DOCKED_FOLLOWER
{
    if(drone_state == STANDBY || drone_state == DOCKED_FOLLOWER)
        disarm_system();
}    

void Drone::disarm_frame() // for DOCKED_LEADER (send disarm_drone() to followers)
{

}
    
void Drone::takeoff_drone() // for drones in STANDBY
{
    if(drone_state == STANDBY)
        takeoff_system();   
}    
    
void Drone::takeoff_frame() // for DOCKED_LEADER (send attitude and thrust to followers)
{

}
    
void Drone::land_drone() // for any undocked drone
{
    if(drone_state == STANDBY || drone_state == NEEDS_SERVICE)
        land_system();  
}
    
void Drone::land_frame() // for DOCKED_LEADER (send attitude and thrust to followers)
{

}