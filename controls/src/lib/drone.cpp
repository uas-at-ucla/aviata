#include "drone.hpp"
#include "px4_io.hpp"

Drone::Drone()
{
	//TODO: default constructor?
}

Drone::Drone(std::string connection_url)
{
	system = connect_to_pixhawk(connection_url);
}

void Drone::arm_drone() // for drones in STANDBY / DOCKED_FOLLOWER
{
    //if(drone_state == STANDBY || drone_state == DOCKED_FOLLOWER)
    //    arm_system(system);
}   

void Drone::arm_frame() // for DOCKED_LEADER (send arm_drone() to followers)
{

}    

void Drone::disarm_drone() // for drones in STANDBY / DOCKED_FOLLOWER
{
    //if(drone_state == STANDBY || drone_state == DOCKED_FOLLOWER)
    //    disarm_system(system);
}    

void Drone::disarm_frame() // for DOCKED_LEADER (send disarm_drone() to followers)
{

}
    
void Drone::takeoff_drone() // for drones in STANDBY
{
    //if(drone_state == STANDBY)
    //    takeoff_system(system);   
}    
    
void Drone::takeoff_frame() // for DOCKED_LEADER (send attitude and thrust to followers)
{

}
    
void Drone::land_drone() // for any undocked drone
{
    //if(drone_state == STANDBY || drone_state == NEEDS_SERVICE)
    //    land_system(system);  
}
    
void Drone::land_frame() // for DOCKED_LEADER (send attitude and thrust to followers)
{

}