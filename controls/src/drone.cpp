#include "drone.hpp"
#include "px4_io.hpp"

drone::drone()
{
	//TODO: default constructor?
}

drone::drone(std::string connection_url)
{
	system = connect_to_pixhawk(connection_url);
}

void drone::arm_drone() // for drones in STANDBY / DOCKED_FOLLOWER
{
    //if(drone_state == STANDBY || drone_state == DOCKED_FOLLOWER)
    //    arm_system(system);
}   

void drone::arm_frame() // for DOCKED_LEADER (send arm_drone() to followers)
{

}    

void drone::disarm_drone() // for drones in STANDBY / DOCKED_FOLLOWER
{
    //if(drone_state == STANDBY || drone_state == DOCKED_FOLLOWER)
    //    disarm_system(system);
}    

void drone::disarm_frame() // for DOCKED_LEADER (send disarm_drone() to followers)
{

}
    
void drone::takeoff_drone() // for drones in STANDBY
{
    //if(drone_state == STANDBY)
    //    takeoff_system(system);   
}    
    
void drone::takeoff_frame() // for DOCKED_LEADER (send attitude and thrust to followers)
{

}
    
void drone::land_drone() // for any undocked drone
{
    //if(drone_state == STANDBY || drone_state == NEEDS_SERVICE)
    //    land_system(system);  
}
    
void drone::land_frame() // for DOCKED_LEADER (send attitude and thrust to followers)
{

}