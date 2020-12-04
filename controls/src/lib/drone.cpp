#include "drone.hpp"
#include "px4_io.hpp"

Drone::Drone()
{
	//TODO: default constructor?
    std::cout<<"\n\n\nbonkers.\n\n\n"<<std::endl;
}

Drone::Drone(std::string drone_id, std::string connection_url)
{
    system = connect_to_pixhawk(connection_url);
    this->drone_id = drone_id;
    telemValues = new Telemetry();
    init_telem();
    drone_state = STANDBY;

    drone_status.drone_id = this->drone_id;
    drone_status.drone_state = this->drone_state;
    drone_status.docking_slot = this->docking_slot;
}

Drone::~Drone()
{
    delete telemValues;
}

void Drone::init_telem()
{
    telem.subscribe_position([&](mavsdk::Telemetry::Position position) {
        telemValues->position = position;
    });
    telem.subscribe_attitude_quaternion([&](mavsdk::Telemetry::Quaternion quaternion){
        telemValues->quarternion = quaternion;
    });
    telem.subscribe_battery([&](mavsdk::Telemetry::Battery battery){
        telemValues->battery = battery;
    });
}
    

void Drone::update_drone_status()
{
    drone_status.drone_state = drone_state;
    drone_status.docking_slot = docking_slot;
    drone_status.gps_position = telemValues->position;
    drone_status.yaw = telemValues->quarternion.z; //TODO: verify 
    drone_status.battery_percent = telemValues->battery.remaining_percent;
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