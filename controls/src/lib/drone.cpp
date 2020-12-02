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
    this.drone_id = drone_id;
    init_telem();
    drone_state = STANDBY;

    drone_status.drone_id = this.drone_id;
    drone_status.drone_state = drone_state;
    drone_status.docking_slot = docking_slot;
}

void Drone::init_telem()
{
    telem = std::make_shared<mavsdk::Telemetry>(system);

    telemetry->subscribe_position([&gps_position](mavsdk::Telemetry::Position position) {
        gps_position = position;
    }); 

    telemetry->subscribe_attitude_quaternion([&att_quaternion](mavsdk::Telemetry::AttitudeQuaternionCallback quaternion ){
        att_quaternion = quaternion;
    });

    telemetry->subscribe_battery([&batt](mavsdk::Telemetry::BatteryCallback battery){
        batt = battery;
    });

    // more telemetry
}

void update_drone_status()
{
    drone_status.drone_state = drone_state;
    drone_status.docking_slot = docking_slot;
    drone_status.gps_position = gps_position;
    drone_status.yaw = att_quaternion.z; //TODO: verify 
    drone_status.battery_percent = batt.remaining_percent;
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