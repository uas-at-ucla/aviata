#ifndef DRONE_HPP
#define DRONE_HPP

#include <string>
#include <iostream>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "dronetelemetry.hpp"
#include "px4_io.hpp"

enum DroneState {
    STANDBY,
    ARRIVING,
    DOCKING,
    DOCKED_FOLLOWER,
    DOCKED_LEADER,
    UNDOCKING,
    DEPARTING,
    NEEDS_SERVICE
};

// reference values (copy of values within Drone)
struct DroneStatus {
    std::string drone_id;
    DroneState drone_state;
    uint8_t docking_slot;

    float battery_percent; 
    mavsdk::Telemetry::Position gps_position;
    float yaw;
};

class Drone
{
	public:
    Drone(std::string drone_id, PX4IO& px4_io);
    
	// AVIATA 
	//std::string drone_name;
    std::string drone_id;
    DroneState drone_state;
    DroneStatus drone_status;
    uint8_t docking_slot = -1;

    //TELEMETRY
    DroneTelemetry telemValues; // DroneTelemetry object

    void update_drone_status(); // call before sending data

    void arm_drone(); // for drones in STANDBY / DOCKED_FOLLOWER
    
    void arm_frame(); // for DOCKED_LEADER (send arm_drone() to followers)
    
    void disarm_drone(); // for drones in STANDBY / DOCKED_FOLLOWER
    
    void disarm_frame(); // for DOCKED_LEADER (send disarm_drone() to followers)

    void takeoff_drone(); // for drones in STANDBY
    
    void takeoff_frame(); // for DOCKED_LEADER (send attitude and thrust to followers)

    void land_drone(); // for any undocked drone
    
    void land_frame(); // for DOCKED_LEADER (send attitude and thrust to followers)

    void undock();

    void dock(int n); 

    void become_leader();

    void become_follower(); //for successful sender of request_new_leader

    void get_leader_setpoint(float q[4], float* thrust);

    void set_follower_setpoint(float q[4], float* thrust);
    
private:
    PX4IO& px4_io;
};

#endif
