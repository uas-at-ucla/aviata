#ifndef DRONE_HPP
#define DRONE_HPP

#include <string>
#include <mavsdk/mavsdk.h>

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

struct DroneStatus {
    std::string id;
    DroneState state;
    uint8_t docking_slot;
    // add telemetry
};

class Drone
{
	public:
    Drone();
    Drone(std::string connection_url);

	// AVIATA 
	std::string drone_name;
    DroneState drone_state;

	//MAVSDK
	std::string connection_url;
	std::shared_ptr<mavsdk::System> system;

    void arm_drone(); // for drones in STANDBY / DOCKED_FOLLOWER
    
    void arm_frame(); // for DOCKED_LEADER (send arm_drone() to followers)
    
    void disarm_drone(); // for drones in STANDBY / DOCKED_FOLLOWER
    
    void disarm_frame(); // for DOCKED_LEADER (send disarm_drone() to followers)

    void takeoff_drone(); // for drones in STANDBY
    
    void takeoff_frame(); // for DOCKED_LEADER (send attitude and thrust to followers)

    void land_drone(); // for any undocked drone
    
    void land_frame(); // for DOCKED_LEADER (send attitude and thrust to followers)

    private:
};

#endif
