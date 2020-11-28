
 #define USE_MAV //enable mavsdk var types

#include <string>

#ifdef USE_MAV
#include <mavsdk/mavsdk.h>
// #include <mavsdk/plugins/system/system.h> // from the PX4 Guide it seems system is part of mavsdk.h
#endif

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

class drone
{
	public:
    drone::drone();
    drone::drone(std::String connection_url);

	// AVIATA 
	std::String drone_name;
    DroneState drone_state;

	//MAVSDK
	std::String connection_url;
	mavsdk::System* system;

    void arm_drone(); // for drones in STANDBY / DOCKED_FOLLOWER
    void arm_drone(std::String drone_name); // arm individual drone
    
    void arm_frame(); // for DOCKED_LEADER (send arm_drone() to followers)
    
    void disarm_drone(); // for drones in STANDBY / DOCKED_FOLLOWER
    void disarm_drone(std::String drone_name);
    
    void disarm_frame(); // for DOCKED_LEADER (send disarm_drone() to followers)

    void takeoff_drone(); // for drones in STANDBY
    void takeoff_drone(std::String drone_name);    
    
    void takeoff_frame(); // for DOCKED_LEADER (send attitude and thrust to followers)

    void land_drone(); // for any undocked drone
    void land_drone(std::String drone_name);    
    
    void land_frame(); // for DOCKED_LEADER (send attitude and thrust to followers)

    private:
}

