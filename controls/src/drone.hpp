
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
	// AVIATA 
	std::String drone_name;
    DroneState drone_state;

	//MAVSDK
	std::String connection_url;
	mavsdk::System* system;

    private:
}

