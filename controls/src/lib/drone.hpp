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

// same types from mavsdk so it can directly pass in data from px4
struct GpsPosition { double latitude_deg; double longitude_deg; float absolute_altitude_m; float relative_altitude_m; }; //https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_position.html
struct AttQuatternion { float w; float x; float y; float z; }; //https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_quaternion.html
struct Battery { float voltage_v; float remaining_percent; }; //https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_battery.html

// reference values (copy of values within Drone)
struct DroneStatus {
    std::string drone_id;
    DroneState drone_state;
    uint8_t docking_slot;

    float battery_percent; 
    GpsPosition gps_position;
    float yaw;
};

class Drone
{
	public:
    Drone();
    Drone(std::string drone_id, std::string connection_url);

	// AVIATA 
	//std::string drone_name;
    std::string drone_id;
    DroneState drone_state;
    DroneStatus drone_status;
    uint8_t docking_slot = -1;

	//MAVSDK
	std::string connection_url;
	std::shared_ptr<mavsdk::System> system;
    std::shared_ptr<mavsdk::Telemetry> telem;

    //TELEMETRY
    GpsPosition gps_position;
    AttQuatternion att_quaternion;
    Battery batt;

    void init_telem();

    void update_drone_status(); // call before sending data

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
