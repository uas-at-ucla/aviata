#ifndef DRONE_HPP
#define DRONE_HPP

#include <string>
#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

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

class Telemetry
{
    public:
        mavsdk::Telemetry::Position position; //https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_position.html
        mavsdk::Telemetry::Battery battery; //https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_quaternion.html
        mavsdk::Telemetry::Quaternion quarternion; //https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_battery.html
};

class Drone
{
	public:
    Drone();
    ~Drone();
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
    mavsdk::Telemetry telem = mavsdk::Telemetry(system);

    //TELEMETRY
    Telemetry* telemValues; // pointer to Telemetry object

    void init_telem(); // initializes telemetry values

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
