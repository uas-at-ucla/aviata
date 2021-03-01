#ifndef DRONE_HPP
#define DRONE_HPP

#include <string>
#include <iostream>
#include <vector>

#include "../mavlink/v2.0/common/mavlink.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "dronetelemetry.hpp"
#include "px4_io.hpp"
#include "network.hpp"

#include "aviata/srv/drone_command.hpp"
#include "aviata/msg/follower_setpoint.hpp"

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
    Drone(std::string drone_id);
    ~Drone();

    int run(std::string connection_url);
    int test_lead_att_target(std::string connection_url);
    int test_follow_att_target(std::string connection_url);
    int lead_standalone(std::string connection_url);
    int follow_standalone(std::string connection_url);
    int lead_as_0(std::string connection_url);
    int follow_as_1(std::string connection_url);
    
private:
    const std::string drone_id;

    // APIs
    std::shared_ptr<Network> network;
    PX4IO px4_io;
    DroneTelemetry telemValues;

    // State
    DroneState drone_state;
    DroneStatus drone_status;
    uint8_t docking_slot = 0;
    std::map<std::string, DroneStatus> swarm; // map by ID

    // Leader
    uint8_t leader_increment = 0;
    uint8_t leader_increment_next = 0;

    // Command Request Lists
    std::vector<CommandRequest> drone_command_requests;
    std::vector<CommandRequest> drone_command_responses; // TODO: log to file at end of flight?

    void basic_lead();
    void basic_follow();

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

    void command_handler(aviata::srv::DroneCommand::Request::SharedPtr request, 
                         aviata::srv::DroneCommand::Response::SharedPtr response);

    void check_command_requests();
};

#endif
