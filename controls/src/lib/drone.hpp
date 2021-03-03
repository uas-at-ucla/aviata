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
    // mavsdk::Telemetry::Position gps_position;
    float gps_position[4];
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
    std::map<std::string, DroneStatus> swarm; // map by drone_id

    // Leader
    uint8_t leader_increment = 0;
    uint8_t leader_increment_next = 0;

    uint8_t leader_follower_listened = 0; // keep track of how many followers acknowledged LISTEN_NEW_LEADER command

    // Command Request Lists
    std::vector<CommandRequest> drone_command_requests;
    std::vector<CommandRequest> drone_command_responses; // TODO: log to file at end of flight?

    void basic_lead();
    void basic_follow();

    void update_drone_status(); // call before sending data

    int arm_drone(); // for drones in STANDBY / DOCKED_FOLLOWER
    
    int arm_frame(); // for DOCKED_LEADER (send arm_drone() to followers)
    
    int disarm_drone(); // for drones in STANDBY / DOCKED_FOLLOWER
    
    int disarm_frame(); // for DOCKED_LEADER (send disarm_drone() to followers)

    int takeoff_drone(); // for drones in STANDBY
    
    int takeoff_frame(); // for DOCKED_LEADER (send attitude and thrust to followers)

    int land_drone(); // for any undocked drone
    
    int land_frame(); // for DOCKED_LEADER (send attitude and thrust to followers)

    int undock();

    int dock(int n); 

    int become_leader();

    int become_follower(); //for successful sender of request_new_leader

    void get_leader_setpoint(float q[4], float* thrust);

    void set_follower_setpoint(float q[4], float* thrust);

    void send_drone_command(std::string other_drone_id, DroneCommand drone_command, int param, std::string request_origin,
                            std::function<void(uint8_t ack)> callback);

    void command_handler(const aviata::srv::DroneCommand::Request::SharedPtr request, 
                         aviata::srv::DroneCommand::Response::SharedPtr response);

    void check_command_requests();
};

#endif
