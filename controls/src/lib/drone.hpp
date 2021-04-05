#ifndef DRONE_HPP
#define DRONE_HPP

#include <string>
#include <iostream>
#include <vector>

#include "../mavlink/v2.0/common/mavlink.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "dronestatus.hpp"
#include "dronetelemetry.hpp"
#include "px4_io.hpp"
#include "network.hpp"

#include "aviata/srv/drone_command.hpp"
#include "aviata/msg/follower_setpoint.hpp"

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
    const std::string _drone_id;

    // APIs
    std::shared_ptr<Network> _network;
    PX4IO _px4_io;
    DroneTelemetry _telem_values;

    // State
    DroneState _drone_state;
    DroneStatus _drone_status;
    uint8_t _docking_slot = 0;
    std::map<std::string, DroneStatus> _swarm; // map by drone_id

    bool _kill_switch_engaged;
    bool _armed;

    // Leader
    uint8_t _leader_seq_num = 0;

    // uint8_t leader_follower_listened = 0; // keep track of how many followers acknowledged LISTEN_NEW_LEADER command
    uint8_t _leader_follower_armed = 0;
    uint8_t _leader_follower_disarmed = 0;

    // Command Request Lists
    std::vector<CommandRequest> _drone_command_requests;
    std::vector<CommandRequest> _drone_command_responses; // TODO: log to file at end of flight?

    void basic_lead();
    void basic_follow();

    bool valid_leader_msg(uint8_t leader_seq_num);

    void update_drone_status(); // call before sending data

    uint8_t arm_drone(); // for drones in STANDBY / DOCKED_FOLLOWER
    
    uint8_t arm_frame(); // for DOCKED_LEADER (send arm_drone() to followers)
    
    uint8_t disarm_drone(); // for drones in STANDBY / DOCKED_FOLLOWER
    
    uint8_t disarm_frame(); // for DOCKED_LEADER (send disarm_drone() to followers)

    uint8_t takeoff_drone(); // for drones in STANDBY
    
    uint8_t takeoff_frame(); // for DOCKED_LEADER (send attitude and thrust to followers)

    uint8_t land_drone(); // for any undocked drone
    
    uint8_t land_frame(); // for DOCKED_LEADER (send attitude and thrust to followers)

    uint8_t undock();

    uint8_t dock(int n); 

    uint8_t become_leader(uint8_t leader_seq_num);

    uint8_t become_follower(); //for successful sender of request_new_leader

    void init_leader();

    void deinit_leader();

    void send_drone_command(std::string other_drone_id, DroneCommand drone_command, int8_t param, std::string request_origin,
                            std::function<void(uint8_t ack)> callback);

    void command_handler(const aviata::srv::DroneCommand::Request::SharedPtr request, 
                         aviata::srv::DroneCommand::Response::SharedPtr response);

    void check_command_requests();
};

#endif
