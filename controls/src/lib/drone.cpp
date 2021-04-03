#include "drone.hpp"

using namespace std::chrono;

Drone::Drone(std::string drone_id) : _drone_id(drone_id), _px4_io(drone_id), _telem_values(_px4_io)
{
    _telem_values.init_telem();
    _drone_state = STANDBY;
    update_drone_status();

    _px4_io.subscribe_armed([this](bool is_armed) {
        _armed = is_armed;
    });

    _px4_io.subscribe_status_text([this](mavsdk::Telemetry::StatusText status_text) {
        if (status_text.text == "Kill-switch engaged")
            _kill_switch_engaged = true;
        else if (status_text.text == "Kill-switch disengaged")
            _kill_switch_engaged = false;
    });

    Network::init();
    _network = std::make_shared<Network>(drone_id);
    // init publishers
    _network->init_drone_status_publisher();
    _network->init_drone_debug_publisher();
    // subscribe drone status (also updates command clients)
    _network->subscribe_drone_status([&](const aviata::msg::DroneStatus::SharedPtr ds_rec) {
        // remove from swarm if leaving frame
        if ( static_cast<DroneState>(ds_rec->drone_state) == UNDOCKING )
        {
            auto it = _swarm.find(ds_rec->drone_id);
            _swarm.erase(it);
            // but keeps service client
        }
        else
        {
            DroneStatus rec;
            rec.drone_id = ds_rec->drone_id;
            rec.drone_state = static_cast<DroneState>(ds_rec->drone_state);
            rec.docking_slot = ds_rec->docking_slot;
            rec.battery_percent = ds_rec->battery_percent;
            std::copy(std::begin(ds_rec->gps_position), std::end(ds_rec->gps_position), std::begin(rec.gps_position));
            rec.yaw = ds_rec->yaw;

            _swarm[rec.drone_id] = rec;
            _network->init_drone_command_client(ds_rec->drone_id);
        }
    });
    // start drone command servicer
    _network->init_drone_command_service([this](const aviata::srv::DroneCommand::Request::SharedPtr request,
                                               aviata::srv::DroneCommand::Response::SharedPtr response) {
        command_handler(request, response);
    });
}

Drone::~Drone()
{
    Network::shutdown();
}

int Drone::run(std::string connection_url)
{
    // Do all the things
    return 0;
}

int Drone::test_lead_att_target(std::string connection_url)
{
    if (_px4_io.connect_to_pixhawk(connection_url, 5) == false)
    {
        return 1;
    }

    _px4_io.arm_system();

    _network->init_follower_setpoint_publisher();

    _px4_io.subscribe_attitude_target([this](const mavlink_attitude_target_t &attitude_target) {
        aviata::msg::FollowerSetpoint follower_setpoint;
        std::copy(std::begin(attitude_target.q), std::end(attitude_target.q), std::begin(follower_setpoint.q));
        follower_setpoint.thrust = attitude_target.thrust;
        follower_setpoint.aviata_yaw_est = attitude_target.aviata_yaw_est;
        follower_setpoint.leader_seq_num = _leader_seq_num;
        _network->publish_follower_setpoint(follower_setpoint);
        std::cout << "attitude_target thrust: " << attitude_target.thrust << std::endl;
    });

    bool began_descent = false;
    std::cout << "Taking off!" << std::endl;
    int64_t takeoff_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    _px4_io.takeoff_system();

    while (true)
    {
        if (!began_descent && duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - takeoff_time >= 20000)
        {
            std::cout << "Landing!" << std::endl;
            _px4_io.land_system();
            began_descent = true;
        }
        _px4_io.call_queued_mavsdk_callbacks();
        Network::spin_some(_network);
    }

    return 0;
}

int Drone::test_follow_att_target(std::string connection_url)
{
    if (_px4_io.connect_to_pixhawk(connection_url, 5) == false)
    {
        return 1;
    }

    _px4_io.arm_system(); // TODO manage auto disarm for follower drones

    mavlink_set_attitude_target_t initial_attitude_target;
    initial_attitude_target.q[0] = 1;
    initial_attitude_target.q[1] = 0;
    initial_attitude_target.q[2] = 0;
    initial_attitude_target.q[3] = 0;
    initial_attitude_target.thrust = 0;

    // Fulfill PX4 requirement to "already be receiving a stream of target setpoints (>2Hz)" before offboard mode can be engaged
    // TODO the optimal way to do this will be to to attempt entering offboard mode each time until it works.
    for (int i = 0; i < 50; i++)
    {
        _px4_io.set_attitude_target(initial_attitude_target);
        std::this_thread::sleep_for(milliseconds(50));
    }

    _px4_io.set_offboard_mode();

    _network->subscribe_follower_setpoint([this](const aviata::msg::FollowerSetpoint::SharedPtr follower_setpoint) {
        mavlink_set_attitude_target_t attitude_target;
        std::copy(std::begin(follower_setpoint->q), std::end(follower_setpoint->q), std::begin(attitude_target.q));
        attitude_target.thrust = follower_setpoint->thrust;
        attitude_target.aviata_yaw_est = follower_setpoint->aviata_yaw_est;
        _leader_seq_num = follower_setpoint->leader_seq_num;
        _px4_io.set_attitude_target(attitude_target);
        std::cout << "follower_setpoint thrust: " << follower_setpoint->thrust << std::endl;
    });

    while (true)
    {
        _px4_io.call_queued_mavsdk_callbacks();
        Network::spin_some(_network);
    }

    return 0;
}

int Drone::lead_standalone(std::string connection_url)
{
    if (_px4_io.connect_to_pixhawk(connection_url, 5) == false) {
        return 1;
    }
    basic_lead();
    return 0;
}

int Drone::follow_standalone(std::string connection_url)
{
    if (_px4_io.connect_to_pixhawk(connection_url, 5) == false) {
        return 1;
    }
    basic_follow();
    return 0;
}

int Drone::lead_as_0(std::string connection_url)
{
    if (_px4_io.connect_to_pixhawk(connection_url, 5) == false) {
        return 1;
    }

    while (_px4_io.undock() != 1) {}
    std::cout << "Drone is undocked." << std::endl;
    while (_px4_io.dock(0, nullptr, 0) != 1) {}
    std::cout << "Docking command sent successfully!" << std::endl;
    basic_lead();

    return 0;
}

int Drone::follow_as_1(std::string connection_url)
{
    if (_px4_io.connect_to_pixhawk(connection_url, 5) == false) {
        return 1;
    }

    while (_px4_io.undock() != 1) {}
    std::cout << "Drone is undocked." << std::endl;
    while (_px4_io.dock(1, nullptr, 0) != 1) {}
    std::cout << "Docking command sent successfully!" << std::endl;
    basic_follow();

    return 0;
}


void Drone::basic_lead()
{
    _network->init_follower_setpoint_publisher();

    bool armed = false;
    bool kill_switch_engaged = false;

    _px4_io.subscribe_armed([&armed](bool is_armed) {
        armed = is_armed;
    });

    _px4_io.subscribe_status_text([&kill_switch_engaged](mavsdk::Telemetry::StatusText status_text) {
        if (status_text.text == "Kill-switch engaged") {
            kill_switch_engaged = true;
        } else if (status_text.text == "Kill-switch disengaged") {
            kill_switch_engaged = false;
        }
    });

    _px4_io.subscribe_attitude_target([this, &armed, &kill_switch_engaged](const mavlink_attitude_target_t &attitude_target) {
        if (armed && !kill_switch_engaged) {
            aviata::msg::FollowerSetpoint follower_setpoint;
            std::copy(std::begin(attitude_target.q), std::end(attitude_target.q), std::begin(follower_setpoint.q));
            follower_setpoint.thrust = attitude_target.thrust;
            follower_setpoint.aviata_yaw_est = attitude_target.aviata_yaw_est;
            follower_setpoint.leader_seq_num = _leader_seq_num;
            _network->publish_follower_setpoint(follower_setpoint);
        }
    });

    while (true) {
        _px4_io.call_queued_mavsdk_callbacks();
        Network::spin_some(_network);
    }
}

void Drone::basic_follow()
{
    bool in_offboard = false;
    bool armed = false;
    int64_t last_msg_time = 0;

    _px4_io.subscribe_flight_mode([&in_offboard](mavsdk::Telemetry::FlightMode flight_mode) {
        in_offboard = (flight_mode == mavsdk::Telemetry::FlightMode::Offboard);
    });

    _px4_io.subscribe_armed([&armed](bool is_armed) {
        armed = is_armed;
    });

    _network->subscribe_follower_setpoint([this, &in_offboard, &armed, &last_msg_time](const aviata::msg::FollowerSetpoint::SharedPtr follower_setpoint) {
        if (!valid_leader_msg(follower_setpoint->leader_seq_num)) {
            return;
        }
        
        last_msg_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        mavlink_set_attitude_target_t attitude_target;
        std::copy(std::begin(follower_setpoint->q), std::end(follower_setpoint->q), std::begin(attitude_target.q));
        attitude_target.thrust = follower_setpoint->thrust;
        attitude_target.aviata_yaw_est = follower_setpoint->aviata_yaw_est;
        _px4_io.set_attitude_target(attitude_target);

        if (!in_offboard) {
            if (_px4_io.set_offboard_mode() == 1) {
                in_offboard = true;
            }
        }
        if (in_offboard && !armed) {
            if (_px4_io.arm() == 1) {
                armed = true;
            }
        }
    });

    while (true) {
        if (duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - last_msg_time > 100) {
            if (armed) {
                if (_px4_io.disarm() == 1) {
                    armed = false;
                    _px4_io.set_hold_mode(); // Take out of offboard mode to prevent annoying failsafe beeps
                }
            }
        }
        _px4_io.call_queued_mavsdk_callbacks();
        Network::spin_some(_network);
    }
}

// TODO add new program that can switch between states, and call check_command_requests() in the loop

bool Drone::valid_leader_msg(uint8_t sending_leader_seq_num) {
    // switch to this leader if its sequence number comes after the current sequence number, else only accept if the sequence number matches the current sequence number
    if ((int8_t) (sending_leader_seq_num - _leader_seq_num) > 0) { // utilize unsigned integer overflow, then cast to signed int8 to check order (because sequence numbers are mod 256)
        _leader_seq_num = sending_leader_seq_num;
        return true;
    } else if (sending_leader_seq_num == _leader_seq_num) {
        return true;
    }
    return false;
}

void Drone::update_drone_status()
{
    _drone_status.drone_state = _drone_state;
    _drone_status.docking_slot = _docking_slot;
    std::copy(std::begin(_telem_values.dronePosition), std::end(_telem_values.dronePosition), std::begin(_drone_status.gps_position));
    _drone_status.yaw = _telem_values.droneEulerAngle.yaw_deg;
    _drone_status.battery_percent = _telem_values.droneBattery.remaining_percent;
}

uint8_t Drone::arm_drone() // for drones in STANDBY / DOCKED_FOLLOWER
{
    if (_drone_state == STANDBY || _drone_state == DOCKED_FOLLOWER)
        return _px4_io.arm_system();
    else if (_drone_state == DOCKED_LEADER)
    {
        if (_leader_follower_armed != 0) // protect against infinite recursive call for leader
            return 1;
        return arm_frame();
    }
    _network->publish_drone_debug("Arm Drone FAILED: improper DroneState = " + _drone_state);
    return 0;
}

uint8_t Drone::arm_frame() // for DOCKED_LEADER (send arm_drone() to followers)
{
    if (_drone_state == DOCKED_LEADER)
    {
        _leader_follower_armed = _swarm.size();
        _leader_follower_disarmed = 0;
        for (const auto &[id, status] : _swarm)
        {
            send_drone_command(id, ARM, -1, "arm, " + _drone_id,
                               [this](uint8_t ack) {
                                   if (ack == 1)
                                       _leader_follower_armed--;
                                   else
                                   {
                                       _network->publish_drone_debug("Arm Frame FAILED: Ack = " + ack);
                                       // TODO: cancel tasks?
                                       disarm_frame();
                                   }
                                   if (_leader_follower_armed == 0) //last drone armed
                                       {
                                           _px4_io.arm_system(); // arm itself last
                                           _network->publish_drone_debug("Arm Frame SUCCESS");
                                       }
                                   _network->publish_drone_debug("Arm Frame in progress: awaiting ack for = " + _leader_follower_armed);

                               });
        }
        return 1;
    }
    _network->publish_drone_debug("Arm Frame FAILED: leader improper DroneState = " + _drone_state);
    return 0;
}

uint8_t Drone::disarm_drone() 
{
    if (_drone_state == STANDBY || _drone_state == DOCKED_FOLLOWER || _drone_state == NEEDS_SERVICE)
        return _px4_io.disarm_system();
    else if (_drone_state == DOCKED_LEADER)
    {
        if (_leader_follower_disarmed != 0) // protect against infinite recursive call for leader
            return 1;
        return disarm_frame();
    }
    _network->publish_drone_debug("Disarm Drone FAILED: improper DroneState = " + _drone_state);
    return 0;
}

uint8_t Drone::disarm_frame() 
{
    if (_drone_state == DOCKED_LEADER)
    {
        _leader_follower_disarmed = _swarm.size();
        _leader_follower_armed = 0;
        for (const auto &[id, status] : _swarm)
        {
            send_drone_command(id, DISARM, -1, "disarm, " + _drone_id,
                               [this](uint8_t ack) {
                                   if (ack == 1)
                                       _leader_follower_disarmed--;
                                   else
                                   {
                                       _network->publish_drone_debug("Disarm Frame FAILED: Ack = " + ack);
                                       // TODO: kill switch of some sort?
                                   }
                                   if (_leader_follower_disarmed == 0) //last drone armed
                                       {
                                           _px4_io.disarm_system(); // disarm itself last
                                           _network->publish_drone_debug("Disarm Frame SUCCESS");
                                       }
                                   _network->publish_drone_debug("Disarm Frame in progress: awaiting ack for = " + _leader_follower_disarmed);

                               });
        }
        return 1;
    }
    _network->publish_drone_debug("Disarm Frame FAILED: leader improper DroneState = " + _drone_state);
    return 0;
}

uint8_t Drone::takeoff_drone() // for drones in STANDBY or leader
{
    if (_drone_state == STANDBY)
        return _px4_io.takeoff_system();
    else if (_drone_state == DOCKED_LEADER)
        return takeoff_frame();

    _network->publish_drone_debug("Takeoff Drone FAILED: improper DroneState = " + _drone_state);
    return 0;
}

uint8_t Drone::takeoff_frame() // for DOCKED_LEADER (send attitude and thrust to followers)
{
    if (_drone_state == DOCKED_LEADER)
    {
        // TODO: start setpoint following for followers
        _network->publish_drone_debug("Frame taking off");
        return _px4_io.takeoff_system();
    }
    _network->publish_drone_debug("Takeoff Frame FAILED: improper DroneState = " + _drone_state);
    return 0;
}

uint8_t Drone::land_drone() // for any undocked drone or leader
{
    if (_drone_state == STANDBY || _drone_state == NEEDS_SERVICE)
        return _px4_io.land_system();
     else if (_drone_state == DOCKED_LEADER)
         return land_frame();

    _network->publish_drone_debug("Land Drone FAILED: improper DroneState = " + _drone_state);
    return 0;
}

uint8_t Drone::land_frame() // for DOCKED_LEADER (send attitude and thrust to followers)
{
    if (_drone_state == DOCKED_LEADER)
    {
        _network->publish_drone_debug("Frame landing");
        return _px4_io.land_system();
    }
    _network->publish_drone_debug("Land Frame FAILED: improper DroneState = " + _drone_state);
    return 0;
}

uint8_t Drone::undock()
{
    if (_drone_state == DOCKED_FOLLOWER)
    {
        _drone_state = UNDOCKING;
        // TODO implment control
        return 1;
    }
    return 0;
}

// @param n frame position
uint8_t Drone::dock(int n)
{
    // TODO possibly integrate controls stuff through ROS here?
    return 1;
}

uint8_t Drone::become_leader(uint8_t sending_leader_seq_num)
{
    if (!valid_leader_msg(sending_leader_seq_num)) {
        return 0;
    }

    // TODO: also check if battery % is above a certain threshold, and check if not about to undock
    if (_drone_state == DOCKED_FOLLOWER) 
    {
        _leader_seq_num += (uint8_t) 1; // increment by 1, or wrap around to 0 if overflow
        _drone_state = DOCKED_LEADER;

        // // Tell all drones to listen for new leader
        // leader_follower_listened = swarm.size();
        // for (const auto &[id, status] : swarm)
        // {
        //     send_drone_command(id, LISTEN_NEW_LEADER, leader_seq_num_next, "listen new leader, " + drone_id,
        //                        [this](uint8_t ack) { //
        //                            if (ack == 1)
        //                                leader_follower_listened--;
        //                            if (leader_follower_listened == 0) //last follower acknowledged
        //                            {
        //                                drone_state = DOCKED_LEADER;
        //                                leader_seq_num = leader_seq_num_next;
        //                                network->publish_drone_debug("Become Leader SUCCESS");

        //                                // TODO: Start Leader stuff
        //                                init_leader();
                                       
        //                            }
        //                            network->publish_drone_debug("Become Leader in progress: awaiting ack for = " + leader_follower_listened);
        //                        });
        // }
        return 1;
    }

    _network->publish_drone_debug("Become Leader FAILED: improper DroneState = " + _drone_state);
    return 0;
}

uint8_t Drone::become_follower() //for successful sender of request_new_leader
{
    if (_drone_state == DOCKED_LEADER)
    {
        float highest_batt = 0;
        std::string highest_batt_drone = "";
        for (const auto& [id, status] : _swarm)
        {
            // determine drone to become leader
            if (status.battery_percent > highest_batt)
            {
                highest_batt = status.battery_percent;
                highest_batt_drone = id;
            }
        }
        _network->publish_drone_debug("Identified new leader: drone_id = " + highest_batt_drone);
        
        send_drone_command(highest_batt_drone, BECOME_LEADER, _leader_seq_num, "become new leader, " + _drone_id,
        [this](uint8_t ack) {
            if (ack == 1) {
                _drone_state = DOCKED_FOLLOWER;
                _network->publish_drone_debug("Become Follower SUCCESS");
                
                // TODO: End Leader Stuff
                deinit_leader();
            } else {
                _network->publish_drone_debug("Become Follower: Requested leader rejected, trying again...");
                become_follower();
            }
        });
        return 1;
    }

    _network->publish_drone_debug("Become Follower FAILED: improper DroneState = " + _drone_state);
    return 0;
}

void Drone::init_leader()
{
    _network->init_follower_setpoint_publisher();
    // send follower setpoints
    _px4_io.subscribe_attitude_target([this](const mavlink_attitude_target_t &attitude_target) {
        if (!_kill_switch_engaged) {
            aviata::msg::FollowerSetpoint follower_setpoint;
            std::copy(std::begin(attitude_target.q), std::end(attitude_target.q), std::begin(follower_setpoint.q));
            follower_setpoint.thrust = attitude_target.thrust;
            follower_setpoint.aviata_yaw_est = attitude_target.aviata_yaw_est;
            follower_setpoint.leader_seq_num = _leader_seq_num;
            _network->publish_follower_setpoint(follower_setpoint);
        }
    });
}

void Drone::deinit_leader()
{
    _network->deinit_follower_setpoint_publisher();
    _px4_io.subscribe_attitude_target(nullptr); // possibly unecessary since publisher is deinitialized
}

// @brief async, adds to list of command requests, call check_command_requests() to process
void Drone::send_drone_command(std::string other_drone_id, DroneCommand drone_command, int8_t param, std::string request_origin,
                               std::function<void(uint8_t ack)> callback)
{
    CommandRequest com;
    com.other_drone_id = other_drone_id;
    com.drone_command = drone_command;
    com.param = param;
    com.callback = std::make_shared<std::function<void(uint8_t)>>(callback);
    com.command_request = _network->send_drone_command_async(other_drone_id, drone_command, param);
    com.request_origin = request_origin;
    com.timestamp_request = std::chrono::high_resolution_clock::now();
    // com.callback = std::make_shared<std::function<void(uint8_t)>>([](uint8_t ack) {}); // TODO
    _drone_command_requests.push_back(com);
}

// @brief to be used as callback for service server
void Drone::command_handler(const aviata::srv::DroneCommand::Request::SharedPtr request,
                            aviata::srv::DroneCommand::Response::SharedPtr response)
{
    switch (request->command)
    {
    case DroneCommand::REQUEST_SWAP:

        break;
    case DroneCommand::REQUEST_UNDOCK:

        break;
    case DroneCommand::REQUEST_DOCK:

        break;
    case DroneCommand::TERMINATE_FLIGHT:

        break;
    case DroneCommand::UNDOCK:

        break;
    case DroneCommand::DOCK:

        break;
    case DroneCommand::CANCEL_DOCKING:

        break;
    case DroneCommand::SETPOINT:

        break;
    case DroneCommand::LEADER_SETPOINT:

        break;
    case DroneCommand::BECOME_LEADER:
        response->ack = become_leader(request->param);
        break;
    case DroneCommand::REQUEST_NEW_LEADER:
        response->ack = become_follower();
        break;
    // case DroneCommand::LISTEN_NEW_LEADER:
    //     _leader_seq_num_next = request->param;
    //     response->ack = 1;
    //     break;
    case DroneCommand::ARM:
        response->ack = arm_drone();
        break;
    case DroneCommand::DISARM:
        response->ack = disarm_drone();
        break;
    case DroneCommand::TAKEOFF:
        response->ack = takeoff_drone();
        break;
    case DroneCommand::LAND:
        response->ack = land_drone();
        break;
    default:
        response->ack = 0;
    }
}

// to be called in each loop
void Drone::check_command_requests()
{
    for (auto it = _drone_command_requests.begin(); it != _drone_command_requests.end();)
    {
        if (it->command_request.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            it->ack = it->command_request.get()->ack;
            it->timestamp_response = std::chrono::high_resolution_clock::now();

            // let the callback handle the ack
            (*it->callback)(it->ack);

            // Copy completed request to drone_command_responses vector
            CommandRequest done = *it;
            _drone_command_responses.push_back(done);
            it = _drone_command_requests.erase(it);
        }
        else
        {
            ++it;
        }
    }
}
