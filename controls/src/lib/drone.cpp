#include "drone.hpp"

using namespace std::chrono;

Drone::Drone(std::string drone_id, DroneSettings drone_settings) : _drone_id(drone_id), _drone_settings(drone_settings), 
                                                                   _px4_io(drone_id, drone_settings), _telem_values(_px4_io)
{
    Network::init();
    _network = std::make_shared<Network>(drone_id);
}

Drone::~Drone()
{
    _network = nullptr;
    Network::shutdown();
}

bool Drone::init(DroneState initial_state, uint8_t docking_slot, std::string connection_url) {
    if (_px4_io.connect_to_pixhawk(connection_url, 5) == false) {
        return false;
    }

    // Universal initialization
    while (_px4_io.undock() != 1) {}

    _telem_values.init_telem();

    _px4_io.subscribe_status_text([this](mavsdk::Telemetry::StatusText status_text) {
        if (status_text.text == "Kill-switch engaged")
            _kill_switch_engaged = true;
        else if (status_text.text == "Kill-switch disengaged")
            _kill_switch_engaged = false;
    });

    _px4_io.subscribe_armed([this](bool is_armed) {
        _armed = is_armed;
    });

    _px4_io.subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode flight_mode) {
        _flight_mode = flight_mode;
    });

    // init publishers
    _network->init_publisher<DRONE_STATUS>();
    _network->init_publisher<DRONE_DEBUG>();
    // subscribe drone status (also updates command clients)
    _network->subscribe<DRONE_STATUS>([&](const aviata::msg::DroneStatus::SharedPtr ds_rec) {
        // Don't process own ID
        if (_drone_id == ds_rec->drone_id) {
            return;
        }

        // // remove from swarm if leaving frame
        // if ( static_cast<DroneState>(ds_rec->drone_state) == UNDOCKING )
        // {
        //     auto it = _swarm.find(ds_rec->drone_id);
        //     _swarm.erase(it);
        //     // but keeps service client
        // }
        // else
        // {
            DroneStatus rec;
            rec.drone_id = ds_rec->drone_id;
            rec.drone_state = static_cast<DroneState>(ds_rec->drone_state);
            rec.docking_slot = ds_rec->docking_slot;
            rec.battery_percent = ds_rec->battery_percent;
            std::copy(std::begin(ds_rec->gps_position), std::end(ds_rec->gps_position), std::begin(rec.gps_position));
            rec.yaw = ds_rec->yaw;

            _swarm[rec.drone_id] = rec;
            _network->init_drone_command_client_if_needed(ds_rec->drone_id);
        // }
    });
    
    // start drone command service
    _network->init_drone_command_service([this](const aviata::srv::DroneCommand::Request::SharedPtr request,
                                               aviata::srv::DroneCommand::Response::SharedPtr response) {
        command_handler(request, response);
    });

    // State-specific initialization
    _drone_state = initial_state;

    if (_drone_state == DOCKED_FOLLOWER || _drone_state == DOCKED_LEADER) {
        _docking_slot = docking_slot;
        while (_px4_io.dock(_docking_slot, nullptr, 0) != 1) {} // TODO add missing_drones to init() and pass here
        std::cout << "Initial docking command sent successfully!" << std::endl;
    }

    switch (_drone_state) {
        case DOCKED_FOLLOWER:
            init_follower();
            break;
        case DOCKED_LEADER:
            init_leader();
            break;
        default:
            std::cout << "Invalid initial state: " << _drone_state << std::endl;
            return false;
    }

    return true;
}

void Drone::run()
{
    while (true) {
        int64_t current_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

        // Send status every second
        if (current_time - _last_status_publish_time >= 1000) {
            publish_drone_status();
            _last_status_publish_time = current_time;
        }

        switch (_drone_state)
        {
            case DOCKED_FOLLOWER:
                // TODO Make this a longer timeout and try to land instead of disarm
                if (current_time - _last_setpoint_msg_time > 250) {
                    if (_armed) {
                        if (_px4_io.disarm() == 1) {
                            _armed = false;
                            _px4_io.set_hold_mode(); // Take out of offboard mode to prevent annoying failsafe beeps
                        }
                    }
                }
                break;
            
            case DOCKED_LEADER:
                if (_need_to_enter_hold_mode) {
                    if (_px4_io.set_hold_mode() == 1) {
                        _flight_mode = mavsdk::Telemetry::FlightMode::Hold;
                        _need_to_enter_hold_mode = false;
                    }
                }
                break;
        }

        _px4_io.call_queued_mavsdk_callbacks();
        Network::spin_some(_network);
        _network->check_command_requests();
    }
}

void Drone::init_follower() {
    _last_setpoint_msg_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

    _network->subscribe<FOLLOWER_SETPOINT>([this](const aviata::msg::FollowerSetpoint::SharedPtr follower_setpoint) {
        if (!valid_leader_msg(follower_setpoint->leader_seq_num)) {
            return;
        }
        
        _last_setpoint_msg_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        mavlink_set_attitude_target_t attitude_target;
        std::copy(std::begin(follower_setpoint->q), std::end(follower_setpoint->q), std::begin(attitude_target.q));
        attitude_target.thrust = follower_setpoint->thrust;
        attitude_target.body_roll_rate = follower_setpoint->aviata_yaw_est; // body_roll_rate indicates aviata_yaw_est
        attitude_target.body_pitch_rate = (float) follower_setpoint->aviata_docking_slot; // body_pitch_rate indicates aviata_docking_slot
        _px4_io.set_attitude_target(attitude_target);

        if (_flight_mode != mavsdk::Telemetry::FlightMode::Offboard) {
            if (_px4_io.set_offboard_mode() == 1) {
                _flight_mode = mavsdk::Telemetry::FlightMode::Offboard;
            }
        }
        if (_flight_mode == mavsdk::Telemetry::FlightMode::Offboard && !_armed) {
            if (_px4_io.arm() == 1) {
                _armed = true;
            }
        }
    });
}

void Drone::init_leader() {
    _network->init_publisher<FOLLOWER_SETPOINT>();
    // send follower setpoints
    _px4_io.subscribe_attitude_target([this](const mavlink_attitude_target_t &attitude_target) {
        if (_armed && !_kill_switch_engaged) {
            aviata::msg::FollowerSetpoint follower_setpoint;
            std::copy(std::begin(attitude_target.q), std::end(attitude_target.q), std::begin(follower_setpoint.q));
            follower_setpoint.thrust = attitude_target.thrust;
            follower_setpoint.aviata_yaw_est = attitude_target.body_roll_rate; // body_roll_rate indicates aviata_yaw_est
            follower_setpoint.aviata_docking_slot = (uint8_t) attitude_target.body_pitch_rate; // body_pitch_rate indicates aviata_docking_slot
            follower_setpoint.leader_seq_num = _leader_seq_num;
            _network->publish<FOLLOWER_SETPOINT>(follower_setpoint);
        }
    });
}

void Drone::transition_leader_to_follower() {
    // deinit leader
    _network->deinit_publisher<FOLLOWER_SETPOINT>();
    _px4_io.unsubscribe_attitude_target();

    _drone_state = DOCKED_FOLLOWER;
    init_follower();
}

void Drone::transition_follower_to_leader() {
    // deinit follower
    _network->unsubscribe<FOLLOWER_SETPOINT>();

    _drone_state = DOCKED_LEADER;
    _need_to_enter_hold_mode = true;
    if (_px4_io.set_hold_mode() == 1) { // try to enter hold mode once, but if it fails it will continue to try in run()
        _flight_mode = mavsdk::Telemetry::FlightMode::Hold;
        _need_to_enter_hold_mode = false;
    }
    init_leader();
}

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

void Drone::publish_drone_status()
{
    aviata::msg::DroneStatus drone_status;
    drone_status.drone_id = _drone_id;
    drone_status.drone_state = _drone_state;
    drone_status.docking_slot = _docking_slot;
    std::copy(std::begin(_telem_values.dronePosition), std::end(_telem_values.dronePosition), std::begin(drone_status.gps_position));
    drone_status.yaw = _telem_values.droneEulerAngle.yaw_deg;
    drone_status.battery_percent = _telem_values.droneBattery.remaining_percent;
    _network->publish<DRONE_STATUS>(drone_status);
}

uint8_t Drone::arm_drone() // for drones in STANDBY / DOCKED_FOLLOWER
{
    if (_drone_state == STANDBY || _drone_state == DOCKED_FOLLOWER) {
        return _px4_io.arm_system();
    }
    _network->publish_drone_debug("Arm Drone FAILED: improper DroneState = " + _drone_state);
    return 0;
}

uint8_t Drone::arm_frame() // for DOCKED_LEADER (send arm_drone() to followers)
{
    if (_drone_state == DOCKED_LEADER)
    {
        // _leader_follower_armed = _swarm.size();
        // _leader_follower_disarmed = 0;
        for (const auto &[id, status] : _swarm)
        {
            // TODO publish FOLLOWER_ARM topic instead

            // send_drone_command(id, ARM, -1, "arm, " + _drone_id,
            //                    [this](uint8_t ack) {
            //                        if (ack == 1)
            //                            _leader_follower_armed--;
            //                        else
            //                        {
            //                            _network->publish_drone_debug("Arm Frame FAILED: Ack = " + ack);
            //                            // TODO: cancel tasks?
            //                            disarm_frame();
            //                        }
            //                        if (_leader_follower_armed == 0) //last drone armed
            //                            {
            //                                _px4_io.arm_system(); // arm itself last
            //                                _network->publish_drone_debug("Arm Frame SUCCESS");
            //                            }
            //                        _network->publish_drone_debug("Arm Frame in progress: awaiting ack for = " + _leader_follower_armed);

            //                    });
        }
        return 1;
    }
    _network->publish_drone_debug("Arm Frame FAILED: leader improper DroneState = " + _drone_state);
    return 0;
}

uint8_t Drone::disarm_drone() 
{
    if (_drone_state == STANDBY || _drone_state == DOCKED_FOLLOWER || _drone_state == NEEDS_SERVICE) {
        return _px4_io.disarm_system();
    }

    _network->publish_drone_debug("Disarm Drone FAILED: improper DroneState = " + _drone_state);
    return 0;
}

uint8_t Drone::disarm_frame() 
{
    if (_drone_state == DOCKED_LEADER)
    {
        // _leader_follower_disarmed = _swarm.size();
        // _leader_follower_armed = 0;
        for (const auto &[id, status] : _swarm)
        {
            // TODO publish FOLLOWER_DISARM topic instead

            // send_drone_command(id, DISARM, -1, "disarm, " + _drone_id,
            //                    [this](uint8_t ack) {
            //                        if (ack == 1)
            //                            _leader_follower_disarmed--;
            //                        else
            //                        {
            //                            _network->publish_drone_debug("Disarm Frame FAILED: Ack = " + ack);
            //                            // TODO: kill switch of some sort?
            //                        }
            //                        if (_leader_follower_disarmed == 0) //last drone armed
            //                            {
            //                                _px4_io.disarm_system(); // disarm itself last
            //                                _network->publish_drone_debug("Disarm Frame SUCCESS");
            //                            }
            //                        _network->publish_drone_debug("Disarm Frame in progress: awaiting ack for = " + _leader_follower_disarmed);

            //                    });
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
        // _drone_state = UNDOCKING; // TODO make function transition_follower_to_undocking()
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
        transition_follower_to_leader();
        _network->publish_drone_debug("Become Leader SUCCESS");

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

uint8_t Drone::become_follower()
{
    if (_drone_state == DOCKED_LEADER)
    {
        float highest_batt = 0;
        std::string highest_batt_drone = "";
        for (const auto& [id, status] : _swarm)
        {
            // determine drone to become leader
            if (status.drone_state == DOCKED_FOLLOWER && status.battery_percent > highest_batt)
            {
                highest_batt = status.battery_percent;
                highest_batt_drone = id;
            }
        }

        if (highest_batt_drone == "") {
            _network->publish_drone_debug("Become Follower FAILED: Not aware of any other docked drones.");
            return 0;
        }
        _network->publish_drone_debug("Identified new leader: drone_id = " + highest_batt_drone);
        
        _network->send_drone_command(highest_batt_drone, BECOME_LEADER, _leader_seq_num, "become new leader, " + _drone_id,
        [this](uint8_t ack) {
            if (ack == 1) {
                transition_leader_to_follower();
                _network->publish_drone_debug("Become Follower SUCCESS");
            } else {
                // Note that BECOME_FOLLOWER will still respond with an ack=1 in this scenario.
                _network->publish_drone_debug("Become Follower FAILED: Found suitable successor (and sent ack), but successor rejected the request.");
            }
        });
        return 1;
    }

    _network->publish_drone_debug("Become Follower FAILED: improper DroneState = " + _drone_state);
    return 0;
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
    case DroneCommand::REQUEST_NEW_LEADER:
        response->ack = become_follower();
        break;
    case DroneCommand::BECOME_LEADER:
        response->ack = become_leader(request->param);
        break;
    // case DroneCommand::LISTEN_NEW_LEADER:
    //     _leader_seq_num_next = request->param;
    //     response->ack = 1;
    //     break;
    default:
        response->ack = 0;
    }
}
