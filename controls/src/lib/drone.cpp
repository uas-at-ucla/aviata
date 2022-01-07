#include "drone.hpp"
#include "fastdds_config.hpp"

using namespace std::chrono;
using namespace mavsdk;
using std::this_thread::sleep_for;

Drone::Drone(std::string drone_id, DroneSettings drone_settings, Target t) : 
    _drone_id(drone_id), _drone_settings(drone_settings), 
    _px4_io(drone_id, drone_settings), _telem_values(_px4_io), camera(t), 
    image_analyzer(), m_target_info(t), m_north(0), m_east(0), m_down(-5), m_yaw(0), 
    _follower_setpoint_timeout(drone_settings.sim ? 2000 : 1000),
    _docking_slot_is_occupied(drone_settings.n_docking_slots, false)
{
    Network::init();
    _network = std::make_shared<Network>(drone_id);
}

Drone::~Drone()
{
    _network = nullptr;
    Network::shutdown();
}

bool Drone::init(DroneState initial_state, int8_t docking_slot, std::string connection_url) {
    if (_px4_io.connect_to_pixhawk(connection_url, 5) == false) {
        return false;
    }

    // Universal initialization
    _telem_values.init_telem();

    _px4_io.subscribe_status_text([this](mavsdk::Telemetry::StatusText status_text) {
        if (status_text.text == "Kill-switch engaged") {
            _kill_switch_engaged = true;
            if (_drone_state == DOCKED_LEADER) {
                aviata::msg::Empty disarm;
                _network->publish<FOLLOWER_DISARM>(disarm);
            }
        } else if (status_text.text == "Kill-switch disengaged") {
            _kill_switch_engaged = false;
        }
    });

    _px4_io.subscribe_armed([this](bool is_armed) {
        if (_drone_state == DOCKED_LEADER && _armed && !is_armed) {
            // Disarm followers if leader was disarmed
            aviata::msg::Empty disarm;
            _network->publish<FOLLOWER_DISARM>(disarm);
        }
        _armed = is_armed;
    });

    _px4_io.subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode flight_mode) {
        _flight_mode = flight_mode;
    });

    // init publishers
    _network->init_publisher<FRAME_DISARM>(); // TODO Temporary failsafe option until we have something more robust
    _network->init_publisher<DRONE_STATUS>();
    _network->init_publisher<DRONE_DEBUG>();
    // subscribe drone status (also updates command clients)
    _network->subscribe<DRONE_STATUS>([this](const aviata::msg::DroneStatus::SharedPtr ds_rec) {
        // Don't process own ID
        if (ds_rec->drone_id == _drone_id) {
            return;
        }

        DroneStatus rec;
        rec.drone_id = ds_rec->drone_id;
        rec.ip_address = ds_rec->ip_address;
        rec.mavlink_sys_id = ds_rec->mavlink_sys_id;
        rec.drone_state = static_cast<DroneState>(ds_rec->drone_state);
        rec.docking_slot = ds_rec->docking_slot;
        rec.battery_percent = ds_rec->battery_percent;
        std::copy(std::begin(ds_rec->gps_position), std::end(ds_rec->gps_position), std::begin(rec.gps_position));
        rec.yaw = ds_rec->yaw;

        receive_drone_status(rec);
    });
    
    // start drone command service
    _network->init_drone_command_service([this](const aviata::srv::DroneCommand::Request::SharedPtr request,
                                               aviata::srv::DroneCommand::Response::SharedPtr response) {
        command_handler(request, response);
    });

    while (_px4_io.set_mixer_undocked() != 1) {}
    _network->publish_drone_debug("Initialized PX4 mixer to undocked.");

    // State-specific initialization
    _drone_state = initial_state;
    _docking_slot = docking_slot;

    switch (_drone_state) {
        case DOCKED_FOLLOWER:
            init_docked();
            init_follower();
            break;
        case DOCKED_LEADER:
            init_docked();
            init_leader();
            break;
        case STANDBY:
            init_standby();
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
                if (current_time - _last_setpoint_msg_time > _follower_setpoint_timeout) {
                    if (_armed) {
                        aviata::msg::Empty disarm;
                        _network->publish<FRAME_DISARM>(disarm); // TODO Temporary failsafe option. Eventually, some sort of landing failsafe might be better than disarming. In this particular case, we have to plan for possible future network outages.
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
            case ARRIVING:
                if (fly_to_central_target()) {
                    initiate_docking(STAGE_1);
                }
                break;
            case DOCKING_STAGE_1:
            {
                int stage_1_result = do_docking(STAGE_1);
                switch (stage_1_result){
                    case DOCKING_SUCCESS:
                        initiate_docking(STAGE_2);
                        break;
                    case DOCKING_FAILURE:
                        land_drone();
                        break;
                    default:
                        break;
                }
                break;
            }
            case DOCKING_STAGE_2:
                int stage_2_result = do_docking(STAGE_2);
                switch(stage_2_result){
                    case DOCKING_SUCCESS:
                        transition_docking_to_docked();
                        break;
                    case DOCKING_FAILURE:
                        land_drone();
                        break;
                    case RESTART_DOCKING:
                        initiate_docking(STAGE_1);
                        break;
                    default:
                        break;
                }
                break;
        }

        _px4_io.call_queued_mavsdk_callbacks();
        Network::spin_some(_network);
        _network->check_command_requests();
    }
}

// in update and status subscriptions (do the status trick), update frame (make global frame variable), if frame changes and missing drones not too much, set config, or set docked if flag set

void Drone::init_docked() {
    // publish position it is docked
    aviata::msg::DockingInfo docked;
    docked.drone_id = _drone_id;
    docked.docking_slot = _docking_slot;
    docked.arriving = true;
    _network->publish<DOCKING_INFO>(docked);

    _docking_slot_is_occupied[_docking_slot] = true;

    std::vector<uint8_t> missing_drones = generate_missing_drones_list();
    if (missing_drones.size() <= _drone_settings.max_missing_drones) {
        if (_px4_io.set_mixer_docked(_docking_slot, missing_drones.data(), missing_drones.size()) == 1) {
            _network->publish_drone_debug("MAVLink docking command sent successfully!");
        } else {
            _network->publish_drone_debug("MAVLink docking command failed, and that's pretty bad becuase there's no failsafe here!");
        }
    } else {
        _network->publish_drone_debug("Not ready for docked flight - Need to discover more drones.");
        _need_to_discover_more_drones = true;
    }

    _network->subscribe<DOCKING_INFO>([this](const aviata::msg::DockingInfo::SharedPtr docking_update) {
        // Don't process own ID
        if (docking_update->drone_id == _drone_id) {
            return;
        }
        
        DroneStatus rec;
        if (_swarm.find(docking_update->drone_id) != _swarm.end()) {
            DroneStatus rec = _swarm[docking_update->drone_id]; // update status of relevant drone
        } else {
            rec.drone_id = docking_update->drone_id;
        }
        rec.docking_slot = docking_update->docking_slot;
        if (docking_update->arriving) {
            rec.drone_state = DOCKED_FOLLOWER;
        } else {
            rec.drone_state = UNDOCKING;
        }
        
        receive_drone_status(rec);
    });
}

void Drone::init_follower() {
    _last_setpoint_msg_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

    //subscribe follower stuff
    _network->subscribe<FOLLOWER_ARM>([this](const aviata::msg::Empty::SharedPtr follower_arm) {    arm_drone();    });
    _network->subscribe<FOLLOWER_DISARM>([this](const aviata::msg::Empty::SharedPtr follower_disarm) {  disarm_drone();    });
    _network->subscribe<FOLLOWER_SETPOINT>([this](const aviata::msg::FollowerSetpoint::SharedPtr follower_setpoint) {
        if (!valid_leader_msg(follower_setpoint->leader_seq_num)) {
            return;
        }
        
        _last_setpoint_msg_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        mavlink_set_attitude_target_t attitude_target;
        attitude_target.q[0] = follower_setpoint->q[0];
        attitude_target.q[1] = follower_setpoint->q[1];
        attitude_target.q[2] = follower_setpoint->q[2];
        attitude_target.q[3] = follower_setpoint->q[3];
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
    _network->subscribe<REFERENCE_ATTITUDE>([this](const aviata::msg::ReferenceAttitude::SharedPtr reference_attitude) {  });
}

void Drone::init_leader() {
    // init leader publishers
    _network->init_publisher<FOLLOWER_ARM>();
    _network->init_publisher<FOLLOWER_DISARM>();

    // subscribe leader topics
    _network->subscribe<FRAME_ARM>([this](const aviata::msg::Empty::SharedPtr frame_arm) {    arm_frame();    });
    _network->subscribe<FRAME_DISARM>([this](const aviata::msg::Empty::SharedPtr frame_disarm) {    disarm_frame();    });
    _network->subscribe<FRAME_TAKEOFF>([this](const aviata::msg::Empty::SharedPtr frame_takeoff) {    takeoff_frame();    });
    _network->subscribe<FRAME_LAND>([this](const aviata::msg::Empty::SharedPtr frame_land) {    land_frame();    });    
    _network->subscribe<FRAME_SETPOINT>([this](const aviata::msg::FrameSetpoint::SharedPtr frame_setpoint) {    /* TODO */    });    

    // send follower setpoints
    _network->init_publisher<FOLLOWER_SETPOINT>();
    _px4_io.subscribe_attitude_target([this](const mavlink_attitude_target_t &attitude_target) {
        if (_armed && !_kill_switch_engaged) {
            aviata::msg::FollowerSetpoint follower_setpoint;
            follower_setpoint.q[0] = attitude_target.q[0];
            follower_setpoint.q[1] = attitude_target.q[1];
            follower_setpoint.q[2] = attitude_target.q[2];
            follower_setpoint.q[3] = attitude_target.q[3];
            follower_setpoint.thrust = attitude_target.thrust;
            follower_setpoint.aviata_yaw_est = attitude_target.body_roll_rate; // body_roll_rate indicates aviata_yaw_est
            follower_setpoint.aviata_docking_slot = (uint8_t) (attitude_target.body_pitch_rate+0.5f); // body_pitch_rate indicates aviata_docking_slot
            follower_setpoint.leader_seq_num = _leader_seq_num;
            _network->publish<FOLLOWER_SETPOINT>(follower_setpoint);
        }
    });
    _network->init_publisher<REFERENCE_ATTITUDE>();
}

void Drone::init_standby() {
    // wait for docking command

    // TODO uncomment to test docking immediately
    // eventually have callback here to listen for docking command
    // transition_standby_to_docking();
}

void Drone::transition_standby_to_docking() {
    arm_drone();
    takeoff_drone();

    // wait until we're done taking off before proceeding
    Telemetry::LandedState curr_state = _px4_io.telemetry_ptr()->landed_state();
    //TODO update instances of _px4_io.telemetry_ptr()->subscribe... to use mavsdk_callback_manager.subscribe_mavsdk_callback() in px4_io.cpp.
    _px4_io.telemetry_ptr()->subscribe_landed_state([this, &curr_state](Telemetry::LandedState landed_state) {
        if (curr_state != landed_state) {
            curr_state = landed_state;
        }
    });

    while (curr_state != Telemetry::LandedState::InAir)
    {
        sleep_for(seconds(1));
    }

    std::cout << "Finished taking off" << std::endl;

    _px4_io.telemetry_ptr()->subscribe_landed_state(nullptr);

    Offboard::VelocityBodyYawspeed initial_setpoint{};
    _px4_io.offboard_ptr()->set_velocity_body(initial_setpoint);
    
    if (_px4_io.set_offboard_mode() == 1) {
        _flight_mode = mavsdk::Telemetry::FlightMode::Offboard;
        std::cout << "Offboard successfully started for docking drone" << std::endl;
    }
    _drone_state = ARRIVING;
}

void Drone::transition_leader_to_follower() {
    // deinit leader
    _network->unsubscribe<FRAME_ARM>();
    _network->unsubscribe<FRAME_DISARM>();
    _network->unsubscribe<FRAME_TAKEOFF>();
    _network->unsubscribe<FRAME_LAND>();
    _network->unsubscribe<FRAME_SETPOINT>();

    _network->deinit_publisher<FOLLOWER_ARM>();
    _network->deinit_publisher<FOLLOWER_DISARM>();
    _network->deinit_publisher<FOLLOWER_SETPOINT>();
    _px4_io.unsubscribe_attitude_target();
    _network->deinit_publisher<REFERENCE_ATTITUDE>();

    _drone_state = DOCKED_FOLLOWER;
    init_follower();
}

void Drone::transition_docking_to_docked() {
    _drone_state = DOCKED_FOLLOWER;
    init_docked();
    init_follower();
}

void Drone::transition_follower_to_leader() {
    // deinit follower
    _network->unsubscribe<FOLLOWER_ARM>();
    _network->unsubscribe<FOLLOWER_DISARM>();
    _network->unsubscribe<FOLLOWER_SETPOINT>();
    _network->unsubscribe<REFERENCE_ATTITUDE>();

    _drone_state = DOCKED_LEADER;
    _need_to_enter_hold_mode = true;
    if (_px4_io.set_hold_mode() == 1) { // try to enter hold mode once, but if it fails it will continue to try in run()
        _flight_mode = mavsdk::Telemetry::FlightMode::Hold;
        _need_to_enter_hold_mode = false;
    }
    init_leader();
}

std::vector<uint8_t> Drone::generate_missing_drones_list() {
    std::vector<uint8_t> missing_drones;
    for (uint8_t i = 0; i < _docking_slot_is_occupied.size(); i++) {
        if (!_docking_slot_is_occupied[i])
        {
            missing_drones.push_back(i);
        }
    }
    return missing_drones;
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
    drone_status.ip_address = mesh_ip_address;
    drone_status.mavlink_sys_id = _px4_io.drone_system_id();
    drone_status.drone_state = _drone_state;
    drone_status.docking_slot = _docking_slot;
    std::copy(std::begin(_telem_values.dronePosition), std::end(_telem_values.dronePosition), std::begin(drone_status.gps_position));
    drone_status.yaw = _telem_values.droneEulerAngle.yaw_deg;
    drone_status.battery_percent = _telem_values.droneBattery.remaining_percent;
    _network->publish<DRONE_STATUS>(drone_status);
}

void Drone::receive_drone_status(const DroneStatus& rec) {
    _swarm[rec.drone_id] = rec;
    _network->init_drone_command_client_if_needed(rec.drone_id);

    if (rec.docking_slot >= 0) {
        bool docking_slot_is_occupied = rec.drone_state == DOCKED_FOLLOWER || rec.drone_state == DOCKED_LEADER;
        if (!docking_slot_is_occupied) { // check if anyone else occupies the docking slot
            for (const auto& [id, status] : _swarm) {
                if (status.docking_slot == rec.docking_slot && (status.drone_state == DOCKED_FOLLOWER || status.drone_state == DOCKED_LEADER)) {
                    docking_slot_is_occupied = true;
                    break;
                }
            }
        }

        if (docking_slot_is_occupied != _docking_slot_is_occupied[rec.docking_slot]) { // if occupied state changed
            _docking_slot_is_occupied[rec.docking_slot] = docking_slot_is_occupied;
            std::vector<uint8_t> missing_drones = generate_missing_drones_list();

            if (_need_to_discover_more_drones) {
                if (missing_drones.size() <= _drone_settings.max_missing_drones) {
                    if (_px4_io.set_mixer_docked(_docking_slot, missing_drones.data(), missing_drones.size()) == 1) {
                        _network->publish_drone_debug("MAVLink docking command sent successfully!");
                    } else {
                        _network->publish_drone_debug("MAVLink docking command failed, and that's pretty bad becuase there's no failsafe here!");
                    }
                    _need_to_discover_more_drones = false;
                }
            } else {
                if (missing_drones.size() <= _drone_settings.max_missing_drones) {
                    _px4_io.set_mixer_configuration(missing_drones.data(), missing_drones.size());
                } else {
                    _network->publish_drone_debug("FATAL: Too many missing drones!");
                    aviata::msg::Empty disarm;
                    _network->publish<FRAME_DISARM>(disarm); // TODO Temporary failsafe option. Eventually, some sort of landing failsafe might be better than disarming.
                }
            }
        }
    }
}

uint8_t Drone::arm_drone() // for drones in STANDBY / DOCKED_FOLLOWER
{
    if (_drone_state == STANDBY || _drone_state == DOCKED_FOLLOWER) {
        if (_need_to_discover_more_drones) {
            _network->publish_drone_debug("Arm Drone FAILED: not enough docked drones discovered");
            return 0;
        }
        return _px4_io.wait_for_arm();
    }
    _network->publish_drone_debug("Arm Drone FAILED: improper DroneState = " + std::to_string(_drone_state));
    return 0;
}

uint8_t Drone::arm_frame() // for DOCKED_LEADER (send arm_drone() to followers)
{
    if (_drone_state == DOCKED_LEADER)
    {
        aviata::msg::Empty arm;
        _network->publish<FOLLOWER_ARM>(arm);
        _network->publish_drone_debug("ARM FRAME");        
        // TODO: check if all drones on frame have armed

        // _leader_follower_armed = _swarm.size();
        // _leader_follower_disarmed = 0;
        // for (const auto &[id, status] : _swarm)
        // {
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
            //                                _px4_io.wait_for_arm(); // arm itself last
            //                                _network->publish_drone_debug("Arm Frame SUCCESS");
            //                            }
            //                        _network->publish_drone_debug("Arm Frame in progress: awaiting ack for = " + _leader_follower_armed);

            //                    });
        // }
        return 1;
    }
    _network->publish_drone_debug("Arm Frame FAILED: leader improper DroneState = " + std::to_string(_drone_state));
    return 0;
}

uint8_t Drone::disarm_drone() 
{
    if (_drone_state == STANDBY || _drone_state == DOCKED_FOLLOWER || _drone_state == NEEDS_SERVICE) {
        // return _px4_io.wait_for_disarm(); // waits for the drone to not be in the air
        return _px4_io.disarm();
    }

    _network->publish_drone_debug("Disarm Drone FAILED: improper DroneState = " + std::to_string(_drone_state));
    return 0;
}

uint8_t Drone::disarm_frame() 
{
    if (_drone_state == DOCKED_LEADER)
    {
        aviata::msg::Empty disarm;
        _network->publish<FOLLOWER_DISARM>(disarm);
        _network->publish_drone_debug("DISARM FRAME");
        // TODO: check if all drones on frame have disarmed
        
        // _leader_follower_disarmed = _swarm.size();
        // _leader_follower_armed = 0;
        // for (const auto &[id, status] : _swarm)
        // {
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
            //                                _px4_io.wait_for_disarm(); // disarm itself last
            //                                _network->publish_drone_debug("Disarm Frame SUCCESS");
            //                            }
            //                        _network->publish_drone_debug("Disarm Frame in progress: awaiting ack for = " + _leader_follower_disarmed);

            //                    });
        // }
        return 1;
    }
    _network->publish_drone_debug("Disarm Frame FAILED: leader improper DroneState = " + std::to_string(_drone_state));
    return 0;
}

uint8_t Drone::takeoff_drone() // for drones in STANDBY or leader
{
    if (_drone_state == STANDBY)
        return _px4_io.takeoff_system();
    else if (_drone_state == DOCKED_LEADER)
        return takeoff_frame();

    _network->publish_drone_debug("Takeoff Drone FAILED: improper DroneState = " + std::to_string(_drone_state));
    return 0;
}

uint8_t Drone::takeoff_frame() // for DOCKED_LEADER (send attitude and thrust to followers)
{
    if (_drone_state == DOCKED_LEADER)
    {
        _network->publish_drone_debug("Frame taking off");
        return _px4_io.takeoff_system();
    }
    _network->publish_drone_debug("Takeoff Frame FAILED: improper DroneState = " + std::to_string(_drone_state));
    return 0;
}

uint8_t Drone::land_drone() // for any undocked drone or leader
{
    if (_drone_state == STANDBY || _drone_state == NEEDS_SERVICE)
        return _px4_io.land_system();
     else if (_drone_state == DOCKED_LEADER)
         return land_frame();

    _network->publish_drone_debug("Land Drone FAILED: improper DroneState = " + std::to_string(_drone_state));
    return 0;
}

uint8_t Drone::land_frame() // for DOCKED_LEADER (send attitude and thrust to followers)
{
    if (_drone_state == DOCKED_LEADER)
    {
        _network->publish_drone_debug("Frame landing");
        return _px4_io.land_system();
    }
    _network->publish_drone_debug("Land Frame FAILED: improper DroneState = " + std::to_string(_drone_state));
    return 0;
}

uint8_t Drone::init_state(DroneState initial_state, int8_t docking_slot) {
    if (_drone_state == STANDBY) {
        DroneState prev_drone_state = _drone_state;
        int8_t prev_docking_slot = _docking_slot;
        _drone_state = initial_state;
        _docking_slot = docking_slot;

        switch (initial_state) {
            case DOCKED_FOLLOWER:
                init_docked();
                init_follower();
                break;
            case DOCKED_LEADER:
                init_docked();
                init_leader();
                break;
            case STANDBY:
                break;
            default:
                _network->publish_drone_debug("Invalid initial state: " + std::to_string(initial_state));
                _drone_state = prev_drone_state;
                _docking_slot = prev_docking_slot;
                return 0;
        }
        return 1;
    }
    _network->publish_drone_debug("Init State FAILED: improper DroneState = " + std::to_string(_drone_state));
    return 0;
}

uint8_t Drone::undock()
{
    if (_drone_state == DOCKED_FOLLOWER)
    {
        _drone_state = UNDOCKING; // TODO make function transition_follower_to_undocking()
        _px4_io.set_mixer_undocked();
        _docking_slot_is_occupied[_docking_slot] = false;
        
        // TODO move DockingInfo publish to transition_follower_to_undocking()
        aviata::msg::DockingInfo undocked;
        undocked.drone_id = _drone_id;
        undocked.docking_slot = _docking_slot;
        undocked.arriving = false;
        _network->publish<DOCKING_INFO>(undocked);
        _network->unsubscribe<DOCKING_INFO>(); 
        return 1;
    }
    return 0;
}

uint8_t Drone::dock(uint8_t docking_slot) {
    if (_drone_state == STANDBY) {
        _docking_slot = docking_slot;
        transition_standby_to_docking();
        return 1;
    }

    _network->publish_drone_debug("Begin Docking FAILED: improper DroneState = " + std::to_string(_drone_state));
    return 0;
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
        return 1;
    }

    _network->publish_drone_debug("Become Leader FAILED: improper DroneState = " + std::to_string(_drone_state));
    return 0;
}

uint8_t Drone::become_follower(int8_t new_leader_docking_slot)
{
    if (_drone_state == DOCKED_LEADER)
    {
        std::string new_leader_drone = "";

        if (new_leader_docking_slot >= 0) {
            for (const auto& [id, status] : _swarm) {
                if (status.drone_state == DOCKED_FOLLOWER && status.docking_slot == new_leader_docking_slot) {
                    new_leader_drone = id;
                    break;
                }
            }
        } else {
            // If new leader not specified, find drone with highest battery
            float highest_batt = -1;
            for (const auto& [id, status] : _swarm)
            {
                // determine drone to become leader
                if (status.drone_state == DOCKED_FOLLOWER && status.battery_percent > highest_batt)
                {
                    highest_batt = status.battery_percent;
                    new_leader_drone = id;
                }
            }
        }

        if (new_leader_drone == "") {
            _network->publish_drone_debug("Become Follower FAILED: Not aware of any other docked drones.");
            return 0;
        }
        _network->publish_drone_debug("Identified new leader: drone_id = " + new_leader_drone);
        
        _network->send_drone_command(new_leader_drone, BECOME_LEADER, _leader_seq_num, -1, "become new leader, " + _drone_id,
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

    _network->publish_drone_debug("Become Follower FAILED: improper DroneState = " + std::to_string(_drone_state));
    return 0;
}

/**
 * Attempts a particular stage of the docking process
 * Previously used to have 1 function per stage, but code got too long/duplicated
 * Easier to have 1 function with conditional separation of logic for small parts that are unique to a stage
 * 
 * @param target_id is the target to dock to
 * @param stage is which stage we're in
 * @return true if successful
 * */
DockingIterationResult Drone::do_docking(int stage)
{
    // --- simulation-specific code

    float swarm_alt = 0.0f;
    float swarm_lat = 0.0f;
    float swarm_lon = 0.0f;
    int swarm_size = 0;
    for (const auto& [id, status] : _swarm) //Calculate average location of swarm (approximates central target location)
    {
        if (status.drone_state == DOCKED_FOLLOWER || status.drone_state == DOCKED_LEADER)
        {
            swarm_size++;
            swarm_alt += status.gps_position[2];
            swarm_lat += status.gps_position[0];
            swarm_lon += status.gps_position[1];
        }
    }
    if (swarm_size == 0) {
        return ITERATION_SUCCESS; // need to wait for valid coordinates
    }
    swarm_alt /= swarm_size;
    swarm_lat /= swarm_size;
    swarm_lon /= swarm_size;

    geometry::CoordinateTransformation::GlobalCoordinate gc_2;
    gc_2.latitude_deg = swarm_lat;
    gc_2.longitude_deg = swarm_lon;
    geometry::CoordinateTransformation::LocalCoordinate lc = docking_status.ct->local_from_global(gc_2);
    float north = (float) lc.north_m;
    float east = (float) lc.east_m;

    Target t; 
    t.lat = north; 
    t.lon = east;
    t.alt = swarm_alt;
    t.yaw = 0.0; // yaw is 0 for now, will need to calculate
    camera.update_target(t);

    // --- end simulation-specific code


    std::string log_tag = "Stage " + std::to_string(stage);
    uint8_t target_id = _docking_slot;
    log(log_tag, std::to_string(target_id));

    // Initialize vars
    Mat img;
    Velocities velocities; 
    Errors errs;
    Offboard::VelocityBodyYawspeed change{};
    std::string tags_detected = "";

    img = camera.update_current_image(m_east, m_north, m_down * -1.0, m_yaw, stage == STAGE_1 ? 0 : target_id);
    bool is_tag_detected = image_analyzer.processImage(img, stage == STAGE_1 ? 0 : target_id, docking_status.tags, errs); // central target has id = 0

    if (is_tag_detected)
    {
        if (stage == STAGE_1) offset_errors(errs, target_id); // adjusts errors for stage 1 to stage 2 transition
        else errs.alt -= 0.35;
        velocities = docking_status.pid.getVelocities(errs.x, errs.y, errs.alt, errs.yaw, 1.5);
        log(log_tag, "Apriltag found! errors: x=" + std::to_string(errs.x) + " y=" + std::to_string(errs.y) + " z=" + std::to_string(errs.alt) + " yaw=" + std::to_string(errs.yaw)
                        + " velocities: x=" + std::to_string(velocities.x) + " y=" + std::to_string(velocities.y) + " z=" + std::to_string(velocities.alt) + 
                        " yaw_speed=" + std::to_string(velocities.yaw) + " successful frames: " + std::to_string(docking_status.successful_frames));

        // Check if we're centered enough to consider transitioning
        float tolerance = stage == STAGE_1 ? STAGE_1_TOLERANCE : STAGE_2_TOLERANCE;
        if (errs.x < tolerance && errs.x > -1.0 * tolerance && 
            errs.y < tolerance && errs.y > -1.0 * tolerance &&
            errs.alt < tolerance && errs.alt > -1.0 * tolerance && 
            errs.yaw < 5.0 && errs.yaw > -5.0)
        {
            docking_status.successful_frames++;
        }
        else
        {
            docking_status.successful_frames = 0;
        }

        // Transition if we've been centered for over 1 second (stage 1) or docking detected (stage 2)
        if ((stage == STAGE_1 && docking_status.successful_frames > 10) || (stage == STAGE_2 && docking_detector.is_docked())) {
            return DOCKING_SUCCESS;
        }

        // Dynamically adjust z velocity to only descend when close enough to being centered
        // double safe_view = 2 * (errs.alt - velocities.alt * 0.1) * tan(to_radians(CAMERA_FOV_VERTICAL / 2));
        // if (abs(errs.x) >= safe_view || abs(errs.y) >= safe_view) {
        //     velocities.alt = -0.1;
        //     velocities.yaw = 0.0;
        // }
        float frame_size_meters_x = 640 * errs.tag_pixel_ratio;
        float frame_size_meters_y = 480 * errs.tag_pixel_ratio;
        if (abs(errs.x) >= 0.5 * (frame_size_meters_x / 2) || abs(errs.y) >= 0.5 * (frame_size_meters_y / 2)) {
            velocities.alt = -0.1;
            velocities.yaw = 0.0;
        }
        
        change.forward_m_s = velocities.y;
        change.right_m_s = velocities.x;
        change.down_m_s = velocities.alt;
        change.yawspeed_deg_s = velocities.yaw;
        docking_status.failed_frames = 0;
    } else {
        log(log_tag, "Failed to find Apriltag, number missed frames: " + std::to_string(docking_status.failed_frames)); 

        if (docking_status.failed_frames > 10) {
            change.forward_m_s = 0.0;
            change.right_m_s = 0.0;
            change.down_m_s = 0.0;
            change.yawspeed_deg_s = 0.0;
        } else {
            change.forward_m_s = velocities.y * 0.8;
            change.right_m_s = velocities.x * 0.8;
            change.down_m_s = velocities.alt * 0.8;
            change.yawspeed_deg_s = 0.0;
        }
        docking_status.failed_frames++;
    }

    if (m_down > 5.0) {
        log(log_tag, "Emergency land, too high! Altitude: " + std::to_string(m_down), true);
        land_drone(); // emergency land, too high
        return DOCKING_FAILURE;
    }

    _px4_io.offboard_ptr()->set_velocity_body(change);

    return ITERATION_SUCCESS;
}

// Safely fly to central target
// Note that this DOES NOT include takeoff, needs to be handled separately
bool Drone::fly_to_central_target() {
    float swarm_alt = 0.0f;
    float swarm_lat = 0.0f;
    float swarm_lon = 0.0f;
    int swarm_size = 0;
    for (const auto& [id, status] : _swarm) //Calculate average location of swarm (approximates central target location)
    {
        if (status.drone_state == DOCKED_FOLLOWER || status.drone_state == DOCKED_LEADER)
        {
            swarm_size++;
            swarm_alt += status.gps_position[2];
            swarm_lat += status.gps_position[0];
            swarm_lon += status.gps_position[1];
        }
    }
    if (swarm_size == 0) {
        return false; // need to wait for valid coordinates
    }
    swarm_alt /= swarm_size;
    swarm_lat /= swarm_size;
    swarm_lon /= swarm_size;

    Telemetry::Position m_gps_position = _px4_io.telemetry_ptr()->position();

    if (swarm_alt >= m_gps_position.absolute_altitude_m - DOCKING_HEIGHT_PRECONDITION){ // Drone too low, needs to fly up
        Offboard::VelocityBodyYawspeed change{};
        change.down_m_s = -1.0f;
        _px4_io.offboard_ptr()->set_velocity_body(change);
    } else if (swarm_lat - m_gps_position.latitude_deg <= PRECONDITION_TOLERANCE && swarm_lat - m_gps_position.latitude_deg >= -1.0 * PRECONDITION_TOLERANCE &&
               swarm_lon - m_gps_position.longitude_deg <= PRECONDITION_TOLERANCE && swarm_lon - m_gps_position.longitude_deg >= -1.0 * PRECONDITION_TOLERANCE){
        // don't fly to target if already above it
        Offboard::VelocityBodyYawspeed change{};
        change.down_m_s = 0.0f;
        _px4_io.offboard_ptr()->set_velocity_body(change);
        return true;
    } else { // Drone high enough, needs to fly to central target GPS location
        _px4_io.goto_gps_position(swarm_lat, swarm_lon, swarm_alt, 0.0);
    }
    return false;
}

// callback to support docking simulation
void Drone::set_position(float north, float east, float down)
{
    m_north = north;
    m_east = east;
    m_down = down;
}

// callback to support docking simulation
void Drone::set_yaw(float yaw)
{
    m_yaw = yaw;
}

/**
 * Sets up docking status for each stage of docking 
 * */
void Drone::initiate_docking(int stage) {
    // set up telemetry updates; happens in background through callbacks (required for docking simulation)
    const Telemetry::Result set_rate_result_p = _px4_io.telemetry_ptr()->set_rate_position_velocity_ned(20);
    const Telemetry::Result set_rate_result_a = _px4_io.telemetry_ptr()->set_rate_attitude(20);
    if (set_rate_result_a != Telemetry::Result::Success || set_rate_result_p != Telemetry::Result::Success)
    {
        std::cout << "Setting attitude rate possibly failed:" << set_rate_result_a << '\n';
        std::cout << "Setting position rate possibly failed:" << set_rate_result_p << '\n';
    }

    // clear any existing callbacks, then subscribe to getting periodic telemetry updates with lambda function
    // (required for docking simulation)
    _px4_io.telemetry_ptr()->subscribe_position_velocity_ned(nullptr);
    _px4_io.telemetry_ptr()->subscribe_attitude_euler(nullptr);

    Telemetry::Position current_gps_pos = _px4_io.telemetry_ptr()->position();
    geometry::CoordinateTransformation::GlobalCoordinate gc;
    gc.latitude_deg = current_gps_pos.latitude_deg;
    gc.longitude_deg = current_gps_pos.longitude_deg;
    docking_status.ct = new geometry::CoordinateTransformation(gc);

    _px4_io.telemetry_ptr()->subscribe_position([this](Telemetry::Position p) {
        geometry::CoordinateTransformation::GlobalCoordinate gc_2;
        gc_2.latitude_deg = p.latitude_deg;
        gc_2.longitude_deg = p.longitude_deg;
        geometry::CoordinateTransformation::LocalCoordinate lc = docking_status.ct->local_from_global(gc_2);
        set_position((float) lc.north_m, (float) lc.east_m, p.absolute_altitude_m * -1.0);
    });
    _px4_io.telemetry_ptr()->subscribe_attitude_euler([this](Telemetry::EulerAngle e) {
        set_yaw(e.yaw_deg);
    });

    _drone_state = stage == STAGE_1 ? DOCKING_STAGE_1 : DOCKING_STAGE_2;
    PIDController temp;
    docking_status.pid = temp;
    docking_status.tags = "";
    docking_status.failed_frames = 0;
    docking_status.successful_frames = 0;
    docking_status.prev_iter_detection = false;
    docking_status.has_centered = false;
    docking_status.docking_attempts = 0;
}

//Utility method to adjust errors for stage 1
void Drone::offset_errors(Errors &errs, int id)
{
    float target_offset = id <= 3 ? abs(id - 3) * 45 : (11 - id) * 45;
    float x = errs.x + BOOM_LENGTH / 2 * cos(to_radians(target_offset - errs.yaw));
    float y = errs.y + BOOM_LENGTH / 2 * sin(to_radians(target_offset - errs.yaw));
    errs.alt -= ALTITUDE_DISP;
    errs.x = x;
    errs.y = y;
    float yaw = errs.yaw + 90 - ((id - 1) * 45);
    errs.yaw = yaw;
}

// @brief to be used as callback for service server
void Drone::command_handler(const aviata::srv::DroneCommand::Request::SharedPtr request,
                            aviata::srv::DroneCommand::Response::SharedPtr response)
{
    switch (request->command)
    {
    case DroneCommand::INIT_STATE:
        response->ack = init_state(static_cast<DroneState>(request->param1), request->param2);
    case DroneCommand::UNDOCK:
        response->ack = undock();
        break;
    case DroneCommand::DOCK:
        response->ack = dock(request->param1);
        break;
    case DroneCommand::CANCEL_DOCKING:
        // TODO
        break;
    case DroneCommand::REQUEST_NEW_LEADER:
        response->ack = become_follower(request->param1);
        break;
    case DroneCommand::BECOME_LEADER:
        response->ack = become_leader(request->param1);
        break;
    default:
        response->ack = 0;
    }
}
