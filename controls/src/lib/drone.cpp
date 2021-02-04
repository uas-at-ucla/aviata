#include "drone.hpp"

using namespace std::chrono;

Drone::Drone(std::string drone_id) : drone_id(drone_id), px4_io(drone_id), telemValues(px4_io)
{
    Network::init();
    network = std::make_shared<Network>(drone_id);

    telemValues.init_telem();
    drone_state = STANDBY;

    drone_status.drone_id = drone_id;
    drone_status.drone_state = drone_state;
    drone_status.docking_slot = docking_slot;
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
    if (px4_io.connect_to_pixhawk(connection_url, 5) == false)
    {
        return 1;
    }

    px4_io.arm_system();

    network->init_follower_setpoint_publisher();

    px4_io.subscribe_attitude_target([this](const mavlink_attitude_target_t &attitude_target) {
        aviata::msg::FollowerSetpoint follower_setpoint;
        std::copy(std::begin(attitude_target.q), std::end(attitude_target.q), std::begin(follower_setpoint.q));
        follower_setpoint.thrust = attitude_target.thrust;
        network->publish_follower_setpoint(follower_setpoint);
        std::cout << "attitude_target thrust: " << attitude_target.thrust << std::endl;
    });

    bool began_descent = false;
    std::cout << "Taking off!" << std::endl;
    int64_t takeoff_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    px4_io.takeoff_system();

    while (true)
    {
        if (!began_descent && duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - takeoff_time >= 20000)
        {
            std::cout << "Landing!" << std::endl;
            px4_io.land_system();
            began_descent = true;
        }
        px4_io.call_queued_mavsdk_callbacks();
        Network::spin_some(network);
    }

    return 0;
}

int Drone::test_follow_att_target(std::string connection_url)
{
    if (px4_io.connect_to_pixhawk(connection_url, 5) == false)
    {
        return 1;
    }

    px4_io.arm_system(); // TODO manage auto disarm for follower drones

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
        px4_io.set_attitude_target(initial_attitude_target);
        std::this_thread::sleep_for(milliseconds(50));
    }

    px4_io.set_offboard_mode();

    network->subscribe_follower_setpoint([this](const aviata::msg::FollowerSetpoint::SharedPtr follower_setpoint) {
        mavlink_set_attitude_target_t attitude_target;
        std::copy(std::begin(follower_setpoint->q), std::end(follower_setpoint->q), std::begin(attitude_target.q));
        attitude_target.thrust = follower_setpoint->thrust;
        px4_io.set_attitude_target(attitude_target);
        std::cout << "follower_setpoint thrust: " << follower_setpoint->thrust << std::endl;
    });

    while (true)
    {
        px4_io.call_queued_mavsdk_callbacks();
        Network::spin_some(network);
    }

    return 0;
}

int Drone::lead_standalone(std::string connection_url)
{
    if (px4_io.connect_to_pixhawk(connection_url, 5) == false) {
        return 1;
    }
    basic_lead();
    return 0;
}

int Drone::follow_standalone(std::string connection_url)
{
    if (px4_io.connect_to_pixhawk(connection_url, 5) == false) {
        return 1;
    }
    basic_follow();
    return 0;
}

int Drone::lead_as_0(std::string connection_url)
{
    if (px4_io.connect_to_pixhawk(connection_url, 5) == false) {
        return 1;
    }

    while (px4_io.undock() != 1) {}
    std::cout << "Drone is undocked." << std::endl;
    while (px4_io.dock(0, nullptr, 0) != 1) {}
    std::cout << "Docking command sent successfully!" << std::endl;
    basic_lead();

    return 0;
}

int Drone::follow_as_1(std::string connection_url)
{
    if (px4_io.connect_to_pixhawk(connection_url, 5) == false) {
        return 1;
    }

    while (px4_io.undock() != 1) {}
    std::cout << "Drone is undocked." << std::endl;
    while (px4_io.dock(1, nullptr, 0) != 1) {}
    std::cout << "Docking command sent successfully!" << std::endl;
    basic_follow();

    return 0;
}


void Drone::basic_lead()
{
    network->init_follower_setpoint_publisher();

    bool armed = false;
    bool kill_switch_engaged = false;

    px4_io.subscribe_armed([&armed](bool is_armed) {
        armed = is_armed;
    });

    px4_io.subscribe_status_text([&kill_switch_engaged](mavsdk::Telemetry::StatusText status_text) {
        if (status_text.text == "Kill-switch engaged") {
            kill_switch_engaged = true;
        } else if (status_text.text == "Kill-switch disengaged") {
            kill_switch_engaged = false;
        }
    });

    px4_io.subscribe_attitude_target([this, &armed, &kill_switch_engaged](const mavlink_attitude_target_t &attitude_target) {
        if (armed && !kill_switch_engaged) {
            aviata::msg::FollowerSetpoint follower_setpoint;
            std::copy(std::begin(attitude_target.q), std::end(attitude_target.q), std::begin(follower_setpoint.q));
            follower_setpoint.thrust = attitude_target.thrust;
            network->publish_follower_setpoint(follower_setpoint);
        }
    });

    while (true) {
        px4_io.call_queued_mavsdk_callbacks();
        Network::spin_some(network);
    }
}

void Drone::basic_follow()
{
    bool in_offboard = false;
    bool armed = false;
    int64_t last_msg_time = 0;

    px4_io.subscribe_flight_mode([&in_offboard](mavsdk::Telemetry::FlightMode flight_mode) {
        in_offboard = (flight_mode == mavsdk::Telemetry::FlightMode::Offboard);
    });

    px4_io.subscribe_armed([&armed](bool is_armed) {
        armed = is_armed;
    });

    network->subscribe_follower_setpoint([this, &in_offboard, &armed, &last_msg_time](const aviata::msg::FollowerSetpoint::SharedPtr follower_setpoint) {
        last_msg_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        mavlink_set_attitude_target_t attitude_target;
        std::copy(std::begin(follower_setpoint->q), std::end(follower_setpoint->q), std::begin(attitude_target.q));
        attitude_target.thrust = follower_setpoint->thrust;
        px4_io.set_attitude_target(attitude_target);
        if (!in_offboard) {
            if (px4_io.set_offboard_mode() == 1) {
                in_offboard = true;
            }
        }
        if (in_offboard && !armed) {
            if (px4_io.arm() == 1) {
                armed = true;
            }
        }
    });

    while (true) {
        if (duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - last_msg_time > 100) {
            if (armed) {
                if (px4_io.disarm() == 1) {
                    armed = false;
                    px4_io.set_manual_mode(); // Take out of offboard mode to prevent annoying failsafe beeps
                }
            }
        }
        px4_io.call_queued_mavsdk_callbacks();
        Network::spin_some(network);
    }
}

void Drone::update_drone_status()
{
    drone_status.drone_state = drone_state;
    drone_status.docking_slot = docking_slot;
    drone_status.gps_position = telemValues.dronePosition;
    drone_status.yaw = telemValues.droneQuarternion.z; //TODO: verify
    drone_status.battery_percent = telemValues.droneBattery.remaining_percent;
}

void Drone::arm_drone() // for drones in STANDBY / DOCKED_FOLLOWER
{
    if (drone_state == STANDBY || drone_state == DOCKED_FOLLOWER)
        px4_io.arm_system();
}

void Drone::arm_frame() // for DOCKED_LEADER (send arm_drone() to followers)
{
}

void Drone::disarm_drone() // for drones in STANDBY / DOCKED_FOLLOWER
{
    if (drone_state == STANDBY || drone_state == DOCKED_FOLLOWER)
        px4_io.disarm_system();
}

void Drone::disarm_frame() // for DOCKED_LEADER (send disarm_drone() to followers)
{
}

void Drone::takeoff_drone() // for drones in STANDBY
{
    if (drone_state == STANDBY)
        px4_io.takeoff_system();
}

void Drone::takeoff_frame() // for DOCKED_LEADER (send attitude and thrust to followers)
{
}

void Drone::land_drone() // for any undocked drone
{
    if (drone_state == STANDBY || drone_state == NEEDS_SERVICE)
        px4_io.land_system();
}

void Drone::land_frame() // for DOCKED_LEADER (send attitude and thrust to followers)
{
}

void Drone::undock()
{
    if (drone_state == DOCKED_FOLLOWER)
        drone_state = UNDOCKING;
    // TODO implment control
}

// @param n frame position
void Drone::dock(int n)
{
    // TODO possibly integrate controls stuff through ROS here?
}

void Drone::become_leader()
{
    if (drone_state == DOCKED_FOLLOWER)
    {
        drone_state = DOCKED_LEADER;
        // TODO implement controls
    }
}

void Drone::become_follower() //for successful sender of request_new_leader
{
    if (drone_state == DOCKED_LEADER)
    {
        drone_state = DOCKED_FOLLOWER;
    }

    if (1) //undock request
    {
        drone_state = UNDOCKING;
    }
}

void Drone::get_leader_setpoint(float q[4], float *thrust)
{
    q = telemValues.q_target;
    thrust = &(telemValues.thrust_target);
}

void Drone::set_follower_setpoint(float q[4], float *thrust)
{
    // if (drone_state == DOCKED_FOLLOWER) //is this check necessary?
    //     // offset calculations
    //     px4_io.set_attitude_and_thrust(q, thrust); // TODO update function call
}

// @brief to be used as callback for service server
void Drone::command_handler(aviata::srv::DroneCommand::Request::SharedPtr request,
                            aviata::srv::DroneCommand::Response::SharedPtr response)
{
    switch (request->command)
    {
    case REQUEST_SWAP:

        break;
    case REQUEST_UNDOCK:

        break;
    case REQUEST_DOCK:

        break;
    case TERMINATE_FLIGHT:

        break;
    case UNDOCK:

        break;
    case DOCK:

        break;
    case CANCEL_DOCKING:

        break;
    case SETPOINT:

        break;
    case LEADER_SETPOINT:

        break;
    case BECOME_LEADER:

        break;
    case REQUEST_NEW_LEADER:

        break;
    case ARM:

        break;
    case DISARM:

        break;
    case TAKEOFF:

        break;
    case LAND:

        break;
    default:
        response->ack = 0;
    }
}
