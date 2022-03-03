#ifndef DRONE_HPP
#define DRONE_HPP

#include <string>
#include <iostream>
#include <vector>
#include <future>
#include <memory>
#include <Eigen/Dense>

#include "MAVSDK/src/core/mavsdk.h"
#include "MAVSDK/src/plugins/telemetry/include/plugins/telemetry/telemetry.h"
#include "MAVSDK/src/plugins/action/include/plugins/action/action.h"
#include "MAVSDK/src/plugins/offboard/include/plugins/offboard/offboard.h"
#include "MAVSDK/src/core/geometry.h"

#include "drone_types.hpp"
#include "px4_telemetry.hpp"
#include "px4_io.hpp"
#include "network.hpp"

#include "aviata/srv/drone_command.hpp"
#include "aviata/msg/follower_setpoint.hpp"

#include "image_analyzer.hpp"
#include "util.hpp"
#include "docking_detector.hpp"
#include "pid_controller.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>

#if USE_RASPI_CAMERA == 1
    #include "raspi_camera.hpp"
    typedef RaspiCamera Camera;
#else
    #include "camera_simulator.hpp"
    typedef CameraSimulator Camera;
#endif

//Docking constant for altitude adjustment
const float ALTITUDE_DISP = BOOM_LENGTH / 2 / tan(to_radians(CAMERA_FOV_VERTICAL / 2)) * 1.4;

using namespace mavsdk;

class Drone
{
public:
    Drone(std::string drone_id, DroneSettings drone_settings, Target target);
    ~Drone();

    bool init(DroneState initial_state, int8_t docking_slot, std::string connection_url);
    void run();
    
private:
    const std::string _drone_id;
    const DroneSettings _drone_settings;
    const double _follower_setpoint_timeout;

    // APIs
    std::shared_ptr<Network> _network;
    PX4IO _px4_io;
    PX4Telemetry _px4_telem;

    // State
    DroneState _drone_state;

    // More state information
    int8_t _docking_slot;
    bool _kill_switch_engaged = false;
    bool _armed = false;
    Telemetry::FlightMode _flight_mode = Telemetry::FlightMode::Unknown;
    uint8_t _leader_seq_num = 0;
    double _last_setpoint_msg_time;
    bool _need_to_discover_more_drones = false;
    const AviataFrameInfo& _frame_info;
    std::vector<bool> _docking_slot_is_occupied;

    //Docking members
    Camera camera;
    ImageAnalyzer image_analyzer;
    Target m_target_info;
    DockingStatus docking_status;
    DockingDetector docking_detector;

    float m_north;
    float m_east;
    float m_down;
    float m_yaw;

    // Status of other drones
    std::map<std::string, DroneStatus> _swarm; // map by drone_id

    // State helper functions
    void init_docked();
    void init_follower();
    void init_leader();
    void init_standby();

    // State transition functions
    void transition_standby_to_docking();
    void transition_docking_to_docked();
    void transition_leader_to_follower();
    void transition_follower_to_leader();

    std::vector<uint8_t> generate_missing_drones_list();

    bool valid_leader_msg(uint8_t leader_seq_num);

    void publish_drone_status();

    void receive_drone_status(const DroneStatus& rec);

    // Functions that respond to commands
    uint8_t arm_drone(); // for drones in STANDBY / DOCKED_FOLLOWER
    uint8_t arm_frame(); // for DOCKED_LEADER (send arm_drone() to followers)
    
    uint8_t disarm_drone(); // for drones in STANDBY / DOCKED_FOLLOWER
    uint8_t disarm_frame(); // for DOCKED_LEADER (send disarm_drone() to followers)
    
    uint8_t takeoff_drone(); // for drones in STANDBY
    uint8_t takeoff_frame(); // for DOCKED_LEADER (send attitude and thrust to followers)
   
    uint8_t land_drone(); // for any undocked drone
    uint8_t land_frame(); // for DOCKED_LEADER (send attitude and thrust to followers)

    uint8_t init_state(DroneState state, int8_t docking_slot);
    uint8_t undock();
    uint8_t dock(uint8_t docking_slot);
    uint8_t become_leader(uint8_t leader_seq_num);
    uint8_t become_follower(int8_t new_leader_docking_slot); //for successful sender of request_new_leader

    // docking integrated functions
    DockingIterationResult do_docking(int stage); 
    bool fly_to_central_target();
    void initiate_docking(int stage);
    void set_position(float north, float east, float down);
    void set_yaw(float yaw);
    void offset_errors(Errors &errs, int target_id); // Helper method for docking stage 1 error adjustment

    void command_handler(const aviata::srv::DroneCommand::Request::SharedPtr request, 
                         aviata::srv::DroneCommand::Response::SharedPtr response);
};

#endif
