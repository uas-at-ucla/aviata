#ifndef DRONESTATUS_HPP
#define DRONESTATUS_HPP

class Mat;
class PIDController;

struct DroneSettings {
    bool sim;
    bool modify_px4_mixers;
};

enum DroneState {
    STANDBY,
    ARRIVING,
    DOCKING_STAGE_1,
    DOCKING_STAGE_2,
    DOCKED_FOLLOWER,
    DOCKED_LEADER,
    UNDOCKING,
    DEPARTING,
    NEEDS_SERVICE
};

enum DockingIterationResult{
    DOCKING_SUCCESS,
    DOCKING_FAILURE,
    ITERATION_SUCCESS,
    RESTART_DOCKING
};

struct DroneStatus {
// struct DroneStatus_Docked { // update naming convention?
    std::string drone_id;
    DroneState drone_state;
    uint8_t docking_slot;

    float battery_percent; 
    float gps_position[4];
    float yaw;
};

struct DroneStatus_Attitude {
    std::string drone_id;


};

struct DroneStatus_ {
    std::string drone_id;


};

class DockingStatus{
    auto offboard=Offboard{m_system};
    PIDController pid(m_dt);
    std::string tags;
    Mat img;

    int failed_frames;
    int successful_frames;
    int docking_attempts;

    bool prev_iter_detection;
    bool has_centered;
};

#endif
