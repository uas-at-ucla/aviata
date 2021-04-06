#ifndef DRONESTATUS_HPP
#define DRONESTATUS_HPP

struct DroneSettings {
    bool sim;
    bool modify_px4_mixers;
};

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

#endif
