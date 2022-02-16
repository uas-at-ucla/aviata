#ifndef DRONESTATUS_HPP
#define DRONESTATUS_HPP

#define NEED_MIXER_TYPES
#include "aviata_mixers.h"

#include "pid_controller.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include "MAVSDK/src/core/mavsdk.h"
#include "MAVSDK/src/plugins/telemetry/include/plugins/telemetry/telemetry.h"
#include "MAVSDK/src/plugins/action/include/plugins/action/action.h"
#include "MAVSDK/src/plugins/offboard/include/plugins/offboard/offboard.h"
#include "MAVSDK/src/core/geometry.h"

struct DroneSettings {
    bool sim;
    bool modify_px4_mixers;
    AviataAirframe frame;
    uint8_t max_missing_drones;
};

enum DroneState {
    STANDBY,
    ARRIVING, // AKA Docking Stage 0
    DOCKING_STAGE_1,
    DOCKING_STAGE_2,
    DOCKED_FOLLOWER,
    DOCKED_LEADER,
    UNDOCKING,
    DEPARTING,
    NEEDS_SERVICE
};

struct DroneStatus {
    std::string drone_id;
    std::string ip_address {""};
    uint8_t mavlink_sys_id {0};
    DroneState drone_state;
    int8_t docking_slot;

    float battery_percent {0}; 
    float gps_position[4] {0};
    float yaw {0};
};

class DockingStatus{
    public:
        DockingStatus() {
            tags = "";
            failed_frames = 0;
            successful_frames = 0;
            docking_attempts = 0;
            prev_iter_detection = false;
            has_centered = false;
        }

        mavsdk::geometry::CoordinateTransformation* ct;
        PIDController pid;
        std::string tags;
        cv::Mat img;

        int failed_frames;
        int successful_frames;
        int docking_attempts;

        bool prev_iter_detection;
        bool has_centered;
};

enum DockingIterationResult{
    DOCKING_SUCCESS,
    DOCKING_FAILURE,
    ITERATION_SUCCESS,
    RESTART_DOCKING
};

struct SetupOptions {
    std::string drone_id;
    DroneState state;
    int8_t docking_slot;
    std::string connection_url;
    AviataAirframe frame;
    int8_t max_missing_drones;
};

#endif
