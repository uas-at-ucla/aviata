////
// UTILITY FUNCTIONS
////

#ifndef UTIL_H_
#define UTIL_H_

#include <iostream>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <Eigen/Dense>

// Constants

#define CENTRAL_TAG_SIZE 0.135 // to be changed according to actual target size
#define PERIPHERAL_TAG_SIZE 0.06 // likewise

// Camera information for simulator, measured in degrees
#define CAMERA_FOV_VERTICAL 48.8
#define CAMERA_FOV_HORIZONTAL 62.2
#define DISPLAY_SCALE 1750

// Target information, measured in centimeters
#define TARGET_SIZE 234
#define DRONE_RADIUS 1.1135 // measured in meters, 1m boom + half the central tag side length (possibly slightly off for corners but good enough)


#define MARGIN 10 // apriltag detection margin

#define BOOM_LENGTH 1.00

#define OVERSHOOT_CONSTANT 0.6

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define LOG_CONSOLE_TEXT "\033[34m"   // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

//Drone-specific docking constants, measured in meters 
#define MAX_ATTEMPTS 3
#define MAX_HEIGHT 10
#define DOCKING_HEIGHT_PRECONDITION 5.0
#define MAX_HEIGHT_STAGE_2 3
#define PRECONDITION_TOLERANCE 0.20
#define STAGE_1_TOLERANCE 0.10
#define STAGE_2_TOLERANCE 0.05

#define STAGE_1 1
#define STAGE_2 2

// Logs
inline void log(const std::string &tag, const std::string msg, bool err = false)
{
    std::time_t t = std::time(nullptr);
    std::cout << (err ? ERROR_CONSOLE_TEXT : LOG_CONSOLE_TEXT) 
              << "[" 
              << std::put_time(std::localtime(&t), "%r")
              << "|"
              << tag 
              << "] " 
              << NORMAL_CONSOLE_TEXT 
              << msg 
              << std::endl;
}

// Math functions
inline double to_radians(double degrees)
{
    return degrees * M_PI / 180.0;
}

inline double to_degrees(double radians)
{
    return radians * 180.0 / M_PI;
}

// cmath::abs() is only implemented for int, long int, or long long int
inline float absolute_value(float val) {
    if (val < 0.0) {
        return -1.0 * val;
    } else {
        return val;
    }
}

inline float heading_from_att_q(const Eigen::Quaternionf& q) {
    // The heading vector (x,y) is the first two entries of the body X vector, or X column of the rotation matrix.
    // So this is equivalent to `Eigen::Vector3f body_x = q.toRotationMatrix().col(0); return atan2f(body_x(1), body_x(0));`
    float a = q.w();
    float b = q.x();
    float c = q.y();
    float d = q.z();
    float body_x_0 = a*a + b*b - c*c - d*d;
    float body_x_1 = 2 * (b*c + a*d);
    return atan2f(body_x_1, body_x_0);
}

inline Eigen::Vector3f body_z_from_att_q(const Eigen::Quaternionf& q) {
    // Equivalent to `return q.toRotationMatrix().col(2);`
    float a = q.w();
    float b = q.x();
    float c = q.y();
    float d = q.z();
    return Eigen::Vector3f(
        2 * (a * c + b * d),
        2 * (c * d - a * b),
        a * a - b * b - c * c + d * d
    );
}

// Target type
struct Target
{
    float lat = 0;
    float lon = 0;
    float alt = 0;
    float yaw = 0;
};

struct Errors
{
    float x = 0;
    float y = 0;
    float alt = 0;
    float yaw = 0;
    float tag_pixel_ratio = 0;
};

struct Velocities
{
    float x = 0;
    float y = 0;
    float alt = 0;
    float yaw = 0;
};

#endif // UTIL_H_
