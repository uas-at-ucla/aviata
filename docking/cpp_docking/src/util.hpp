////
// UTILITY FUNCTIONS
////

#ifndef UTIL_H_
#define UTIL_H_

#include <iostream>
#include <cmath>
#include <ctime>
#include <iomanip>

// Constants

#define CENTRAL_TAG_SIZE 0.135 // to be changed according to actual target size
#define PERIPHERAL_TAG_SIZE 0.08 // likewise

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
    float horiz_percentage_offset = 0;
};

struct Velocities
{
    float x = 0;
    float y = 0;
    float alt = 0;
    float yaw = 0;
};

#endif // UTIL_H_