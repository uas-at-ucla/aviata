////
// UTILITY FUNCTIONS
////

#ifndef UTIL_H_
#define UTIL_H_

#include <iostream>
#include <cmath>

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define LOG_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

// Logs
inline void log(const std::string& tag, const std::string msg, bool err=false)
{
    std::cout << (err ? ERROR_CONSOLE_TEXT : LOG_CONSOLE_TEXT) << "[" << tag << "] " << NORMAL_CONSOLE_TEXT << msg << std::endl;
}

// Math functions
inline double to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

inline double to_degrees(double radians) {
    return radians * 180.0 / M_PI;
}

// Target type
struct Target {
    float lat = 0;
    float lon = 0;
    float alt = 0;
    float yaw = 0;
};

#endif // UTIL_H_