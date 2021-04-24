#ifndef PID_H
#define PID_H

#include <array>

//PID Controller class
class PIDController
{
public:
    PIDController(bool overshoot = false);
    ~PIDController();
    Velocities getVelocities(float x_err, float y_err, float alt_err, float rot_err, float max_speed);

private:
    bool m_overshoot_adjust;
    Errors m_prev_errs;
    Errors m_sums; 
};
#endif //PID_H