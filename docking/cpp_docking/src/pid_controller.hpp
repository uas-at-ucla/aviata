#ifndef PID_H
#define PID_H

#include <array>

//PID Controller class
class PIDController
{
public:
    PIDController(float dt,bool overshoot=false);
    ~PIDController();
    std::array<float, 4> getVelocities(float x_err, float y_err, float alt_err, float yaw_err,float max_speed);

private:
    float m_dt;
    bool m_overshoot_adjust;
    std::array<float, 4> m_prev_errs;
};
#endif //PID_H