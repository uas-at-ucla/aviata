#ifndef PID_H
#define PID_H

#include <array>

//PID Controller class
class PIDController
{
public:
    PIDController(float dt);
    ~PIDController();
    std::array<float, 3> getVelocities(float x_err, float y_err, float alt_err, float max_speed);

private:
    float m_dt;
    std::array<float, 3> m_prev_errs;
};
#endif //PID_H