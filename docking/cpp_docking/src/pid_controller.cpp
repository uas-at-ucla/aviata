#include "util.hpp"
#include "pid_controller.hpp"
#include <cstdlib>
#include <array>
#include <algorithm>
#include <string>

PIDController::PIDController(float dt,bool overshoot)
    : m_dt(dt),m_overshoot_adjust(overshoot)
{
    m_prev_errs = {0, 0, 0};
}

PIDController::~PIDController()
{
}

/**
 * Apply PID logic to calculated errors to determine appropriate velocity.
 * 
 * @param x_err x (east) offset in meters
 * @param y_err y (north) offset in meters
 * @param alt_err z (altitude) offset in meters
 * @param max_speed cap on the calculated errors
 * @return 3-element array for x, y, z velocity in meters / second
 * 
 * */
std::array<float, 3> PIDController::getVelocities(float x_err, float y_err, float alt_err, float max_speed)
{
    std::array<float, 3> ans = {0, 0, 0};
    float ku_nv = 2.6;
    float ku_ev = 2.15;
    float ku_dv = 3.5;

    float tu_nv=0.75;
    float tu_ev=0.90;
    float tu_dv=0.45;

    float kp_nv;
    float kp_ev;
    float kp_dv;
    float kd_nv;
    float kd_ev;
    float kd_dv;
    if(!m_overshoot_adjust){ 
        kp_nv = 0.6 * ku_nv;
        kp_ev = 0.6 * ku_ev;
        kp_dv = 0.6 * ku_dv;

        kd_nv = 0.15;
        kd_ev = 0.15;
        kd_dv = 0.12;
    }
    else{
        kp_nv = 0.2 * ku_nv;
        kp_ev = 0.2 * ku_ev;
        kp_dv = 0.2 * ku_dv;

        kd_nv = 0.066*ku_nv*tu_nv;
        kd_ev = 0.066*ku_ev*tu_ev;
        kd_dv = 0.066*ku_dv*tu_dv;
    }

    float ev = x_err * kp_ev + (x_err - m_prev_errs[0]) / m_dt * kd_ev;
    float nv = y_err * kp_nv + (y_err - m_prev_errs[1]) / m_dt * kd_nv;
    float dv = alt_err * kp_dv + (alt_err - m_prev_errs[2]) / m_dt * kd_dv;

    if (absolute_value(dv) > max_speed)
    {
        dv = max_speed * dv / absolute_value(dv);
    }

    if (absolute_value(ev) > max_speed)
    {
        ev = max_speed * ev / absolute_value(ev);
    }

    if (absolute_value(nv) > max_speed)
    {
        nv = max_speed * nv / absolute_value(nv);
    }

    m_prev_errs[0] = x_err;
    m_prev_errs[1] = y_err;
    m_prev_errs[2] = alt_err;

    ans[0] = ev;
    ans[1] = nv;
    ans[2] = dv;

    return ans;
}