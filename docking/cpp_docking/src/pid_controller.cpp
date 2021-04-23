#include "util.hpp"
#include "pid_controller.hpp"
#include <cstdlib>
#include <array>
#include <algorithm>
#include <string>

PIDController::PIDController(bool overshoot)
    : m_overshoot_adjust(overshoot)
{
    m_prev_errs = {0, 0, 0, 0};
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
Velocities PIDController::getVelocities(float x_err, float y_err, float alt_err, float rot_err, float max_speed)
{
    Velocities ans;

    float kp_nv, kp_ev, kp_dv, kp_rv;
    float kd_nv, kd_ev, kd_dv, kd_rv;

    kp_nv = 1;//0.6 * ku_nv;
    kp_ev = 1;//0.6 * ku_ev;
    kp_dv = 0.6;// * ku_dv;

    kd_nv = 0.1; //0.15;
    kd_ev = 0.1; //0.15;
    kd_dv = 0.12;

    kp_rv = 1;
    kd_rv = 1;

    float ev = x_err * kp_ev + (x_err - m_prev_errs.x) * kd_ev;
    float nv = y_err * kp_nv + (y_err - m_prev_errs.y) * kd_nv;
    float dv = alt_err * kp_dv + (alt_err - m_prev_errs.alt) * kd_dv;
    float rv = rot_err * kp_rv + (rot_err - m_prev_errs.yaw) * kd_rv;

    if (absolute_value(dv) > 0.1) // use different max speed to funnel down
    {
        dv = 0.1 * dv / absolute_value(dv);
    }

    if (absolute_value(ev) > max_speed)
    {
        ev = max_speed * ev / absolute_value(ev);
    }

    if (absolute_value(nv) > max_speed)
    {
        nv = max_speed * nv / absolute_value(nv);
    }

    if (absolute_value(rv) > 45) 
    {
        rv = 45 * rv / absolute_value(rv); // max = 45 degrees / second
    }

    m_prev_errs.x = x_err;
    m_prev_errs.y = y_err;
    m_prev_errs.alt = alt_err;
    m_prev_errs.yaw = rot_err;

    ans.x = ev;
    ans.y = nv;
    ans.alt = dv;
    ans.yaw = rv;

    return ans;
}