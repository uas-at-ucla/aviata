#ifndef PID
#define PID

#include "util.hpp"
#include "pid_controller.hpp"
#include <cstdlib>
#include <algorithm>
#include <string>

PIDController::PIDController(float dt)
    : m_dt(dt)
{
    m_prev_errs = new float[3];
    m_prev_errs[0]=0;
    m_prev_errs[1]=0;
    m_prev_errs[2]=0;
}
PIDController::~PIDController()
{
    delete[] m_prev_errs;
}

float *PIDController::getVelocities(float x_err, float y_err, float alt_err, float max_speed)
{
    float *ans = new float[3];
    float ku_nv = 2.6;
    float ku_ev = 2.15;
    float ku_dv = 3.5;

    float kp_nv = 0.6 * ku_nv;
    float kp_ev = 0.6 * ku_ev;
    float kp_dv = 0.6 * ku_dv;

    float kd_nv = 0.15;
    float kd_ev = 0.15;
    float kd_dv = 0.12;

    float ev = x_err * kp_ev + (x_err - m_prev_errs[0]) / m_dt * kd_ev;
    float nv = y_err * kp_nv + (y_err - m_prev_errs[1]) / m_dt * kd_nv;
    float dv = alt_err * kp_dv + (alt_err - m_prev_errs[2]) / m_dt * kd_dv;

    if (abs(dv) > max_speed)
    {
        dv = max_speed * dv / abs(dv);
    }

    if (abs(ev) > max_speed) {
        ev = max_speed * ev / abs(ev);
    }

    if (abs(nv) > max_speed) {
        nv = max_speed * nv / abs(nv);
    }

    m_prev_errs[0] = x_err;
    m_prev_errs[1] = y_err;
    m_prev_errs[2] = alt_err;

    ans[0] = ev;
    ans[1] = nv;
    ans[2] = dv;

    return ans;
}
#endif