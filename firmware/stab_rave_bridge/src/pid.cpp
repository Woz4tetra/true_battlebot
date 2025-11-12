#include "pid.h"

namespace pid
{
    Pid::Pid(const PidConfig &config)
        : kp(config.kp), ki(config.ki), kd(config.kd), kf(config.kf), i_zone(config.i_zone), i_max(config.i_max), tolerance(config.tolerance), i_accum(0.0), prev_error(0.0), has_prev_error(false)
    {
    }

    void Pid::reset()
    {
        i_accum = 0.0;
        prev_error = 0.0;
        has_prev_error = false;
    }

    float Pid::update(float setpoint, float measurement, float dt)
    {
        float error = setpoint - measurement;
        if (fabs(error) < tolerance)
        {
            return 0.0;
        }
        if (dt <= 0.0)
        {
            return 0.0;
        }

        float output = 0.0;
        output += _calculate_p(error);
        output += _calculate_i(error, dt);
        output += _calculate_d(error, dt);
        output += _calculate_f(setpoint);
        return output;
    }

    float Pid::_calculate_p(float error)
    {
        if (kp == 0.0)
        {
            return 0.0;
        }
        return kp * error;
    }

    float Pid::_calculate_i(float error, float dt)
    {
        if (ki == 0.0)
        {
            return 0.0;
        }

        // Check integral zone
        if (i_zone < 0.0)
        {
            // No i_zone limit (equivalent to Python's None)
            i_accum += error;
        }
        else if (fabs(error) < i_zone)
        {
            i_accum += error;
        }

        // Apply integral windup protection
        if (i_max != 0.0)
        {
            float max_i_term = i_max / ki;
            if (i_accum > 0.0)
            {
                i_accum = fmin(i_accum, max_i_term);
            }
            else
            {
                i_accum = fmax(i_accum, -max_i_term);
            }
        }

        return ki * i_accum * dt;
    }

    float Pid::_calculate_d(float error, float dt)
    {
        if (kd == 0.0)
        {
            return 0.0;
        }

        if (!has_prev_error)
        {
            prev_error = error;
            has_prev_error = true;
            return 0.0; // No derivative on first call
        }

        float output = kd * (error - prev_error) / dt;
        prev_error = error;
        return output;
    }

    float Pid::_calculate_f(float setpoint)
    {
        if (kf == 0.0)
        {
            return 0.0;
        }
        return kf * setpoint;
    }
}