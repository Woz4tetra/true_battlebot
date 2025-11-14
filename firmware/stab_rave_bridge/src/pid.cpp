#include "pid.h"
#include <cmath>

namespace pid
{
    Pid::Pid(const PidConfig &config)
        : kp(config.kp), ki(config.ki), kd(config.kd), kf(config.kf), i_zone(config.i_zone), i_max(config.i_max), tolerance(config.tolerance), continuous(config.continuous), min_input(config.min_input), max_input(config.max_input), i_accum(0.0), prev_error(0.0), has_prev_error(false), error(0.0)
    {
    }

    void Pid::reset()
    {
        i_accum = 0.0;
        prev_error = 0.0;
        error = 0.0;
        has_prev_error = false;
    }

    float Pid::update(float setpoint, float measurement, float dt)
    {
        error = setpoint - measurement;

        // Handle continuous input (angle wrapping)
        if (continuous)
        {
            error = _wrap_error(error);
        }

        if (fabs(error) < tolerance)
        {
            return 0.0;
        }
        if (dt <= 0.0)
        {
            // In embedded systems, we might prefer to return 0 or last output
            // instead of throwing an exception
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

        float derivative_error = error - prev_error;

        // Handle continuous input for derivative term
        if (continuous)
        {
            derivative_error = _wrap_error(derivative_error);
        }

        float output = kd * derivative_error / dt;
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

    float Pid::_repeat(float value, float length)
    {
        // Loops the value, so it's never larger than length and never smaller than 0
        if (length <= 0.0f)
            return 0.0f;

        return value - floor(value / length) * length;
    }

    float Pid::_input_modulus(float value, float min_value, float max_value)
    {
        float modulus = max_value - min_value;

        value -= min_value;
        value = _repeat(value, modulus);
        value += min_value;

        return value;
    }

    float Pid::_wrap_error(float error)
    {
        // For angle errors, we want the shortest path
        // Convert error to range [-input_range/2, input_range/2]
        float input_range = max_input - min_input;
        float half_range = input_range / 2.0f;

        // Use input_modulus to wrap to [0, input_range), then shift to [-half_range, half_range)
        float wrapped = _input_modulus(error + half_range, 0.0f, input_range) - half_range;

        return wrapped;
    }

    float Pid::get_error()
    {
        return error;
    }
}