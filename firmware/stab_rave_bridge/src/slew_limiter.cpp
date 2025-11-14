#include "slew_limiter.h"
#include <Arduino.h>
#include <cmath>

namespace slew_limiter
{
    SlewLimiter::SlewLimiter(float acceleration_limit, float deceleration_limit, float initial_value, float deadzone_value)
        : prev_value(initial_value), deadzone(fabs(deadzone_value))
    {
        // If deceleration_limit not provided (negative), use acceleration_limit
        if (deceleration_limit < 0.0f)
        {
            deceleration_limit = acceleration_limit;
        }

        accel_limit = fabs(acceleration_limit);
        decel_limit = -fabs(deceleration_limit);
    }

    float SlewLimiter::clamp(float value, float lower, float upper)
    {
        if (value < lower)
            return lower;
        if (value > upper)
            return upper;
        return value;
    }

    float SlewLimiter::calculate(float input, float dt)
    {
        // Calculate the change needed
        float change_needed = input - prev_value;

        // Check if both current and target values are within deadzone
        bool current_in_deadzone = (fabs(prev_value) <= deadzone);
        bool target_in_deadzone = (fabs(input) <= deadzone);

        // If both are in deadzone, allow immediate change (no rate limiting)
        if (current_in_deadzone && target_in_deadzone)
        {
            prev_value = input;
            return prev_value;
        }

        // If transitioning from deadzone to outside deadzone, allow immediate jump to deadzone edge
        if (current_in_deadzone && !target_in_deadzone)
        {
            // Jump to deadzone edge in the direction of the target
            if (input > 0.0f)
            {
                prev_value = deadzone;
            }
            else
            {
                prev_value = -deadzone;
            }
            // Recalculate change needed from new position
            change_needed = input - prev_value;
        }

        // Determine if we're accelerating (moving away from zero) or decelerating (moving towards zero)
        bool is_accelerating;

        if (fabs(input) > fabs(prev_value))
        {
            // Target is further from zero than current value - accelerating
            is_accelerating = true;
        }
        else
        {
            // Target is closer to zero than current value - decelerating
            is_accelerating = false;
        }

        // Choose the appropriate limit based on acceleration/deceleration
        float rate_limit;
        if (is_accelerating)
        {
            rate_limit = accel_limit * dt;
        }
        else
        {
            rate_limit = fabs(decel_limit) * dt; // decel_limit is stored as negative
        }

        // Apply rate limiting
        if (fabs(change_needed) <= rate_limit)
        {
            // Can reach target in this time step
            prev_value = input;
        }
        else
        {
            // Apply rate limiting in the direction of change
            if (change_needed > 0.0f)
            {
                prev_value += rate_limit;
            }
            else
            {
                prev_value -= rate_limit;
            }
        }

        return prev_value;
    }

    void SlewLimiter::reset(float value)
    {
        prev_value = value;
    }

    float SlewLimiter::get_output() const
    {
        return prev_value;
    }
}