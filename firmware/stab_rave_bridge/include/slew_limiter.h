#pragma once

namespace slew_limiter
{
    class SlewLimiter
    {
    public:
        /**
         * Implements a slew rate limiter that constrains the rate of change of a value.
         *
         * Args:
         * acceleration_limit: Maximum rate of increase per second (positive value)
         * deceleration_limit: Maximum rate of decrease per second (positive value, optional)
         *                    If not provided, uses same value as acceleration_limit
         * initial_value: Starting value for the limiter
         * deadzone: Range around zero where rate limiting is not applied (optional)
         */
        SlewLimiter(float acceleration_limit, float deceleration_limit = -1.0f, float initial_value = 0.0f, float deadzone = 0.0f);

        /**
         * Calculate the rate-limited output for a given input
         *
         * Args:
         *     input: The desired target value
         *     dt: time since last update
         *
         * Returns:
         *     The rate-limited output value
         */
        float calculate(float input, float dt);

        /**
         * Reset the limiter to a specific value and current time
         *
         * Args:
         *     value: The value to reset to
         */
        void reset(float value);

        /**
         * Get the current output value without updating
         *
         * Returns:
         *     Current output value
         */
        float get_output() const;

    private:
        /**
         * Clamp a value between lower and upper bounds
         *
         * Args:
         *     value: Value to clamp
         *     lower: Lower bound
         *     upper: Upper bound
         *
         * Returns:
         *     Clamped value
         */
        static float clamp(float value, float lower, float upper);

        // Configuration parameters
        float accel_limit;
        float decel_limit;
        float deadzone;

        // State variables
        float prev_value;
    };
}