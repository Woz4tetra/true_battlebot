#include <Arduino.h>

#ifndef __SPEED_PID_H__
#define __SPEED_PID_H__

namespace speed_pid
{
    class SpeedPID
    {
    private:
        float target;
        float error_sum, prev_error;
        float feedforward;
        uint32_t current_time, prev_update_time;
        float dt;
        float out;
        float deadzone_command;
        float K_ff; // feedforward constant

    public:
        float Kp, Ki, Kd;
        float error_sum_clamp;
        float command_min, command_max;
        float epsilon; // values that are basically zero

        SpeedPID();
        void set_target(float target);
        float get_target();
        void reset();
        float limit(float value);
        float compute(float measurement);
        float get_last_command() { return out; };
        void set_deadzones(float K_ff, float deadzone_command);

        static float sign(float x);
        static int sign(int x);
    };
}

#endif // __SPEED_PID_H__