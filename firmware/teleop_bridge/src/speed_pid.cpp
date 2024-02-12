#include <speed_pid.h>

using namespace speed_pid;

SpeedPID::SpeedPID()
{
    target = 0.0;
    error_sum = 0.0;
    prev_error = 0.0;
    feedforward = 0.0;
    current_time = 0;
    prev_update_time = 0;
    dt = 0.0;
    out = 0.0;
    command_min = -255;
    command_max = 255;
    epsilon = 1E-3;

    K_ff = 0.0;
    deadzone_command = 0.0;
    standstill_deadzone_command = 0.0;
    error_sum_clamp = 0.0;
    Kp = 1.0;
    Ki = 0.0;
    Kd = 0.0;
}

float SpeedPID::sign(float x)
{
    return (x > 0) - (x < 0);
}

int SpeedPID::sign(int x)
{
    return (x > 0) - (x < 0);
}

void SpeedPID::set_deadzones(float K_ff, float deadzone_command, float standstill_deadzone_command)
{
    this->K_ff = K_ff;
    this->deadzone_command = deadzone_command;
    this->standstill_deadzone_command = standstill_deadzone_command;
}

void SpeedPID::set_target(float target)
{
    if (K_ff != 0.0)
    {
        if (abs(target) < epsilon)
        {
            feedforward = 0.0;
        }
        else
        {
            feedforward = K_ff * target;
        }
    }
    if (sign(target) != this->target)
    {
        error_sum = 0.0;
    }
    this->target = target;
    prev_update_time = 0;
}

float SpeedPID::get_target()
{
    return target;
}

void SpeedPID::reset()
{
    prev_error = 0.0;
    error_sum = 0.0;
    set_target(0.0);
}

float SpeedPID::limit(float command)
{
    if (abs(command) < epsilon)
    {
        return 0.0;
    }

    if (command > command_max)
    {
        return command_max;
    }
    if (command < command_min)
    {
        return command_min;
    }

    if (abs(command) < deadzone_command)
    {
        return sign(command) * deadzone_command;
    }

    return command;
}

float SpeedPID::compute(float measurement)
{
    current_time = millis();

    if (current_time - prev_update_time == 0)
    {
        return limit(out);
    }
    else if (current_time - prev_update_time < 0)
    { // edge case for timer looping
        prev_update_time = 0;
    }

    float error = target - measurement;
    dt = (current_time - prev_update_time) * 1E-3;
    prev_update_time = current_time;

    out = 0.0;
    if (Kp > 0.0)
    {
        out += Kp * error;
    }
    if (Kd > 0.0)
    {
        out += Kd * (error - prev_error) / dt;
        prev_error = error;
    }
    if (Ki > 0.0)
    {
        out += Ki * error_sum * dt;
        error_sum += error;
        if (error_sum_clamp >= 0.0 && abs(error_sum) > error_sum_clamp)
        {
            error_sum = sign(error_sum) * error_sum_clamp;
        }
    }
    out += feedforward;

    if (standstill_deadzone_command != 0.0 && abs(target) > epsilon && abs(measurement) < epsilon)
    {
        out = sign(target) * standstill_deadzone_command;
    }

    return limit(out);
}
