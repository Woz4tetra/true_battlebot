#include <motors/esc_tank_controller.h>

using namespace base_controller;
using namespace esc_tank_controller;

EscTankController::EscTankController() : BaseController()
{
}

void EscTankController::begin()
{
}

void EscTankController::set_motor(uint8_t channel, int velocity)
{
    int pulse_width;
    int speed = abs(velocity);

    // reverse is 800-1500μs
    if (velocity < 0)
    {
        pulse_width = map(speed, 0, 255, STOP_MOTOR_PULSE_WIDTH, 800);
    }
    // forward is 1500-2200μs
    else if (velocity > 0)
    {
        pulse_width = map(speed, 0, 255, STOP_MOTOR_PULSE_WIDTH, 2200);
    }
    // stop is 1500μs
    else
    {
        pulse_width = STOP_MOTOR_PULSE_WIDTH;
    }

    switch (channel)
    {
    case LEFT_CHANNEL:
        left_pulse_width_ = pulse_width;
        left_velocity_ = velocity;
        break;
    case RIGHT_CHANNEL:
        right_pulse_width_ = pulse_width;
        right_velocity_ = velocity;
        break;
    default:
        Serial.print("Invalid channel ");
        Serial.println(channel);
        break;
    }
}

int EscTankController::get_motor(uint8_t channel)
{
    switch (channel)
    {
    case LEFT_CHANNEL:
        return left_velocity_;
    case RIGHT_CHANNEL:
        return right_velocity_;
    default:
        return 0;
    }
}

int EscTankController::get_pulse_width(uint8_t channel)
{
    switch (channel)
    {
    case LEFT_CHANNEL:
        return left_pulse_width_;
    case RIGHT_CHANNEL:
        return right_pulse_width_;
    default:
        return 0;
    }
}

void EscTankController::stop_all_motors()
{
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        set_motor(i, 0);
    }
}
