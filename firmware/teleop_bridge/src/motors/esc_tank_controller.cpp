#include <motors/esc_tank_controller.h>

using namespace base_controller;
using namespace esc_tank_controller;

EscTankController::EscTankController(int left_output_pin, int right_output_pin) : BaseController()
{
    left_output_pin_ = left_output_pin;
    right_output_pin_ = right_output_pin;
}

void EscTankController::begin()
{
    left_servo_.attach(left_output_pin_);
    right_servo_.attach(right_output_pin_);
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
        left_servo_.writeMicroseconds(pulse_width);
        left_velocity_ = velocity;
        break;
    case RIGHT_CHANNEL:
        right_servo_.writeMicroseconds(pulse_width);
        right_velocity_ = velocity;
        break;
    default:
        Serial.print("Invalid channel ");
        Serial.println(channel);
        break;
    }
}

void EscTankController::get_motor(uint8_t channel, int &velocity)
{
    switch (channel)
    {
    case LEFT_CHANNEL:
        velocity = left_velocity_;
        break;
    case RIGHT_CHANNEL:
        velocity = right_velocity_;
        break;
    default:
        velocity = 0;
        break;
    }
}

void EscTankController::stop_all_motors()
{
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        set_motor(i, 0);
    }
}
