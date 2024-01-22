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

void EscTankController::set_motor(uint8_t channel, uint8_t speed, int8_t direction)
{
    int pulse_width;

    // reverse is 800-1100μs
    if (direction < 0)
    {
        pulse_width = map(speed, 0, 255, 800, 1100);
    }
    // forward is 1900-2200μs
    else if (direction > 0)
    {
        pulse_width = map(speed, 0, 255, 1900, 2200);
    }
    // stop is 1500μs
    else
    {
        pulse_width = STOP_MOTOR_PULSE_WIDTH;
    }

    switch (channel)
    {
    case LEFT_CHANNEL:
        right_servo_.writeMicroseconds(pulse_width);
        break;
    case RIGHT_CHANNEL:
        left_servo_.writeMicroseconds(pulse_width);
        break;
    default:
        Serial.print("Invalid channel ");
        Serial.println(channel);
        break;
    }
}
