#include <motors/esc_tank_controller.h>

using namespace base_controller;
using namespace esc_tank_controller;
using namespace esc_motor;

EscTankController::EscTankController(EscMotor *left_motor, EscMotor *right_motor) : BaseController()
{
    left_motor_ = left_motor;
    right_motor_ = right_motor;
}

void EscTankController::begin()
{
    left_motor_->begin();
    right_motor_->begin();
}

void EscTankController::update()
{
    left_motor_->update();
    right_motor_->update();
}

void EscTankController::set_motor(uint8_t channel, int command)
{
    switch (channel)
    {
    case LEFT_CHANNEL:
        left_motor_->set_command(command);
        break;
    case RIGHT_CHANNEL:
        right_motor_->set_command(command);
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
        return left_motor_->get_command();
    case RIGHT_CHANNEL:
        return right_motor_->get_command();
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
