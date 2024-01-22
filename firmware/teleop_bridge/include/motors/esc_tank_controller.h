#include <ESP32Servo.h>
#include <motors/base_controller.h>

#ifndef __ESC_TANK_CONTROLLER_H__
#define __ESC_TANK_CONTROLLER_H__

namespace esc_tank_controller
{
    const int NUM_CHANNELS = 2;
    const int LEFT_CHANNEL = 0;
    const int RIGHT_CHANNEL = 1;
    const int STOP_MOTOR_PULSE_WIDTH = 1500;

    class EscTankController : public base_controller::BaseController
    {
    public:
        EscTankController(int left_output_pin, int right_output_pin);
        void begin();
        void set_motor(uint8_t channel, uint8_t speed, int8_t direction);
        void get_motor(uint8_t channel, uint8_t &speed, int8_t &direction);
        void stop_all_motors();

    private:
        int left_output_pin_, right_output_pin_;
        Servo left_servo_, right_servo_;
        uint8_t left_speed_ = 0, right_speed_ = 0;
        int8_t left_direction_ = 0, right_direction_ = 0;
    };
} // namespace esc_tank_controller

#endif // __ESC_TANK_CONTROLLER_H__
