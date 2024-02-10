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
        EscTankController();
        void begin();
        void set_motor(uint8_t channel, int velocity);
        int get_motor(uint8_t channel);
        void stop_all_motors();
        int get_num_channels() { return NUM_CHANNELS; }
        int get_pulse_width(uint8_t channel);

    private:
        int left_velocity_ = 0, right_velocity_ = 0;
        int left_pulse_width_ = 0, right_pulse_width_ = 0;
    };
} // namespace esc_tank_controller

#endif // __ESC_TANK_CONTROLLER_H__
