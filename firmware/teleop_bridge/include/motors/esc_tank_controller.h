#include <motors/base_controller.h>
#include <motors/esc_motor.h>

#ifndef __ESC_TANK_CONTROLLER_H__
#define __ESC_TANK_CONTROLLER_H__

namespace esc_tank_controller
{
    const int NUM_CHANNELS = 2;
    const int LEFT_CHANNEL = 0;
    const int RIGHT_CHANNEL = 1;

    class EscTankController : public base_controller::BaseController
    {
    public:
        EscTankController(esc_motor::EscMotor *left_motor, esc_motor::EscMotor *right_motor);
        void begin();
        void update();
        void set_motor(uint8_t channel, int velocity);
        int get_motor(uint8_t channel);
        void stop_all_motors();
        int get_num_channels() { return NUM_CHANNELS; }

    private:
        esc_motor::EscMotor *left_motor_, *right_motor_;
    };
} // namespace esc_tank_controller

#endif // __ESC_TANK_CONTROLLER_H__
