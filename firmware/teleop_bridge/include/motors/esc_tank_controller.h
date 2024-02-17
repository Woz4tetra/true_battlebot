#include <motors/base_controller.h>
#include <motors/esc_motor.h>
#include <feedback/imu_sensor.h>
#include <speed_pid.h>

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
        EscTankController(
            esc_motor::EscMotor *left_motor,
            esc_motor::EscMotor *right_motor,
            imu_sensor::ImuSensor *imu_sensor_inst,
            speed_pid::SpeedPID *angular_pid,
            float base_width,
            float wheel_radius);
        void begin();
        void update();
        void set_motor(uint8_t channel, float velocity);
        int get_command(uint8_t channel);
        void stop_all_motors();
        int get_num_channels() { return NUM_CHANNELS; }

    private:
        esc_motor::EscMotor *left_motor_, *right_motor_;
        imu_sensor::ImuSensor *imu_sensor_;
        speed_pid::SpeedPID *angular_pid_;
        float base_width_;
        float ground_velocity_to_frequency_;
        float left_setpoint_ = 0.0f, right_setpoint_ = 0.0f;
    };
} // namespace esc_tank_controller

#endif // __ESC_TANK_CONTROLLER_H__
