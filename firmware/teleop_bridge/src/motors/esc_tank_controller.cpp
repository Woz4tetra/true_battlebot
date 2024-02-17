#include <motors/esc_tank_controller.h>

using namespace base_controller;
using namespace esc_tank_controller;
using namespace esc_motor;

EscTankController::EscTankController(
    EscMotor *left_motor,
    EscMotor *right_motor,
    imu_sensor::ImuSensor *imu_sensor_inst,
    speed_pid::SpeedPID *angular_pid,
    float base_width,
    float wheel_radius) : BaseController()
{
    left_motor_ = left_motor;
    right_motor_ = right_motor;
    imu_sensor_ = imu_sensor_inst;
    base_width_ = base_width;
    angular_pid_ = angular_pid;
    ground_velocity_to_frequency_ = 1.0 / (M_PI * 2 * wheel_radius);
}

void EscTankController::begin()
{
    left_motor_->begin();
    right_motor_->begin();
}

void EscTankController::update()
{
    float left_output, right_output;
    if (left_setpoint_ != 0.0 && right_setpoint_ != 0.0 && imu_sensor_->has_data())
    {
        float angular_vel = (right_setpoint_ - left_setpoint_) / base_width_;
        float feedback_angular = imu_sensor_->get_angular_z();
        angular_pid_->set_target(angular_vel);
        float output_ang_vel = angular_pid_->compute(feedback_angular);
        float correction = output_ang_vel * base_width_ / 2;
        left_output = left_setpoint_ - correction;
        right_output = right_setpoint_ + correction;
    }
    else
    {
        left_output = left_setpoint_;
        right_output = right_setpoint_;
    }

    int left_command = frequency_to_command(left_output * ground_velocity_to_frequency_);
    int right_command = frequency_to_command(right_output * ground_velocity_to_frequency_);

    left_motor_->set_command(left_command);
    right_motor_->set_command(right_command);
}

void EscTankController::set_motor(uint8_t channel, float velocity_mps)
{
    switch (channel)
    {
    case LEFT_CHANNEL:
        left_setpoint_ = velocity_mps;
        break;
    case RIGHT_CHANNEL:
        right_setpoint_ = velocity_mps;
        break;
    default:
        Serial.print("Invalid channel ");
        Serial.println(channel);
        break;
    }
}

int EscTankController::get_command(uint8_t channel)
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
