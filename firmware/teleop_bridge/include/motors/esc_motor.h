#include <Arduino.h>
#include <ESP32Servo.h>
#include <feedback/base_feedback.h>
#include <speed_pid.h>

#ifndef __ESC_MOTOR_H__
#define __ESC_MOTOR_H__

namespace esc_motor
{
    const int STOP_MOTOR_PULSE_WIDTH = 1500;
    const int MINIMUM_BACKWARD_PULSE_WIDTH = 1390;
    const int MINIMUM_FORWARD_PULSE_WIDTH = 1610;

    class EscMotor
    {
    public:
        EscMotor(int pin, float max_speed, base_feedback::BaseFeedback *feedback, speed_pid::SpeedPID *pid);
        void begin();
        void update();
        void stop() { set_command(0); }
        void set_command(int command);
        int get_command();
        float get_velocity() { return pid_->get_target(); }

    private:
        int pin_;
        float max_speed_;
        int counter_;
        Servo *servo_;
        base_feedback::BaseFeedback *feedback_;
        speed_pid::SpeedPID *pid_;

        int velocity_to_pulse(float velocity);
        int compute_pwm_percentages(int pulse_width);
        void update_servo(int pulse_width);
    };
} // namespace esc_motor

#endif // __ESC_MOTOR_H__