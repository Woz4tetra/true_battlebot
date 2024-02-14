#include <Arduino.h>
#include <ESP32Servo.h>
#include <motors/esc_command_lookup.h>

#ifndef __ESC_MOTOR_H__
#define __ESC_MOTOR_H__

namespace esc_motor
{
    const int STOP_MOTOR_PULSE_WIDTH = 1500;
    const int STOP_MOTOR_COMMAND = 0;
    const int LOWER_CUTOFF_COMMAND = -40;
    const int UPPER_CUTOFF_COMMAND = 40;

    class EscMotor
    {
    public:
        EscMotor(int pin, bool flipped);
        void begin();
        void stop() { set_command(0); }
        void set_command(int command);
        int get_command();

    private:
        int pin_;
        int last_command_ = 0;
        int counter_;
        bool flipped_;
        Servo *servo_;

        int command_to_pulse(int command);
        int compute_pwm_percentages(int command);
        void update_servo(int pulse_width);
    };
} // namespace esc_motor

#endif // __ESC_MOTOR_H__