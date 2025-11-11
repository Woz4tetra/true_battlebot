#pragma once
#include <s3servo.h>
#include <Arduino.h>
namespace esc
{
#define Servo s3servo

    const int MIN_ANGLE = 0;
    const int MAX_ANGLE = 180;
    const int NEUTRAL_ANGLE = 90;
    const int SPREAD_PULSE = 90;
    const int MAX_PULSE = NEUTRAL_ANGLE + SPREAD_PULSE;
    const int MIN_PULSE = NEUTRAL_ANGLE - SPREAD_PULSE;
    const float LOWER_DEADZONE_PERCENT = -6.5;
    const float UPPER_DEADZONE_PERCENT = 2.5;

    class Esc
    {
    private:
        Servo *servo;
        const int pin;
        const int pwm_channel;
        float command;
        float apply_uneven_deadzone(float signed_percent);
        int scale_percent_to_pulse(float signed_percent);
        void write_angle(int angle);

    public:
        Esc(int pin, int pwm_channel);
        void begin();
        void stop();
        void write(float signed_percent);
        float get_command();
    };

}