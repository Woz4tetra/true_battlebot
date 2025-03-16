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
    const float DEADZONE_PERCENT = 10.0;

    class Esc
    {
    private:
        Servo *servo;
        const int pin;
        const int pwm_channel;
        int scale_percent_to_pulse(float signed_percent);
        void write_angle(int angle);

    public:
        Esc(int pin, int pwm_channel);
        void begin();
        void stop();
        void write(float signed_percent);
    };

}