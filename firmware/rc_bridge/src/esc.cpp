#include <esc.h>

using namespace esc;

Esc::Esc(int pin, int pwm_channel) : pin(pin), pwm_channel(pwm_channel)
{
    servo = new Servo();
}

void Esc::begin()
{
    servo->attach(pin, pwm_channel);
    stop();
}

void Esc::write(float signed_percent)
{
    write_angle(scale_percent_to_pulse(signed_percent));
}

int Esc::scale_percent_to_pulse(float signed_percent)
{
    if (abs(signed_percent) < DEADZONE_PERCENT)
        return NEUTRAL_ANGLE;
    float angle = (MAX_PULSE - MIN_PULSE) / 200.0 * (signed_percent + 100.0) + MIN_PULSE;
    return (int)min((float)MAX_ANGLE, max((float)MIN_ANGLE, angle));
}

void Esc::stop()
{
    write_angle(NEUTRAL_ANGLE);
}

void Esc::write_angle(int angle)
{
    servo->write(angle);
}
