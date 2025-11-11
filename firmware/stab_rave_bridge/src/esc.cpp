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
    float scaled_percent = apply_uneven_deadzone(signed_percent);
    command = scaled_percent;
    int angle = scale_percent_to_pulse(scaled_percent);
    write_angle(angle);
}

float Esc::apply_uneven_deadzone(float signed_percent)
{
    float scaled_percent;
    // Scale percent based on uneven deadzone
    if (signed_percent > 0)
        scaled_percent = signed_percent + LOWER_DEADZONE_PERCENT;
    else
        scaled_percent = signed_percent - UPPER_DEADZONE_PERCENT;
    return scaled_percent;
}

int Esc::scale_percent_to_pulse(float signed_percent)
{
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

float Esc::get_command()
{
    return command;
}
