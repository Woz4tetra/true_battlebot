#include <motors/esc_motor.h>

using namespace esc_motor;

EscMotor::EscMotor(int pin, bool flipped)
{
    pin_ = pin;
    flipped_ = flipped;
    servo_ = new Servo();
}

void EscMotor::begin()
{
    servo_->attach(pin_);
    stop();
}

void EscMotor::set_command(int command)
{
    if (flipped_)
    {
        command = -command;
    }
    update_servo(command);
}

int EscMotor::get_command()
{
    return last_command_;
}

int EscMotor::command_to_pulse(int command)
{
    int speed = abs(command);
    int pulse_width;
    // reverse is 800-1500μs
    if (command < 0)
    {
        pulse_width = map(speed, 0, 255, STOP_MOTOR_PULSE_WIDTH, 800);
    }
    // forward is 1500-2200μs
    else if (command > 0)
    {
        pulse_width = map(speed, 0, 255, STOP_MOTOR_PULSE_WIDTH, 2200);
    }
    // stop is 1500μs
    else
    {
        pulse_width = STOP_MOTOR_PULSE_WIDTH;
    }

    return pulse_width;
}

int EscMotor::compute_pwm_percentages(int command)
{
    float on_counts = 0.0;
    if (command < STOP_MOTOR_COMMAND)
    {
        on_counts = 100.0 * (float)(STOP_MOTOR_COMMAND - command) / (UPPER_CUTOFF_COMMAND - STOP_MOTOR_COMMAND);
    }
    else
    {
        on_counts = 100.0 * (float)(command - STOP_MOTOR_COMMAND) / (STOP_MOTOR_COMMAND - LOWER_CUTOFF_COMMAND);
    }
    return (int)on_counts;
}

void EscMotor::update_servo(int command)
{
    int on_counts = compute_pwm_percentages(command);
    if ((LOWER_CUTOFF_COMMAND <= command || command <= UPPER_CUTOFF_COMMAND) && command != STOP_MOTOR_COMMAND)
    {
        if (counter_ > on_counts)
        {
            command = STOP_MOTOR_COMMAND;
        }
        else
        {
            command = command > STOP_MOTOR_COMMAND ? UPPER_CUTOFF_COMMAND : LOWER_CUTOFF_COMMAND;
        }
    }
    servo_->writeMicroseconds(command_to_pulse(command));
    counter_++;
    if (counter_ >= 30)
    {
        counter_ = 0;
    }
}