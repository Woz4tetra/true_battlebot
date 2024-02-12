#include <motors/esc_motor.h>

using namespace esc_motor;
using namespace base_feedback;
using namespace speed_pid;

EscMotor::EscMotor(int pin, float max_speed, BaseFeedback *feedback, SpeedPID *pid)
{
    max_speed_ = max_speed;
    pin_ = pin;
    servo_ = new Servo();
    pid_ = pid;
    feedback_ = feedback;
}

void EscMotor::begin()
{
    servo_->attach(pin_);
    stop();
}

void EscMotor::update()
{
    float command_velocity;
    if (feedback_->has_feedback())
    {
        command_velocity = pid_->compute(feedback_->get_feedback());
    }
    else
    {
        command_velocity = get_velocity();
    }
    int pulse_width = velocity_to_pulse(command_velocity);
    update_servo(pulse_width);
}

void EscMotor::set_command(int command)
{
    pid_->set_target((float)command / 255.0 * max_speed_);
}

int EscMotor::get_command()
{
    return (int)(get_velocity() / max_speed_ * 255.0);
}

int EscMotor::velocity_to_pulse(float velocity)
{
    int command = (int)(velocity / max_speed_ * 255.0);
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

int EscMotor::compute_pwm_percentages(int pulse_width)
{
    float on_counts = 0.0;
    if (pulse_width < STOP_MOTOR_PULSE_WIDTH)
    {
        on_counts = 100.0 * (float)(STOP_MOTOR_PULSE_WIDTH - pulse_width) / (MINIMUM_FORWARD_PULSE_WIDTH - STOP_MOTOR_PULSE_WIDTH);
    }
    else
    {
        on_counts = 100.0 * (float)(pulse_width - STOP_MOTOR_PULSE_WIDTH) / (STOP_MOTOR_PULSE_WIDTH - MINIMUM_BACKWARD_PULSE_WIDTH);
    }
    return (int)on_counts;
}

void EscMotor::update_servo(int pulse_width)
{
    int on_counts = compute_pwm_percentages(pulse_width);
    if ((MINIMUM_BACKWARD_PULSE_WIDTH <= pulse_width || pulse_width <= MINIMUM_FORWARD_PULSE_WIDTH) && pulse_width != STOP_MOTOR_PULSE_WIDTH)
    {
        if (counter_ > on_counts)
        {
            pulse_width = STOP_MOTOR_PULSE_WIDTH;
        }
        else
        {
            pulse_width = pulse_width > STOP_MOTOR_PULSE_WIDTH ? MINIMUM_FORWARD_PULSE_WIDTH : MINIMUM_BACKWARD_PULSE_WIDTH;
        }
    }
    servo_->writeMicroseconds(pulse_width);
    counter_++;
    if (counter_ >= 100)
    {
        counter_ = 0;
    }
}