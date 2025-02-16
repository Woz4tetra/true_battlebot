#include <Arduino.h>
#include <Servo.h>

#define CHANNEL_A 1
#define CHANNEL_B 3

const unsigned long PWM_TIMEOUT = 10000; // 10 millisecond (100 Hz)
const unsigned long LOW_PERIOD = 19000;  // 19 millisecond (nominally 50 Hz)
const unsigned long HIGH_PERIOD = 21000; // 21 millisecond (nominally 50 Hz)
float duty_cycle_a, duty_cycle_b;

const float min_cycle = 0.89945;
const float max_cycle = 0.950652;

#define SERVO_LEFT 4
#define SERVO_RIGHT 13

Servo servo_left;
Servo servo_right;

const int neutral_pulse = 1500;
const int spread_pulse = 500;
const int max_pulse = neutral_pulse + spread_pulse;
const int min_pulse = neutral_pulse - spread_pulse;

float scale_cycle_to_percent(float duty_cycle)
{
    float percent = -2.0 / (max_cycle - min_cycle) * (duty_cycle - min_cycle) + 1.0;
    return min(1.0, max(-1.0, percent));
}

int scale_percent_to_pulse(float signed_percent)
{
    float angle = (max_pulse - min_pulse) / 2.0 * (signed_percent + 1.0) + min_pulse;
    return (int)min(MAX_PULSE_WIDTH, max(MIN_PULSE_WIDTH, angle));
}

bool read_pwm(int pin, float &duty_cycle)
{
    unsigned long high_duration = pulseIn(pin, HIGH, PWM_TIMEOUT);
    unsigned long low_duration = pulseIn(pin, LOW, PWM_TIMEOUT);

    unsigned long period = high_duration + low_duration;
    if (period < LOW_PERIOD || period > HIGH_PERIOD)
    {
        return false;
    }

    duty_cycle = (float)high_duration / (float)period;
    return true;
}

void write_escs(int left_pulse, int right_pulse)
{
    servo_left.writeMicroseconds(left_pulse);
    servo_right.writeMicroseconds(right_pulse);
}

void initialize_escs()
{
    write_escs(MIN_PULSE_WIDTH, MIN_PULSE_WIDTH);
    delay(3000);
    write_escs(neutral_pulse, neutral_pulse);
    delay(1000);
}

void setup()
{
    Serial.begin(115200);
    pinMode(CHANNEL_A, INPUT);
    pinMode(CHANNEL_B, INPUT);

    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);

    Serial.println("Initializing ESCs");
    initialize_escs();
}

void loop()
{
    float next_duty_cycle_a, next_duty_cycle_b;
    if (read_pwm(CHANNEL_A, next_duty_cycle_a))
        duty_cycle_a = next_duty_cycle_a;
    if (read_pwm(CHANNEL_B, next_duty_cycle_b))
        duty_cycle_b = next_duty_cycle_b;

    float a_percent = -1 * scale_cycle_to_percent(duty_cycle_a);
    float b_percent = scale_cycle_to_percent(duty_cycle_b);

    float left_command = a_percent - b_percent;
    float right_command = a_percent + b_percent;

    int left_pulse = scale_percent_to_pulse(left_command);
    int right_pulse = scale_percent_to_pulse(right_command);

    write_escs(left_pulse, right_pulse);
}
