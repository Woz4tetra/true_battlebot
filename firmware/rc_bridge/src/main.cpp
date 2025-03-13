#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <s3servo.h>

#define MIN_ANGLE 0
#define MAX_ANGLE 180
#define Servo s3servo

float led_intensity = 0.0;
int led_cycle_index = 0;
int led_cycle_direction = 1;

#define CHANNEL_A A1
#define CHANNEL_B A2

const unsigned long LOW_PERIOD = 19000;  // 19 millisecond (nominally 50 Hz)
const unsigned long HIGH_PERIOD = 21000; // 21 millisecond (nominally 50 Hz)
volatile unsigned long pulse_in_begin_a = 0, pulse_in_begin_b = 0;
volatile unsigned long pulse_in_end_a = 0, pulse_in_end_b = 0;
volatile unsigned long pulse_duration_a = 0, pulse_duration_b = 0;
volatile bool pulse_in_available_a = false, pulse_in_available_b = false;
float duty_cycle_a = 0.0, duty_cycle_b = 0.0;

const float min_cycle = 5.0;
const float max_cycle = 10.0;

#define SERVO_LEFT 9
#define SERVO_RIGHT 8

Servo servo_left;
Servo servo_right;

const int neutral_angle = 90;
const int spread_pulse = 90;
const int max_pulse = neutral_angle + spread_pulse;
const int min_pulse = neutral_angle - spread_pulse;

const float deadzone_percent = 10.0;

typedef struct
{
    float x;
    float y;
    float z;
} vector3_t;
vector3_t *accel_vec;
float right_side_up_threshold = -2.0;
float upside_down_threshold = -15.0;
bool is_upside_down = false;

Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
bool accel_initialized = false;

vector3_t *make_unit_vector(float x, float y, float z)
{
    float magnitude = sqrt(x * x + y * y + z * z);
    vector3_t *unit_vector = (vector3_t *)malloc(sizeof(vector3_t));
    unit_vector->x = x / magnitude;
    unit_vector->y = y / magnitude;
    unit_vector->z = z / magnitude;
    return unit_vector;
}

float scale_cycle_to_percent(float duty_cycle)
{
    float percent = -200.0 / (max_cycle - min_cycle) * (duty_cycle - min_cycle) + 100.0;
    return min(100.0f, max(-100.0f, percent));
}

int scale_percent_to_pulse(float signed_percent)
{
    if (abs(signed_percent) < deadzone_percent)
        return neutral_angle;
    float angle = (max_pulse - min_pulse) / 200.0 * (signed_percent + 100.0) + min_pulse;
    return (int)min((float)MAX_ANGLE, max((float)MIN_ANGLE, angle));
}

void channel_a_interrupt()
{
    if (digitalRead(CHANNEL_A) == HIGH)
    {
        unsigned long now_a = micros();
        pulse_duration_a = now_a - pulse_in_begin_a;
        pulse_in_begin_a = now_a;
    }
    else
    {
        pulse_in_end_a = micros();
        pulse_in_available_a = true;
    }
}

void channel_b_interrupt()
{
    if (digitalRead(CHANNEL_B) == HIGH)
    {
        unsigned long now_b = micros();
        pulse_duration_b = now_b - pulse_in_begin_b;
        pulse_in_begin_b = now_b;
    }
    else
    {
        pulse_in_end_b = micros();
        pulse_in_available_b = true;
    }
}

bool read_durations_channel_a(unsigned long &high_duration, unsigned long &low_duration)
{
    if (!pulse_in_available_a)
        return false;
    noInterrupts();
    high_duration = pulse_in_end_a - pulse_in_begin_a;
    low_duration = pulse_duration_a - high_duration;
    pulse_in_available_a = false;
    interrupts();
    return true;
}

bool read_durations_channel_b(unsigned long &high_duration, unsigned long &low_duration)
{
    if (!pulse_in_available_b)
        return false;
    noInterrupts();
    high_duration = pulse_in_end_b - pulse_in_begin_b;
    low_duration = pulse_duration_b - high_duration;
    pulse_in_available_b = false;
    interrupts();
    return true;
}

bool read_pwm(unsigned long high_duration, unsigned long low_duration, float &duty_cycle)
{
    unsigned long period = high_duration + low_duration;
    if (period < LOW_PERIOD || period > HIGH_PERIOD)
    {
        return false;
    }

    duty_cycle = (float)high_duration / (float)period * 100;
    return true;
}

void write_escs(int left_angle, int right_angle)
{
    servo_left.write(left_angle);
    servo_right.write(right_angle);
}

void initialize_escs()
{
    write_escs(neutral_angle, neutral_angle);
}

void set_builtin_led(int value)
{
    analogWrite(LED_BUILTIN, value);
}

void cycle_led(float intensity)
{
    led_cycle_index = min(255, max(0, led_cycle_direction + 1));
    set_builtin_led(led_cycle_index);
    if (led_cycle_index < 0 || led_cycle_index >= 255)
    {
        led_cycle_direction *= -1;
    }
}

void pulse_led()
{
    for (int count = 0; count < 255; count += 5)
    {
        set_builtin_led(count);
        delay(1);
    }
    for (int count = 255; count > 0; count -= 5)
    {
        set_builtin_led(count);
        delay(1);
    }
    set_builtin_led(0);
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting setup");
    accel_vec = make_unit_vector(0.0, 0.0, -1.0);
    pinMode(LED_BUILTIN, OUTPUT);
    for (int count = 0; count < 2; count++)
        pulse_led();

    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);
    initialize_escs();

    pinMode(CHANNEL_A, INPUT);
    pinMode(CHANNEL_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(CHANNEL_A), channel_a_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CHANNEL_B), channel_b_interrupt, CHANGE);

    accel_initialized = accel.begin();
    if (!accel_initialized)
    {
        Serial.println("Could not initialize accelerometer");
        for (int count = 0; count < 10; count++)
            pulse_led();
    }
    accel.setTrimOffsets(0, 0, 0);

    set_builtin_led(255);
    Serial.println("Setup complete");
}

bool get_accel(vector3_t *accel_vec)
{
    if (!accel_initialized)
        return false;
    sensors_event_t event;
    accel.getEvent(&event);

    accel_vec->x = event.acceleration.x;
    accel_vec->y = event.acceleration.y;
    accel_vec->z = event.acceleration.z;
    return true;
}

bool get_is_upside_down(vector3_t *accel_vec)
{
    if (!get_accel(accel_vec))
        return is_upside_down;

    float z = accel_vec->z;

    if (z > right_side_up_threshold)
        is_upside_down = false;
    else if (z < upside_down_threshold)
        is_upside_down = true;
    return is_upside_down;
}

void loop()
{
    unsigned long high_duration_a, low_duration_a;
    unsigned long high_duration_b, low_duration_b;
    float next_duty_cycle_a, next_duty_cycle_b;

    if (read_durations_channel_a(high_duration_a, low_duration_a) &&
        read_pwm(high_duration_a, low_duration_a, next_duty_cycle_a))
        duty_cycle_a = next_duty_cycle_a;
    if (read_durations_channel_b(high_duration_b, low_duration_b) &&
        read_pwm(high_duration_b, low_duration_b, next_duty_cycle_b))
        duty_cycle_b = next_duty_cycle_b;

    float a_percent, b_percent;
    if (duty_cycle_a == 0.0)
        a_percent = 0.0;
    else
        a_percent = scale_cycle_to_percent(duty_cycle_a);

    if (duty_cycle_b == 0.0)
        b_percent = 0.0;
    else
        b_percent = -1 * scale_cycle_to_percent(duty_cycle_b);

    led_intensity = (abs(a_percent) + abs(b_percent)) / 2.0;

    is_upside_down = get_is_upside_down(accel_vec);

    if (is_upside_down)
        a_percent *= -1;

    float left_command = a_percent - b_percent;
    float right_command = a_percent + b_percent;
    float max_command = max(abs(left_command), abs(right_command));
    if (max_command > 100.0)
    {
        left_command = left_command / max_command * 100.0;
        right_command = right_command / max_command * 100.0;
    }

    int left_pulse = scale_percent_to_pulse(left_command);
    int right_pulse = scale_percent_to_pulse(right_command);

    write_escs(left_pulse, right_pulse);

    Serial.print("A: ");
    Serial.print(duty_cycle_a, 3);
    Serial.print("\tB: ");
    Serial.print(duty_cycle_b, 3);
    Serial.print("\tX: ");
    Serial.print(accel_vec->x, 3);
    Serial.print("\tY: ");
    Serial.print(accel_vec->y, 3);
    Serial.print("\tZ: ");
    Serial.print(accel_vec->z, 3);
    Serial.print("\tUpside down: ");
    Serial.println(is_upside_down);

    delay(10);
    cycle_led(led_intensity);
}
