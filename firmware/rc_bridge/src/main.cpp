#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_DotStar.h>
#include <SPI.h>

#define NUMPIXELS 1
#define DATAPIN 7
#define CLOCKPIN 8
Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);
unsigned int rainbow_color_index = 0;

#define CHANNEL_A 2
#define CHANNEL_B 4

const unsigned long PWM_TIMEOUT = 50000;
const unsigned long LOW_PERIOD = 19000;  // 19 millisecond (nominally 50 Hz)
const unsigned long HIGH_PERIOD = 21000; // 21 millisecond (nominally 50 Hz)
volatile unsigned long pulse_in_begin_a = 0, pulse_in_begin_b = 0;
volatile unsigned long pulse_in_end_a = 0, pulse_in_end_b = 0;
volatile unsigned long pulse_duration_a = 0, pulse_duration_b = 0;
volatile bool pulse_in_available_a = false, pulse_in_available_b = false;
float duty_cycle_a, duty_cycle_b;

const float min_cycle = 5.0;
const float max_cycle = 10.0;

#define SERVO_LEFT 1
#define SERVO_RIGHT 3

Servo servo_left;
Servo servo_right;

const int neutral_pulse = 1500;
const int spread_pulse = 500;
const int max_pulse = neutral_pulse + spread_pulse;
const int min_pulse = neutral_pulse - spread_pulse;

float scale_cycle_to_percent(float duty_cycle)
{
    float percent = -200.0 / (max_cycle - min_cycle) * (duty_cycle - min_cycle) + 100.0;
    return min(100.0, max(-100.0, percent));
}

int scale_percent_to_pulse(float signed_percent)
{
    float angle = (max_pulse - min_pulse) / 200.0 * (signed_percent + 100.0) + min_pulse;
    return (int)min(MAX_PULSE_WIDTH, max(MIN_PULSE_WIDTH, angle));
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

void cycle_rainbow_led(float intensity)
{
    strip.setPixelColor(0, strip.ColorHSV(rainbow_color_index, 255, 255));
    strip.setBrightness((int)(intensity * 0.01 * 255));
    strip.show();
    rainbow_color_index = (rainbow_color_index + 1000) % 65535;
}

void setup()
{
    strip.begin();
    strip.show();

    strip.setPixelColor(0, 0xFF0000); // red
    strip.show();

    Serial.begin(115200);
    pinMode(CHANNEL_A, INPUT);
    pinMode(CHANNEL_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(CHANNEL_A), channel_a_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CHANNEL_B), channel_b_interrupt, CHANGE);

    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);

    Serial.println("Initializing ESCs");

    strip.setPixelColor(0, 0x00FF00); // green
    strip.show();

    initialize_escs();

    strip.setPixelColor(0, 0x0000FF); // blue
    strip.show();
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

    float a_percent = scale_cycle_to_percent(duty_cycle_a);
    float b_percent = -1 * scale_cycle_to_percent(duty_cycle_b);

    float led_intensity = (abs(a_percent) + abs(b_percent)) / 2.0;

    float left_command = a_percent + b_percent;
    float right_command = a_percent - b_percent;

    Serial.print("L: ");
    Serial.print(left_command);
    Serial.print(" R: ");
    Serial.println(right_command);

    int left_pulse = scale_percent_to_pulse(left_command);
    int right_pulse = scale_percent_to_pulse(right_command);

    write_escs(left_pulse, right_pulse);

    cycle_rainbow_led(led_intensity);
    delay(10);
}
