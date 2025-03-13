#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <s3servo.h>
#include <AlfredoCRSF.h>
#include <Adafruit_NeoPixel.h>

#define MIN_ANGLE 0
#define MAX_ANGLE 180
#define Servo s3servo

float led_intensity = 0.0;
int led_cycle_index = 0;
int led_cycle_direction = 1;

#define CRSF_SERIAL Serial2
#define MAIN_SERIAL Serial
// AlfredoCRSF crsf;

const float min_cycle = 1000.0;
const float max_cycle = 2000.0;

#define SERVO_LEFT A2
#define SERVO_RIGHT A3

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

const int NUM_PIXELS = 1;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

vector3_t *make_unit_vector(float x, float y, float z)
{
    float magnitude = sqrt(x * x + y * y + z * z);
    vector3_t *unit_vector = (vector3_t *)malloc(sizeof(vector3_t));
    unit_vector->x = x / magnitude;
    unit_vector->y = y / magnitude;
    unit_vector->z = z / magnitude;
    return unit_vector;
}

float scale_channel_to_percent(float duty_cycle)
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
    pixels.fill(pixels.Color(value, 0, 0));
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
    MAIN_SERIAL.begin(115200);
    delay(1000);
    MAIN_SERIAL.println("Starting setup");

#if defined(NEOPIXEL_POWER)
    // If this board has a power control pin, we must set it to output and high
    // in order to enable the NeoPixels_-> We put this in an #if defined so it can
    // be reused for other boards without compilation errors
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
    MAIN_SERIAL.println("Set neopixel power");
#endif

    pixels.begin();
    pixels.setBrightness(20);

    accel_vec = make_unit_vector(0.0, 0.0, -1.0);
    for (int count = 0; count < 2; count++)
        pulse_led();

    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);
    initialize_escs();

    accel_initialized = accel.begin();
    if (!accel_initialized)
    {
        MAIN_SERIAL.println("Could not initialize accelerometer");
        for (int count = 0; count < 10; count++)
            pulse_led();
    }
    accel.setTrimOffsets(0, 0, 0);

    set_builtin_led(255);

    CRSF_SERIAL.begin(CRSF_BAUDRATE, SERIAL_8N1, RX1, TX1);
    // crsf.begin(CRSF_SERIAL);

    MAIN_SERIAL.println("Setup complete");
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
    if (CRSF_SERIAL.available())
    {
        while (CRSF_SERIAL.available())
        {
            // crsf.update();
            unsigned char c = CRSF_SERIAL.read();
            if (c == 0xC8)
            {
                MAIN_SERIAL.print('\n');
            }
            MAIN_SERIAL.print(c, HEX);
            MAIN_SERIAL.print(' ');
        }
    }

    float channel_a, channel_b;
    // if (crsf.isLinkUp())
    if (false)
    {
        MAIN_SERIAL.println("Link up");
        channel_a = 0.0;
        channel_b = 0.0;
    }
    else
    {
        channel_a = 0.0;
        channel_b = 0.0;
    }

    float a_percent, b_percent;
    if (channel_a == 0.0)
        a_percent = 0.0;
    else
        a_percent = scale_channel_to_percent(channel_a);

    if (channel_b == 0.0)
        b_percent = 0.0;
    else
        b_percent = -1 * scale_channel_to_percent(channel_b);

    led_intensity = (abs(a_percent) + abs(b_percent)) / 2.0;

    is_upside_down = get_is_upside_down(accel_vec);

    if (is_upside_down)
        a_percent *= -1;

    float left_command = a_percent + b_percent;
    float right_command = a_percent - b_percent;
    float max_command = max(abs(left_command), abs(right_command));
    if (max_command > 100.0)
    {
        left_command = left_command / max_command * 100.0;
        right_command = right_command / max_command * 100.0;
    }

    int left_pulse = scale_percent_to_pulse(left_command);
    int right_pulse = scale_percent_to_pulse(right_command);

    write_escs(left_pulse, right_pulse);

    // MAIN_SERIAL.print("A: ");
    // MAIN_SERIAL.print(channel_a, 3);
    // MAIN_SERIAL.print("\tB: ");
    // MAIN_SERIAL.print(channel_b, 3);
    // MAIN_SERIAL.print("\tX: ");
    // MAIN_SERIAL.print(accel_vec->x, 3);
    // MAIN_SERIAL.print("\tY: ");
    // MAIN_SERIAL.print(accel_vec->y, 3);
    // MAIN_SERIAL.print("\tZ: ");
    // MAIN_SERIAL.print(accel_vec->z, 3);
    // MAIN_SERIAL.print("\tUpside down: ");
    // MAIN_SERIAL.println(is_upside_down);

    delay(10);
    cycle_led(led_intensity);
}
