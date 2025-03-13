#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <AlfredoCRSF.h>
#include <Adafruit_NeoPixel.h>

#define RXD1 18
#define TXD1 17
#define CRSF_SERIAL Serial1
#define MAIN_SERIAL Serial
AlfredoCRSF crsf;

const float min_cycle = 0.0;
const float max_cycle = 2000.0;

const int ESC_LEFT = 9;
const int ESC_LEFT_CHANNEL = 2;
const int ESC_RIGHT = 8;
const int ESC_RIGHT_CHANNEL = 3;
const int PWM_FREQ = 50;
const int PWM_RESOLUTION = 16;

const int neutral_pulse = 1250;
const int max_pulse = 2000;
const int min_pulse = 500;

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
uint32_t reconnect_timer = 0;
const uint32_t RECONNECT_INTERVAL = 1000;

const int NUM_PIXELS = 1;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
int rainbow_tick = 0;

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
        return neutral_pulse;
    float pulse = (max_pulse - min_pulse) / 200.0 * (signed_percent + 100.0) + min_pulse;
    return (int)min((float)max_pulse, max((float)min_pulse, pulse));
}

void write_escs(int left_duty_cycle, int right_duty_cycle)
{
    ledcWrite(ESC_LEFT_CHANNEL, left_duty_cycle);
    ledcWrite(ESC_RIGHT_CHANNEL, right_duty_cycle);
}

void initialize_escs()
{
    ledcSetup(ESC_LEFT, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(ESC_RIGHT, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(ESC_LEFT, ESC_LEFT_CHANNEL);
    ledcAttachPin(ESC_RIGHT, ESC_RIGHT_CHANNEL);

    delay(500);
    write_escs(neutral_pulse, neutral_pulse);
}

void set_builtin_led(int value)
{
    pixels.fill(pixels.Color(value, 0, 0));
    pixels.show();
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

void cycle_rainbow_led(int tick, int brightness)
{
    for (int i = 0; i < NUM_PIXELS; i++)
    {
        pixels.setPixelColor(i, pixels.ColorHSV(tick * 65536 / 255 + i * 65536 / NUM_PIXELS, 255, 255));
    }
    pixels.setBrightness(brightness);
    pixels.show();
}

void connect_accelerometer()
{
    if (accel.begin())
    {
        MAIN_SERIAL.println("Accelerometer reconnected");
        accel_initialized = true;
        accel.setTrimOffsets(0, 0, 0);
    }
    else
    {
        MAIN_SERIAL.println("Could not reconnect accelerometer");
        accel_initialized = false;
        for (int count = 0; count < 10; count++)
            pulse_led();
    }
}

bool get_accel(vector3_t *accel_vec)
{
    if (!accel_initialized && !crsf.isLinkUp())
    {
        if (millis() - reconnect_timer > RECONNECT_INTERVAL)
        {
            connect_accelerometer();
            reconnect_timer = millis();
        }
        return false;
    }
    sensors_event_t event;
    uint32_t start_time = millis();
    accel.getEvent(&event);
    uint32_t end_time = millis();

    if (end_time - start_time > 250)
    {
        accel_initialized = false;
        MAIN_SERIAL.println("Could not get accelerometer event");
        return false;
    }

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

    initialize_escs();
    connect_accelerometer();
    set_builtin_led(255);
    CRSF_SERIAL.begin(CRSF_BAUDRATE, SERIAL_8N1, RXD1, TXD1);
    crsf.begin(CRSF_SERIAL);

    MAIN_SERIAL.println("Setup complete");
}

void loop()
{
    crsf.update();

    float channel_a, channel_b;
    if (crsf.isLinkUp())
    {
        const crsf_channels_t *channels = crsf.getChannelsPacked();
        channel_a = channels->ch0;
        channel_b = channels->ch3;
    }
    else
    {
        MAIN_SERIAL.println("CRSF link down");
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

    int led_intensity = (int)(2.35 * (abs(a_percent) + abs(b_percent)) / 2.0) + 20;

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

    MAIN_SERIAL.print("A: ");
    MAIN_SERIAL.print(channel_a, 3);
    MAIN_SERIAL.print("\tB: ");
    MAIN_SERIAL.print(channel_b, 3);
    MAIN_SERIAL.print("\tLeft: ");
    MAIN_SERIAL.print(left_pulse);
    MAIN_SERIAL.print("\tRight: ");
    MAIN_SERIAL.print(right_pulse);
    MAIN_SERIAL.print("\tX: ");
    MAIN_SERIAL.print(accel_vec->x, 3);
    MAIN_SERIAL.print("\tY: ");
    MAIN_SERIAL.print(accel_vec->y, 3);
    MAIN_SERIAL.print("\tZ: ");
    MAIN_SERIAL.print(accel_vec->z, 3);
    MAIN_SERIAL.print("\tUpside down: ");
    MAIN_SERIAL.println(is_upside_down);

    delay(10);
    cycle_rainbow_led(rainbow_tick, led_intensity);
    rainbow_tick = (rainbow_tick + 5) % 255;
}
