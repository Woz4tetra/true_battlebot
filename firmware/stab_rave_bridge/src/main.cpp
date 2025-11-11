#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoOTA.h>
#include <crsf_bridge.h>
#include <esc.h>
#include <updown_sensor.h>
#include <diagnostics_server.h>
#include <s3servo.h>

#define MAIN_SERIAL Serial
#define Servo s3servo

crsf_bridge::CrsfBridge *crsf;
crsf_bridge::radio_data_t *radio_data;

#define LEFT_ESC A6
#define RIGHT_ESC A2
#define BACK_ESC A3
#define LIFTER_SERVO_PIN A7

esc::Esc *left_esc;
esc::Esc *right_esc;
esc::Esc *back_esc;
Servo lifter_servo;

updown_sensor::UpdownSensor *updown;

diagnostics_server::DiagnosticsServer *diagnostics;
diagnostics_server::telemetry_data_t *telemetry_data;

const int NUM_PIXELS = 1;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
int rainbow_tick = 0, led_intensity = 20;

bool is_loading_firmware = false;

const float WHEEL_ANGLES[3] = {240.0f, 0.0f, 120.0f};
const float DEG2RAD = M_PI / 180.0;

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

void set_led_intensity(float percent)
{
    led_intensity = (int)(2.35 * min(100.0f, max(-100.0f, percent))) + 20;
}

int convert_percent_to_servo(float percent)
{
    // Convert -100 to 100 percent to 0 to 180 degrees
    return (int)(0.9 * (percent + 100.0));
}

void stop_escs()
{
    left_esc->stop();
    right_esc->stop();
    back_esc->stop();
    set_led_intensity(0);
}

void setup_ota()
{
    stop_escs();

    ArduinoOTA
        .onStart([]()
                 {
    is_loading_firmware = true;
    stop_escs();
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    MAIN_SERIAL.println("Start updating " + type); })
        .onEnd([]()
               { MAIN_SERIAL.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { MAIN_SERIAL.printf("Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                 {
    MAIN_SERIAL.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      MAIN_SERIAL.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      MAIN_SERIAL.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      MAIN_SERIAL.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      MAIN_SERIAL.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      MAIN_SERIAL.println("End Failed");
    } });

    ArduinoOTA.begin();
}

void mix_motor_outputs(crsf_bridge::radio_data_t *radio_data, float &left_command, float &right_command, float &back_command)
{
    float linear_vx = -1 * radio_data->a_percent;
    float angular_v = radio_data->b_percent;
    float linear_vy = -1 * radio_data->c_percent;

    left_command = linear_vx * sin(WHEEL_ANGLES[0] * DEG2RAD) + linear_vy * cos(WHEEL_ANGLES[0] * DEG2RAD) + angular_v;
    right_command = linear_vx * sin(WHEEL_ANGLES[1] * DEG2RAD) + linear_vy * cos(WHEEL_ANGLES[1] * DEG2RAD) + angular_v;
    back_command = linear_vx * sin(WHEEL_ANGLES[2] * DEG2RAD) + linear_vy * cos(WHEEL_ANGLES[2] * DEG2RAD) + angular_v;

    float max_command = max(abs(left_command), max(abs(right_command), abs(back_command)));
    if (max_command > 100.0)
    {
        left_command = left_command / max_command * 100.0;
        right_command = right_command / max_command * 100.0;
        back_command = back_command / max_command * 100.0;
    }
}

void print_telemetry_data(diagnostics_server::telemetry_data_t *telemetry_data)
{
    MAIN_SERIAL.print("A: ");
    MAIN_SERIAL.print(telemetry_data->radio_data.a_percent, 3);
    MAIN_SERIAL.print("\tB: ");
    MAIN_SERIAL.print(telemetry_data->radio_data.b_percent, 3);
    MAIN_SERIAL.print("\tC: ");
    MAIN_SERIAL.print(telemetry_data->radio_data.c_percent, 3);
    MAIN_SERIAL.print("\tLeft: ");
    MAIN_SERIAL.print(telemetry_data->left_command, 3);
    MAIN_SERIAL.print("\tRight: ");
    MAIN_SERIAL.print(telemetry_data->right_command, 3);
    MAIN_SERIAL.print("\tBack: ");
    MAIN_SERIAL.print(telemetry_data->back_command, 3);
    MAIN_SERIAL.print("\tX: ");
    MAIN_SERIAL.print(telemetry_data->grav_vec.x, 3);
    MAIN_SERIAL.print("\tY: ");
    MAIN_SERIAL.print(telemetry_data->grav_vec.y, 3);
    MAIN_SERIAL.print("\tZ: ");
    MAIN_SERIAL.print(telemetry_data->grav_vec.z, 3);
    MAIN_SERIAL.print("\tUpside down: ");
    MAIN_SERIAL.print(telemetry_data->is_upside_down);
    MAIN_SERIAL.print("\tLifter: ");
    MAIN_SERIAL.print(telemetry_data->lifter_command);
    MAIN_SERIAL.print("\n");
}

void setup()
{
    MAIN_SERIAL.begin(115200);
    MAIN_SERIAL.println("Starting setup");

#if defined(NEOPIXEL_POWER)
    // If this board has a power control pin, we must set it to output and high
    // in order to enable the NeoPixels_-> We put this in an #if defined so it can
    // be reused for other boards without compilation errors
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
    MAIN_SERIAL.println("Set neopixel power");
#endif
    left_esc = new esc::Esc(LEFT_ESC, 0);
    right_esc = new esc::Esc(RIGHT_ESC, 1);
    back_esc = new esc::Esc(BACK_ESC, 2);
    left_esc->begin();
    right_esc->begin();
    back_esc->begin();
    lifter_servo.attach(LIFTER_SERVO_PIN, 3);
    delay(200); // Wait for the ESCs to initialize

    pixels.begin();
    pixels.setBrightness(20);

    for (int count = 0; count < 2; count++)
        pulse_led();

    Wire1.begin();
    updown = new updown_sensor::UpdownSensor();
    if (!updown->begin())
    {
        for (int count = 0; count < 10; count++)
            pulse_led();
    }
    set_builtin_led(255);
    radio_data = (crsf_bridge::radio_data_t *)malloc(sizeof(crsf_bridge::radio_data_t));
    crsf = new crsf_bridge::CrsfBridge();
    crsf->begin();

    diagnostics = new diagnostics_server::DiagnosticsServer();
    diagnostics->begin();
    telemetry_data = (diagnostics_server::telemetry_data_t *)malloc(sizeof(diagnostics_server::telemetry_data_t));

    setup_ota();

    MAIN_SERIAL.println("Setup complete");
}

void loop()
{
    delay(10);
    cycle_rainbow_led(rainbow_tick, led_intensity);
    rainbow_tick = (rainbow_tick + 5) % 255;

    ArduinoOTA.handle();
    if (is_loading_firmware)
        return;

    if (!crsf->update(radio_data))
    {
        MAIN_SERIAL.println("Disconnected from radio");
        stop_escs();
        return;
    }

    if (!radio_data->armed)
    {
        MAIN_SERIAL.println("Disarmed.");
        stop_escs();
        return;
    }

    set_led_intensity((abs(radio_data->a_percent) + abs(radio_data->b_percent) + abs(radio_data->c_percent)) / 3.0);

    bool is_upside_down;
    switch (radio_data->flip_switch_state)
    {
    case crsf_bridge::UP:
        is_upside_down = true;
        break;
    case crsf_bridge::MIDDLE:
        is_upside_down = false;
        break;
    case crsf_bridge::DOWN:
        is_upside_down = updown->get_is_upside_down(radio_data->connected);
        break;

    default:
        is_upside_down = false;
        break;
    }

    updown_sensor::vector3_t *orientation = updown->get_orientation();

    if (is_upside_down)
    {
        radio_data->a_percent *= -1;
        radio_data->c_percent *= -1;
    }
    float left_command, right_command, back_command;
    mix_motor_outputs(radio_data, left_command, right_command, back_command);
    int lifter_angle = convert_percent_to_servo(radio_data->lifter_command);

    left_esc->write(left_command);
    right_esc->write(back_command);
    back_esc->write(right_command);
    lifter_servo.write(lifter_angle);

    telemetry_data->radio_data = *radio_data;
    telemetry_data->is_upside_down = is_upside_down;
    telemetry_data->grav_vec = *updown->get();
    telemetry_data->max_grav_vec = *updown->get_max();
    telemetry_data->min_grav_vec = *updown->get_min();
    telemetry_data->left_command = left_command;
    telemetry_data->right_command = back_command;
    telemetry_data->back_command = right_command;
    telemetry_data->left_scaled_command = left_esc->get_command();
    telemetry_data->right_scaled_command = right_esc->get_command();
    telemetry_data->back_scaled_command = back_esc->get_command();
    telemetry_data->lifter_command = lifter_angle;
    telemetry_data->orientation = *orientation;

    if (radio_data->button_state)
        diagnostics->write_telemetry(telemetry_data);
    print_telemetry_data(telemetry_data);

    crsf->send_telemetry(
        orientation->x,
        orientation->y,
        orientation->z);
}
