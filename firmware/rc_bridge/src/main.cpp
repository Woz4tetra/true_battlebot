#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoOTA.h>
#include <crsf_bridge.h>
#include <esc.h>
#include <updown_sensor.h>
#include <diagnostics_server.h>

#define MAIN_SERIAL Serial

crsf_bridge::CrsfBridge *crsf;
crsf_bridge::radio_data_t *radio_data;

#define LEFT_ESC A2
#define RIGHT_ESC A3

esc::Esc *left_esc;
esc::Esc *right_esc;

updown_sensor::UpdownSensor *accel;

diagnostics_server::DiagnosticsServer *diagnostics;
diagnostics_server::telemetry_data_t *telemetry_data;

const int NUM_PIXELS = 1;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
int rainbow_tick = 0, led_intensity = 20;

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

void stop_escs()
{
    left_esc->stop();
    right_esc->stop();
    set_led_intensity(0);
}

void setup_ota()
{
    stop_escs();

    ArduinoOTA
        .onStart([]()
                 {
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

void mix_motor_outputs(crsf_bridge::radio_data_t *radio_data, float &left_command, float &right_command)
{
    left_command = radio_data->a_percent + radio_data->b_percent;
    right_command = radio_data->a_percent - radio_data->b_percent;
    float max_command = max(abs(left_command), abs(right_command));
    if (max_command > 100.0)
    {
        left_command = left_command / max_command * 100.0;
        right_command = right_command / max_command * 100.0;
    }
}

void print_telemetry_data(diagnostics_server::telemetry_data_t *telemetry_data)
{
    MAIN_SERIAL.print("A: ");
    MAIN_SERIAL.print(telemetry_data->radio_data.a_percent, 3);
    MAIN_SERIAL.print("\tB: ");
    MAIN_SERIAL.print(telemetry_data->radio_data.b_percent, 3);
    MAIN_SERIAL.print("\tLeft: ");
    MAIN_SERIAL.print(telemetry_data->left_command, 3);
    MAIN_SERIAL.print("\tRight: ");
    MAIN_SERIAL.print(telemetry_data->right_command, 3);
    MAIN_SERIAL.print("\tX: ");
    MAIN_SERIAL.print(telemetry_data->accel_vec.x, 3);
    MAIN_SERIAL.print("\tY: ");
    MAIN_SERIAL.print(telemetry_data->accel_vec.y, 3);
    MAIN_SERIAL.print("\tZ: ");
    MAIN_SERIAL.print(telemetry_data->accel_vec.z, 3);
    MAIN_SERIAL.print("\tUpside down: ");
    MAIN_SERIAL.print(telemetry_data->is_upside_down);
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
    left_esc->begin();
    right_esc->begin();
    delay(500); // Wait for the ESCs to initialize

    pixels.begin();
    pixels.setBrightness(20);

    for (int count = 0; count < 2; count++)
        pulse_led();

    accel = new updown_sensor::UpdownSensor();
    if (!accel->begin())
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

    set_led_intensity((abs(radio_data->a_percent) + abs(radio_data->b_percent)) / 2.0);

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
        is_upside_down = accel->get_is_upside_down(radio_data->connected);
        break;

    default:
        is_upside_down = false;
        break;
    }

    if (is_upside_down)
        radio_data->a_percent *= -1;

    float left_command, right_command;
    mix_motor_outputs(radio_data, left_command, right_command);

    left_esc->write(left_command);
    right_esc->write(right_command);

    telemetry_data->radio_data = *radio_data;
    telemetry_data->is_upside_down = is_upside_down;
    telemetry_data->accel_vec = *accel->get();
    telemetry_data->max_accel_vec = *accel->get_max();
    telemetry_data->min_accel_vec = *accel->get_min();
    telemetry_data->left_command = left_command;
    telemetry_data->right_command = right_command;

    diagnostics->write_telemetry(telemetry_data);
    print_telemetry_data(telemetry_data);
}
