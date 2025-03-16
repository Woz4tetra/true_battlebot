#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>

namespace updown_sensor
{
    typedef struct
    {
        float x;
        float y;
        float z;
    } vector3_t;

    const float RIGHT_SIDE_UP_THRESHOLD = -2.0;
    const float UPSIDE_DOWN_THRESHOLD = -15.0;
    const uint32_t RECONNECT_INTERVAL = 1000;

    class UpdownSensor
    {
    private:
        Adafruit_ADXL375 *accel;
        bool initialized = false;
        vector3_t *accel_vec;
        bool is_upside_down = false;
        uint32_t reconnect_timer = 0;
        vector3_t *make_unit_vector(float x, float y, float z);
        bool get_accel(bool radio_connected);

    public:
        UpdownSensor();
        bool begin();
        bool get_is_upside_down(bool radio_connected);
        vector3_t *get() { return accel_vec; }
    };
}