#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

namespace updown_sensor
{
    typedef struct
    {
        float x;
        float y;
        float z;
    } vector3_t;

    const float RIGHT_SIDE_UP_THRESHOLD = 1.0;
    const float UPSIDE_DOWN_THRESHOLD = -1.0;
    const uint32_t RECONNECT_INTERVAL = 1000;
    const uint32_t SAMPLE_INTERVAL = 100;

    class UpdownSensor
    {
    private:
        Adafruit_BNO055 *sensor;
        bool initialized = false;
        vector3_t *grav_vec;
        vector3_t *max_grav_vec;
        vector3_t *min_grav_vec;
        bool is_upside_down = false;
        uint32_t reconnect_timer = 0;
        uint32_t sample_timer = 0;
        vector3_t *make_unit_vector(float x, float y, float z);
        bool update_sensor(bool radio_connected);
        vector3_t *init_vector3(float x, float y, float z);

    public:
        UpdownSensor();
        bool begin();
        bool get_is_upside_down(bool radio_connected);
        vector3_t *get() { return grav_vec; }
        vector3_t *get_max() { return max_grav_vec; }
        vector3_t *get_min() { return min_grav_vec; }
    };
}