#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <crsf_bridge.h>
#include <updown_sensor.h>

namespace diagnostics_server
{
    typedef struct telemetry_data
    {
        crsf_bridge::radio_data_t radio_data;
        bool is_upside_down;
        updown_sensor::vector3_t grav_vec, max_grav_vec, min_grav_vec;
        float left_command, right_command, back_command;
        float left_scaled_command, right_scaled_command, back_scaled_command;
        int lifter_angle;
        float lifter_command;
        updown_sensor::vector3_t orientation;
    } telemetry_data_t;

    class DiagnosticsServer
    {
    private:
        WiFiUDP udp;
        int sequence_number = 0;

    public:
        DiagnosticsServer();
        void begin();
        void write_telemetry(telemetry_data_t *telemetry_data);
        String get_ip();
    };
}