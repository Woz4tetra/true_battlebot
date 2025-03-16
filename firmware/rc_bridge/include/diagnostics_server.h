#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

namespace diagnostics_server
{
    class DiagnosticsServer
    {
    private:
        WiFiUDP udp;

    public:
        DiagnosticsServer();
        void begin();
        String get_ip();
    };
}