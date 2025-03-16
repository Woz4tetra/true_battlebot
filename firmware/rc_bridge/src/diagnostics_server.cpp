#include <diagnostics_server.h>

using namespace diagnostics_server;

DiagnosticsServer::DiagnosticsServer()
{
}

void DiagnosticsServer::begin()
{
    const char *SSID = "MR-STABS";
    const char *PASSWORD = "havocbots";
    const int PORT = 4176;

    WiFi.softAP(SSID, PASSWORD);
    udp.begin(PORT);
}

String DiagnosticsServer::get_ip()
{
    return WiFi.softAPIP().toString();
}
