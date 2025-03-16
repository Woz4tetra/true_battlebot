#include <diagnostics_server.h>

using namespace diagnostics_server;
const char *SSID = "MR-STABS";
const char *PASSWORD = "havocbots";
const char *SEND_IP = "192.168.4.2";
const int PORT = 4176;

DiagnosticsServer::DiagnosticsServer()
{
}

void DiagnosticsServer::begin()
{
    WiFi.softAP(SSID, PASSWORD);
    udp.begin(PORT);
}

String DiagnosticsServer::get_ip()
{
    return WiFi.softAPIP().toString();
}

void DiagnosticsServer::write_telemetry(telemetry_data_t *telemetry_data)
{
    JsonDocument doc;
    doc["a_percent"] = telemetry_data->radio_data.a_percent;
    doc["b_percent"] = telemetry_data->radio_data.b_percent;
    doc["armed"] = telemetry_data->radio_data.armed;
    doc["lifter_command"] = telemetry_data->radio_data.lifter_command;
    doc["connected"] = telemetry_data->radio_data.connected;
    doc["flip_switch_state"] = telemetry_data->radio_data.flip_switch_state;
    doc["is_upside_down"] = telemetry_data->is_upside_down;
    doc["accel"]["x"] = telemetry_data->accel_vec.x;
    doc["accel"]["y"] = telemetry_data->accel_vec.y;
    doc["accel"]["z"] = telemetry_data->accel_vec.z;
    doc["max_accel"]["x"] = telemetry_data->max_accel_vec.x;
    doc["max_accel"]["y"] = telemetry_data->max_accel_vec.y;
    doc["max_accel"]["z"] = telemetry_data->max_accel_vec.z;
    doc["min_accel"]["x"] = telemetry_data->min_accel_vec.x;
    doc["min_accel"]["y"] = telemetry_data->min_accel_vec.y;
    doc["min_accel"]["z"] = telemetry_data->min_accel_vec.z;
    doc["left_command"] = telemetry_data->left_command;
    doc["right_command"] = telemetry_data->right_command;
    doc["time"] = millis();
    doc["sequence_number"] = sequence_number;

    udp.beginPacket(SEND_IP, PORT);
    serializeJson(doc, udp);
    udp.endPacket();
    sequence_number++;
}
