#include <diagnostics_server.h>

using namespace diagnostics_server;
const char *SSID = "STAB-RAVE";
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
    doc["c_percent"] = telemetry_data->radio_data.c_percent;
    doc["armed"] = telemetry_data->radio_data.armed;
    doc["lifter_command"] = telemetry_data->radio_data.lifter_command;
    doc["connected"] = telemetry_data->radio_data.connected;
    doc["button_state"] = telemetry_data->radio_data.button_state;
    doc["flip_switch_state"] = telemetry_data->radio_data.flip_switch_state;
    doc["is_upside_down"] = telemetry_data->is_upside_down;
    doc["gravity"]["x"] = telemetry_data->grav_vec.x;
    doc["gravity"]["y"] = telemetry_data->grav_vec.y;
    doc["gravity"]["z"] = telemetry_data->grav_vec.z;
    doc["max_gravity"]["x"] = telemetry_data->max_grav_vec.x;
    doc["max_gravity"]["y"] = telemetry_data->max_grav_vec.y;
    doc["max_gravity"]["z"] = telemetry_data->max_grav_vec.z;
    doc["min_gravity"]["x"] = telemetry_data->min_grav_vec.x;
    doc["min_gravity"]["y"] = telemetry_data->min_grav_vec.y;
    doc["min_gravity"]["z"] = telemetry_data->min_grav_vec.z;
    doc["orientation"]["x"] = telemetry_data->orientation.x;
    doc["orientation"]["y"] = telemetry_data->orientation.y;
    doc["orientation"]["z"] = telemetry_data->orientation.z;
    doc["left_command"] = telemetry_data->left_command;
    doc["right_command"] = telemetry_data->right_command;
    doc["back_command"] = telemetry_data->back_command;
    doc["left_scaled_command"] = telemetry_data->left_scaled_command;
    doc["right_scaled_command"] = telemetry_data->right_scaled_command;
    doc["back_scaled_command"] = telemetry_data->back_scaled_command;
    doc["time"] = millis();
    doc["sequence_number"] = sequence_number;

    udp.beginPacket(SEND_IP, PORT);
    serializeJson(doc, udp);
    udp.endPacket();
    sequence_number++;
}
