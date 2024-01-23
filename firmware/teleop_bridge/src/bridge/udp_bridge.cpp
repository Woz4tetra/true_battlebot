#include <bridge/udp_bridge.h>

using namespace bridge;
using namespace udp_bridge;

UdpBridge::UdpBridge(config_info_p config, char *buffer, base_controller::BaseController *controller) : BaseBridge()
{
    device_config_ = config;
    state_ = INIT;
    controller_ = controller;
    buffer_ = buffer;
}

bool UdpBridge::update()
{
    switch (state_)
    {
    case INIT:
        init_callback();
        break;
    case CONNECTING:
        connecting_callback();
        break;
    case READY:
        return ready_callback();
    default:
        break;
    }
    return false;
}

void UdpBridge::init_callback()
{
    char *ssid = device_config_->wifi_info;
    char *password = ssid + SSID_LENGTH;
    WiFi.begin(ssid, password);
    state_ = CONNECTING;
}

void UdpBridge::connecting_callback()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        return;
    }
    state_ = READY;

    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());

    // Setup UDP
    UDP.begin(device_config_->port);
    Serial.print("Listening on UDP port ");
    Serial.println(device_config_->port);
}

bool UdpBridge::ready_callback()
{
    // Check for UDP packets. If a packet is received, process it.
    bool processed_packet = false;
    if (UDP.parsePacket())
    {
        read_length_ = UDP.read(buffer_, PACKET_MAX_LENGTH);
        if (process_packet(buffer_, read_length_))
        {
            processed_packet = true;
        }
    }
    return processed_packet;
}

void UdpBridge::set_motor(uint8_t channel, int velocity)
{
    controller_->set_motor(channel, velocity);
}

void UdpBridge::respond_to_config(config_info_p config_info)
{
    Serial.println("Setting config over UDP is not allowed.");
}

void UdpBridge::respond_to_ping(ping_packet_p packet)
{
    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
    UDP.write(packet->bytes, packet->data.size); // Use union to interpret struct as bytes
    UDP.endPacket();
}