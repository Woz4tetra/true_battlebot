#include <bridge/udp_bridge.h>

using namespace bridge;
using namespace udp_bridge;
using namespace imu_sensor;

UdpBridge::UdpBridge(
    config_info_p config,
    char *read_buffer,
    uint8_t *write_buffer,
    base_controller::BaseController *controller,
    ImuSensor *imu_sensor_inst) : BaseBridge()
{
    device_config_ = config;
    state_ = INIT;
    controller_ = controller;
    read_buffer_ = read_buffer;
    write_buffer_ = write_buffer;
    imu_sensor_inst_ = imu_sensor_inst;
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
        read_length_ = UDP.read(read_buffer_, PACKET_MAX_LENGTH);
        if (process_packet(read_buffer_, read_length_))
        {
            processed_packet = true;
        }
    }
    send_imu();
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

void UdpBridge::respond_to_ping(ping_info_p packet)
{
    write_buffer_ = (uint8_t *)packet;
    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
    UDP.write(write_buffer_, sizeof(ping_info_t));
    UDP.endPacket();
}

void UdpBridge::send_imu()
{
    if (!imu_sensor_inst_->is_connected())
    {
        return;
    }
    bridge::imu_data_p imu_struct = (bridge::imu_data_p)write_buffer_;
    if (imu_sensor_inst_->get_imu_data(imu_struct))
    {
        imu_struct->size = sizeof(bridge::imu_data_t);
        imu_struct->type = bridge::IMU;
        imu_struct->device_id = device_config_->device_id;
        UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
        UDP.write(write_buffer_, sizeof(bridge::imu_data_t));
        UDP.endPacket();
    }
}