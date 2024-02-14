#include <bridge/base_bridge.h>

using namespace bridge;

bool BaseBridge::process_motor_packet(char *packet, int packet_size)
{
    // sizeof(motor_description_t) defines the minimum size of a motor packet.
    // If there are more than one channels, the packet will be larger than this.
    if (packet_size < sizeof(motor_description_t))
    {
        Serial.println("Motor packet too small");
        return false;
    }

    // Cast packet to motor_description_p so we can access the fields.
    // The memory layout of the packet should be the same as the struct.
    motor_description_p motor_desc = (motor_description_p)packet;

    // Set the motor speed and direction for each channel
    for (int channel = 0; channel < motor_desc->num_channels; channel++)
    {
        int16_t velocity_command = motor_desc->commands[channel].velocity;
        float velocity_mps = velocity_command / 1000.0;
        set_motor(channel, velocity_mps);
    }

    return true;
}

bool BaseBridge::process_ping_packet(char *packet, int packet_size)
{
    // Check that the packet size matches the size of the ping_info_t struct
    if (packet_size < sizeof(ping_info_t))
    {
        Serial.println("Ping packet too small");
        return false;
    }

    // Cast packet to ping_info_p so we can access the fields.
    ping_info_p ping_info = (ping_info_p)packet;
    ping_info->size = sizeof(ping_info_t);
    ping_info->type = PING;
    ping_info->device_id = device_config_->device_id;
    ping_info->time = ping_info->time; // Echo the time back to the sender
    respond_to_ping(ping_info);
    return true;
}

bool BaseBridge::process_config_packet(char *packet, int packet_size)
{
    // Check that the packet size matches the size of the config_info_t struct
    if (packet_size < sizeof(config_info_t))
    {
        Serial.print("Config packet too small. Expected ");
        Serial.print(sizeof(config_info_t));
        Serial.print(" bytes, got ");
        Serial.print(packet_size);
        Serial.println(" bytes");
        return false;
    }

    respond_to_config((config_info_p)packet);
    return true;
}

bool BaseBridge::process_packet(char *packet, int packet_size)
{
    // Check that the packet size is at least the size of a header.
    // Other structs extend the header struct and thus will be at least this size.
    if (packet_size < sizeof(header_t))
    {
        Serial.println("Packet header too small");
        return false;
    }
    // Cast packet to header_p so we can access the fields.
    header_p header = (header_p)packet;

    // Check that the packet size matches the size in the header
    if (packet_size != header->size)
    {
        Serial.print("Packet length doesn't match header. Expected ");
        Serial.print(header->size);
        Serial.print(" bytes, got ");
        Serial.print(packet_size);
        Serial.println(" bytes");
        return false;
    }

    // If the device ID is NULL_DEVICE_ID, the config is not set. Only process config packets.
    if (device_config_->device_id == NULL_DEVICE_ID)
    {
        if (header->type == CONFIG)
        {
            return process_config_packet(packet, packet_size);
        }
        else
        {
            Serial.println("No device ID set. Cannot process packet");
            return false;
        }
    }

    // Check that the device ID matches the device ID in the header.
    // If the packet is a ping packet, the device ID doesn't matter.
    if (header->device_id != device_config_->device_id && header->type != PING)
    {
        return false;
    }

    // Process the packet based on the type in the header
    switch (header->type)
    {
    case MOTOR:
        return process_motor_packet(packet, packet_size);
    case PING:
        return process_ping_packet(packet, packet_size);
    case CONFIG:
        return process_config_packet(packet, packet_size);
    default:
        return false;
    }
}
