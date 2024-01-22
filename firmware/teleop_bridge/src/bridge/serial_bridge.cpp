#include <bridge/serial_bridge.h>

using namespace bridge;
using namespace serial_bridge;
using namespace persistent_config;

SerialBridge::SerialBridge(config_info_p config, char *buffer, PersistentConfig *persistent_config) : BaseBridge()
{
    device_config_ = config;
    state_ = WAITING_FOR_CONFIG;
    buffer_ = buffer;
    persistent_config_ = persistent_config;
}

bool SerialBridge::update()
{
    // Serial doesn't process packets once config is set.
    if (state_ == WAITING_FOR_CONFIG)
    {
        if (device_config_->device_id != NULL_DEVICE_ID)
        {
            state_ = READY;
        }
        else if (process_serial_packet())
        {
            return true;
        }
    }
    return false;
}

void SerialBridge::set_motor(uint8_t channel, int velocity)
{
    Serial.println("Setting motor over serial is not allowed.");
}

void SerialBridge::respond_to_ping(ping_packet_p packet)
{
    Serial.println("Responding to ping over serial is not allowed.");
}

void SerialBridge::respond_to_config(config_info_p config_info)
{
    // Copy the config into the shared device_config struct
    *device_config_ = *config_info;

    // Write the config to EEPROM
    persistent_config_->write(device_config_);

    // Ensure the config was written correctly
    if (!persistent_config_->is_set())
    {
        device_config_->device_id = NULL_DEVICE_ID;
        Serial.println("Failed to write config to EEPROM");
        return;
    }
    else
    {
        Serial.println("Config set in EEPROM");
    }

    uint16_t packet_size = sizeof(config_info_t);
    config_packet_t packet;
    packet.data = *config_info;

    // Respond to the config packet with the received packet
    Serial.write(SERIAL_PACKET_C0);
    Serial.write(SERIAL_PACKET_C1);
    Serial.write(packet_size & 0xFF);
    Serial.write((packet_size >> 8) & 0xFF);
    Serial.write(packet.bytes, packet_size);
}

int SerialBridge::get_serial_packet_length()
{
    if (Serial.available() < 4)
    {
        return 0;
    }
    char c = Serial.read();
    if (c == SERIAL_PACKET_C0)
    {
        c = Serial.read();
        if (c == SERIAL_PACKET_C1)
        {
            char size_1 = Serial.read();
            char size_2 = Serial.read();
            return (size_2 << 8) | size_1;
        }
    }
    return 0;
}

bool SerialBridge::process_serial_packet()
{
    if (serial_read_length_ > 0)
    {
        if (read_length_ < serial_read_length_)
        {
            buffer_[read_length_++] = Serial.read();
        }
        else
        {
            if (process_packet(buffer_, read_length_))
            {
                return true;
            }
            read_length_ = 0;
            serial_read_length_ = 0;
        }
    }
    else
    {
        serial_read_length_ = get_serial_packet_length();
    }
    return false;
}