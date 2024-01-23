#include <Arduino.h>

#ifndef __BRIDGE_STRUCTS_H__
#define __BRIDGE_STRUCTS_H__
/**
 * Packet structs
 */

namespace bridge
{
    // The maximum size of a UDP and serial packet for this device.
    const int PACKET_MAX_LENGTH = 512;

    // HEADER
    typedef struct header
    {
        uint16_t size;
        uint8_t type; // Corresponds to HeaderType
        uint8_t device_id;
    } header_t, *header_p;

    enum HeaderType
    {
        MOTOR = 0x01,
        PING = 0x02,
        CONFIG = 0x03
        // Put more commands here if needed
    };

    // MOTOR
    typedef struct motor_command
    {
        int8_t direction;
        uint8_t speed;
    } motor_command_t, *motorCommand_p;

    typedef struct motor_description : header
    {
        uint8_t num_channels;
        motor_command_t commands[1];
    } motor_description_t, *motor_description_p;

    // PING
    typedef struct ping_info : header
    {
        uint32_t time;
    } ping_info_t, *ping_info_p;

    typedef union ping_packet
    {
        ping_info_t data;
        uint8_t bytes[sizeof(ping_info_t)];
    } ping_packet_t, *ping_packet_p;

    // CONFIG
    const int SSID_LENGTH = 33;
    const int PASSWORD_LENGTH = 64;
    const int NULL_DEVICE_ID = 0xff;
    typedef struct config_info : header
    {
        uint16_t port;
        char wifi_info[SSID_LENGTH + PASSWORD_LENGTH];
    } config_info_t, *config_info_p;

    typedef union config_packet
    {
        config_info_t data;
        uint8_t bytes[sizeof(config_info_t)];
    } config_packet_t, *config_packet_p;

}; // namespace bridge

#endif // __BRIDGE_STRUCTS_H__