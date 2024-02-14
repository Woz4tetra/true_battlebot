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
        CONFIG = 0x03,
        IMU = 0x04,
        // Put more commands here if needed
    };

    // MOTOR
    typedef struct motor_command
    {
        int16_t velocity; // ground meters per second * 1000
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

    // CONFIG
    const int SSID_LENGTH = 33;
    const int PASSWORD_LENGTH = 64;
    const int NULL_DEVICE_ID = 0xff;
    typedef struct config_info : header
    {
        uint16_t port;
        char wifi_info[SSID_LENGTH + PASSWORD_LENGTH];
    } config_info_t, *config_info_p;

    // IMU
    typedef struct vector_3d
    {
        float x;
        float y;
        float z;
    } vector_3d_t, *vector_3d_p;

    typedef struct imu_data : header
    {
        vector_3d_t accel;
        vector_3d_t gyro;
        vector_3d_t orientation;
    } imu_data_t, *imu_data_p;

}; // namespace bridge

#endif // __BRIDGE_STRUCTS_H__