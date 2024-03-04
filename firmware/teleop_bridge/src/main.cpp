/**
 * teleop_bridge
 *
 * This is the firmware for all battlebots controlled by the ESP. It listens
 * for UDP packets on a configurable port and sets the motor speed and
 * direction based on the contents of the packet.
 */
#include <Arduino.h>
#include <bridge/udp_bridge.h>
#include <bridge/serial_bridge.h>
#include <bridge/persistent_config.h>
#include <motors/esc_tank_controller.h>
#include <motors/esc_motor.h>
#include <feedback/imu_sensor.h>
#include <status_lights/status_neopixel.h>

#define CHANNEL_1 17 // A1 -> channel 1 (left)
#define CHANNEL_2 9  // A2 -> channel 2 (right)

const float WHEEL_RADIUS = 0.2f;
const float BASE_WIDTH = 0.1f;

char UDP_READ_BUFFER[bridge::PACKET_MAX_LENGTH];
uint8_t UDP_WRITE_BUFFER[bridge::PACKET_MAX_LENGTH];
char SERIAL_READ_BUFFER[bridge::PACKET_MAX_LENGTH];

bridge::config_info_p DEVICE_CONFIG;
udp_bridge::UdpBridge *udp_interface;
serial_bridge::SerialBridge *serial_interface;
esc_tank_controller::EscTankController *controller;
esc_motor::EscMotor *left_motor, *right_motor;
imu_sensor::ImuSensor *imu_sensor_inst;
persistent_config::PersistentConfig *persistent_config_inst;
status_neopixel::StatusNeopixel *neopixel_status;

const int COMMAND_TIMEOUT = 1000; // Stop motors if no command is received for this many milliseconds
uint32_t last_command = 0;        // The last time a packet was received (milliseconds)

const int IMU_SEND_INTERVAL = 100;
uint32_t last_imu_send = 0;

int get_max_command()
{
    int max_speed = 0;
    for (int channel = 0; channel < controller->get_num_channels(); channel++)
    {
        int command = controller->get_command(channel);
        int speed = abs(command);
        if (speed > max_speed)
        {
            max_speed = speed;
        }
    }
    return max_speed;
}

/**
 * @brief Setup the teleop bridge
 *
 * This method sets up the teleop bridge by initializing relevent components.
 */
void setup()
{
    Serial.begin(115200);

    // Reset config
    DEVICE_CONFIG = new bridge::config_info_t;
    DEVICE_CONFIG->device_id = bridge::NULL_DEVICE_ID;

    persistent_config_inst = persistent_config::PersistentConfig::get_instance();

    imu_sensor_inst = new imu_sensor::ImuSensor();

    left_motor = new esc_motor::EscMotor(CHANNEL_1, true);
    right_motor = new esc_motor::EscMotor(CHANNEL_2, false);

    speed_pid::SpeedPID *angular_pid = new speed_pid::SpeedPID();
    angular_pid->Kp = 6.0;
    angular_pid->Ki = 0.1;
    angular_pid->Kd = 0.02;

    controller = new esc_tank_controller::EscTankController(left_motor, right_motor, imu_sensor_inst, angular_pid, BASE_WIDTH, WHEEL_RADIUS);
    udp_interface = udp_bridge::UdpBridge::get_instance(DEVICE_CONFIG, UDP_READ_BUFFER, UDP_WRITE_BUFFER, controller, imu_sensor_inst);
    serial_interface = serial_bridge::SerialBridge::get_instance(DEVICE_CONFIG, SERIAL_READ_BUFFER, persistent_config_inst);
    neopixel_status = new status_neopixel::StatusNeopixel();
    neopixel_status->begin();
    persistent_config_inst->begin();

    if (!imu_sensor_inst->begin())
    {
        Serial.println("Failed to initialize IMU!");
    }

    controller->begin();
    controller->stop_all_motors();

    // Read config from EEPROM
    if (persistent_config_inst->is_set())
    {
        Serial.println("Config is set");
        persistent_config_inst->read(DEVICE_CONFIG);
        Serial.print("Device ID: ");
        Serial.println(DEVICE_CONFIG->device_id);
        Serial.print("Port: ");
        Serial.println(DEVICE_CONFIG->port);
        Serial.print("SSID: ");
        Serial.println(DEVICE_CONFIG->wifi_info);
    }
    else
    {
        Serial.println("Config is not set. Waiting for config via serial.");
    }

    Serial.println("teleop_bridge setup complete");
}

/**
 * @brief The main loop
 *
 * This method listens for UDP packets and processes them.
 * If no packets are received for a while, it turns off the motors.
 * It also checks the serial port for a "clear" command.
 */
void loop()
{
    uint32_t now = millis();
    serial_interface->update();
    serial_bridge::SerialBridgeState serial_state = serial_interface->get_state();
    if (serial_state == serial_bridge::WAITING_FOR_CONFIG)
    {
        neopixel_status->set_state(status_base::WAITING_FOR_CONFIG);
    }
    else
    {
        // Check if the serial port has a "clear" command
        persistent_config_inst->check_clear();

        if (udp_interface->update())
        {
            last_command = now;
        }
        udp_bridge::UdpBridgeState udp_state = udp_interface->get_state();
        switch (udp_state)
        {
        case udp_bridge::INIT:
        case udp_bridge::CONNECTING:
            neopixel_status->set_state(status_base::CONNECTING);
            break;
        case udp_bridge::READY:
            if (now - last_command > COMMAND_TIMEOUT)
            {
                controller->stop_all_motors();
                neopixel_status->set_state(status_base::TIMED_OUT);
            }
            else
            {
                neopixel_status->set_state(status_base::OK);
                neopixel_status->set_speed_readout(get_max_command());
            }

            imu_sensor_inst->update();
            controller->update();

            break;
        default:
            break;
        }
    }
    neopixel_status->update();
}
