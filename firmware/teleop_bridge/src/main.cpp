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
#include <status_lights/status_neopixel.h>

char UDP_READ_BUFFER[bridge::PACKET_MAX_LENGTH];
char SERIAL_READ_BUFFER[bridge::PACKET_MAX_LENGTH];

bridge::config_info_p DEVICE_CONFIG;
udp_bridge::UdpBridge *udp_interface;
serial_bridge::SerialBridge *serial_interface;
base_controller::BaseController *controller;
persistent_config::PersistentConfig *persistent_config_inst;
status_neopixel::StatusNeopixel *neopixel_status;

uint32_t last_command = 0; // The last time a packet was received (milliseconds)

/**
 * @brief Setup the mini_bot
 *
 * This method sets up the mini_bot by initializing the neopixel, reading the
 * config from EEPROM, and connecting to WiFi.
 */
void setup()
{
    Serial.begin(115200);

    // Reset config
    DEVICE_CONFIG = new bridge::config_info_t;
    DEVICE_CONFIG->device_id = bridge::NULL_DEVICE_ID;

    persistent_config_inst = persistent_config::PersistentConfig::get_instance();
    controller = new esc_tank_controller::EscTankController(17, 9); // TODO make this configurable from build args
    udp_interface = udp_bridge::UdpBridge::get_instance(DEVICE_CONFIG, UDP_READ_BUFFER, controller);
    serial_interface = serial_bridge::SerialBridge::get_instance(DEVICE_CONFIG, SERIAL_READ_BUFFER, persistent_config_inst);
    neopixel_status = new status_neopixel::StatusNeopixel();
    neopixel_status->begin();

    persistent_config_inst->begin();

    // Read config from EEPROM
    if (persistent_config_inst->is_set())
    {
        Serial.println("Config is set");
        persistent_config_inst->read(DEVICE_CONFIG);
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
            neopixel_status->set_state(status_base::CONNECTING);
            break;
        case udp_bridge::CONNECTING:
            neopixel_status->set_state(status_base::CONNECTING);
            break;
        case udp_bridge::READY:
            if (now - last_command > 1000)
            {
                controller->stop_all_motors();
                neopixel_status->set_state(status_base::TIMED_OUT);
            }
            else
            {
                neopixel_status->set_state(status_base::OK);
            }
            break;
        default:
            break;
        }
    }
    neopixel_status->update();
}
