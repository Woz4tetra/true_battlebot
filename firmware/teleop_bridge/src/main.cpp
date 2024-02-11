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

#define CHANNEL_1 17 // A1 -> channel 1 (left)
#define CHANNEL_2 9  // A2 -> channel 2 (right)

Servo left_servo, right_servo;
const int STOP_PULSE = esc_tank_controller::STOP_MOTOR_PULSE_WIDTH;
volatile int left_pulse_width = STOP_PULSE;
volatile int right_pulse_width = STOP_PULSE;
volatile int left_on_counts = 0, right_on_counts = 0;
volatile int left_counter = 0, right_counter = 0;

char UDP_READ_BUFFER[bridge::PACKET_MAX_LENGTH];
char SERIAL_READ_BUFFER[bridge::PACKET_MAX_LENGTH];

bridge::config_info_p DEVICE_CONFIG;
udp_bridge::UdpBridge *udp_interface;
serial_bridge::SerialBridge *serial_interface;
esc_tank_controller::EscTankController *controller;
persistent_config::PersistentConfig *persistent_config_inst;
status_neopixel::StatusNeopixel *neopixel_status;

const int COMMAND_TIMEOUT = 250; // Stop motors if no command is received for this many milliseconds
uint32_t last_command = 0;       // The last time a packet was received (milliseconds)

#define MINIMUM_BACKWARD_PULSE_WIDTH 1390
#define MINIMUM_FORWARD_PULSE_WIDTH 1610
hw_timer_t *Timer0_Cfg = NULL, *Timer1_Cfg = NULL;

// void IRAM_ATTR LeftTimer_ISR()
void update_left_servo()
{
    int pulse_width = left_pulse_width;
    if ((MINIMUM_BACKWARD_PULSE_WIDTH <= left_pulse_width || left_pulse_width <= MINIMUM_FORWARD_PULSE_WIDTH) && left_pulse_width != STOP_PULSE)
    {
        if (left_counter > left_on_counts)
        {
            pulse_width = STOP_PULSE;
        }
        else
        {
            pulse_width = left_pulse_width > STOP_PULSE ? MINIMUM_FORWARD_PULSE_WIDTH : MINIMUM_BACKWARD_PULSE_WIDTH;
        }
    }
    left_servo.writeMicroseconds(pulse_width);
    left_counter++;
    if (left_counter >= 100)
    {
        left_counter = 0;
    }
}

// void IRAM_ATTR RightTimer_ISR()
void update_right_servo()
{
    int pulse_width = right_pulse_width;
    if ((MINIMUM_BACKWARD_PULSE_WIDTH <= right_pulse_width || right_pulse_width <= MINIMUM_FORWARD_PULSE_WIDTH) && right_pulse_width != STOP_PULSE)
    {
        if (right_counter > right_on_counts)
        {
            pulse_width = STOP_PULSE;
        }
        else
        {
            pulse_width = right_pulse_width > STOP_PULSE ? MINIMUM_FORWARD_PULSE_WIDTH : MINIMUM_BACKWARD_PULSE_WIDTH;
        }
    }
    right_servo.writeMicroseconds(pulse_width);
    right_counter++;
    if (right_counter >= 100)
    {
        right_counter = 0;
    }
}

int get_max_motor_speed()
{
    int max_speed = 0;
    for (int channel = 0; channel < controller->get_num_channels(); channel++)
    {
        int velocity = controller->get_motor(channel);
        int speed = abs(velocity);
        if (speed > max_speed)
        {
            max_speed = speed;
        }
    }
    return max_speed;
}

int compute_pwm_percentages(int pulse_width)
{
    float on_counts = 0.0;
    if (pulse_width < STOP_PULSE)
    {
        on_counts = 100.0 * (float)(STOP_PULSE - pulse_width) / (MINIMUM_FORWARD_PULSE_WIDTH - STOP_PULSE);
    }
    else
    {
        on_counts = 100.0 * (float)(pulse_width - STOP_PULSE) / (STOP_PULSE - MINIMUM_BACKWARD_PULSE_WIDTH);
    }
    return (int)on_counts;
}

/**
 * @brief Setup the teleop bridge
 *
 * This method sets up the teleop bridge by initializing relevent components.
 */
void setup()
{
    Serial.begin(115200);

    left_servo.attach(CHANNEL_1);
    right_servo.attach(CHANNEL_2);

    left_servo.writeMicroseconds(STOP_PULSE);
    right_servo.writeMicroseconds(STOP_PULSE);

    // Timer0_Cfg = timerBegin(0, 80, true);
    // timerAttachInterrupt(Timer0_Cfg, &LeftTimer_ISR, true);
    // timerAlarmWrite(Timer0_Cfg, 100, true); // 0.1ms
    // timerAlarmEnable(Timer0_Cfg);

    // Timer1_Cfg = timerBegin(1, 80, true);
    // timerAttachInterrupt(Timer1_Cfg, &RightTimer_ISR, true);
    // timerAlarmWrite(Timer1_Cfg, 100, true); // 0.1ms
    // timerAlarmEnable(Timer1_Cfg);

    // Reset config
    DEVICE_CONFIG = new bridge::config_info_t;
    DEVICE_CONFIG->device_id = bridge::NULL_DEVICE_ID;

    persistent_config_inst = persistent_config::PersistentConfig::get_instance();
    controller = new esc_tank_controller::EscTankController();
    udp_interface = udp_bridge::UdpBridge::get_instance(DEVICE_CONFIG, UDP_READ_BUFFER, controller);
    serial_interface = serial_bridge::SerialBridge::get_instance(DEVICE_CONFIG, SERIAL_READ_BUFFER, persistent_config_inst);
    neopixel_status = new status_neopixel::StatusNeopixel();
    neopixel_status->begin();
    persistent_config_inst->begin();

    controller->begin();
    controller->stop_all_motors();

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
        case udp_bridge::CONNECTING:
            neopixel_status->set_state(status_base::CONNECTING);
            break;
        case udp_bridge::READY:
            if (now - last_command > COMMAND_TIMEOUT)
            {
                controller->stop_all_motors();
                neopixel_status->set_state(status_base::TIMED_OUT);
                left_pulse_width = esc_tank_controller::STOP_MOTOR_PULSE_WIDTH;
                right_pulse_width = esc_tank_controller::STOP_MOTOR_PULSE_WIDTH;
            }
            else
            {
                left_pulse_width = controller->get_pulse_width(esc_tank_controller::LEFT_CHANNEL);
                right_pulse_width = controller->get_pulse_width(esc_tank_controller::RIGHT_CHANNEL);
                neopixel_status->set_state(status_base::OK);
                neopixel_status->set_speed_readout(get_max_motor_speed());
            }
            left_on_counts = compute_pwm_percentages(left_pulse_width);
            right_on_counts = compute_pwm_percentages(right_pulse_width);

            update_left_servo();
            update_right_servo();
            break;
        default:
            break;
        }
    }
    neopixel_status->update();
}
