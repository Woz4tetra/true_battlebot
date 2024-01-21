/**
 * mini_bot
 *
 * This is the firmware for the mini_bot. It listens for UDP packets on a
 * configurable port and sets the motor speed and direction based on the
 * contents of the packet.
 */
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include <ESP32Servo.h>

/**
 * Neopixels
 */
const int NUM_PIXELS = 1; // This board has a single built in neopixel.
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

/**
 * WiFi and UDP
 */

// The maximum size of a UDP and serial packet for this device.
#define PACKET_MAX_LENGTH 512
WiFiUDP UDP;                         // A UDP instance to let us send and receive packets over UDP
int read_length = 0;                 // Current length of the packet being read
char read_buffer[PACKET_MAX_LENGTH]; // Buffer to read packets into

const int NUM_MOTORS = 2; // This robot has two motors

int active_mode_hue = 0;                  // While motor commands are active, the hue of neopixel will cycle
int idle_mode_color = 0;                  // When no motor commands are active, flash a single channel with this color (red)
uint32_t prev_packet_time = 0;            // The last time a packet was received (milliseconds)
const uint32_t PACKET_WARN_TIMEOUT = 500; // If no packets are received for this long, flash yellow (milliseconds)
const uint32_t PACKET_STOP_TIMEOUT = 500; // If no packets are received for this long, turn off motors (milliseconds)

const int CONFIG_SET_ADDRESS = 1; // EEPROM address to store whether config is set

// The first two bytes of a serial packet
const char SERIAL_PACKET_C0 = 'b';
const char SERIAL_PACKET_C1 = 'w';

/**
 * PWM
 */
Servo left_servo;
Servo right_servo;
const int LEFT_OUTPUT_PIN = 17;
const int RIGHT_OUTPUT_PIN = 9;

/**
 * Packet structs
 */

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
#define SSID_LENGTH 33
#define PASSWORD_LENGTH 64
typedef struct config_info : header
{
    uint16_t port;
    char wifi_info[SSID_LENGTH + PASSWORD_LENGTH];
} config_info_t, *config_info_p;

config_info_p device_config;

/**
 * @brief Blink the neopixel white and black
 *
 * @param state True to set to white, false to set to black
 */
void blink(bool state)
{
    if (state)
    {
        pixels.fill(0xFFFFFF);
    }
    else
    {
        pixels.fill(0x000000);
    }
    pixels.show();
}

/**
 * @brief Blink the neopixel on and off for a number of cycles
 *
 * @param num_cycles The number of cycles to blink
 */
void cycle_blink(int num_cycles)
{
    for (int i = 0; i < num_cycles * 2; i++)
    {
        blink(i % 2);
        delay(100);
    }
}

/**
 * @brief Check if the config is set in EEPROM
 *
 * @return True if the config is set, false otherwise
 */
bool is_config_set()
{
    uint8_t is_config;
    EEPROM.get(CONFIG_SET_ADDRESS, is_config);
    return is_config == 0x01;
}

/**
 * @brief Unset the config in EEPROM
 */
void unset_config()
{
    EEPROM.put(CONFIG_SET_ADDRESS, 0x00);
    if (EEPROM.commit())
    {
        Serial.println("Config unset");
    }
    else
    {
        Serial.println("Failed to unset config");
    }
}

/**
 * @brief Write the config to EEPROM
 *
 * @param config The config struct pointer to write
 */
void write_config(config_info_p config)
{
    EEPROM.put(CONFIG_SET_ADDRESS, 0x01);
    EEPROM.put(CONFIG_SET_ADDRESS + 1, *config);
    if (EEPROM.commit())
    {
        Serial.println("Config written to EEPROM");
    }
    else
    {
        Serial.println("Failed to write config to EEPROM");
    }
}

/**
 * @brief Read the config from EEPROM
 *
 * @param config The config struct pointer to read into
 */
void read_config(config_info_p config)
{
    config_info_t read_config;
    EEPROM.get(CONFIG_SET_ADDRESS + 1, read_config);
    *config = read_config;
}

/**
 * @brief Check if the serial port has a "clear" command
 *
 * If the serial port has a "clear" command, unset the config and restart the
 * device.
 */
void check_clear()
{
    if (Serial.available())
    {
        String command = Serial.readStringUntil('\n');
        if (command.equals("clear"))
        {
            unset_config();
            ESP.restart();
        }
    }
}

/**
 * @brief Get the length of the next serial packet
 *
 * @return The length of the next serial packet, or 0 if no packet is available
 */
int get_serial_packet_length()
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

/**
 * @brief Set the config from the serial port
 *
 * This method reads packets from the serial port until a valid config packet is
 * received. It then writes the config to EEPROM.
 */
void set_config_from_serial()
{
    int blue_channel = 0;
    int serial_packet_size = 0;
    while (device_config->device_id == 255)
    {
        if (serial_packet_size > 0)
        {
            if (read_length < serial_packet_size)
            {
                read_buffer[read_length++] = Serial.read();
            }
            else
            {
                if (process_packet(read_buffer, read_length))
                {
                    Serial.println("Config set via serial");
                }
                else
                {
                    Serial.println("Invalid config packet");
                }
                read_length = 0;
                serial_packet_size = 0;
            }
        }
        else
        {
            serial_packet_size = get_serial_packet_length();
            pixels.fill(pixels.Color(0, 0, blue_channel));
            blue_channel = (blue_channel + 1) % 255;
            pixels.show();
        }
    }
}

/**
 * @brief Setup the mini_bot
 *
 * This method sets up the mini_bot by initializing the neopixel, reading the
 * config from EEPROM, and connecting to WiFi.
 */
void setup()
{
    Serial.begin(115200);

#if defined(NEOPIXEL_POWER)
    // If this board has a power control pin, we must set it to output and high
    // in order to enable the NeoPixels. We put this in an #if defined so it can
    // be reused for other boards without compilation errors
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

    pixels.begin();           // INITIALIZE NeoPixel pixels object (REQUIRED)
    pixels.setBrightness(20); // not so bright

    cycle_blink(2);

    // Initialize EEPROM
    if (!EEPROM.begin(PACKET_MAX_LENGTH))
    {
        while (true)
        {
            Serial.println("failed to initialize EEPROM");
            delay(1000);
        }
    }
    delay(100);

    // Reset config
    device_config = new config_info_t;
    device_config->device_id = 255;

    // Read config from EEPROM
    if (is_config_set())
    {
        Serial.println("Config is set");
        read_config(device_config);
    }
    else
    {
        Serial.println("Config is not set. Waiting for config via serial.");
        set_config_from_serial();
        cycle_blink(2);
    }

    Serial.print("Config set. Device ID: ");
    Serial.println(device_config->device_id);

    // Connect to WiFi
    char *ssid = device_config->wifi_info;
    char *password = ssid + SSID_LENGTH;
    WiFi.begin(ssid, password);
    int blink_count = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(100);
        check_clear();
        blink(blink_count % 2);
        Serial.print(".");
        blink_count++;
    }
    blink(false);

    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());

    // Setup UDP
    UDP.begin(device_config->port);
    Serial.print("Listening on UDP port ");
    Serial.println(device_config->port);

    // Setup servo control
    Serial.println("Setting up servo control...");
    left_servo.attach(LEFT_OUTPUT_PIN);
    right_servo.attach(RIGHT_OUTPUT_PIN);
    // initialize them to their neutral positions
    left_servo.writeMicroseconds(1500);
    right_servo.writeMicroseconds(1500);

    Serial.println("mini_bot setup complete");
}

/**
 * @brief Set a motor channel
 *
 * @param channel The motor channel to set (0 or 1)
 * @param speed The speed to set the motor to (0-255)
 * @param direction The direction to set the motor to (-1 backwards, 0 stop, 1 forward)
 */
void set_motor(uint8_t channel, uint8_t speed, int8_t direction)
{
  int position = 1500;

  // reverse is 800-1100μs
  if (direction < 0)
  {
    position = map(speed, 0, 255, 800, 1100);
  }
  // forward is 1900-2200μs
  else if (direction > 0)
  {
    position = map(speed, 0, 255, 1900, 2200);
  }

  if (channel) {
    right_servo.writeMicroseconds(position);
  }
  else {
    left_servo.writeMicroseconds(position);
  }

  // Serial.print("Setting motor ");
  // Serial.print(channel);
  // Serial.print(" to speed ");
  // Serial.print(speed);
  // Serial.print(" , direction ");
  // Serial.print(direction);
  // Serial.prit(" and position ");
  // Serial.println(position);
}

/**
 * @brief Process a motor packet
 *
 * @param packet The packet to process
 * @param packet_size The size of the packet
 * @return True if the packet was processed successfully, false otherwise
 */
bool process_motor_packet(char *packet, int packet_size)
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

    // Check that the packet size matches the number of channels
    if (motor_desc->num_channels > NUM_MOTORS)
    {
        Serial.println("Too many motor channels");
        return false;
    }

    // Set the motor speed and direction for each channel
    int max_speed = 0;
    for (int channel = 0; channel < motor_desc->num_channels; channel++)
    {
        uint8_t speed = motor_desc->commands[channel].speed;
        int8_t direction = motor_desc->commands[channel].direction;
        set_motor(channel, speed, direction);
        if (speed > max_speed)
        {
            max_speed = speed;
        }
    }

    // Set the neopixel brightness based on the max speed
    pixels.setBrightness(min(255, max(20, max_speed)));
    return true;
}

/**
 * @brief Process a ping packet
 *
 * @param packet The packet to process
 * @param packet_size The size of the packet
 * @return True if the packet was processed successfully, false otherwise
 */
bool process_ping_packet(char *packet, int packet_size)
{
    // If the device ID is 255, assume the rest of the config is not set
    // and thus wifi credentials are not set. In this case, we cannot respond
    // to ping packets via UDP.
    if (device_config->device_id == 255)
    {
        Serial.println("No wifi credentials set. Cannot respond to ping");
        return false;
    }

    // Check that the packet size matches the size of the ping_info_t struct
    if (packet_size < sizeof(ping_info_t))
    {
        Serial.println("Ping packet too small");
        return false;
    }

    // Cast packet to ping_info_p so we can access the fields.
    ping_info_p ping_info = (ping_info_p)packet;
    ping_packet_t ping_packet;
    ping_packet.data.size = sizeof(ping_info_t);
    ping_packet.data.type = PING;
    ping_packet.data.device_id = device_config->device_id;
    ping_packet.data.time = ping_info->time; // Echo the time back to the sender
    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
    UDP.write(ping_packet.bytes, ping_packet.data.size); // Use union to interpret struct as bytes
    UDP.endPacket();
    return true;
}

/**
 * @brief Process a config packet
 *
 * @param packet The packet to process
 * @param packet_size The size of the packet
 * @return True if the packet was processed successfully, false otherwise
 */
bool process_config_packet(char *packet, int packet_size)
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
    // Cast packet to config_info_p so we can access the fields.
    config_info_p config_info = (config_info_p)packet;

    // Copy the config into the global device_config struct
    *device_config = *config_info;

    // Write the config to EEPROM
    write_config(device_config);

    // Ensure the config was written correctly
    if (!is_config_set())
    {
        Serial.println("Failed to write config to EEPROM");
        return false;
    }
    else
    {
        Serial.println("Config set in EEPROM");
    }

    // Respond to the config packet with the received packet
    Serial.write(SERIAL_PACKET_C0);
    Serial.write(SERIAL_PACKET_C1);
    Serial.write(packet_size & 0xFF);
    Serial.write((packet_size >> 8) & 0xFF);
    Serial.write(packet, packet_size);
    return true;
}
/**
 * @brief Process a packet
 *
 * @param packet The packet to process
 * @param packet_size The size of the packet
 * @return True if the packet was processed successfully, false otherwise
 */
bool process_packet(char *packet, int packet_size)
{
    // Check that the packet size is at least the size of a header.
    // Other structs extend the header struct and thus will be at least this size.
    if (packet_size < sizeof(header_t))
    {
        Serial.println("Packet header too small");
        return false;
    }
    // Cast packet to header_p so we can access the fields.
    header_p header = (header_p)read_buffer;

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

    // If the device ID is 255, the config is not set. Only process config packets.
    if (device_config->device_id == 255)
    {
        if (header->type == CONFIG)
        {
            return process_config_packet(read_buffer, packet_size);
        }
        else
        {
            Serial.println("No device ID set. Cannot process packet");
            return false;
        }
    }

    // Check that the device ID matches the device ID in the header.
    if (header->device_id != device_config->device_id)
    {
        return false;
    }

    // Process the packet based on the type in the header
    switch (header->type)
    {
    case MOTOR:
        return process_motor_packet(read_buffer, packet_size);
    case PING:
        return process_ping_packet(read_buffer, packet_size);
    case CONFIG:
        return process_config_packet(read_buffer, packet_size);
    default:
        return false;
    }
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

    // Check for UDP packets. If a packet is received, process it.
    int packet_size = UDP.parsePacket();
    if (packet_size)
    {
        read_length = UDP.read(read_buffer, PACKET_MAX_LENGTH);
        if (process_packet(read_buffer, read_length))
        {
            prev_packet_time = now;
        }
    }

    // Check for serial packets. If a packet is received, process it.
    check_clear();

    // If no packets have been received for a while, turn off motors
    uint32_t packet_delay = now - prev_packet_time;
    if (packet_delay > PACKET_STOP_TIMEOUT)
    {
        // No packets received in a while, turn off motors
        for (int channel = 0; channel < NUM_MOTORS; channel++)
        {
            set_motor(channel, 0, 0);
        }
        pixels.setBrightness(20);
        pixels.fill(pixels.Color(idle_mode_color / 5, 0, 0));
        idle_mode_color = (idle_mode_color + 1) % 1275;
    }
    else if (packet_delay > PACKET_WARN_TIMEOUT)
    {
        pixels.setBrightness(20);
        int channel = idle_mode_color / 3;
        pixels.fill(pixels.Color(channel, channel, 0));
        idle_mode_color = (idle_mode_color + 1) % 765;
    }
    else
    {
        pixels.fill(pixels.gamma32(pixels.ColorHSV((uint16_t)active_mode_hue)));
        active_mode_hue = (active_mode_hue + 50) % 0x10000;
    }
    pixels.show();
}
