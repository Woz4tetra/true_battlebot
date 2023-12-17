#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <EEPROM.h>

#define PACKET_MAX_LENGTH 512

const int NUM_PIXELS = 1;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

WiFiUDP UDP;
int read_length = 0;
char read_buffer[PACKET_MAX_LENGTH];

const int NUM_MOTORS = 2;

int active_mode_hue = 0;
int idle_mode_color = 0;
uint32_t prev_packet_time = 0;
const uint32_t PACKET_TIMEOUT = 500;

const int CONFIG_SET_ADDRESS = 1;

const char SERIAL_PACKET_C0 = 'b';
const char SERIAL_PACKET_C1 = 'w';
int serial_packet_size = 0;

typedef struct header
{
    uint16_t size;
    uint8_t type;
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
    uint8_t direction;
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

bool is_config_set()
{
    uint8_t is_config;
    EEPROM.get(CONFIG_SET_ADDRESS, is_config);
    return is_config == 0x01;
}

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

void read_config(config_info_p config)
{
    config_info_t read_config;
    EEPROM.get(CONFIG_SET_ADDRESS + 1, read_config);
    *config = read_config;
}

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

bool has_serial_packet()
{
    if (serial_packet_size > 0)
    {
        return true;
    }
    if (Serial.available() < 4)
    {
        return false;
    }
    char c = Serial.read();
    if (c == SERIAL_PACKET_C0)
    {
        c = Serial.read();
        if (c == SERIAL_PACKET_C1)
        {
            char size_1 = Serial.read();
            char size_2 = Serial.read();
            serial_packet_size = (size_2 << 8) | size_1;
            return true;
        }
    }
    return false;
}

void cycle_blink(int num_cycles)
{
    for (int i = 0; i < num_cycles * 2; i++)
    {
        blink(i % 2);
        delay(100);
    }
}

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

    if (!EEPROM.begin(PACKET_MAX_LENGTH))
    {
        while (true)
        {
            Serial.println("failed to initialize EEPROM");
            delay(1000);
        }
    }
    delay(100);

    device_config = new config_info_t;
    device_config->device_id = 255;
    if (is_config_set())
    {
        Serial.println("Config is set");
        read_config(device_config);
    }
    else
    {
        Serial.println("Config is not set. Waiting for config via serial.");
        int blue_channel = 0;
        while (device_config->device_id == 255)
        {
            if (has_serial_packet())
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
                pixels.fill(pixels.Color(0, 0, blue_channel));
                blue_channel = (blue_channel + 1) % 255;
                pixels.show();
            }
        }
        cycle_blink(2);
    }
    Serial.print("Config set. Device ID: ");
    Serial.println(device_config->device_id);

    Serial.print("Connecting to ");
    Serial.println(device_config->wifi_info);

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

    UDP.begin(device_config->port);
    Serial.print("Listening on UDP port ");
    Serial.println(device_config->port);

    Serial.println("mini_bot setup complete");
}

void set_motor(uint8_t channel, uint8_t speed, uint8_t direction)
{
    // Serial.print("Setting motor ");
    // Serial.print(channel);
    // Serial.print(" to speed ");
    // Serial.print(speed);
    // Serial.print(" and direction ");
    // Serial.println(direction);
}

bool process_motor_packet(char *packet, int packet_size)
{
    if (packet_size < sizeof(motor_description_t))
    {
        Serial.println("Motor packet too small");
        return false;
    }
    motor_description_p motor_desc = (motor_description_p)packet;

    if (motor_desc->num_channels > NUM_MOTORS)
    {
        Serial.println("Too many motor channels");
        return false;
    }

    int max_speed = 0;
    for (int channel = 0; channel < motor_desc->num_channels; channel++)
    {
        uint8_t speed = motor_desc->commands[channel].speed;
        uint8_t direction = motor_desc->commands[channel].direction;
        set_motor(channel, speed, direction);
        if (speed > max_speed)
        {
            max_speed = speed;
        }
    }
    pixels.setBrightness(min(255, max(20, max_speed)));
    return true;
}

bool process_ping_packet(char *packet, int packet_size)
{
    if (device_config->port == -1)
    {
        Serial.println("No wifi credentials set. Cannot respond to ping");
        return false;
    }
    if (packet_size < sizeof(ping_info_t))
    {
        Serial.println("Ping packet too small");
        return false;
    }
    ping_info_p ping_info = (ping_info_p)packet;
    ping_packet_t ping_packet;
    ping_packet.data.size = sizeof(ping_info_t);
    ping_packet.data.type = PING;
    ping_packet.data.device_id = device_config->device_id;
    ping_packet.data.time = ping_info->time;
    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
    UDP.write(ping_packet.bytes, ping_packet.data.size);
    UDP.endPacket();
    return true;
}

bool process_config_packet(char *packet, int packet_size)
{
    if (packet_size < sizeof(config_info_t))
    {
        Serial.print("Config packet too small. Expected ");
        Serial.print(sizeof(config_info_t));
        Serial.print(" bytes, got ");
        Serial.print(packet_size);
        Serial.println(" bytes");
        return false;
    }
    config_info_p config_info = (config_info_p)packet;

    *device_config = *config_info;
    write_config(device_config);
    if (!is_config_set())
    {
        Serial.println("Failed to write config to EEPROM");
        return false;
    }
    else
    {
        Serial.println("Config set in EEPROM");
    }
    Serial.write(SERIAL_PACKET_C0);
    Serial.write(SERIAL_PACKET_C1);
    Serial.write(packet_size & 0xFF);
    Serial.write((packet_size >> 8) & 0xFF);
    Serial.write(packet, packet_size);
    return true;
}

bool process_packet(char *packet, int packet_size)
{
    if (packet_size < sizeof(header_t))
    {
        Serial.println("Packet header too small");
        return false;
    }
    header_p header = (header_p)read_buffer;
    if (packet_size != header->size)
    {
        Serial.print("Packet length doesn't match header. Expected ");
        Serial.print(header->size);
        Serial.print(" bytes, got ");
        Serial.print(packet_size);
        Serial.println(" bytes");
        return false;
    }

    if (device_config->device_id != 255 && header->device_id != device_config->device_id)
    {
        return false;
    }

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

void loop()
{
    uint32_t now = millis();

    int packet_size = UDP.parsePacket();
    if (packet_size)
    {
        read_length = UDP.read(read_buffer, PACKET_MAX_LENGTH);
        if (process_packet(read_buffer, read_length))
        {
            prev_packet_time = now;
        }
    }

    check_clear();

    if (now - prev_packet_time > PACKET_TIMEOUT)
    {
        // No packets received in a while, turn off motors
        for (int channel = 0; channel < NUM_MOTORS; channel++)
        {
            set_motor(channel, 0, 0);
        }
        pixels.setBrightness(20);
        pixels.fill(pixels.Color(idle_mode_color, 0, 0));
        idle_mode_color = (idle_mode_color + 1) % 255;
    }
    else
    {
        pixels.fill(pixels.gamma32(pixels.ColorHSV((uint16_t)active_mode_hue)));
        active_mode_hue = (active_mode_hue + 50) % 0x10000;
    }
    pixels.show();
}
