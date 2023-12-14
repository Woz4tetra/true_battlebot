#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define WIFI_SSID "HavocNet"
#define WIFI_PASS "havocmachinesthatbreakrobots"
#define UDP_PORT 4176
#define DEVICE_ID 1

const int NUM_PIXELS = 1;
const int SHOW_DELAY = 1;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

WiFiUDP UDP;
#define PACKET_MAX_LENGTH 255
char packet[PACKET_MAX_LENGTH];

const int NUM_MOTORS = 2;

int active_mode_hue = 0;
int idle_mode_color = 0;
uint32_t prev_packet_time = 0;
const uint32_t PACKET_TIMEOUT = 500;

typedef struct header
{
    uint16_t size;
    uint8_t type;
    uint8_t device_id;
} header_t, *header_p;

enum HeaderType
{
    MOTOR = 0x01,
    // Put more commands here if needed
};

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

    for (int i = 0; i < 4; i++)
    {
        blink(i % 2);
        delay(100);
    }

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int blink_count = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(100);
        blink(blink_count % 2);
        Serial.print(".");
        blink_count++;
    }
    blink(false);

    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());

    UDP.begin(UDP_PORT);
    Serial.print("Listening on UDP port ");
    Serial.println(UDP_PORT);

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

bool process_packet()
{
    int packet_size = UDP.parsePacket();
    if (!packet_size)
    {
        return false;
    }

    int read_size = UDP.read(packet, PACKET_MAX_LENGTH);
    if (read_size < sizeof(header_t))
    {
        Serial.println("Packet header too small");
        return false;
    }
    header_p header = (header_p)packet;
    if (read_size < header->size)
    {
        Serial.println("Packet length doesn't match header");
        return false;
    }

    if (header->device_id != DEVICE_ID)
    {
        return false;
    }

    if (header->type == MOTOR)
    {
        motor_description_p motor_desc = (motor_description_p)packet;
        if (read_size < sizeof(motor_description_t))
        {
            Serial.println("Motor packet too small");
            return false;
        }

        if (motor_desc->num_channels > NUM_MOTORS)
        {
            Serial.println("Too many motor channels");
            return false;
        }

        for (int channel = 0; channel < motor_desc->num_channels; channel++)
        {
            uint8_t speed = motor_desc->commands[channel].speed;
            uint8_t direction = motor_desc->commands[channel].direction;
            set_motor(channel, speed, direction);
        }
    }

    return true;
}

void loop()
{
    uint32_t now = millis();
    if (process_packet())
    {
        prev_packet_time = now;
    }
    if (now - prev_packet_time > PACKET_TIMEOUT)
    {
        // No packets received in a while, turn off motors
        for (int channel = 0; channel < NUM_MOTORS; channel++)
        {
            set_motor(channel, 0, 0);
        }
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
