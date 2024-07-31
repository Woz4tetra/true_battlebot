#include <Arduino.h>

const int LED_PIN = 13;
const int INTERRUPT_PIN_0 = 2;
const int INTERRUPT_PIN_1 = 3;

volatile unsigned int hall_0_sensor_ticks = 0;
volatile unsigned int hall_1_sensor_ticks = 0;

unsigned int prev_hall_0_sensor_ticks = 0;
unsigned int prev_hall_1_sensor_ticks = 0;

bool led_state = false;

void interrupt_0_handler()
{
    hall_0_sensor_ticks++;
}

void interrupt_1_handler()
{
    hall_1_sensor_ticks++;
}

void setup()
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    pinMode(INTERRUPT_PIN_0, INPUT_PULLUP);
    pinMode(INTERRUPT_PIN_1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_0), interrupt_0_handler, RISING);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_1), interrupt_1_handler, RISING);
    Serial.println("Ready");
}

void loop()
{
    if (prev_hall_0_sensor_ticks != hall_0_sensor_ticks || prev_hall_1_sensor_ticks != hall_1_sensor_ticks)
    {
        prev_hall_0_sensor_ticks = hall_0_sensor_ticks;
        prev_hall_1_sensor_ticks = hall_1_sensor_ticks;
        led_state = !led_state;
        digitalWrite(LED_PIN, led_state);
        Serial.print(hall_0_sensor_ticks);
        Serial.print("\t");
        Serial.println(hall_1_sensor_ticks);
    }
    if (Serial.available() > 0)
    {
        int reset_value = Serial.parseInt();
        hall_0_sensor_ticks = reset_value;
        hall_1_sensor_ticks = reset_value;
        while (Serial.available() > 0)
        {
            Serial.read();
        }
    }
    delay(5);
}