#include <Arduino.h>

const int LED_PIN = 13;
const int INTERRUPT_PIN_0 = 2;
const int INTERRUPT_PIN_1 = 3;

volatile unsigned int hall_0_sensor_ticks = 0;
volatile unsigned int hall_1_sensor_ticks = 0;

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
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_0), interrupt_0_handler, RISING);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_1), interrupt_1_handler, RISING);
}

void loop()
{
    Serial.print(hall_0_sensor_ticks);
    Serial.print("\t");
    Serial.println(hall_1_sensor_ticks);
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