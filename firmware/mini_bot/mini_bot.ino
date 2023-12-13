#include <Adafruit_NeoPixel.h>

const int NUM_PIXELS = 1;
const int SHOW_DELAY = 1;
Adafruit_NeoPixel pixels(NUM_PIXELS, 5, NEO_GRB + NEO_KHZ800);

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
        pixels.fill(0xFFFFFF);
        pixels.show();
        delay(100);
        pixels.fill(0x000000);
        pixels.show();
        delay(100);
    }

    Serial.println("mini_bot setup complete");
}

void loop()
{
    for (int hue = 0; hue < 0x10000; hue += 30)
    {
        pixels.fill(pixels.gamma32(pixels.ColorHSV((uint16_t)hue)));
        pixels.show();
        delay(SHOW_DELAY);
    }
}
