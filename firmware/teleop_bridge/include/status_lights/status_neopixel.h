#include <Adafruit_NeoPixel.h>

#ifndef __STATUS_NEOPIXEL_H__
#define __STATUS_NEOPIXEL_H__
/**
 * Neopixels
 */
const int NUM_PIXELS = 1; // This board has a single built in neopixel.
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
int active_mode_hue = 0; // While motor commands are active, the hue of neopixel will cycle
int idle_mode_color = 0; // When no motor commands are active, flash a single channel with this color (red)

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

void begin()
{
#if defined(NEOPIXEL_POWER)
    // If this board has a power control pin, we must set it to output and high
    // in order to enable the NeoPixels. We put this in an #if defined so it can
    // be reused for other boards without compilation errors
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

    pixels.begin();           // INITIALIZE NeoPixel pixels object (REQUIRED)
    pixels.setBrightness(20); // not so bright
}

#endif // __STATUS_NEOPIXEL_H__