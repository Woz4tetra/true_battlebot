#include <status_lights/status_neopixel.h>

using namespace status_neopixel;

StatusNeopixel::StatusNeopixel() : status_base::StatusBase()
{
    pixels_ = new Adafruit_NeoPixel(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
}

void StatusNeopixel::begin()
{
#if defined(NEOPIXEL_POWER)
    // If this board has a power control pin, we must set it to output and high
    // in order to enable the NeoPixels_-> We put this in an #if defined so it can
    // be reused for other boards without compilation errors
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

    pixels_->begin();           // INITIALIZE NeoPixel pixels object (REQUIRED)
    pixels_->setBrightness(20); // not so bright
}

void StatusNeopixel::update()
{
    uint32_t now = millis();
    switch (state_)
    {
    case status_base::WAITING_FOR_CONFIG:
        if (now - last_update_ < 250)
        {
            return;
        }
        if (blink_mode_color == 0)
        {
            blink_mode_color = 255;
        }
        else
        {
            blink_mode_color = 0;
        }
        pixels_->setBrightness(20);
        pixels_->fill(pixels_->Color(0, 0, blink_mode_color));
        break;
    case status_base::CONNECTING:
        if (now - last_update_ < 250)
        {
            return;
        }
        if (blink_mode_color == 0)
        {
            blink_mode_color = 255;
        }
        else
        {
            blink_mode_color = 0;
        }
        pixels_->setBrightness(20);
        pixels_->fill(pixels_->Color(blink_mode_color, blink_mode_color, blink_mode_color));
        break;
    case status_base::TIMED_OUT:
        if (now - last_update_ < 50)
        {
            return;
        }
        pixels_->setBrightness(20);
        pixels_->fill(pixels_->Color(idle_mode_color_, 0, 0));
        idle_mode_color_ = (idle_mode_color_ + 1) % 255;
        break;
    case status_base::OK:
        if (now - last_update_ < 20)
        {
            return;
        }
        pixels_->fill(pixels_->gamma32(pixels_->ColorHSV((uint16_t)active_mode_hue_)));
        active_mode_hue_ = (active_mode_hue_ + 100) % 0x10000;
        break;
    default:
        break;
    }
    pixels_->show();
}
