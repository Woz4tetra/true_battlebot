#include <Adafruit_NeoPixel.h>
#include <status_lights/status_base.h>

#ifndef __STATUS_NEOPIXEL_H__
#define __STATUS_NEOPIXEL_H__

namespace status_neopixel
{
    const int NUM_PIXELS = 1; // This board has a single built in neopixel.
    class StatusNeopixel : public status_base::StatusBase
    {
    public:
        StatusNeopixel();
        void begin();
        void update();
        void set_speed_readout(int speed);

    private:
        Adafruit_NeoPixel *pixels_;
        int active_mode_hue_ = 0;
        int idle_mode_color_ = 0;
        int blink_mode_color = 0;
        uint32_t last_update_ = 0;
    };
} // namespace status_neopixel

#endif // __STATUS_NEOPIXEL_H__