#pragma once
#include <Arduino.h>
#include <AlfredoCRSF.h>

namespace crsf_bridge
{
#define RXD1 18
#define TXD1 17
#define CRSF_SERIAL Serial1

    const float MIN_CYCLE = 200.0;
    const float LOWER_CYCLE = 800.0;
    const float MID_CYCLE = 1000.0;
    const float UPPER_CYCLE = 1200.0;
    const float MAX_CYCLE = 1700.0;

    typedef enum three_state_switch
    {
        DOWN = 0,
        MIDDLE = 1,
        UP = 2
    } three_state_switch_t;

    typedef struct radio_data
    {
        float a_percent, b_percent;
        bool armed, lifter_command, connected, button_state;
        three_state_switch_t flip_switch_state;
    } radio_data_t;

    class CrsfBridge
    {
    private:
        AlfredoCRSF *crsf;
        three_state_switch_t get_switch_state(float channel_value);
        bool get_button_state(float channel_value);
        float scale_channel_to_percent(float channel_value);

    public:
        CrsfBridge();
        void begin();
        bool update(radio_data_t *radio_data);
    };
}