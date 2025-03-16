#include <crsf_bridge.h>

using namespace crsf_bridge;

CrsfBridge::CrsfBridge()
{
    crsf = new AlfredoCRSF();
}

void CrsfBridge::begin()
{
    CRSF_SERIAL.begin(CRSF_BAUDRATE, SERIAL_8N1, RXD1, TXD1);
    crsf->begin(CRSF_SERIAL);
}

bool CrsfBridge::update(radio_data_t *radio_data)
{
    crsf->update();
    three_state_switch_t flip_switch_state;
    if (crsf->isLinkUp())
    {
        const crsf_channels_t *channels = crsf->getChannelsPacked();
        radio_data->a_percent = -1 * scale_channel_to_percent(channels->ch0);
        radio_data->b_percent = scale_channel_to_percent(channels->ch3);
        radio_data->armed = channels->ch4 > MID_CYCLE;
        radio_data->flip_switch_state = get_switch_state(channels->ch5);
        radio_data->button_state = get_button_state(channels->ch6);
        radio_data->lifter_command = channels->ch7 > UPPER_CYCLE;
        radio_data->connected = true;
        return true;
    }
    else
    {
        radio_data->a_percent = 0.0;
        radio_data->b_percent = 0.0;
        radio_data->armed = false;
        radio_data->flip_switch_state = MIDDLE;
        radio_data->button_state = false;
        radio_data->lifter_command = false;
        radio_data->connected = false;
        return false;
    }
}

float CrsfBridge::scale_channel_to_percent(float channel_value)
{
    float percent;
    if (channel_value < MID_CYCLE)
        percent = -100.0 / (MID_CYCLE - MIN_CYCLE) * (MID_CYCLE - channel_value);
    else
        percent = 100.0 / (MAX_CYCLE - MID_CYCLE) * (channel_value - MID_CYCLE);
    return min(100.0f, max(-100.0f, percent));
}

three_state_switch_t CrsfBridge::get_switch_state(float channel_value)
{
    if (channel_value < LOWER_CYCLE)
        return DOWN;
    else if (channel_value < UPPER_CYCLE)
        return MIDDLE;
    else
        return UP;
}

bool CrsfBridge::get_button_state(float channel_value)
{
    return channel_value > MID_CYCLE;
}
