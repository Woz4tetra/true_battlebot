#include <Arduino.h>
#include <bridge/structs.h>

#ifndef __BRIDGE_BASE_BRIDGE_H__
#define __BRIDGE_BASE_BRIDGE_H__
class BaseBridge
{
public:
    /**
     * @brief Process a packet if available.
     *
     * @return True if a packet was processed, false otherwise
     */
    virtual bool update();

protected:
    bridge::config_info_p device_config_;

    virtual void set_motor(uint8_t channel, uint8_t speed, int8_t direction);
    virtual void respond_to_ping(bridge::ping_packet_p packet);
    virtual void respond_to_config(bridge::config_info_p config_info);
    bool process_packet(char *packet, int packet_size);

private:
    /**
     * @brief Process a motor packet. Calls set_motor for the respective channel.
     *
     * @param packet The packet to process
     * @param packet_size The size of the packet
     * @return True if the packet was processed successfully, false otherwise
     */
    bool process_motor_packet(char *packet, int packet_size);

    /**
     * @brief Process a ping packet. Calls respond_to_ping.
     *
     * @param packet The packet to process
     * @param packet_size The size of the packet
     * @return True if the packet was processed successfully, false otherwise
     */
    bool process_ping_packet(char *packet, int packet_size);

    /**
     * @brief Process a config packet. Calls respond_to_config.
     *
     * @param packet The packet to process
     * @param packet_size The size of the packet
     * @return True if the packet was processed successfully, false otherwise
     */
    bool process_config_packet(char *packet, int packet_size);
};
#endif // __BRIDGE_BASE_BRIDGE_H__