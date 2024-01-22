#include <Arduino.h>
#include <bridge/base_bridge.h>
#include <bridge/structs.h>
#include <bridge/persistent_config.h>

#ifndef __BRIDGE_SERIAL_BRIDGE_H__
#define __BRIDGE_SERIAL_BRIDGE_H__
namespace serial_bridge
{
    enum SerialBridgeState
    {
        WAITING_FOR_CONFIG,
        READY
    };

    // The first two bytes of a serial packet
    const char SERIAL_PACKET_C0 = 'b';
    const char SERIAL_PACKET_C1 = 'w';

    class SerialBridge : public BaseBridge
    {
    public:
        static SerialBridge *get_instance(bridge::config_info_p config, char *buffer, persistent_config::PersistentConfig *persistent_config)
        {
            static SerialBridge instance(config, buffer, persistent_config);
            return &instance;
        }
        bool update();
        SerialBridgeState get_state() { return state_; }

    protected:
        void set_motor(uint8_t channel, uint8_t speed, int8_t direction);
        void respond_to_ping(bridge::ping_packet_p packet);
        void respond_to_config(bridge::config_info_p config_info);

    private:
        SerialBridge(bridge::config_info_p config, char *buffer, persistent_config::PersistentConfig *persistent_config);
        SerialBridgeState state_;
        char *buffer_;
        int read_length_ = 0;
        int serial_read_length_ = 0;
        persistent_config::PersistentConfig *persistent_config_;

        /**
         * @brief Get the length of the next serial packet
         *
         * @return The length of the next serial packet, or 0 if no packet is available
         */
        int get_serial_packet_length();

        /**
         * @brief Process a serial packet
         *
         * @return True if the packet was processed successfully, false otherwise
         */
        bool process_serial_packet();
    };

} // namespace serial_bridge
#endif // __BRIDGE_SERIAL_BRIDGE_H__