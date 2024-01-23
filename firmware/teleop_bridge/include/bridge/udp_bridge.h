#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <bridge/base_bridge.h>
#include <bridge/structs.h>
#include <motors/base_controller.h>

#ifndef __BRIDGE_UDP_BRIDGE_H__
#define __BRIDGE_UDP_BRIDGE_H__
namespace udp_bridge
{
    enum UdpBridgeState
    {
        INIT,
        CONNECTING,
        READY
    };

    class UdpBridge : public BaseBridge
    {
    public:
        static UdpBridge *get_instance(bridge::config_info_p config, char *buffer, base_controller::BaseController *controller)
        {
            static UdpBridge instance(config, buffer, controller);
            return &instance;
        }
        bool update();
        UdpBridgeState get_state() { return state_; }

    protected:
        void set_motor(uint8_t channel, int velocity);
        void respond_to_ping(bridge::ping_packet_p packet);
        void respond_to_config(bridge::config_info_p config_info);

    private:
        UdpBridge(bridge::config_info_p config, char *buffer, base_controller::BaseController *controller);
        WiFiUDP UDP; // A UDP instance to let us send and receive packets over UDP
        UdpBridgeState state_;
        base_controller::BaseController *controller_;
        char *buffer_;
        int read_length_ = 0;

        void init_callback();
        void connecting_callback();
        bool ready_callback();
    };
} // namespace udp_bridge

#endif // __BRIDGE_UDP_BRIDGE_H__