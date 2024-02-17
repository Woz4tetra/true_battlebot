#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <bridge/base_bridge.h>
#include <bridge/structs.h>
#include <motors/base_controller.h>
#include <feedback/imu_sensor.h>

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
        static UdpBridge *get_instance(
            bridge::config_info_p config,
            char *read_buffer,
            uint8_t *write_buffer,
            base_controller::BaseController *controller,
            imu_sensor::ImuSensor *imu_sensor_inst)
        {
            static UdpBridge instance(config, read_buffer, write_buffer, controller, imu_sensor_inst);
            return &instance;
        }
        bool update();
        UdpBridgeState get_state() { return state_; }
        void send_imu();

    protected:
        void set_motor(uint8_t channel, float velocity);
        void respond_to_ping(bridge::ping_info_p packet);
        void respond_to_config(bridge::config_info_p config_info);

    private:
        UdpBridge(
            bridge::config_info_p config,
            char *read_buffer,
            uint8_t *write_buffer,
            base_controller::BaseController *controller,
            imu_sensor::ImuSensor *imu_sensor_inst);
        WiFiUDP UDP; // A UDP instance to let us send and receive packets over UDP
        UdpBridgeState state_;
        base_controller::BaseController *controller_;
        imu_sensor::ImuSensor *imu_sensor_inst_;
        char *read_buffer_;
        int read_length_ = 0;

        uint8_t *write_buffer_;

        void init_callback();
        void connecting_callback();
        bool ready_callback();
    };
} // namespace udp_bridge

#endif // __BRIDGE_UDP_BRIDGE_H__