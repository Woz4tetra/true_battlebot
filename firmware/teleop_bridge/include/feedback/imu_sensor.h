#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <bridge/structs.h>

#ifndef __IMU_SENSOR_H__
#define __IMU_SENSOR_H__

namespace imu_sensor
{
    class ImuSensor
    {
    public:
        ImuSensor();
        bool begin();
        bool update();
        float get_angular_z() { return angular_vel_->gyro.y; } // IMU is mounted sideways
        bool is_connected() { return _connected; }
        bool get_imu_data(bridge::imu_data_p imu_data);
        bool has_data();

    private:
        Adafruit_BNO055 *bno;
        bool _connected = false;
        sensors_event_t *orientation_, *angular_vel_, *accel_;
        uint32_t last_update_time_ = 0;
    };
} // namespace imu_sensor

#endif // __IMU_SENSOR_H__