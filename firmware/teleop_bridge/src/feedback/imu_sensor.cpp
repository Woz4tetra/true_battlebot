#include <feedback/imu_sensor.h>

using namespace imu_sensor;

ImuSensor::ImuSensor()
{
    bno = new Adafruit_BNO055(55, 0x28);
    orientation_ = nullptr;
    angular_vel_ = nullptr;
    accel_ = nullptr;
}

bool ImuSensor::begin()
{
    bool success = bno->begin();
    _connected = success;
    if (!success)
    {
        return false;
    }

    orientation_ = new sensors_event_t;
    angular_vel_ = new sensors_event_t;
    accel_ = new sensors_event_t;

    // Use external crystal for better accuracy
    bno->setExtCrystalUse(true);
    return true;
}

bool ImuSensor::update()
{
    if (!_connected)
    {
        return false;
    }
    bno->getEvent(orientation_, Adafruit_BNO055::VECTOR_EULER);
    bno->getEvent(angular_vel_, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno->getEvent(accel_, Adafruit_BNO055::VECTOR_LINEARACCEL);
    return true;
}

bool ImuSensor::has_data()
{
    if (!_connected)
    {
        return false;
    }
    if (orientation_ == nullptr || angular_vel_ == nullptr || accel_ == nullptr)
    {
        return false;
    }
    return true;
}

bool ImuSensor::get_imu_data(bridge::imu_data_p imu_data)
{
    if (!has_data())
    {
        return false;
    }
    imu_data->accel.x = accel_->acceleration.x;
    imu_data->accel.y = accel_->acceleration.y;
    imu_data->accel.z = accel_->acceleration.z;

    imu_data->gyro.x = angular_vel_->gyro.x;
    imu_data->gyro.y = angular_vel_->gyro.y;
    imu_data->gyro.z = angular_vel_->gyro.z;

    imu_data->orientation.x = orientation_->orientation.x;
    imu_data->orientation.y = orientation_->orientation.y;
    imu_data->orientation.z = orientation_->orientation.z;

    return true;
}
