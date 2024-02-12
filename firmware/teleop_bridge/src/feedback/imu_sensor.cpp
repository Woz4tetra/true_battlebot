#include <feedback/imu_sensor.h>

using namespace imu_sensor;

ImuSensor::ImuSensor()
{
    bno = new Adafruit_BNO055(55, 0x28);
}

bool ImuSensor::begin()
{
    bool success = bno->begin();
    _connected = success;
    if (!success)
    {
        return false;
    }

    // Use external crystal for better accuracy
    bno->setExtCrystalUse(true);
    return true;
}

void ImuSensor::update()
{
    bno->getEvent(orientation_, Adafruit_BNO055::VECTOR_EULER);
    bno->getEvent(angular_vel_, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno->getEvent(accel_, Adafruit_BNO055::VECTOR_LINEARACCEL);
}

void ImuSensor::get_imu_data(bridge::imu_data_p imu_data)
{
    imu_data->accel.x = accel_->acceleration.x;
    imu_data->accel.y = accel_->acceleration.y;
    imu_data->accel.z = accel_->acceleration.z;

    imu_data->gyro.x = angular_vel_->gyro.x;
    imu_data->gyro.y = angular_vel_->gyro.y;
    imu_data->gyro.z = angular_vel_->gyro.z;

    imu_data->orientation.x = orientation_->orientation.x;
    imu_data->orientation.y = orientation_->orientation.y;
    imu_data->orientation.z = orientation_->orientation.z;
}
