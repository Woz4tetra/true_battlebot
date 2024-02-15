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
    delay(250); // Give the sensor time to initialize
    last_update_time_ = millis();

    return true;
}

bool ImuSensor::update()
{
    if (!_connected)
    {
        return false;
    }

    uint32_t current_time = millis();

    if (current_time - last_update_time_ > 500)
    {
        Serial.println("BNO055 disconnected or has a fault. Trying to reestablish connection...");
        if (bno->begin())
        {
            Serial.println("Connection to BNO055 reestablished.");
        }
        last_update_time_ = millis();
        return false; // Wait for next update to try again
    }
    sensors_event_t orientation, angular_vel, accel;
    bno->getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);
    bno->getEvent(&angular_vel, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno->getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    if (accel.acceleration.x != accel_->acceleration.x &&
        accel.acceleration.y != accel_->acceleration.y &&
        accel.acceleration.z != accel_->acceleration.z)
    {
        last_update_time_ = current_time;
    }
    *orientation_ = orientation;
    *angular_vel_ = angular_vel;
    *accel_ = accel;

    filtered_accel_ = filter_k_ * filtered_accel_ + (1 - filter_k_) * accel.acceleration.y;

    return true;
}

float ImuSensor::get_angular_z()
{
    // IMU is mounted sideways

    float angular_z = angular_vel_->gyro.y;
    if (filtered_accel_ < 0.0)
    {
        angular_z *= -1;
    }

    return angular_z;
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
