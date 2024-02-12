#include <feedback/imu_feedback.h>

using namespace base_feedback;
using namespace imu_feedback;

ImuFeedback::ImuFeedback(imu_sensor::ImuSensor *sensor, float radius) : BaseFeedback()
{
    sensor_ = sensor;
    radius_ = radius;
}

float ImuFeedback::get_feedback()
{
    return sensor_->get_angular_z() * radius_;
}

bool ImuFeedback::has_feedback()
{
    return sensor_->is_connected();
}
