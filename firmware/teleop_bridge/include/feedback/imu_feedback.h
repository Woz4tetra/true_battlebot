#include <Arduino.h>
#include <feedback/base_feedback.h>
#include <feedback/imu_sensor.h>

#ifndef __IMU_FEEDBACK_H__
#define __IMU_FEEDBACK_H__

namespace imu_feedback
{
    class ImuFeedback : public base_feedback::BaseFeedback
    {
    public:
        ImuFeedback(imu_sensor::ImuSensor *sensor, float radius);
        void begin() {}
        float get_feedback();
        bool has_feedback();

    private:
        imu_sensor::ImuSensor *sensor_;
        float radius_;
    };
} // namespace imu_feedback

#endif // __IMU_FEEDBACK_H__