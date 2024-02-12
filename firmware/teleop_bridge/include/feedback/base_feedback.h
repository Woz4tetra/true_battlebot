#include <Arduino.h>

#ifndef __BASE_FEEDBACK_H__
#define __BASE_FEEDBACK_H__

namespace base_feedback
{
    class BaseFeedback
    {
    public:
        BaseFeedback() {}

        /**
         * @brief Attach pins and set up the feedback
         *
         */
        virtual void begin();

        /**
         * @brief Get feedback on the motor
         *
         * @return float The feedback value
         */
        virtual float get_feedback();

        /**
         * @brief Check if the sensor has a valid feedback value
         *
         * @return true If feedback is available
         * @return false If feedback is not available
         */
        virtual bool has_feedback();
    };
} // namespace base_feedback

#endif // __BASE_FEEDBACK_H__