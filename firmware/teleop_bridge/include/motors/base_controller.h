#include <Arduino.h>

#ifndef __BASE_CONTROLLER_H__
#define __BASE_CONTROLLER_H__

namespace base_controller
{
    class BaseController
    {
    public:
        BaseController() {}

        /**
         * @brief Attach pins and set up the controller
         *
         */
        virtual void begin();

        /**
         * @brief Set a motor channel
         *
         * @param channel The motor channel to set
         * @param speed The speed to set the motor to (0..255)
         * @param direction The direction to set the motor to (negative == backwards, 0 == stop, positive == forward)
         */
        virtual void set_motor(uint8_t channel, float velocity);

        /**
         * @brief Get the motor channel
         *
         * @param channel The motor channel to get
         * @return int The speed of the motor
         */
        virtual int get_command(uint8_t channel);

        /**
         * @brief Stop all motors
         */
        virtual void stop_all_motors();

        /**
         * @brief Get the number of channels
         *
         * @return int The number of channels
         */
        virtual int get_num_channels();
    };
} // namespace base_controller

#endif // __BASE_CONTROLLER_H__
