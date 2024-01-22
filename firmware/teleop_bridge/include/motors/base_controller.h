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
         * @brief Set a motor channel
         *
         * @param channel The motor channel to set
         * @param speed The speed to set the motor to (0..255)
         * @param direction The direction to set the motor to (negative == backwards, 0 == stop, positive == forward)
         */
        virtual void set_motor(uint8_t channel, uint8_t speed, int8_t direction);
    };
} // namespace base_controller

#endif // __BASE_CONTROLLER_H__
