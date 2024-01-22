#include <Arduino.h>

#ifndef __STATUS_BASE_H__
#define __STATUS_BASE_H__

namespace status_base
{
    enum StatusState
    {
        WAITING_FOR_CONFIG,
        CONNECTING,
        TIMED_OUT,
        OK
    };

    class StatusBase
    {
    public:
        StatusBase() { state_ = WAITING_FOR_CONFIG; }
        virtual void begin();
        virtual void update();
        void set_state(StatusState state) { state_ = state; }
        virtual void set_speed_readout(int speed);
        StatusState get_state() { return state_; }

    protected:
        StatusState state_;
    };
} // namespace status_base

#endif // __STATUS_BASE_H__
