#include <Arduino.h>

#ifndef __STATUS_BASE_H__
#define __STATUS_BASE_H__

uint32_t prev_packet_time = 0;            // The last time a packet was received (milliseconds)
const uint32_t PACKET_WARN_TIMEOUT = 250; // If no packets are received for this long, flash yellow (milliseconds)
const uint32_t PACKET_STOP_TIMEOUT = 500; // If no packets are received for this long, turn off motors (milliseconds)

#endif // __STATUS_BASE_H__
