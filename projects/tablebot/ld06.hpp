/*
 * Copyright (c) 2023, Michael E. Ferguson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _TABLEBOT_LD06_HPP_
#define _TABLEBOT_LD06_HPP_

/*
 * NOTE: Each packet from laser is max 107 bytes
 *
 * Typical packet appears to have 12 readings in it - meaning we expect 375
 * packets per second (each packet being 45 bytes). This is 16875 bytes per
 * second.
 */

/*
 * LD06 Packet structures
 */
typedef struct __attribute__((packed))
{
  uint16_t range;           // Range in millimeters
  uint8_t confidence;       // Around 200 for white objects within 6m
} ld06_measurement_t;

typedef struct __attribute__((packed, aligned(sizeof(uint32_t))))
{
  uint8_t start_byte;       // Always 0x54
  uint8_t length;           // Lower 5 bits are number of data measurements
  uint16_t radar_speed;     // Degrees per second - 10hz is 3600
  uint16_t start_angle;     // Angle in 0.01 degree increments, 0 is forward
  ld06_measurement_t data[32];
  uint16_t end_angle;       // Angle in 0.01 degree increments, 0 is forward
  uint8_t crc;
} ld06_packet_t;

/*
 * Driver for the LD06 Lidar
 */
template <typename T>
class LD06
{
  enum
  {
    BUS_READING_START,
    BUS_READING_LENGTH,
    BUS_READING_RADAR_SPEED_L,
    BUS_READING_RADAR_SPEED_H,
    BUS_READING_START_ANGLE_L,
    BUS_READING_START_ANGLE_H,
    BUS_READING_DATA,
    BUS_READING_END_ANGLE_L,
    BUS_READING_END_ANGLE_H,
    BUS_READING_CRC,
    BUS_ERROR,
  };

public:
  LD06()
  {
    state_ = BUS_READING_START;
    timeout_ = 10;
    packets_ = errors_ = timeouts_ = 0;

    // Packet size is 9 + 3 * 32 = 105, then aligned to 32-bits
    static_assert(sizeof(ld06_packet_t) == (9 + 3 * 32 + 3));
  }

  void init(T* bus)
  {
    // Initialize usart
    bus->init(230400, 8);

    // Setup timer, for 30khz PWM
    TIM12->CR1 &= (uint16_t) ~TIM_CR1_CEN;  // Disable before configuring
    TIM12->ARR = 1400;  // 42mhz / 30khz
    TIM12->PSC = 1;     // Clock is 42mhz, no prescaler   TODO??? But confirmed via scope as 29.8khz
    // CH2 = PWM Mode 1 - high output while CNT < CCR
    TIM12->CCMR1 = TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    // Enable CH2
    TIM12->CCER = TIM_CCER_CC2E;
    // 40% should be about 10hz rotation
    TIM12->CCR2  = 560;
    TIM12->CR1 |= TIM_CR1_CEN;  // Enable
  }

  /**
   * @brief Parse packets from device.
   * @param bus Pointer to device to read from.
   * @param ms Current system time, for timeouts
   * @returns Length of packet parsed, 0 if no packet, -1 if timeout/error
   */
  int8_t update(T* bus, uint32_t ms)
  {
    if (state_ == BUS_READING_START)
    {
      // Not in a packet yet - reset the timeout
      last_byte_ = ms;
    }

    int32_t b = bus->read();
    while (b >= 0)
    {
      last_byte_ = ms;
      if (state_ == BUS_READING_DATA)
      {
        // The most used state_, so we put it first.
        unsigned char * x = (unsigned char *) packet.data;
        x[data_idx_++] = b;
        if (data_idx_ == bytes_to_read_)
        {
          state_ = BUS_READING_END_ANGLE_L;
        }
      }
      else if (state_ == BUS_READING_START)
      {
        packet.start_byte = b;
        if (b == 0x54)
        {
          state_ = BUS_READING_LENGTH;
        }
      }
      else if (state_ == BUS_READING_LENGTH)
      {
        packet.length = b;
        // Reset data index
        data_idx_ = 0;
        // Bytes to read = 3 * length
        // Only the lower 5 bits of the length actually count
        bytes_to_read_ = 3 * (b & 0x1F);
        state_ = BUS_READING_RADAR_SPEED_L;
      }
      else if (state_ == BUS_READING_RADAR_SPEED_L)
      {
        packet.radar_speed = b;
        state_ = BUS_READING_RADAR_SPEED_H;
      }
      else if (state_ == BUS_READING_RADAR_SPEED_H)
      {
        packet.radar_speed += (b  << 8);
        state_ = BUS_READING_START_ANGLE_L;
      }
      else if (state_ == BUS_READING_START_ANGLE_L)
      {
        packet.start_angle = b;
        state_ = BUS_READING_START_ANGLE_H;
      }
      else if (state_ == BUS_READING_START_ANGLE_H)
      {
        packet.start_angle += (b  << 8);
        state_ = BUS_READING_DATA;
      }
      else if (state_ == BUS_READING_END_ANGLE_L)
      {
        packet.end_angle = b;
        state_ = BUS_READING_END_ANGLE_H;
      }
      else if (state_ == BUS_READING_END_ANGLE_H)
      {
        packet.end_angle += (b  << 8);
        state_ = BUS_READING_CRC;
      }
      else if (state_ == BUS_READING_CRC)
      {
        state_ = BUS_READING_START;
        packet.crc = b;
        // TODO: check CRC, return -1 if bad, state to BUS_ERROR
        ++packets_;
        // Now that CRC is computed, make length correct
        packet.length = packet.length % 0x1F;
        last_speed_ = packet.radar_speed;
        // TODO: Add feedback loop for PWM
        return (9 + 3 * packet.length);
      }
      b = bus->read();
    }

    // If we got here, no packet this time around, should we timeout?
    if (ms > last_byte_ + timeout_)
    {
      if (state_ != BUS_ERROR)
      {
        state_ = BUS_ERROR;
        ++timeouts_;
      }
      return -1;
    }

    // No packet, no error
    return 0;
  }

  /** @brief Reset the state of the parser */
  void reset(T* bus)
  {
    // Get rid of any remaining data in DMA, reset to IDLE
    while (bus->read() >= 0)
      ;

    // Go idle, on next parse last_byte will be reinitialized to current time
    state_ = BUS_READING_START;
  }

  ld06_packet_t packet;

private:
  uint8_t state_;
  uint8_t data_idx_;     // Index within data to write next byte
  uint8_t bytes_to_read_;

  uint8_t checksum_;     // Scratch workspace for computing checksum

  uint32_t last_byte_;   // Timestamp of last byte
  uint32_t timeout_;     // Timeout after which we abort packet

  uint32_t packets_;     // Error counters for debugging
  uint32_t timeouts_;
  uint32_t errors_;

  uint16_t last_speed_;  // For control loop
};

#endif  // _TABLEBOT_LD06_HPP_
