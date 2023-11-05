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

#ifndef _LD06_HPP_
#define _LD06_HPP_

/*
 * NOTE: Each packet from laser is max 107 bytes per datasheet - but ROS
 *       driver is hardcoded to 12 readings per packet.
 *
 * Since each packet has 12 readings in it, we expect 375 packets per second
 * (each packet being 47 bytes). This is 17625 bytes per second.
 */
#define EXPECTED_LENGTH_BYTE      0x2C
#define EXPECTED_PACKET_LENGTH    (11 + 3 * 12)

// Timeout in milliseconds
#define LASER_TIMEOUT             10

// Desire laser speed is 10hz - 3600
#define DESIRED_LASER_SPEED       3600
#define MINIMUM_LASER_SPEED       3400
#define MAXIMUM_LASER_SPEED       3800

#define MEASUREMENTS_PER_PACKET   12

/*
 * LD06 Packet structures
 */
typedef struct __attribute__((packed))
{
  uint16_t range;           // Range in millimeters
  uint8_t confidence;       // Around 200 for white objects within 6m
} ld06_measurement_t;

typedef struct __attribute__((packed))
{
  uint8_t start_byte;       // Always 0x54
  uint8_t length;           // Lower 5 bits are number of data measurements
  uint16_t radar_speed;     // Degrees per second - 10hz is 3600
  uint16_t start_angle;     // Angle in 0.01 degree increments, 0 is forward
  ld06_measurement_t data[12];
  uint16_t end_angle;       // Angle in 0.01 degree increments, 0 is forward
  uint16_t timestamp;       // In milliseconds
  uint8_t crc;
} ld06_packet_t;

// From datasheet
static const uint8_t ld06_crc_table[256] =
{
  0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
  0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
  0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
  0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
  0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
  0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
  0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
  0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
  0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
  0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
  0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
  0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
  0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
  0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
  0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
  0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
  0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
  0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
  0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
  0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
  0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
  0x7f, 0x32, 0xe5, 0xa8
};

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
    BUS_READING_DATA,
    BUS_ERROR,
  };

public:
  LD06()
  {
    state_ = BUS_READING_START;
    packets_ = errors_ = crc_errors_ = timeouts_ = 0;
    static_assert(sizeof(ld06_packet_t) == (EXPECTED_PACKET_LENGTH));
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
    control_pwm_ = 560;
    TIM12->CCR2  = control_pwm_;
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
        unsigned char * data = (unsigned char *) &packet;
        data[data_idx_++] = b;
        if (data_idx_ == EXPECTED_PACKET_LENGTH)
        {
          // Next state will be starting again
          state_ = BUS_READING_START;

          // Compute CRC
          uint8_t crc = 0;
          for (uint32_t i = 0; i < EXPECTED_PACKET_LENGTH - 1; i++)
          {
            crc = ld06_crc_table[(crc ^ data[i]) & 0xff];
          }

          // Check CRC is correct
          if (crc == packet.crc)
          {
            ++packets_;
            last_speed_ = packet.radar_speed;
            if (last_speed_ < MINIMUM_LASER_SPEED)
            {
              control_pwm_ += 1;
            }
            else if (last_speed_ > MAXIMUM_LASER_SPEED)
            {
              control_pwm_ -= 1;
            }
            TIM12->CCR2 = control_pwm_;
            return EXPECTED_PACKET_LENGTH;
          }

          // Bad packet, return -1, set state to BUS_ERROR
          ++crc_errors_;
          state_ = BUS_ERROR;
          return -2;
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
        if (b != EXPECTED_LENGTH_BYTE)
        {
          // Not the right value - error
          state_ = BUS_ERROR;
          ++errors_;
          return -1;
        }
        // Reset data index
        data_idx_ = 2;
        state_ = BUS_READING_DATA;
      }
      b = bus->read();
    }

    // If we got here, no packet this time around, should we timeout?
    if (ms > last_byte_ + LASER_TIMEOUT)
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
  uint32_t last_byte_;   // Timestamp of last byte

  uint32_t packets_;     // Error counters for debugging
  uint32_t timeouts_;
  uint32_t errors_;
  uint32_t crc_errors_;

  uint16_t last_speed_;  // For control loop
  uint16_t control_pwm_;
};

#endif  // _LD06_HPP_
