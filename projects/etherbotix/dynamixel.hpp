/*
 * Copyright (c) 2013-2014, Michael E. Ferguson
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

#ifndef __ETHERBOTIX_DYNAMIXEL_HPP__
#define __ETHERBOTIX_DYNAMIXEL_HPP__

// Dynamixel Instructions
#define DYN_READ_DATA       2
#define DYN_WRITE_DATA      3
#define DYN_SYNC_WRITE      131
#define DYN_SYNC_READ       132

/** @brief Dynamixel packet, minus the double 0xff. */
typedef struct
{
  uint8_t id;
  uint8_t length;
  uint8_t error;
  uint8_t parameters[256];
  uint8_t checksum;
} DynamixelPacket_t;

/**
 * @brief A parser for Dynamixel device status packets.
 * @tparam T The usart-like device to read from. Must have a function "read()"
 *         which returns an int16_t. Return value of -1 means "no data".
 */
template <typename T>
class DynamixelParser
{
  enum
  {
    BUS_IDLE,
    BUS_WRITING,
    BUS_READING_FF,
    BUS_READING_2ND_FF,
    BUS_READING_ID,
    BUS_READING_LENGTH,
    BUS_READING_ERROR,
    BUS_READING_PARAMS,
    BUS_READING_CHECKSUM,
    BUS_ERROR
  };

public:
  DynamixelParser()
  {
    state_ = BUS_IDLE;
    timeout_ = 10;  // TODO: 2ms?
    packets_ = errors_ = timeouts_ = 0;
  }

  /**
   * @brief Parse packets from device.
   * @param bus Pointer to device to read from.
   * @param ms Current system time, for timeouts
   * @returns Length of packet parsed, 0 if no packet, -1 if timeout/error
   */
  int8_t parse(T* bus, uint32_t ms)
  {
    if (state_ == BUS_IDLE)
    {
      // Start of new packet
      state_ = BUS_READING_FF;
      last_byte_ = ms;
    }

    int16_t b = bus->read();
    while (b >= 0)
    {
      last_byte_ = ms;
      if (state_ == BUS_READING_PARAMS)
      {
        // Parameters is the most used state_, so we put it first.
        packet.parameters[param_++] = b;
        checksum_ += b;
        if (param_ == packet.length-2)
          state_ = BUS_READING_CHECKSUM;
      }
      else if (state_ == BUS_READING_FF)
      {
        if (b == 0xff)
          state_ = BUS_READING_2ND_FF;
      }
      else if (state_ == BUS_READING_2ND_FF)
      {
        if (b == 0xff)
          state_ = BUS_READING_ID;
        else
          state_ = BUS_READING_FF;  // Restart if we don't have two 0xff
      }
      else if (state_ == BUS_READING_ID)
      {
        if (b != 0xff)
        {
          packet.id = b;
          state_ = BUS_READING_LENGTH;
        }
        //TODO: should we restart if we get three 0xff??
        //else
        //  state_ = BUS_READING_FF;  // Restart
      }
      else if (state_ == BUS_READING_LENGTH)
      {
        if (b < 140)  // Based on 143 character max in RX64 datasheet
        {
          packet.length = b;
          state_ = BUS_READING_ERROR;
        }
        else
          state_ = BUS_READING_FF;  // Restart
      }
      else if (state_ == BUS_READING_ERROR)
      {
        packet.error = b;
        checksum_ = packet.id + packet.length + b;
        param_ = 0;
        state_ = BUS_READING_PARAMS;
      }
      else if (state_ == BUS_READING_CHECKSUM)
      {
        state_ = BUS_IDLE;
        packet.checksum = b;
        checksum_ += b;
        if (checksum_ == 255)
        {
          // Packet is good
          packets_++;
          return packet.length + 4;
        }
        else
        {
          // Packet is bad
          state_ = BUS_ERROR;
          errors_++;
          return -1;
        }
      }
      b = bus->read();
    }

    // If we got here, no packet this time around, should we timeout?
    if (ms > last_byte_ + timeout_)
    {
      if (state_ != BUS_ERROR)
      {
        state_ = BUS_ERROR;
        timeouts_++;
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
    state_ = BUS_IDLE;
  }

  DynamixelPacket_t packet;  /// The actual packet parsed

private:
  uint8_t state_;  // Where we are in parsing
  uint8_t param_;  // The index within packet.parameters when parsing parameters
  uint8_t checksum_;  // Scratch workspace for computing checksum

  uint32_t last_byte_;  // Timestamp of last byte
  uint32_t timeout_;  // Timeout after which we abort packet

  uint32_t packets_;  // Error counters for debugging
  uint32_t timeouts_;
  uint32_t errors_;
};

#endif // __ETHERBOTIX_DYNAMIXEL_HPP__
