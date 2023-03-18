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

#ifndef _TABLEBOT_AX12_HPP_
#define _TABLEBOT_AX12_HPP_

#define AX_READ_DATA            2
#define AX_WRITE_DATA           3

#define AX_GOAL_POSITION_L      30
#define AX_PRESENT_POSITION_L   36

uint8_t ax12_get_checksum(uint8_t * buffer, uint8_t length)
{
  uint8_t checksum = 0;
  // Checksum ignores first 2 bytes (0xff 0xff)
  for (uint8_t i = 2; i < length; i++)
  {
    checksum += buffer[i];
  }
  return (~checksum);
}

template <typename P, typename B>
int ax12_get_register(P * parser, B * bus, uint8_t id, uint8_t start, uint8_t length)
{
  parser->reset(bus);

  uint8_t buffer[8];
  buffer[0] = 0xff;
  buffer[1] = 0xff;
  buffer[2] = id;
  buffer[3] = 4;  // Packet length
  buffer[4] = AX_READ_DATA;
  buffer[5] = start;
  buffer[6] = length;
  buffer[7] = ax12_get_checksum(buffer, 7);
  bus->write(buffer, 8);

  uint32_t stamp = system_state.time;
  while (true)
  {
    int length = parser->parse(bus, system_state.time);
    if (length > 0)
    {
      // Got a response
      if (length == 1)
      {
        return parser->packet.parameters[0];
      }
      else
      {
        return parser->packet.parameters[0] +
               (parser->packet.parameters[1] << 8);
      }
    }
    else if (length < 0)
    {
      // Error in parsing
      return -1;
    }
    else if (system_state.time - stamp > 10)
    {
      // Timed out
      return -1;
    }
  }
}

template <typename P, typename B>
void ax12_set_register(P * parser, B * bus, uint8_t id, uint8_t addr, uint8_t value)
{
  parser->reset(bus);

  uint8_t buffer[8];
  buffer[0] = 0xff;
  buffer[1] = 0xff;
  buffer[2] = id;
  buffer[3] = 4;  // Packet length
  buffer[4] = AX_WRITE_DATA;
  buffer[5] = addr;
  buffer[6] = value;
  buffer[7] = ax12_get_checksum(buffer, 7);
  bus->write(buffer, 8);
}

template <typename P, typename B>
void ax12_set_register2(P * parser, B * bus, uint8_t id, uint8_t addr, uint16_t value)
{
  parser->reset(bus);

  uint8_t buffer[9];
  buffer[0] = 0xff;
  buffer[1] = 0xff;
  buffer[2] = id;
  buffer[3] = 5;  // Packet length
  buffer[4] = AX_WRITE_DATA;
  buffer[5] = addr;
  buffer[6] = value & 0xff;
  buffer[7] = value >> 8;
  buffer[8] = ax12_get_checksum(buffer, 8);
  bus->write(buffer, 9);
}

#endif  // _TABLEBOT_AX12_HPP_
