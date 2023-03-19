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

#ifndef _TABLEBOT_ASSEMBLER_HPP_
#define _TABLEBOT_ASSEMBLER_HPP_

#include "stdbool.h"
#include "ld06.hpp"

// We expect about 450 points per rotation (4500/10hz)
#define ASSEMBLED_SCAN_SIZE     450

// 450 degrees is invalid
#define INVALID_ANGLE           45000

class LaserAssembler
{
public:
  void init()
  {
    for (int i = 0; i < ASSEMBLED_SCAN_SIZE; ++i)
    {
      working_angle[i] = INVALID_ANGLE;
    }
    scans_created = 0;
  }

  bool add_packet(ld06_packet_t * packet)
  {
    // Compute in floating point to reduce discretization issues
    float angle = packet->start_angle * 0.01f;
    float end_angle = packet->end_angle * 0.01f;
    
    float step = (end_angle - angle) / (MEASUREMENTS_PER_PACKET - 1);
    if (angle > end_angle)
    {
      // This packet wraps around 0
      step = (end_angle + 360.0f - angle) / (MEASUREMENTS_PER_PACKET - 1);
    }

    // How many of these data points are new?
    int new_data = 0;

    // Copy points into assembled scan
    for (int i = 0; i < MEASUREMENTS_PER_PACKET; ++i)
    {
      // Compute index within assembled scan
      int idx = ((angle + 0.75f) / 360.0f) * (ASSEMBLED_SCAN_SIZE - 1);
      if (idx >= ASSEMBLED_SCAN_SIZE) idx -= ASSEMBLED_SCAN_SIZE;
      if (idx < 0) idx += ASSEMBLED_SCAN_SIZE;

      if (working_angle[idx] == INVALID_ANGLE)
      {
        // This is a new data point
        ++new_data;
        // Copy range measurement
        working_data[idx] = packet->data[i].range;
        // Figure out exact angle
        working_angle[idx] = angle / 0.01f;  // Convert back to 0.01 degree steps
        if (working_angle[idx] > 36000)
        {
          working_angle[idx] -= 36000;
        }
      }
      angle += step;
    }

    if (new_data < 3)
    {
      // Have we analyzed the whole scan?
      int empty_data = 0;
      for (int i = 0; i < ASSEMBLED_SCAN_SIZE; ++i)
      {
        if (working_angle[i] == INVALID_ANGLE)
        {
          ++empty_data;
        }
      }

      if (empty_data < 20)
      {
        // We have filled in the whole scan
        for (int i = 0; i < ASSEMBLED_SCAN_SIZE; ++i)
        {
          data[i] = working_data[i];
          angles[i] = working_angle[i];
          working_angle[i] = INVALID_ANGLE;
        }
        ++scans_created;
        return true;
      }
    }

    return false;
  }

  // 360 Laser View
  uint16_t data[ASSEMBLED_SCAN_SIZE];
  uint16_t angles[ASSEMBLED_SCAN_SIZE];
  uint32_t scans_created;

private:
  uint16_t working_data[ASSEMBLED_SCAN_SIZE];
  uint16_t working_angle[ASSEMBLED_SCAN_SIZE];
};

#endif  // _TABLEBOT_ASSEMBLER_HPP_
