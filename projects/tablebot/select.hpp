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

#ifndef _TABLEBOT_SELECT_HPP_
#define _TABLEBOT_SELECT_HPP_

#include "tablebot.hpp"

#define MODE_UNSELECTED     0
#define MODE_DONE           255

// Next state (once selected)
int next_selection = -1;

int get_next_mode()
{
  return next_selection;
}

uint16_t select_mode()
{
  static uint32_t last_unlevel_stamp = 0;

  uint16_t mode = system_state.run_state;

  if (mode == MODE_DONE)
  {
    // If we are done, but then rotated away from vertical, re-enter unselected
    if (system_state.accel_z < 2000)
    {
      next_selection = -1;
      last_unlevel_stamp = system_state.time;
      mode = MODE_UNSELECTED;
    }
  }

  if (mode == MODE_UNSELECTED)
  {
    if (system_state.accel_x < -7000)
    {
      // Robot - left side up side
      next_selection = 1;
      last_unlevel_stamp = system_state.time;
    }
    else if (system_state.accel_x > 7000)
    {
      // Robot right side up
      next_selection = 2;
      last_unlevel_stamp = system_state.time;
    }
    else if (system_state.accel_y > 7000)
    {
      // Robot front side up
      next_selection = 3;
      last_unlevel_stamp = system_state.time;
    }
    else if (system_state.accel_z > 7000)
    {
      // Robot is on level - we can start if we have been level for 3 seconds
      if (system_state.time - last_unlevel_stamp > 3000 && next_selection > 0)
      {
        mode = next_selection;
      }
    }
  }

  return mode;
}

#endif  // _TABLEBOT_SELECT_HPP_
