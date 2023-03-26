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

// Current state of robot
int current_selection = -1;

// Next state (once selected)
int next_selection = -1;

uint32_t select_stamp = 0;

int get_next_mode()
{
  if (current_selection > 0 && system_state.time - select_stamp > 1000)
  {
    return current_selection;
  }

  return next_selection;
}

uint8_t select_mode()
{
  uint8_t mode = system_state.run_state;

  if (mode == MODE_DONE)
  {
    // If we are done, but then rotated away from vertical, re-enter unselected
    if (system_state.accel_z < 2000)
    {
      current_selection = -1;
      next_selection = -1;
      select_stamp = system_state.time;
      mode = MODE_UNSELECTED;
    }
  }

  if (mode == MODE_UNSELECTED)
  {
    if (system_state.accel_z > 3000)
    {
      // Robot is somewhat right side up - certainly not tipped enough to trigger a state
      if (current_selection > 0)
      {
        if (system_state.time - select_stamp > 1000)
        {
          // Selection was held for at least 1 second
          next_selection = current_selection;
        }
        current_selection = 0;
        select_stamp = system_state.time;
      }

      if (system_state.accel_x > 3000 || system_state.accel_x < -3000 ||
          system_state.accel_y > 3000 || system_state.accel_y < -3000 ||
          system_state.accel_z < 7000)
      {
        // Not level enough
        select_stamp = system_state.time;
      }

      if (system_state.cliff_left > CLIFF_DETECTED ||
          system_state.cliff_right > CLIFF_DETECTED ||
          system_state.cliff_center > CLIFF_DETECTED)
      {
        // Not on the table
        select_stamp = system_state.time;
      }

      // Have we been level and on table for 1 seconds?
      if (system_state.time - select_stamp > 1000 && next_selection > 0)
      {
        // We are level and on a table
        mode = next_selection;
      }
    }
    else if (system_state.accel_x < -7000)
    {
      // Robot - left side up side
      if (current_selection != 1)
      {
        current_selection = 1;
        select_stamp = system_state.time;
      }
    }
    else if (system_state.accel_x > 7000)
    {
      // Robot right side up
      if (current_selection != 2)
      {
        current_selection = 2;
        select_stamp = system_state.time;
      }
    }
    else if (system_state.accel_y > 7000)
    {
      // Robot front side up
      if (current_selection != 3)
      {
        current_selection = 3;
        select_stamp = system_state.time;
      }
    }
  }

  return mode;
}

#endif  // _TABLEBOT_SELECT_HPP_
