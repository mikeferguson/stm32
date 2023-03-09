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

#ifndef _TABLEBOT_BEHAVIORS_HPP_
#define _TABLEBOT_BEHAVIORS_HPP_
// Phase 1 = drive to end of table and back
#define BEHAVIOR_ID_PHASE_1   1
// Phase 2 = find the cube, push it off the table
#define BEHAVIOR_ID_PHASE_2   2
// Phase 3 = find the cube, push it into the box
#define BEHAVIOR_ID_PHASE_3   3

uint8_t behavior_id = 0;
uint8_t behavior_state = 0;

float last_pose_x;
float last_pose_y;
float last_pose_th;

// Phase 1 states
#define PHASE1_DRIVE_TOWARDS_END  0
#define PHASE1_BACK_UP_A_BIT      1
#define PHASE1_TURN_IN_PLACE      2
#define PHASE1_RETURN_TO_START    3

void run_behavior(uint8_t id, uint32_t stamp)
{
  if (id == 0)
  {
    // Phase is not yet selected
    return;
  }

  if (id != behavior_id)
  {
    // This is our first iteration through - reset things
    behavior_id = id;
    behavior_state = 0;
    last_pose_x = system_state.motor1_pos;  // TOOD: use combined odometry
  }

  if (id == BEHAVIOR_ID_PHASE_1)
  {
    if (behavior_state == PHASE1_DRIVE_TOWARDS_END)
    {
      // If we see the cliff, stop and transition to next state
      if (system_state.cliff_left > 0 || system_state.cliff_right > 0 || system_state.cliff_center > 0)
      {
        m1_pid.update_setpoint(0);
        m2_pid.update_setpoint(0);
        last_pose_x = system_state.pose_x;  // Cache the X position along table length
        behavior_state = PHASE1_BACK_UP_A_BIT;
      }

      // Else drive forward at constant speed
      // TODO: what speed?
      // TODO: use odometry & maintain heading
      m1_pid.update_setpoint(0);
      m2_pid.update_setpoint(0);
    }
    else if (behavior_state == PHASE1_BACK_UP_A_BIT)
    {
      if ((last_pose_x - system_state.pose_x) > 0.15f)
      {
        // Done backing up - stop robot
        m1_pid.update_setpoint(0);
        m2_pid.update_setpoint(0);
        last_pose_th = system_state.pose_th;
        behavior_state = PHASE1_TURN_IN_PLACE;
      }
      else
      {
        // TODO: Backup Command
      }
    }
    else if (behavior_state == PHASE1_TURN_IN_PLACE)
    {
      float angle_error = last_pose_th + 1.57f - system_state.pose_th;
      if (abs(angle_error) < 0.025f)
      {
        // Done rotating in place - stop robot
        m1_pid.update_setpoint(0);
        m2_pid.update_setpoint(0);
        last_pose_th = system_state.pose_th;
        behavior_state = PHASE1_RETURN_TO_START;
      }
      else
      {
        // TODO: Rotate Command based on angular error
      } 
    }
    else if (behavior_state == PHASE1_RETURN_TO_START)
    {
      // Drive only if the cliff sensors aren't tripped
      if (system_state.cliff_left == 0 || system_state.cliff_right == 0 || system_state.cliff_center == 0)
      {
        m1_pid.update_setpoint(100);
        m2_pid.update_setpoint(100);
      }
      else
      {
        // Stop robot
        m1_pid.update_setpoint(0);
        m2_pid.update_setpoint(0);
      }
    }
  }
  else if (id == BEHAVIOR_ID_PHASE_2)
  {

  }
  else if (id == BEHAVIOR_ID_PHASE_3)
  {

  }
}

#endif  // _TABLEBOT_BEHAVIORS_HPP_
