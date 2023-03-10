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

// Phase 1 states
#define PHASE1_DRIVE_TOWARDS_END  0
#define PHASE1_BACK_UP_A_BIT      1
#define PHASE1_TURN_IN_PLACE      2
#define PHASE1_RETURN_TO_START    3

uint16_t behavior_id = 0;
uint8_t behavior_state = 0;

void set_motors(int16_t left, int16_t right)
{
  m1_pid.update_setpoint(left);
  m2_pid.update_setpoint(right);
  system_state.last_motor_command = system_state.time;
}

void run_behavior(uint16_t id, uint32_t stamp)
{
  static float last_pose_x;
  static float last_pose_y;
  static float last_pose_th;

  if (id == MODE_UNSELECTED)
  {
    // Phase is not yet selected
    return;
  }

  if (id != behavior_id)
  {
    // This is our first iteration through - reset things
    behavior_id = id;
    behavior_state = 0;

    // Reset pose
    __disable_irq();
    system_state.pose_x = 0.0f;
    system_state.pose_y = 0.0f;
    system_state.pose_th = 0.0f;
    __enable_irq();

    // Cache starting pose
    last_pose_x = system_state.pose_x;
    last_pose_y = system_state.pose_y;
    last_pose_th = system_state.pose_th;
  }

  if (id == BEHAVIOR_ID_PHASE_1)
  {
    if (behavior_state == PHASE1_DRIVE_TOWARDS_END)
    {
      // If we see the cliff, stop and transition to next state
      if (system_state.cliff_left > CLIFF_DETECTED ||
          system_state.cliff_right > CLIFF_DETECTED ||
          system_state.cliff_center > CLIFF_DETECTED)
      {
        last_pose_x = system_state.pose_x;  // Cache the X position along table length
        behavior_state = PHASE1_BACK_UP_A_BIT;
        set_motors(0, 0);
      }
      else if (system_state.pose_x > 0.7f * TABLE_LENGTH)
      {
        set_motors(SLOW_SPEED, SLOW_SPEED);
      }
      else
      {
        // Else drive forward at constant speed
        // TODO: use odometry & maintain heading
        set_motors(STANDARD_SPEED, STANDARD_SPEED);
      }
    }
    else if (behavior_state == PHASE1_BACK_UP_A_BIT)
    {
      if ((last_pose_x - system_state.pose_x) > 0.15f)
      {
        // Done backing up - stop robot
        set_motors(0, 0);
        last_pose_th = system_state.pose_th;
        behavior_state = PHASE1_TURN_IN_PLACE;
      }
      else
      {
        // Backup
        set_motors(-SLOW_SPEED, -SLOW_SPEED);
      }
    }
    else if (behavior_state == PHASE1_TURN_IN_PLACE)
    {
      float angle_error = last_pose_th + 3.14f - system_state.pose_th;
      if (abs(angle_error) < 0.025f)
      {
        // Done rotating in place - stop robot
        set_motors(0, 0);
        last_pose_th = system_state.pose_th;
        behavior_state = PHASE1_RETURN_TO_START;
      }
      else
      {
        // Rotate based on error
        int16_t speed = angle_error * 75;
        if (speed < 0) speed = -speed;
        if (speed > SLOW_SPEED) speed = SLOW_SPEED;
        if (speed < MIN_SPEED) speed = MIN_SPEED;
        set_motors(speed, -speed);
      } 
    }
    else if (behavior_state == PHASE1_RETURN_TO_START)
    {
      // If we see the cliff, stop and transition to next state
      if (system_state.cliff_left > CLIFF_DETECTED ||
          system_state.cliff_right > CLIFF_DETECTED ||
          system_state.cliff_center > CLIFF_DETECTED)
      {
        system_state.run_state = MODE_DONE;
        set_motors(0, 0);
      }
      else if (system_state.pose_x < 0.3f * TABLE_LENGTH)
      {
        set_motors(SLOW_SPEED, SLOW_SPEED);
      }
      else
      {
        // TODO: use odometery to follow straight line
        set_motors(STANDARD_SPEED, STANDARD_SPEED);
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
