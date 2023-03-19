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
#define BEHAVIOR_ID_PHASE_1       1
// Phase 2 = find the cube, push it off the table
#define BEHAVIOR_ID_PHASE_2       2
// Phase 3 = find the cube, push it into the box
#define BEHAVIOR_ID_PHASE_3       3

// Phase 1 states
#define PHASE1_DRIVE_TOWARDS_END  0
#define PHASE1_BACK_UP_A_BIT      1
#define PHASE1_TURN_IN_PLACE      2
#define PHASE1_RETURN_TO_START    3

// Distance to back up before making in-place 180 degree turn
#define PHASE1_BACKUP_DISTANCE    0.15f

// Phase 2 states
#define PHASE2_SWEEP_NECK         0
#define PHASE2_WAIT_NECK          1
#define PHASE2_ASSEMBLE_SCAN      2
#define PHASE2_ANALYZE_SCAN       3
#define PHASE2_BLOCK_FOUND        4

// TODO: find this value with laser
#define TABLE_LENGTH              1.2192f

uint16_t behavior_id = 0;
uint8_t behavior_state = 0;

#include "segmentation.hpp"

#define MAX_POINTS 300
#define MAX_SEGMENTS 10
point_t line_points[MAX_POINTS];
line_segment_t segments[MAX_SEGMENTS];

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

  static uint32_t latest_scan;
  static uint32_t last_stable_stamp;

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
    system_state.pose_x = 0.15f;  // We aren't right on the end of the table
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
    /******************************************************
     *                                                    *
     *                      PHASE 1                       *
     *                                                    *
     ******************************************************/
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
      else if (system_state.pose_x > 0.8f * TABLE_LENGTH)
      {
        set_motors(SLOW_SPEED, SLOW_SPEED);
      }
      else
      {
        // Else drive forward, keeping robot centered on table
        // Error of 0.05m in Y axis generates ~max_adjustment
        int16_t adjustment = system_state.pose_y * 600;
        int16_t max_adjustment = STANDARD_SPEED * 0.2f;
        if (adjustment > max_adjustment) adjustment = max_adjustment;
        if (adjustment < -max_adjustment) adjustment = -max_adjustment;
        // Moving with axis, so positive error = steer to the right
        set_motors(STANDARD_SPEED + adjustment, STANDARD_SPEED - adjustment);
      }
    }
    else if (behavior_state == PHASE1_BACK_UP_A_BIT)
    {
      if (abs(last_pose_x - system_state.pose_x) > PHASE1_BACKUP_DISTANCE)
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
        // Rotate based on error - slow down as we approach goal angle
        int16_t speed = angle_error * 75;
        if (speed < 0) speed = -speed;
        if (speed > SLOW_SPEED) speed = SLOW_SPEED;
        if (speed < MIN_SPEED) speed = MIN_SPEED;
        // Turn in positive direction (to the left)
        set_motors(-speed, +speed);
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
        // Else drive forward, keeping robot centered on table
        // Error of 0.05m in Y axis generates ~max_adjustment
        int16_t adjustment = system_state.pose_y * 600;
        int16_t max_adjustment = STANDARD_SPEED * 0.2f;
        if (adjustment > max_adjustment) adjustment = max_adjustment;
        if (adjustment < -max_adjustment) adjustment = -max_adjustment;
        // Moving against axis, so positive error = steer to the left
        set_motors(STANDARD_SPEED - adjustment, STANDARD_SPEED + adjustment);
      }
    }
  }
  else if (id == BEHAVIOR_ID_PHASE_2)
  {
    /******************************************************
     *                                                    *
     *                      PHASE 2                       *
     *                                                    *
     ******************************************************/
    if (behavior_state == PHASE2_SWEEP_NECK)
    {
      // Change laser angle as we search for block
      if (system_state.neck_angle > 450 || system_state.neck_angle < 375)
      {
        move_neck(450);
      }
      else
      {
        move_neck(system_state.neck_angle - 5);
      }
      behavior_state = PHASE2_WAIT_NECK;
      last_stable_stamp = 0;
    }
    else if (behavior_state == PHASE2_WAIT_NECK)
    {
      // Read neck angle
      int neck_angle = ax12_get_register(&usart2_parser, &usart2, NECK_SERVO_ID, AX_PRESENT_POSITION_L, 2);
      if ((neck_angle > 0) && (neck_angle - system_state.neck_angle < 5) && (system_state.neck_angle - neck_angle < 5))
      {
        // Has settled - has a new laser scan started?
        if (last_stable_stamp == 0)
        {
          last_stable_stamp = system_state.time;
        }
        else if (system_state.time - last_stable_stamp > 125)
        {
          behavior_state = PHASE2_ASSEMBLE_SCAN;
          latest_scan = assembler.scans_created;
        }
      }
      else
      {
        // Command it again
        move_neck(system_state.neck_angle);
      }
    }
    else if (behavior_state == PHASE2_ASSEMBLE_SCAN)
    {
      // Wait until a new scan is definitely assembled
      if (assembler.scans_created > latest_scan + 2)
      {
        behavior_state = PHASE2_ANALYZE_SCAN;
      }
    }
    else if (behavior_state == PHASE2_ANALYZE_SCAN)
    {
      int points_ct = project_points(line_points, MAX_POINTS, true, 0.5f, -0.1f, 0.2f);

      udp_send_packet((unsigned char *) &line_points, points_ct * 12, return_port);

      // Now process segments
      int segment_ct = extract_segments(line_points, points_ct,
                                        segments, MAX_SEGMENTS, 0.0008f);
      if (segment_ct < 2)
      {
        // No block found
        behavior_state = PHASE2_SWEEP_NECK;
      }
      else
      {
        // we got a possible block
        behavior_state = PHASE2_SWEEP_NECK; //PHASE2_BLOCK_FOUND;
      }
    }
    else if (behavior_state == PHASE2_BLOCK_FOUND)
    {
      // TODO: approach block
    }
  }
  else if (id == BEHAVIOR_ID_PHASE_3)
  {
    /******************************************************
     *                                                    *
     *                      PHASE 3                       *
     *                                                    *
     ******************************************************/
  }
}

#endif  // _TABLEBOT_BEHAVIORS_HPP_
