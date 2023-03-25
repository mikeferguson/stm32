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
// This is a straight forward state machine - we move forward
// through the states and never backwards
#define PHASE1_SETUP_STUFF        0
#define PHASE1_DRIVE_TOWARDS_END  1
#define PHASE1_BACK_UP_A_BIT      2
#define PHASE1_TURN_IN_PLACE      3
#define PHASE1_RETURN_TO_START    4
#define PHASE1_DONE               5

// Distance to back up before making in-place 180 degree turn
#define PHASE1_BACKUP_DISTANCE    0.15f

// Phase 2 states
// This state machine is a bit more complex than Phase 1
//  * We iterate Wait->Assemble->Analyze->Move multiple times until we find the block
//  * Once the block is found we Approach->Push one time and then are finished
#define PHASE2_SETUP_STUFF        0
#define PHASE2_WAIT_STOPPED       1
#define PHASE2_ASSEMBLE_SCAN      2
#define PHASE2_ANALYZE_SCAN       3
#define PHASE2_MOVE_FORWARD       4
#define PHASE2_APPROACH_BLOCK     5
#define PHASE2_PUSH_BLOCK         6

// Phase 3 states
// This is the most complex of the state machines
//  * Wait->Assemble->Analyze is reused for both finding the goal and the block
//  * Selection of goal or block is based on the state of block/goal_pose variables
//  * At the end of the Approach state, we will refind the goal before going into Push
#define PHASE3_SETUP_STUFF        0
#define PHASE3_LOCATE_GOAL        1
#define PHASE3_LOCATE_BLOCK       2
#define PHASE3_WAIT_STOPPED       3
#define PHASE3_ASSEMBLE_SCAN      4
#define PHASE3_ANALYZE_SCAN       5
#define PHASE3_MOVE_FORWARD       6
#define PHASE3_APPROACH_BLOCK     7
#define PHASE3_PUSH_BLOCK         8

// TODO: find this value with laser
#define TABLE_LENGTH              1.2192f

uint16_t behavior_id = 0;

#include "segmentation.hpp"

#define MAX_POINTS 300
point_t line_points[MAX_POINTS];

#define MAX_SEGMENTS 10
line_segment_t segments[MAX_SEGMENTS];

#define MAX_CANDIDATES 10
point_t candidates[MAX_CANDIDATES];

point_t block_pose;
point_t goal_pose;

void set_motors(int16_t left, int16_t right)
{
  m1_pid.update_setpoint(left);
  m2_pid.update_setpoint(right);
  system_state.last_motor_command = system_state.time;
}

// Transfrom from point l (in local) to point g (in global)
void transform_to_global(point_t * g, point_t * l)
{
  float cos_angle, sin_angle;
  arm_sin_cos_f32(system_state.pose_th, &sin_angle, &cos_angle);

  g->x = system_state.pose_x + (cos_angle * l->x - sin_angle * l->y);
  g->y = system_state.pose_y + (sin_angle * l->x + cos_angle * l->y);
  g->z = l->z;
}

// Returns angle in degrees
float simple_atan2(float x, float y)
{
  if (fabs(y) < 0.001f)
  {
    return 0.0f;
  }

  // https://math.stackexchange.com/questions/1098487/atan2-faster-approximation
  float z = y / x;
  if (z < 0.0f)
  {
    z *= -1.0f;
    return -(z * (45.0f - (z - 1.0f) * (14.0f + 3.83f * z)));
  }

  return z * (45.0f - (z - 1.0f) * (14.0f + 3.83f * z));
}

// Move forward unless there is a cliff or we have surpassed limit distance
// Returns distance traveled since last_pose_x or negative if there is a cliff
float move_forward_slowly(float last_pose_x, float limit)
{
  float dist = abs(system_state.pose_x - last_pose_x);

  if (system_state.cliff_left > CLIFF_DETECTED ||
      system_state.cliff_right > CLIFF_DETECTED ||
      system_state.cliff_center > CLIFF_DETECTED)
  {
    set_motors(0, 0);
    return -1.0f;
  }
  else if (dist > limit)
  {
    // Moved a bit, stop the robot
    set_motors(0, 0);
  }
  else
  {
    set_motors(SLOW_SPEED, SLOW_SPEED);
  }
  return dist;
}

// Compute the heading and distance to target
// Values are stored in system_state
void compute_heading_and_dist(point_t * target)
{
  float dx = target->x - system_state.pose_x;
  float dy = target->y - system_state.pose_y;

  system_state.target_dist = dx * dx + dy * dy;

  // Note: yaw is in degrees
  system_state.target_yaw = simple_atan2(dx, dy) - system_state.pose_th;
}

// Turn towards a target
// Returns angular error
float turn_to_target(point_t * target)
{
  compute_heading_and_dist(target);

  if (system_state.target_yaw > 5.0f)
  {
    // In place turn to point at the target
    set_motors(-MIN_SPEED, MIN_SPEED);
  }
  else if (system_state.target_yaw < -5.0f)
  {
    // In place turn to point at the target
    set_motors(MIN_SPEED, -MIN_SPEED);
  }
  else
  {
    set_motors(0, 0);
  }

  return system_state.target_yaw;
}

// Move towards target pose
// Returns distance to target
float approach_target(point_t * target)
{
  compute_heading_and_dist(target);

  // Error of 5 degrees generates ~max_adjustment
  int16_t max_adjustment = SLOW_SPEED * 0.2f;
  int16_t adjustment = (system_state.target_yaw / 5.0f) * max_adjustment;
  if (adjustment > max_adjustment) adjustment = max_adjustment;
  if (adjustment < -max_adjustment) adjustment = -max_adjustment;
  // Positive angular error = steer to the left
  set_motors(SLOW_SPEED - adjustment, SLOW_SPEED + adjustment);

  // Return distance to target
  return system_state.target_dist;
}

// Verify that neck has reached the goal
// Returns > 0 if true
int verify_neck(uint32_t * last_stable_stamp, uint32_t stop_time = 250)
{
  int neck_angle = ax12_get_register(&usart2_parser, &usart2, NECK_SERVO_ID, AX_PRESENT_POSITION_L, 2);
  if ((neck_angle > 0) && (neck_angle - system_state.neck_angle < 5) && (system_state.neck_angle - neck_angle < 5))
  {
    // Has settled - has a new laser scan started?
    if (*last_stable_stamp == 0)
    {
      *last_stable_stamp = system_state.time;
      return 0;
    }
    else if (system_state.time - *last_stable_stamp > stop_time)
    {
      // We've been stopped long enough
      return 1;
    }
  }
  return 0;
}

void run_behavior(uint16_t id, uint32_t stamp)
{
  static float last_pose_x;
  static float last_pose_y;
  static float last_pose_th;

  static uint32_t latest_scan;
  static uint32_t last_stable_stamp;

  static uint32_t max_speed;

  if (id == MODE_UNSELECTED)
  {
    // Phase is not yet selected
    return;
  }

  if (id != behavior_id)
  {
    // This is our first iteration through - reset things
    behavior_id = id;
    system_state.behavior_state = 0;
    last_stable_stamp = 0;

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
    if (system_state.behavior_state == PHASE1_SETUP_STUFF)
    {
      // Angle neck downwards
      system_state.neck_angle = 425;

      // Is neck there yet?
      if (verify_neck(&last_stable_stamp) > 0)
      {
        latest_scan = assembler.scans_created + 1;
        max_speed = STANDARD_SPEED;
        system_state.behavior_state = PHASE1_DRIVE_TOWARDS_END;
      }
      else
      {
         move_neck(425);
      }
    }
    else if (system_state.behavior_state == PHASE1_DRIVE_TOWARDS_END ||
             system_state.behavior_state == PHASE1_RETURN_TO_START)
    {
      // If we see the cliff, stop and transition to next state
      if (system_state.cliff_left > CLIFF_DETECTED ||
          system_state.cliff_right > CLIFF_DETECTED ||
          system_state.cliff_center > CLIFF_DETECTED)
      {
        last_pose_x = system_state.pose_x;  // Cache the X position along table length
        ++system_state.behavior_state;
        set_motors(0, 0);
      }
      else
      {
        // Cliff sensors haven't triggered - can laser still see the table?
        if (max_speed > SLOW_SPEED && assembler.scans_created > latest_scan)
        {
          // New laser scan to inspect - take only points right in front of us
          int points_ct = project_points(line_points, MAX_POINTS, true, 0.1f, -0.1f, 0.2f);

          // Send those points for debugging
          udp_send_packet((unsigned char *) &line_points, points_ct * 12, return_port, PACKET_PROJECTED_POINTS);

          if (points_ct < 5)
          {
            // We've lost the table - SLOW DOWN!
            max_speed = SLOW_SPEED;
          }

          // Note that scan has been processed
          latest_scan = assembler.scans_created;
        }

        // Drive forward, keeping robot centered on table
        // Error of 0.05m in Y axis generates ~max_adjustment
        int16_t adjustment = system_state.pose_y * 600;
        int16_t max_adjustment = max_speed * 0.2f;
        if (adjustment > max_adjustment) adjustment = max_adjustment;
        if (adjustment < -max_adjustment) adjustment = -max_adjustment;
        if (system_state.behavior_state == PHASE1_DRIVE_TOWARDS_END)
        {
          // Moving with axis, so positive error = steer to the right
          set_motors(max_speed + adjustment, max_speed - adjustment);
        }
        else
        {
          // Moving against axis, so positive error = steer to the left
          set_motors(max_speed - adjustment, max_speed + adjustment);
        }
      }
    }
    else if (system_state.behavior_state == PHASE1_BACK_UP_A_BIT)
    {
      if (abs(last_pose_x - system_state.pose_x) > PHASE1_BACKUP_DISTANCE)
      {
        // Done backing up - stop robot
        set_motors(0, 0);
        last_pose_th = system_state.pose_th;
        system_state.behavior_state = PHASE1_TURN_IN_PLACE;
      }
      else
      {
        // Backup
        set_motors(-SLOW_SPEED, -SLOW_SPEED);
      }
    }
    else if (system_state.behavior_state == PHASE1_TURN_IN_PLACE)
    {
      float angle_error = last_pose_th + 3.14f - system_state.pose_th;
      if (abs(angle_error) < 0.025f)
      {
        // Done rotating in place - stop robot
        set_motors(0, 0);
        max_speed = STANDARD_SPEED;
        latest_scan = assembler.scans_created + 1;
        system_state.behavior_state = PHASE1_RETURN_TO_START;
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
    else if (system_state.behavior_state == PHASE1_DONE)
    {
      system_state.run_state = MODE_DONE;
    }
  }
  else if (id == BEHAVIOR_ID_PHASE_2)
  {
    /******************************************************
     *                                                    *
     *                      PHASE 2                       *
     *                                                    *
     ******************************************************/
    if (system_state.behavior_state == PHASE2_SETUP_STUFF)
    {
      // Angle neck downwards, stop robot
      move_neck(425);
      set_motors(0, 0);

      // Now wait for neck to stabilize
      last_stable_stamp = 0;
      system_state.behavior_state = PHASE2_WAIT_STOPPED;
    }
    else if (system_state.behavior_state == PHASE2_WAIT_STOPPED)
    {
      // Read neck angle
      if (verify_neck(&last_stable_stamp) > 0)
      {
        latest_scan = assembler.scans_created;
        system_state.behavior_state = PHASE2_ASSEMBLE_SCAN;
      }
      else
      {
        // Command it again
        move_neck(system_state.neck_angle);
        set_motors(0, 0);
      }
    }
    else if (system_state.behavior_state == PHASE2_ASSEMBLE_SCAN)
    {
      // Wait until a new scan is definitely assembled
      if (assembler.scans_created > latest_scan + 2)
      {
        system_state.behavior_state = PHASE2_ANALYZE_SCAN;
      }
    }
    else if (system_state.behavior_state == PHASE2_ANALYZE_SCAN)
    {
      // Find all points on top of the table, less than 0.5m to either side of the robot
      int points_ct = project_points(line_points, MAX_POINTS, true, 0.5f, 0.0f, 0.2f);

      // Send those points for debugging
      udp_send_packet((unsigned char *) &line_points, points_ct * 12, return_port, PACKET_PROJECTED_POINTS);

      // Now process segments
      int segment_ct = extract_segments(line_points, points_ct,
                                        segments, MAX_SEGMENTS, 0.0008f);

      // Process segments
      int num_candidates = 0;
      for (int s = 0; s < segment_ct; ++s)
      {
        double width_sq = get_segment_width_sq(&segments[s]);
        // Block is 60mm wide - add margin and square it
        if (width_sq < 0.008f && segments[s].points > 3)
        {
          // Candidate
          get_centroid(&segments[s], &candidates[num_candidates]);
          if (++num_candidates >= MAX_CANDIDATES)
          {
            break;
          }
        }
      }

      if (segment_ct == 0 || num_candidates == 0)
      {
        // No segments to select from - move forward
        last_pose_x = system_state.pose_x;
        system_state.behavior_state = PHASE2_MOVE_FORWARD;
      }
      else
      {
        // Transform block to global coordinates
        transform_to_global(&block_pose, &candidates[0]);
        system_state.block_pose_x = block_pose.x;
        system_state.block_pose_y = block_pose.y;
        system_state.block_pose_z = block_pose.z;

        // Now go push it
        if (fabs(turn_to_target(&block_pose)) <= 5.0f)
        {
          system_state.behavior_state = PHASE2_APPROACH_BLOCK;
        }
        else
        {
          // Assemble another scan
          system_state.behavior_state = PHASE2_ASSEMBLE_SCAN;
        }
      }
    }
    else if (system_state.behavior_state == PHASE2_MOVE_FORWARD)
    {
      float dist = move_forward_slowly(last_pose_x, 0.075f);
      if (dist < 0.0f)
      {
        // ERROR - We shouldn't get here
        system_state.run_state = MODE_DONE;
      }
      else if (dist > 0.075f)
      {
        last_stable_stamp = 0;
        system_state.behavior_state = PHASE2_WAIT_STOPPED;
      }
    }
    else if (system_state.behavior_state == PHASE2_APPROACH_BLOCK ||
             system_state.behavior_state == PHASE2_PUSH_BLOCK)
    {
      // Approach block
      if (system_state.cliff_left > CLIFF_DETECTED ||
          system_state.cliff_right > CLIFF_DETECTED ||
          system_state.cliff_center > CLIFF_DETECTED)
      {
        if (system_state.cliff_right > CLIFF_DETECTED &&
            system_state.cliff_left > CLIFF_DETECTED)
        {
          // Definitely stop
          system_state.run_state = MODE_DONE;
          set_motors(0, 0);
        }
        else if (system_state.cliff_right > CLIFF_DETECTED)
        {
          // Rotate around right wheel
          set_motors(MIN_SPEED, 0);
        }
        else if (system_state.cliff_left > CLIFF_DETECTED)
        {
          // Rotate around left wheel
          set_motors(0, MIN_SPEED);
        }
        else
        {
          // Block is blocking our center cliff sensor...
          set_motors(MIN_SPEED, MIN_SPEED);
        }
      }
      else if (system_state.behavior_state == PHASE2_PUSH_BLOCK)
      {
        // Just slow forward movement
        set_motors(SLOW_SPEED, SLOW_SPEED);
      }
      else
      {
        // Approach block
        if (approach_target(&block_pose) < 0.01f)
        {
          system_state.behavior_state = PHASE2_PUSH_BLOCK;
        }
      }
    }
  }
  else if (id == BEHAVIOR_ID_PHASE_3)
  {
    /******************************************************
     *                                                    *
     *                      PHASE 3                       *
     *                                                    *
     ******************************************************/
    if (system_state.behavior_state == PHASE3_SETUP_STUFF)
    {
      // Set neck horizontal, stop robot
      move_neck(512);
      set_motors(0, 0);

      // Clear everything
      block_pose.x = 0.0f;
      block_pose.y = 0.0f;
      block_pose.z = -1.0f;

      goal_pose.x = 0.0f;
      goal_pose.y = 0.0f;
      goal_pose.z = -1.0f;

      // Now wait for neck to stabilize
      last_stable_stamp = 0;
      system_state.behavior_state = PHASE3_WAIT_STOPPED;
    }
    else if (system_state.behavior_state == PHASE3_LOCATE_GOAL)
    {
      // Set neck horizontal, stop robot
      move_neck(512);
      set_motors(0, 0);

      // Now wait for neck to stabilize
      last_stable_stamp = 0;
      system_state.behavior_state = PHASE3_WAIT_STOPPED;
    }
    else if (system_state.behavior_state == PHASE3_LOCATE_BLOCK)
    {
      // Set neck downward, stop robot
      move_neck(425);
      set_motors(0, 0);

      // Now wait for neck to stabilize
      last_stable_stamp = 0;
      system_state.behavior_state = PHASE3_WAIT_STOPPED;
    }
    else if (system_state.behavior_state == PHASE3_WAIT_STOPPED)
    {
      // Read neck angle
      if (verify_neck(&last_stable_stamp) > 0)
      {
        latest_scan = assembler.scans_created;
        system_state.behavior_state = PHASE3_ASSEMBLE_SCAN;
      }
      else
      {
        // Command it again
        move_neck(system_state.neck_angle);
        set_motors(0, 0);
      }
    }
    else if (system_state.behavior_state == PHASE3_ASSEMBLE_SCAN)
    {
      // Wait until a new scan is definitely assembled
      if (assembler.scans_created > latest_scan + 2)
      {
        system_state.behavior_state = PHASE3_ANALYZE_SCAN;
      }
    }
    else if (system_state.behavior_state == PHASE3_ANALYZE_SCAN)
    {
      // Find all points on top of the table, less than 0.5m to either side of the robot
      int points_ct = project_points(line_points, MAX_POINTS, true, 0.5f, 0.0f, 0.2f);

      // Send those points for debugging
      udp_send_packet((unsigned char *) &line_points, points_ct * 12, return_port, PACKET_PROJECTED_POINTS);

      // Now process segments
      int segment_ct = extract_segments(line_points, points_ct,
                                        segments, MAX_SEGMENTS, 0.0008f);

      int num_candidates = 0;
      int selected_segment = 0;
      if (goal_pose.z < 0.0f)
      {
        // We are looking for the goal
        for (int s = 0; s < segment_ct; ++s)
        {
          double width_sq = get_segment_width_sq(&segments[s]);
          // Block is 60mm wide - add margin and square it
          if (width_sq > 0.0625f && segments[s].points > 3)
          {
            // Candidate
            selected_segment = s;
            get_centroid(&segments[s], &candidates[num_candidates]);
            if (++num_candidates >= MAX_CANDIDATES)
            {
              break;
            }
          }
        }

        if (num_candidates == 0)
        {
          // No segments to select from - retry assembling scan
          last_pose_x = system_state.pose_x;
          system_state.behavior_state = PHASE3_ASSEMBLE_SCAN;
        }
        else
        {
          // Send those points for debugging
          line_segment_t & s = segments[selected_segment];
          udp_send_packet((unsigned char *) &line_points[s.start_idx], s.points * 12, return_port, PACKET_SEGMENT_POINTS);

          // Transform goal to global coordinates
          transform_to_global(&goal_pose, &candidates[0]);

          if (block_pose.z < 0.0f)
          {
            // Now find block
            system_state.behavior_state = PHASE3_LOCATE_BLOCK;
          }
          else
          {
            // We already have the block - turn torwards the goal
            if (fabs(turn_to_target(&goal_pose)) <= 5.0f)
            {
              system_state.behavior_state = PHASE3_PUSH_BLOCK;
            }
            else
            {
              // Assemble another scan
              goal_pose.z = -1.0f;
              system_state.behavior_state = PHASE3_ASSEMBLE_SCAN;
            }
          }
        }
      }
      else
      {
        // We are looking for the block
        for (int s = 0; s < segment_ct; ++s)
        {
          double width_sq = get_segment_width_sq(&segments[s]);
          // Block is 60mm wide - add margin and square it
          if (width_sq < 0.008f && segments[s].points > 3)
          {
            // Candidate
            get_centroid(&segments[s], &candidates[num_candidates]);
            if (++num_candidates >= MAX_CANDIDATES)
            {
              break;
            }
          }
        }

        if (num_candidates == 0)
        {
          // No segments to select from - move forward
          last_pose_x = system_state.pose_x;
          system_state.behavior_state = PHASE3_MOVE_FORWARD;
        }
        else
        {
          // Transform block to global coordinates
          transform_to_global(&block_pose, &candidates[0]);
          system_state.block_pose_x = block_pose.x;
          system_state.block_pose_y = block_pose.y;
          system_state.block_pose_z = block_pose.z;

          // Now go push it
          if (fabs(turn_to_target(&block_pose)) <= 5.0f)
          {
            system_state.behavior_state = PHASE3_APPROACH_BLOCK;
          }
          else
          {
            // Assemble another scan
            system_state.behavior_state = PHASE3_ASSEMBLE_SCAN;
          }
        }
      }
    }
    else if (system_state.behavior_state == PHASE3_MOVE_FORWARD)
    {
      float dist = move_forward_slowly(last_pose_x, 0.075f);
      if (dist < 0.0f)
      {
        // ERROR - We shouldn't get here
        system_state.run_state = MODE_DONE;
      }
      else if (dist > 0.075f)
      {
        // Moved a bit, stop the robot
        last_stable_stamp = 0;
        system_state.behavior_state = PHASE3_WAIT_STOPPED;
      }
    }
    else if (system_state.behavior_state == PHASE3_APPROACH_BLOCK)
    {
      // Approach block
      if (system_state.cliff_left > CLIFF_DETECTED ||
          system_state.cliff_right > CLIFF_DETECTED)
      {
        // We shouldn't really get here
        set_motors(0, 0);
      }
      else
      {
        // Approach block
        if (approach_target(&block_pose) < 0.005f)
        {
          // Stop and find the goal again
          set_motors(0, 0);
          goal_pose.z = -1.0f;
          system_state.behavior_state = PHASE3_LOCATE_GOAL;
        }
      }
    }
    else if (system_state.behavior_state == PHASE3_PUSH_BLOCK)
    {
      // Approach block
      if (system_state.cliff_left > CLIFF_DETECTED ||
          system_state.cliff_right > CLIFF_DETECTED ||
          system_state.cliff_center > CLIFF_DETECTED)
      {
        // We shouldn't really get here
        set_motors(0, 0);
      }
      else
      {
        // Approach block
        if (approach_target(&goal_pose) < 0.01f)
        {
          set_motors(0, 0);
          system_state.run_state = MODE_DONE;
        }
      }
    }
  }
}

#endif  // _TABLEBOT_BEHAVIORS_HPP_
