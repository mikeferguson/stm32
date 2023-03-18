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

// Distance to back up before making in-place 180 degree turn
#define PHASE1_BACKUP_DISTANCE    0.15f

// Phase 2 states
#define PHASE2_SWEEP_LASER        0
#define PHASE2_WAIT_LASER         1
#define PHASE2_ASSEMBLE_SCAN      2
#define PHASE2_ANALYZE_SCAN       3
#define PHASE2_BLOCK_FOUND        4

// TODO: find this value with laser
#define TABLE_LENGTH      1.2192f

uint16_t behavior_id = 0;
uint8_t behavior_state = 0;

int last_laser_start_angle;

typedef struct
{
  float x;
  float y;
  float z;
} point_t;

point_t line_points[100];

typedef struct
{
  point_t start;
  point_t end;
  int points;
} line_segment_t;

point_t block;
line_segment_t segments[10];

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
    if (behavior_state == PHASE2_SWEEP_LASER)
    {
      // Change laser angle as we search for block
      if (system_state.neck_angle > 450 || system_state.neck_angle < 375)
      {
        move_neck(450);
      }
      else
      {
        move_neck(system_state.neck_angle - 10);
      }
      behavior_state = PHASE2_WAIT_LASER;
      last_stable_stamp = 0;
    }
    else if (behavior_state == PHASE2_WAIT_LASER)
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
          // Clear laser data
          for (int i = 0; i < ASSEMBLED_SCAN_SIZE; ++i)
          {
            system_state.laser_data[i] = 0;
            system_state.laser_angle[i] = INVALID_ANGLE;
          }
          last_laser_start_angle = -1;
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
      // Compute in floating point and degrees to reduce discretization issues
      float angle = latest_laser_packet.start_angle * 0.01f;
      float end_angle = latest_laser_packet.end_angle * 0.01f;
      float step = (end_angle - angle) / (MEASUREMENTS_PER_PACKET - 1);
      if (angle > end_angle)
      {
        // This packet wraps around 0
        step = (end_angle + 360.0f - angle) / (MEASUREMENTS_PER_PACKET - 1);
      }

      // Process packet if it is new (this loop can run faster than laser packets appear)
      if (latest_laser_packet.start_angle != last_laser_start_angle)
      {
        // How many of these data points are new?
        int new_data = 0;

        for (int i = 0; i < MEASUREMENTS_PER_PACKET; ++i)
        {
          // Compute index within assembled scan
          int idx = ((angle + 0.75f) / 360.0f) * (ASSEMBLED_SCAN_SIZE - 1);
          if (idx >= ASSEMBLED_SCAN_SIZE) idx -= ASSEMBLED_SCAN_SIZE;
          if (idx < 0) idx += ASSEMBLED_SCAN_SIZE;

          if (system_state.laser_angle[idx] == INVALID_ANGLE)
          {
            // This is a new data point
            ++new_data;
            // Copy range measurement
            system_state.laser_data[idx] = latest_laser_packet.data[i].range;
            // Figure out exact angle
            system_state.laser_angle[idx] = angle / 0.01f;  // Convert back to 0.01 degree steps
            if (system_state.laser_angle[idx] > 36000)
            {
              system_state.laser_angle[idx] -= 36000;
            }
          }
          angle += step;
        }

        // Note we have processed this packet
        last_laser_start_angle = latest_laser_packet.start_angle;

        if (new_data < 3)
        {
          // Have we analyzed the whole scan?
          int empty_data = 0;
          for (int i = 0; i < ASSEMBLED_SCAN_SIZE; ++i)
          {
            if (system_state.laser_angle[i] == INVALID_ANGLE)
            {
              ++empty_data;
            }
          }

          if (empty_data < 20)
          {
            // We have filled in the whole scan
            behavior_state = PHASE2_ANALYZE_SCAN;
          }
        }
      }
    }
    else if (behavior_state == PHASE2_ANALYZE_SCAN)
    {
      // Determine where our neck is - AX12 has 1024 ticks over 300 degrees
      float neck_angle = (512 - system_state.neck_angle) * (300.0f / 1023.0f);

      float cos_neck, sin_neck;
      arm_sin_cos_f32(neck_angle, &sin_neck, &cos_neck);

      // Project points to table
      int line_points_idx = 0;
      for (int i = 300; i < 600; i += 2)
      {
        int laser_idx = i;
        if (laser_idx >= ASSEMBLED_SCAN_SIZE)
        {
          laser_idx -= ASSEMBLED_SCAN_SIZE;
        }

        float dist_m = system_state.laser_data[laser_idx] * 0.001f;
        float angle = system_state.laser_angle[laser_idx] * 0.01f;

        if (angle > 360.0f || dist_m < 0.0001f)
        {
          // Not valid
          continue;
        }

        float cos_angle, sin_angle;
        arm_sin_cos_f32(-angle, &sin_angle, &cos_angle);

        // Project to XY plane
        float x = cos_angle * dist_m;
        float y = sin_angle * dist_m;

        // Now handle neck rotation
        float xx = cos_neck * x;
        float zz = -sin_neck * x + 0.127f;  // Neck is 5" off ground

        if (zz < 0.2f && zz > -0.05f &&
            y > -0.5f && y < 0.5f && x > 0.0f)
        {
          line_points[line_points_idx].x = xx;
          line_points[line_points_idx].y = y;
          line_points[line_points_idx].z = zz;
          ++line_points_idx;
        }
      }

      udp_send_packet((unsigned char *) &line_points, line_points_idx * 12, return_port);

      // Now process segments
      int segment_idx = 0;
      int point_idx = 0;
      segments[0].start = line_points[0];
      segments[0].end = line_points[0];
      segments[0].points = 1;
      for (int i = 1; i < line_points_idx; ++i)
      {
        float dx = segments[segment_idx].end.x - line_points[i].x;
        float dy = segments[segment_idx].end.y - line_points[i].y;

        float d = dx * dx + dy * dy;
        if (d < 0.0008f)
        {
          ++segments[segment_idx].points;
          segments[segment_idx].end = line_points[i];
        }
        else
        {
          if (++segment_idx >= 10)
          {
            // Out of segments
            break;
          }
          segments[segment_idx].start = line_points[i];
          segments[segment_idx].end = line_points[i];
          segments[segment_idx].points = 1;
        }
      }

      if (segment_idx < 2)
      {
        // No block found
        behavior_state = PHASE2_SWEEP_LASER;
      }
      else
      {
        // we got a possible block
        behavior_state = PHASE2_SWEEP_LASER; //PHASE2_BLOCK_FOUND;
      }
    }
    else if (behavior_state == PHASE2_BLOCK_FOUND)
    {
      // TODO: approach block
    }
  }
  else if (id == BEHAVIOR_ID_PHASE_3)
  {

  }
}

#endif  // _TABLEBOT_BEHAVIORS_HPP_
