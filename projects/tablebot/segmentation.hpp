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

#ifndef _TABLEBOT_SEGMENTATION_HPP_
#define _TABLEBOT_SEGMENTATION_HPP_

#include <stdbool.h>

typedef struct
{
  float x;
  float y;
  float z;
} point_t;

typedef struct
{
  point_t start;
  point_t end;
  int points;
} line_segment_t;

// Projects the points from the assembler
// Returns the number of points projected
int project_points(point_t * points, int max_points, bool project_neck,
                   float limit_y, float min_z, float max_z)
{
  float cos_neck = 1.0f, sin_neck = 0.0f;
  if (project_neck)
  {
    // Determine where our neck is - AX12 has 1024 ticks over 300 degrees
    float neck_angle = (512 - system_state.neck_angle) * (300.0f / 1023.0f);
    arm_sin_cos_f32(neck_angle, &sin_neck, &cos_neck);
  }

  // Counter for filling our points
  int points_idx = 0;

  // Project points to table
  for (int i = 0; i < ASSEMBLED_SCAN_SIZE; ++i)
  {
    // Offset our index so we start processing in the back half of the scan
    int laser_idx = (i + (ASSEMBLED_SCAN_SIZE / 2)) % ASSEMBLED_SCAN_SIZE;

    float dist_m = assembler.data[laser_idx] * 0.001f;
    float angle = assembler.angles[laser_idx] * 0.01f;

    if (angle > 360.0f || dist_m < 0.0001f || dist_m > 3.0f)
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

    if (zz < max_z && zz > min_z &&
        y > -limit_y && y < limit_y && x > 0.0f)
    {
      points[points_idx].x = xx;
      points[points_idx].y = y;
      points[points_idx].z = zz;
      ++points_idx;
    }

    if (points_idx >= max_points)
    {
      // Can't process anymore points
      return points_idx;
    }
  }

  return points_idx;
}

// Returns the number of segments extracted
int extract_segments(point_t * points, int num_points,
                     line_segment_t * segments, int max_segments,
                     float jump_distance_squared)
{
  int segment_idx = 0;
  int point_idx = 0;

  // Create a first segment
  segments[0].start = points[0];
  segments[0].end = points[0];
  segments[0].points = 1;

  for (int i = 1; i < num_points; ++i)
  {
    float dx = segments[segment_idx].end.x - points[i].x;
    float dy = segments[segment_idx].end.y - points[i].y;

    float d = dx * dx + dy * dy;
    if (d < jump_distance_squared)
    {
      // Add to current segment
      segments[segment_idx].end = points[i];
      ++segments[segment_idx].points;
    }
    else
    {
      if (++segment_idx >= max_segments)
      {
        // Out of segments to fill
        return max_segments;
      }
      // Create new segment
      segments[segment_idx].start = points[i];
      segments[segment_idx].end = points[i];
      segments[segment_idx].points = 1;
    }
  }

  // Return the number of segments extracted
  return ++segment_idx;
}

#endif  // _TABLEBOT_SEGMENTATION_HPP_
