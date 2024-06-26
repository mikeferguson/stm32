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
  int start_idx;
  int points;
} line_segment_t;

// Projects the points from the assembler
// Returns the number of points projected
int project_points(point_t * points, int max_points, bool project_neck,
                   float min_x, float max_x, float limit_y, float min_z, float max_z)
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
    float dist_m = assembler.data[i] * 0.001f;
    float angle = assembler.angles[i] * 0.01f;

    if (angle > 360.0f || dist_m < 0.0001f || dist_m > 3.0f)
    {
      // Not valid
      continue;
    }

    float cos_angle, sin_angle;
    arm_sin_cos_f32(180.0f - angle, &sin_angle, &cos_angle);

    // Project to XY plane
    float x = cos_angle * dist_m;
    float y = sin_angle * dist_m;

    // Now handle neck rotation
    float xx = cos_neck * x;
    float zz = -sin_neck * x + 0.1778f;  // Neck is 7" off ground

    if (xx < max_x && xx > min_x &&
        zz < max_z && zz > min_z &&
        y > -limit_y && y < limit_y)
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
  segments[0].start_idx = point_idx;
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
      // If we skip this point (as noise) does the segment continue?
      int next = i + 1;
      if (next < num_points)
      {
        dx = segments[segment_idx].end.x - points[next].x;
        dy = segments[segment_idx].end.y - points[next].y;
        d = dx * dx + dy * dy;
        if (d < jump_distance_squared)
        {
          // Add to current segment
          segments[segment_idx].end = points[i];
          ++segments[segment_idx].points;
          continue;
        }
      }

      // Can't make the segment continue
      if (++segment_idx >= max_segments)
      {
        // Out of segments to fill
        return max_segments;
      }
      // Create new segment
      segments[segment_idx].start = points[i];
      segments[segment_idx].end = points[i];
      segments[segment_idx].start_idx = point_idx;
      segments[segment_idx].points = 1;
    }
  }

  // Return the number of segments extracted
  return ++segment_idx;
}

float get_segment_width_sq(line_segment_t * segment)
{
  float dx = segment->end.x - segment->start.x;
  float dy = segment->end.y - segment->start.y;
  return dx * dx + dy * dy;
}

void get_centroid(line_segment_t * segment, point_t * point)
{
  point->x = (segment->end.x + segment->start.x) / 2.0;
  point->y = (segment->end.y + segment->start.y) / 2.0;
  point->z = (segment->end.z + segment->start.z) / 2.0;
}

bool has_consensus(point_t * points, int num_points, float consensus_value)
{
  for (int i = 0; i < num_points; ++i)
  {
    for (int j = i + 1; j < num_points; ++j)
    {
      if (fabs(points[j].x - points[i].x) > consensus_value)
      {
        return false;
      }
      if (fabs(points[j].y - points[i].y) > consensus_value)
      {
        return false;
      }
    }
  }

  return true;
}

#endif  // _TABLEBOT_SEGMENTATION_HPP_
