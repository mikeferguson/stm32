/*
 * Copyright (c) 2020, Michael E. Ferguson
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
 *  * Neither the name of Vanadium Labs LLC nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

#ifndef __ETHERBOTIX_MOTOR_TRACE_H__
#define __ETHERBOTIX_MOTOR_TRACE_H__

template <typename T>
void trace_copy(uint8_t * a, T * b, size_t& i)
{
  uint8_t * bb = reinterpret_cast<uint8_t*>(b);
  a[0] = bb[0];
  a[1] = bb[1];
  a[2] = bb[2];
  a[3] = bb[3];
  i += 4;
}

template <size_t MOTOR_TRACE_SIZE>
class MotorTrace
{
public:
  MotorTrace()
  {
    idx_ = 0;
    latch_ = 0;
    is_tracing_ = true;
  }

  void update(int32_t position,
              int16_t velocity,
              int32_t setpoint,
              int32_t command)
  {
    if (is_tracing_)
    {
      position_[idx_] = position;
      velocity_[idx_] = velocity;
      setpoint_[idx_] = setpoint;
      command_[idx_] = command;
      idx_ = (idx_ + 1) % MOTOR_TRACE_SIZE;
    }
  }

  size_t get(uint8_t * buffer, size_t max_bytes)
  {
    if (tracing())
    {
      // Cannot read out trace while actively tracing
      return 0;
    }

    size_t i = 0;
    do
    {
      trace_copy(&buffer[i], &position_[latch_], i);
      trace_copy(&buffer[i], &velocity_[latch_], i);
      trace_copy(&buffer[i], &setpoint_[latch_], i);
      trace_copy(&buffer[i], &command_[latch_], i);
      latch_ = (latch_ + 1) % MOTOR_TRACE_SIZE;

      if (i >= max_bytes)
      {
        // Packet is maxed out
        break;
      }
    }
    while (latch_ != idx_);

    if (latch_ == idx_)
    {
      // We've read out entire trace, restart tracing
      start_tracing();
    }

    // Return number of bytes read
    return i;
  }


  bool tracing()
  {
    return is_tracing_;
  }

  void start_tracing()
  {
    is_tracing_ = true;
  }

  void stop_tracing()
  {
    if (is_tracing_)
    {
      is_tracing_ = false;
      latch_ = idx_;
    }
  }

private:
  int32_t position_[MOTOR_TRACE_SIZE];
  int32_t velocity_[MOTOR_TRACE_SIZE];
  int32_t setpoint_[MOTOR_TRACE_SIZE];
  int32_t command_[MOTOR_TRACE_SIZE];

  uint32_t idx_;
  uint32_t latch_;
  bool is_tracing_;
};

#endif  // __ETHERBOTIX_MOTOR_TRACE_H__
