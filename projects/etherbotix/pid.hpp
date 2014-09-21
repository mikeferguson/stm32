/*
 * Copyright (c) 2012-2014, Michael E. Ferguson
 * Copyright (c) 2009-2011, Vanadium Labs LLC.
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

#ifndef __ETHERBOTIX_PID_HPP__
#define __ETHERBOTIX_PID_HPP__

#include "etherbotix.hpp"

/** @brief Simple fixed-dt PID controller with setpoint interpolation */
class Pid
{
public:
  Pid() :
    max_step_(10)
  {
    reset();
  }

  /** @brief Set gains. */
  void set_gains(float kp, float kd, float ki, int16_t windup)
  {
    kp_ = kp;
    kd_ = kd;
    ki_ = ki;
    windup_ = windup;
    reset();
  }

  /** @brief Set the max step response in a given update */
  void set_max_step(int16_t max_step)
  {
    max_step_ = max_step;
  }

  /** @brief Set new setpoint. */
  void update_setpoint(int16_t setpoint)
  {
    desired_ = setpoint;
  }

  /** @brief Get command based on measurement. */
  int32_t update_pid(int32_t measurement)
  {
    int32_t p_term, d_term, i_term;

    // Interpolate
    if (desired_ > setpoint_)
    {
      setpoint_ += max_step_;
      if (setpoint_ > desired_)
      {
        setpoint_ = desired_;
      }
    }
    else
    {
      setpoint_ -= max_step_;
      if (setpoint_ < desired_)
      {
        setpoint_ = desired_;
      }
    }      

    // Error term
    p_error_ = setpoint_ - measurement;

    // Proportional term
    p_term = kp_ * p_error_;

    // Integral Term
    i_term = ki_ * p_error_ + i_error_;
    if (i_term > windup_)
    {
      i_term = windup_;
    }
    else if (i_term < -windup_)
    {
      i_term = -windup_;
    }
    else
    {
      i_error_ += ki_ * p_error_;
    }

    // Derivative Term
    d_term = kd_ * (p_error_ - p_error_last_);
    p_error_last_ = p_error_;

    // Stop PWM when stopped
    if (setpoint_ == 0 && measurement < max_step_/2 && measurement > -max_step_/2)
    {
      command_ = 0;
    }
    else
    {
      command_ = p_term + i_term + d_term;
    }
    return command_;
  }

  void reset()
  {
    desired_ = 0;
    setpoint_ = 0;
    p_error_last_ = 0;
    p_error_ = 0;
    i_error_ = 0;
  }

private:
  int16_t max_step_;
  int16_t desired_;

  int16_t setpoint_;
  int32_t command_;  // For debugging

  int32_t p_error_;
  int32_t p_error_last_;
  int16_t i_error_;

  float kp_, kd_, ki_;
  float windup_;
};

Pid m1_pid, m2_pid;

#endif  // __ETHERBOTIX_PID_HPP__
