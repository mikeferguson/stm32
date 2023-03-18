/*
 * Copyright (c) 2012-2014, Michael E. Ferguson
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

#ifndef __STM32_CPP_MD01_HPP__
#define __STM32_CPP_MD01_HPP__

#include <stm32f4xx_tim.h>

/** @brief Driver for MD01 Motor Driver, or half of MD03. */
template <unsigned int TIMx, typename A, typename B, typename EN>
class Md01
{
public:
  /**
   *  @brief Initialize the timer.
   *  @param khz The frequency of timer in KHz.
   *  @param resolution The resolution of the timer.
   */
  void init(uint16_t khz, uint16_t resolution)
  {
    resolution_ = resolution;

    // Setup I/O
    A::mode(GPIO_OUTPUT);
    B::mode(GPIO_OUTPUT);
    EN::mode(GPIO_OUTPUT);
    EN::low();
    
    // Disable timer during init
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CR1 &= (uint16_t) ~TIM_CR1_CEN;

    // Set counter mode to center-aligned which causes the counter to count both
    // up and down, trigger capture compare on both up and down count
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CR1 |= TIM_CR1_CMS;
    
    // Compute best prescalar based on resolution
    //  168e6 is the clock frequency, khz is 10e3
    //  divide by 2 because counter is in center-aligned
    uint16_t prescalar = 168000.0f / khz / resolution_ / 2.0f;
    reinterpret_cast<TIM_TypeDef*>(TIMx)->ARR = resolution_-1;
    reinterpret_cast<TIM_TypeDef*>(TIMx)->PSC = prescalar;

    // Enable Output Compare (OC), set to 0
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCMR1 = TIM_OCMode_PWM1;
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCER =  TIM_CCER_CC1E;
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCR1 = 0;
    // Enable main output (needed for OC config to take effect
    reinterpret_cast<TIM_TypeDef*>(TIMx)->BDTR |= TIM_BDTR_MOE;

    // Enable Timer
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CR1 |= TIM_CR1_CEN;
    EN::high();
  }

  void disable()
  {
    EN::low();
  }

  void set(int16_t pwm)
  {
    EN::high();
    if (pwm >= 0)
    {
      if (pwm >= resolution_)
        pwm = resolution_-1;
      A::low();
      B::high();
      reinterpret_cast<TIM_TypeDef*>(TIMx)->CCR1 = pwm;
    }
    else
    {
      if (pwm <= -resolution_)
        pwm = -resolution_+1;
      A::high();
      B::low();
      reinterpret_cast<TIM_TypeDef*>(TIMx)->CCR1 = -pwm;
    }
    pwm_ = pwm;
  }

  int16_t get()
  {
    return pwm_;
  }

  void brake(int16_t pwm)
  {
    A::low();
    B::low();
    if (pwm >= 0)
    {
      if (pwm >= resolution_)
        pwm = resolution_-1;
      reinterpret_cast<TIM_TypeDef*>(TIMx)->CCR1 = pwm;
    }
    else
    {
      if (pwm <= -resolution_)
        pwm = -resolution_+1;
      reinterpret_cast<TIM_TypeDef*>(TIMx)->CCR1 = -pwm;
    }
    pwm_ = pwm;
  }

private:
  uint16_t resolution_;
  int16_t pwm_;
};

#endif  // __STM32_CPP_MD01_HPP__
