/*
 * Copyright (c) 2012, Michael E. Ferguson
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

#ifndef _STM32_CPP_ENCODER_H
#define	_STM32_CPP_ENCODER_H

/**
 *  \brief Interface a quadrature encoder using TIM2-5.
 *  \tparam TIMx the timer base to use, for example TIM2_BASE.
 */
template<unsigned int TIMx>
class Encoder
{
  /* total value since startup */
  int32_t position;
  /* last read of timer */
  uint16_t last_timer;
  /* last difference between timer reads */
  int16_t last_timer_diff;

public:
  /** \brief Initialize the timer. */
  void init(void)
  {
    position = 0;
    last_timer = 0;
    last_timer_diff = 0;

    /* set automatic reload to max */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->ARR = 65535;

    /* setup encoder interface to trigger on up/down both inputs (full resolution) */
    uint16_t tmp = reinterpret_cast<TIM_TypeDef*>(TIMx)->SMCR;
    tmp &= (uint16_t) ~TIM_SMCR_SMS;
    tmp |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    reinterpret_cast<TIM_TypeDef*>(TIMx)->SMCR = tmp;

    /* set TI1, TI2 as capture compare 1 and 2 */
    tmp = reinterpret_cast<TIM_TypeDef*>(TIMx)->CCMR1;
    tmp &= ((uint16_t)~TIM_CCMR1_CC1S) & ((uint16_t)~TIM_CCMR1_CC2S);
    tmp |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCMR1 = tmp;

    /* set the TI1 and the TI2 polarities */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCER &= ((uint16_t)~TIM_CCER_CC1P) & ((uint16_t)~TIM_CCER_CC2P);

    /* enable timer */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CR1 |= TIM_CR1_CEN;
  }

  /**
   *  \brief Read the encoder count.
   *  \returns The current encoder count.
   */
  int32_t read(void)
  {
    uint16_t timer_value = reinterpret_cast<TIM_TypeDef*>(TIMx)->CNT;
    last_timer_diff = timer_value - last_timer;
    last_timer = timer_value;
    position += (int32_t) last_timer_diff;
    return position;
  }

  /**
   *  \brief Get the encoder speed. This assumes you call read() at a regular rate.
   *  \returns The change between the last two read() calls.
   */
  int16_t read_speed(void)
  {
    return last_timer_diff;
  }

  /**
   *  \brief Reset the internal count to a new value.
   *  \param value The value to reset the count to.
   */
  void write(uint32_t value)
  {
    /* disable timer */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CR1 &= ~TIM_CR1_CEN;
    /* set new value */
    position = value;
    last_timer = 0;
    last_timer_diff = 0;
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CNT = 0;
    /* enable timer */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CR1 |= TIM_CR1_CEN;
  }

};

#endif	// _STM32_CPP_ENCODER_H
