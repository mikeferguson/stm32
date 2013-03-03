/*
 * Copyright (c) 2012-2013, Michael E. Ferguson
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

/* 
 * stm32_cpp: a C++ stm32 library
 * Driver for the A4940 motor driver from Allegro.
 *
 * Usage:
 *
 * a4940<TIM1_BASE, 1, 2> motor;
 * ...
 * RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
 * tim1_ch1::mode(GPIO_ALTERNATE | GPIO_AF_TIM1);
 * tim1_ch1n::mode(GPIO_ALTERNATE | GPIO_AF_TIM1);
 * tim1_ch2::mode(GPIO_ALTERNATE | GPIO_AF_TIM1);
 * tim1_ch2n::mode(GPIO_ALTERNATE | GPIO_AF_TIM1);
 * motor.init(period, prescalar);
 * ...
 * motor.set(1.0);
 *
 */

#ifndef _STM32_CPP_A4940_H_
#define	_STM32_CPP_A4940_H_

/* TODO: add fault pin to template */
/* TODO: easy way to do prescalar? */

/* TIMx should be 1 or 8 */
template<unsigned int TIMx, unsigned int OC1, unsigned int OC2>
class A4940
{
  uint16_t period_;

public:
  void init(const uint16_t period, const uint16_t prescalar)
  {
    period_ = period;

    /* disable timer during init */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CR1 &= (uint16_t) ~TIM_CR1_CEN;

    /* set counter mode to basic up counter */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CR1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS));

    /* set period and prescaler */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->ARR = period_-1;
    reinterpret_cast<TIM_TypeDef*>(TIMx)->PSC = prescalar;

    /* set output compare modes */
    if(OC1 < 3){
        reinterpret_cast<TIM_TypeDef*>(TIMx)->CCMR1 |= (uint16_t) (0x68) << (8*(OC1-1)); // pwm mode 1
    }else{
        reinterpret_cast<TIM_TypeDef*>(TIMx)->CCMR2 |= (uint16_t) (0x68) << (8*(OC1-3));
    }

    if(OC2 < 3){
        reinterpret_cast<TIM_TypeDef*>(TIMx)->CCMR1 |= (uint16_t) (0x68) << (8*(OC2-1));
    }else{
        reinterpret_cast<TIM_TypeDef*>(TIMx)->CCMR2 |= (uint16_t) (0x68) << (8*(OC2-3));
    }

    /* enable output compares */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCER &= (uint16_t) ~(TIM_CCER_CC1P<<(4*(OC1-1))); // CHx active high
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCER &= (uint16_t) ~(TIM_CCER_CC1NP<<(4*(OC1-1)));// CHxN active high
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCER |= 0x05<<(4*(OC1-1));                        // enable CHx/CHxN

    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCER &= (uint16_t) ~(TIM_CCER_CC1P<<(4*(OC2-1)));
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCER &= (uint16_t) ~(TIM_CCER_CC1NP<<(4*(OC2-1)));
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCER |= 0x05<<(4*(OC2-1));

    /* generate an update event to reload the Prescaler */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->EGR = 1;

    /* TODO: enable CC4 for current sense? */

    /* Stop TIM1 if we get halted by debugger. */
    //DBGMCU->CR |= DBGMCU_CR_DBG_TIM1_STOP;

    /* enable pwm outputs */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->BDTR |= TIM_BDTR_MOE;

    /* enable, with auto reload */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;

  }

  void invert_outputs(void)
  {
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCER |= TIM_CCER_CC1P<<(4*(OC1-1));               // CHxN active low
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCER |= TIM_CCER_CC1NP<<(4*(OC1-1));              // CHxN active low
    
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCER |= TIM_CCER_CC1P<<(4*(OC2-1));
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCER |= TIM_CCER_CC1NP<<(4*(OC2-1));
  }

  /* Set PWM. Duty should be -1.0 to 1.0 */
  void set(const float duty)
  {
    uint16_t * ccr = (uint16_t*) &(reinterpret_cast<TIM_TypeDef*>(TIMx)->CCR1);
    float pwm = duty * period_;

    if(pwm > 0)
    {
      *(ccr+(2 * (OC2-1))) = 0; // turn blo on
      *(ccr+(2 * (OC1-1))) = pwm;
    }
    else
    {
      *(ccr+(2 * (OC1-1))) = 0; // turn alo on
      *(ccr+(2 * (OC2-1))) = -pwm;
    }
  }

};

#endif	// _STM32_CPP_A4940_H_
