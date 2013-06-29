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
 * Templated injected sampler.
 *
 * Usage:
 *
 *  #define VOLTAGE_SENSE   0   // adc123_in0
 *
 *  // set it up
 *  adc1.init(0);
 *  adc1.setSampleTime(VOLTAGE_SENSE, ADC_SampleTime_15Cycles);
 *
 *  // force a conversion (if trigger isn't actually setup)
 *  adc1.convert();
 *
 *  // some time later
 *  float voltage = (adc1.get_channel1()/4096.0f) * 3.3f;
 *
 */

#ifndef _STM32_CPP_ANALOG_SAMPLER_H_
#define	_STM32_CPP_ANALOG_SAMPLER_H_

#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"

/** \brief This class makes it easy to do injected analog sampling. */
template<unsigned int ADCx>
class AnalogSampler
{
public:
  /** \brief Setup the sampler with a set of channels to sample.
   *  \param trigger The trigger source to use. 
   */  
  void init(int32_t ch1, int32_t ch2 = -1, int32_t ch3 = -1, int32_t ch4 = -1,
            uint32_t trigger = ADC_ExternalTrigInjecConv_T1_TRGO)
  {
    this->user_callback = 0;

    /* PCLK2 = 84MHZ, ADC_CLK has max of 36MHz, divide by 4 */
    ADC->CCR |= ADC_CCR_ADCPRE_0;
    /* Default all channels to 15 cycles */
    reinterpret_cast<ADC_TypeDef*>(ADCx)->SMPR1 = (ADC_SampleTime_15Cycles << 0)
                                                | (ADC_SampleTime_15Cycles << 3)
                                                | (ADC_SampleTime_15Cycles << 6)
                                                | (ADC_SampleTime_15Cycles << 9)
                                                | (ADC_SampleTime_15Cycles << 12)
                                                | (ADC_SampleTime_15Cycles << 15)
                                                | (ADC_SampleTime_15Cycles << 18)
                                                | (ADC_SampleTime_15Cycles << 21)
                                                | (ADC_SampleTime_15Cycles << 24);
    reinterpret_cast<ADC_TypeDef*>(ADCx)->SMPR2 = reinterpret_cast<ADC_TypeDef*>(ADCx)->SMPR1;

    if (ch4 >= 0)
    {
      /* Set up 4-channel injected sequence. */
      reinterpret_cast<ADC_TypeDef*>(ADCx)->JSQR = (4-1)<< 20
                                                 |  ch4 << 15
                                                 |  ch3 << 10
                                                 |  ch2 << 5
                                                 |  ch1;
    }
    else if (ch3 >= 0)
    {
      /* Set up 3-channel injected sequence. */
      reinterpret_cast<ADC_TypeDef*>(ADCx)->JSQR = (3-1)<< 20
                                                 |  ch3 << 15
                                                 |  ch2 << 10
                                                 |  ch1 << 5;
    }
    else if (ch2 >= 0)
    {
      /* Set up 2-channel injected sequence. */
      reinterpret_cast<ADC_TypeDef*>(ADCx)->JSQR = (2-1)<< 20
                                                 |  ch2 << 15
                                                 |  ch1 << 10;
    }
    else
    {
      /* Set up 1-channel injected sequence. */
      reinterpret_cast<ADC_TypeDef*>(ADCx)->JSQR = (1-1)<< 20
                                                 |  ch1 << 15;
    }
    /* Enable scan and end of injected conversion interrupt */
    reinterpret_cast<ADC_TypeDef*>(ADCx)->CR1  = ADC_CR1_SCAN
                                               | ADC_CR1_JEOCIE;
    /* Trigger injected on Timer1 TRGO (rising edge) , turn ADC on */
    reinterpret_cast<ADC_TypeDef*>(ADCx)->CR2  = trigger
                                               | ADC_CR2_JEXTEN_0
                                               | ADC_CR2_ADON;

    // Enable interrupt when ADC injected conversion completes
    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, 2);
  }

  /** \brief Set a callback. */
  void setCallback(void (*user_callback)(uint32_t , uint32_t , uint32_t, uint32_t))
  {
    this->user_callback = user_callback;
  }

  /** \brief Force start of conversion */
  void convert()
  {
    reinterpret_cast<ADC_TypeDef*>(ADCx)->CR2 |= ADC_CR2_JSWSTART;
  }

  /** \brief Read the samples out of registers */
  void inline sample(void)
  {
    channel1 = reinterpret_cast<ADC_TypeDef*>(ADCx)->JDR1;
    channel2 = reinterpret_cast<ADC_TypeDef*>(ADCx)->JDR2;
    channel3 = reinterpret_cast<ADC_TypeDef*>(ADCx)->JDR3;
    channel4 = reinterpret_cast<ADC_TypeDef*>(ADCx)->JDR4;
    if (user_callback)
    {
      user_callback(channel1, channel2, channel3, channel4);
    }
  }

  /** \brief Get the last sampled value from channel 1. */
  uint32_t get_channel1(void) { return channel1; }

  /** \brief Get the last sampled value from channel 2. */
  uint32_t get_channel2(void) { return channel2; }

  /** \brief Get the last sampled value from channel 3. */
  uint32_t get_channel3(void) { return channel3; }

  /** \brief Get the last sampled value from channel 4. */
  uint32_t get_channel4(void) { return channel4; }

  /** \brief Set the sample time (in cycles of the ADC) of a channel. */
  int setSampleTime(const uint16_t channel, const uint16_t time)
  {
    if (!IS_ADC_SAMPLE_TIME(time)) return -1;
    if (channel > 18) return -1;
    if (channel > 9)
    {
      reinterpret_cast<ADC_TypeDef*>(ADCx)->CR1 &= (0xffffffff - (0x7 << ((channel-9)*3)));
      reinterpret_cast<ADC_TypeDef*>(ADCx)->CR1 |= time << ((channel-9)*3);
    }
    else
    {
      reinterpret_cast<ADC_TypeDef*>(ADCx)->CR2 &= (0xffffffff - (0x7 << (channel*3)));
      reinterpret_cast<ADC_TypeDef*>(ADCx)->CR2 |= time << (channel*3);
    }
    return 0;
  }

private:
  uint32_t channel1;
  uint32_t channel2;
  uint32_t channel3;
  uint32_t channel4;

  /** \brief A stored callback to user code to call when an
   *         injected sampling is complete. */
  void (*user_callback)(uint32_t, uint32_t, uint32_t, uint32_t);
};



extern AnalogSampler<ADC1_BASE> adc1;
extern AnalogSampler<ADC2_BASE> adc2;

#endif
