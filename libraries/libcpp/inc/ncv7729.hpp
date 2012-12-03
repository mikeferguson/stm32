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

/* 
 * stm32_cpp: a C++ stm32 library
 * Driver for the NCV7729 motor driver from ON Semi.
 *
 * Usage:
 *
 * Ncv7729<SPI2_BASE, CS, TIM1_BASE, motor_enable, fault, 1> motor;
 * ...
 * RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
 * RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
 * tim1_ch1::mode(GPIO_ALTERNATE | GPIO_AF_TIM1);
 * tim1_ch1n::mode(GPIO_ALTERNATE | GPIO_AF_TIM1);
 * sck::mode(GPIO_ALTERNATE | GPIO_AF_SPI2);
 * miso::mode(GPIO_ALTERNATE | GPIO_AF_SPI2);
 * mosi::mode(GPIO_ALTERNATE | GPIO_AF_SPI2);
 * motor.init();
 * ...
 * motor.set(1.0);
 *
 */

#ifndef _STM32_CPP_NCV7729_H_
#define _STM32_CPP_NCV7729_H_

#include "stm32f4xx_spi.h"
#include "delay.hpp"

#define NCV7729_VERIFICATION_MASK       (0x3F<<8)
#define NCV7729_DATA_MASK               (0xFF)

/* commands */
#define NCV7729_WR_CONFIG               (0x33<<8)
#define NCV7729_RD_ID                   (0x00<<8)
#define NCV7729_RD_REV                  (0x03<<8)
#define NCV7729_RD_CONFIG               (0x28<<8)
#define NCV7729_RD_DIAG                 (0x09<<8)

/* verification data */
#define NCV7729_OK                      (0x2A<<8)
#define NCV7729_ERROR                   (0x2B<<8)

/* CONFIG data */
#define NCV7729_CONFIG_SF_MODE          (1<<7)
#define NCV7729_CONFIG_CHRG_PUMP_MODE   (1<<6)
#define NCV7729_CONFIG_OUT2             (1<<5)
#define NCV7729_CONFIG_OUT1             (1<<4)
#define NCV7729_OC4_9A6                 (0b1100)
#define NCV7729_OC3_6A6_DEFAULT         (0b1000)
#define NCV7729_OC2_5A5                 (0b0100)
#define NCV7729_OC1_2A5                 (0b0000)

/* DIAG data */
#define NCV7729_DIAG_ENABLED            (1<<7)
#define NCV7729_DIAG_OVERTEMP           (1<<6)
#define NCV7729_DIAG_CURRENT_REDUCTION  (1<<5)
#define NCV7729_DIAG_CURRENT_LIMIT      (1<<4)
#define NCV7729_DIAG_SHORTED_LOAD       (0b1100)
#define NCV7729_DIAG_OUT1_SHORT_VS      (0b0001)
#define NCV7729_DIAG_OUT1_SHORT_GND     (0b0010)
#define NCV7729_DIAG_OPEN_LOAD          (0b0011)
#define NCV7729_DIAG_OUT2_SHORT_VS      (0b0100)
#define NCV7729_DIAG_OUT2_SHORT_GND     (0b1000)
#define NCV7729_DIAG_LOAD_MASK          (0b1111)

template<unsigned int SPIx, typename Cs, 
         unsigned int TIMx,typename En, typename Fault, unsigned int CH>
class Ncv7729
{
  
public:
  void init(void)
  {
    /* Setup I/O */
    Cs::mode(GPIO_OUTPUT);
    Cs::high();

    En::mode(GPIO_OUTPUT);
    En::low();
    
    /* it appears we have to hold this low in order for the bridge to enable? */
    Fault::mode(GPIO_OUTPUT);
    Fault::low();

    /* Setup SPIx as follows:
     *  CPOL = 0 : clock is low when idle
     *  CPHA = 1 : sample is taken on falling edge of clk
     */
    SPI_InitTypeDef SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    /*  Max SCLK = 2MHz, 168MHz/256 should be OK */
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(reinterpret_cast<SPI_TypeDef*>(SPIx), &SPI_InitStructure);

    SPI_Cmd(reinterpret_cast<SPI_TypeDef*>(SPIx), ENABLE);

    /* Setup TIMx */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CR1 = 0;
    /* PWM @ 25KHz, 168e6 / 25e3 / 2 (center aligned) = */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->ARR = 3360;
    reinterpret_cast<TIM_TypeDef*>(TIMx)->PSC = 0;
    /* Enable center-aligned, CCIF only on downcount, */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CR1 = (1<<5);
    /* Setup CCRx */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCMR1 =
       (((6<<4) | (1<<3)) << 0) | // OC1
       (((6<<4) | (1<<3)) << 8) ; // OC2 
    /* Use default PWM duty of 50% (locked-antiphase = 0V) */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCR1 = reinterpret_cast<TIM_TypeDef*>(TIMx)->ARR/2;
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCR2 = reinterpret_cast<TIM_TypeDef*>(TIMx)->ARR/2;
    /* Enable capture/compares, then timer */
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CCER =  TIM_CCER_CC1E | TIM_CCER_CC1NE | 
                  TIM_CCER_CC2E | TIM_CCER_CC2NE;
    reinterpret_cast<TIM_TypeDef*>(TIMx)->CR1 |= TIM_CR1_CEN;
    reinterpret_cast<TIM_TypeDef*>(TIMx)->BDTR |= TIM_BDTR_MOE;

    En::high();
  }

  /* send ID(2):COMMAND(6):DATA(8) */
  int16_t read(const uint16_t command)
  {
    Cs::low();

    /* SCS setup time = 100ns minimum */
    delay_ns(100);

    /* Send address to read from */
    SPI_I2S_SendData(reinterpret_cast<SPI_TypeDef*>(SPIx), command);
    while (SPI_I2S_GetFlagStatus(reinterpret_cast<SPI_TypeDef*>(SPIx), SPI_I2S_FLAG_RXNE) == RESET);
    uint16_t d = reinterpret_cast<SPI_TypeDef*>(SPIx)->DR;

    Cs::high();

    if((d & NCV7729_VERIFICATION_MASK) != NCV7729_OK) return -1;

    return (d & NCV7729_DATA_MASK);
  }

  /* send ID(2):COMMAND(6):DATA(8) */
  uint16_t write(const uint16_t command, const uint16_t data)
  {
    Cs::low();
  
    /* SCS setup time = 100ns minimum */
    delay_ns(100);

    /* Send address to read from */
    SPI_I2S_SendData(reinterpret_cast<SPI_TypeDef*>(SPIx), command + data);
    while (SPI_I2S_GetFlagStatus(reinterpret_cast<SPI_TypeDef*>(SPIx), SPI_I2S_FLAG_RXNE) == RESET);
    uint16_t d = reinterpret_cast<SPI_TypeDef*>(SPIx)->DR;
  
    Cs::high();

    if((d & NCV7729_VERIFICATION_MASK) != NCV7729_OK) return -1;

    return (d & NCV7729_DATA_MASK);
  }

  /* set PWM. duty should be -1.0 to 1.0 */
  void set(float duty)
  {
    int d = reinterpret_cast<TIM_TypeDef*>(TIMx)->ARR;
    d = d/2 + (d * duty);

    if (d < 0) d = 0;
    if (d > reinterpret_cast<TIM_TypeDef*>(TIMx)->ARR) d = reinterpret_cast<TIM_TypeDef*>(TIMx)->ARR;

    if(CH == 1) reinterpret_cast<TIM_TypeDef*>(TIMx)->CCR1 = d;
    else if(CH == 2) reinterpret_cast<TIM_TypeDef*>(TIMx)->CCR2 = d;
  }  
};

#endif // _STM32_CPP_NCV7729_H_
