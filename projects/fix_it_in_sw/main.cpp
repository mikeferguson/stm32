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
 * PROJECT: Fix-it-in-software
 * Firmware for custom F4-based battlebot controller.
 * author: Michael E. Ferguson
 */

#include "stm32f4xx.h"

#include "gpio.hpp"
#include "usart.hpp"
#include "encoder.hpp"
#include "ncv7729.hpp"
#include "mini_imu9_v2.hpp"

typedef Gpio<GPIOA_BASE,0> left_enc_a;
typedef Gpio<GPIOA_BASE,1> left_enc_b;
typedef Gpio<GPIOA_BASE,2> radio_ch1; /* TIM9, CH1 */
typedef Gpio<GPIOA_BASE,3> left_CS;
typedef Gpio<GPIOA_BASE,5> radio_ch2; /* TIM2, CH1 */
typedef Gpio<GPIOA_BASE,7> tim1_ch1n;
typedef Gpio<GPIOA_BASE,8> tim1_ch1;
typedef Gpio<GPIOA_BASE,9> tim1_ch2;

typedef Gpio<GPIOB_BASE,0> tim1_ch2n;
typedef Gpio<GPIOB_BASE,1> current_sense; /* A9 */
typedef Gpio<GPIOB_BASE,5> motor_enable;
typedef Gpio<GPIOB_BASE,6> serial_tx; /* USART1 */
typedef Gpio<GPIOB_BASE,7> serial_rx;
typedef Gpio<GPIOB_BASE,8> radio_ch3; /* TIM10, CH1 */
typedef Gpio<GPIOB_BASE,9> radio_ch4; /* TIM11, CH1 */
typedef Gpio<GPIOB_BASE,10> imu_scl; /* I2C2 */
typedef Gpio<GPIOB_BASE,11> imu_sda;
typedef Gpio<GPIOB_BASE,13> sck; /* SPI2 */
typedef Gpio<GPIOB_BASE,14> miso;
typedef Gpio<GPIOB_BASE,15> mosi;

typedef Gpio<GPIOC_BASE,1> voltage_sense; /* A11 */
typedef Gpio<GPIOC_BASE,5> left_fault;
typedef Gpio<GPIOC_BASE,6> right_enc_a;
typedef Gpio<GPIOC_BASE,7> right_enc_b;
typedef Gpio<GPIOC_BASE,8> right_CS;
typedef Gpio<GPIOC_BASE,9> right_fault;
typedef Gpio<GPIOC_BASE,10> top_r_led;
typedef Gpio<GPIOC_BASE,11> top_b_led;
typedef Gpio<GPIOC_BASE,12> top_g_led;
typedef Gpio<GPIOC_BASE,13> bot_r_led;
typedef Gpio<GPIOC_BASE,14> bot_b_led;
typedef Gpio<GPIOC_BASE,15> bot_g_led;

/* two attached encoders */
Encoder<TIM5_BASE> left_enc;
Encoder<TIM3_BASE> right_enc;

/* two NCV7729 motor drivers */
Ncv7729<SPI2_BASE, left_CS, TIM1_BASE, motor_enable, left_fault, 1> left_motor;
Ncv7729<SPI2_BASE, right_CS, TIM1_BASE, motor_enable, right_fault, 2> right_motor;

/* IMU */
MiniImu9v2<I2C2_BASE,
           DMA1_Stream3_BASE, 3 /* stream */, 7 /* channel */,
           imu_scl, imu_sda> imu;

/* debugging via FTDI */
Usart<USART1_BASE, 32> usart1;

/* system clock */
uint32_t system_clock;

int main(void)
{
  NVIC_SetPriorityGrouping(3);

  // enable all GPIO clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

  top_r_led::mode(GPIO_OUTPUT);
  top_g_led::mode(GPIO_OUTPUT);
  top_b_led::mode(GPIO_OUTPUT);
  bot_r_led::mode(GPIO_OUTPUT);
  bot_g_led::mode(GPIO_OUTPUT);
  bot_b_led::mode(GPIO_OUTPUT);

  top_r_led::high();
  top_g_led::high();
  top_b_led::low();
  bot_r_led::low();
  bot_g_led::high();
  bot_b_led::high();

  // setup encoders
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM5EN;
  left_enc_a::mode(GPIO_ALTERNATE | GPIO_AF_TIM5);
  left_enc_b::mode(GPIO_ALTERNATE | GPIO_AF_TIM5);
  right_enc_a::mode(GPIO_ALTERNATE | GPIO_AF_TIM3);
  right_enc_b::mode(GPIO_ALTERNATE | GPIO_AF_TIM3);
  left_enc.init();
  right_enc.init();

  // setup motors
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
  tim1_ch1::mode(GPIO_ALTERNATE | GPIO_AF_TIM1);
  tim1_ch1n::mode(GPIO_ALTERNATE | GPIO_AF_TIM1);
  tim1_ch2::mode(GPIO_ALTERNATE | GPIO_AF_TIM1);
  tim1_ch2n::mode(GPIO_ALTERNATE | GPIO_AF_TIM1);
  sck::mode(GPIO_ALTERNATE | GPIO_AF_SPI2);
  miso::mode(GPIO_ALTERNATE | GPIO_AF_SPI2);
  mosi::mode(GPIO_ALTERNATE | GPIO_AF_SPI2);
  left_motor.init();
  right_motor.init();

  // setup usart
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  usart1.init(115200);
  NVIC_SetPriority(USART1_IRQn, 1);
  NVIC_EnableIRQ(USART1_IRQn);

  // setup imu
  imu.init(100000);

  // setup systick
  SysTick_Config(SystemCoreClock/1000);
  system_clock = 0;
  __enable_irq();
  
  while(1)
  {
    imu.update(system_clock);
  }

}

extern "C"
{

void SysTick_Handler(void)
{
  ++system_clock;
  if( system_clock % 1000 == 0 )
  {
    if( (system_clock / 1000) % 3 == 0)
    {
      left_motor.set(0);
      top_b_led::high();
      top_r_led::low();
    }
    else if( (system_clock / 1000) % 3 == 1)
    {
      left_motor.set(0.1);
      top_r_led::high();
      top_g_led::low();
    }
    else
    {
      left_motor.set(-0.1);
      top_g_led::high();
      top_b_led::low(); 
    }
  }
}

void USART1_IRQHandler(void)
{
  usart1.irq();
}

} /* extern C */
