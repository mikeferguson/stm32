/*
 * Copyright (c) 2014, Michael E. Ferguson
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

#ifndef _ETHERBOTIX_USER_IO_HPP_
#define _ETHERBOTIX_USER_IO_HPP_

usart3_t usart3;

// State of user IO
uint8_t user_io_usart3_active_;
uint8_t user_io_spi2_active_;
uint8_t user_io_tim9_active_;
uint8_t user_io_tim12_active_;
uint8_t user_io_mask_;

inline void user_io_init()
{
  user_io_usart3_active_ = 0;
  user_io_spi2_active_ = 0;
  user_io_tim9_active_ = 0;
  user_io_tim12_active_ = 0;
  user_io_mask_ = 0;
}

inline void user_io_update_mask(uint8_t value)
{
  user_io_mask_ = value;
}

inline void user_io_set_output(uint8_t value)
{
  if (user_io_mask_ & 0x01)
  {
    if (value & 0x01)
      a0_sense::high();
    else
      a0_sense::low();
  }
  if (user_io_mask_ & 0x02)
  {
    if (value & 0x02)
      a1_sense::high();
    else
      a1_sense::low();
  }
  if (user_io_mask_ & 0x04)
  {
    if (value & 0x04)
      a2_sense::high();
    else
      a2_sense::low();
  }
  if ((user_io_mask_ & 0x08) &&
      (user_io_usart3_active_ == 0) &&
      (user_io_spi2_active_ == 0))
  {
    // D3 is also USART3_TX, TIM2_CH3, SPI2_SCK
    if (value & 0x08)
      d3::high();
    else
      d3::low();
  }
  if ((user_io_mask_ & 0x10) &&
      (user_io_usart3_active_ == 0))
  {
    // D4 is also USART3_RX
    if (value & 0x10)
      d4::high();
    else
      d4::low();
  }
  if ((user_io_mask_ & 0x20) &&
      (user_io_tim12_active_ == 0))
  {
    // D5 is also TIM12_CH2
    if (value & 0x20)
      d5::high();
    else
      d5::low();
  }
  if ((user_io_mask_ & 0x40) &&
      (user_io_tim9_active_ == 0))
  {
    // D6 is also TIM9_CH1
    if (value & 0x40)
      d6::high();
    else
      d6::low();
  }
  if ((user_io_mask_ & 0x80) &&
      (user_io_tim9_active_ == 0))
  {
    // D7 is also TIM9_CH2
    if (value & 0x80)
      d7::high();
    else
      d7::low();

  }
}

inline void user_io_set_direction(uint8_t value)
{
  if (user_io_mask_ & 0x01)
  {
    if (value & 0x01)
      a0_sense::mode(GPIO_OUTPUT);
    else
      a0_sense::mode(GPIO_INPUT_ANALOG);
  }
  if (user_io_mask_ & 0x02)
  {
    if (value & 0x02)
      a1_sense::mode(GPIO_OUTPUT);
    else
    {
      a1_sense::mode(GPIO_INPUT_ANALOG);
      // If made input, cancel SPI2 as A1 is also SPI2_MISO
      user_io_spi2_active_ = 0;
    }
  }
  if (user_io_mask_ & 0x04)
  {
    if (value & 0x04)
      a2_sense::mode(GPIO_OUTPUT);
    else
    {
      a2_sense::mode(GPIO_INPUT_ANALOG);
      // If made input, cancel SPI2 as A2 is also SPI2_MOSI
      user_io_spi2_active_ = 0;
    }
  }
  if (user_io_mask_ & 0x08)
  {
    if (value & 0x08)
      d3::mode(GPIO_OUTPUT);
    else
    {
      d3::mode(GPIO_INPUT);
      // If made input, cancel USART3 as D3 is also USART3_TX
      user_io_usart3_active_ = 0;
      // If made input, cancel SPI2 as D3 is also SPI2_SCK
      user_io_spi2_active_ = 0;
    }
  }
  if (user_io_mask_ & 0x10)
  {
    if (value & 0x10)
      d4::mode(GPIO_OUTPUT);
    else
    {
      d4::mode(GPIO_INPUT);
      // If made input, cancel USART3 as D4 is also USART3_RX
      user_io_usart3_active_ = 0;
    }
  }
  if (user_io_mask_ & 0x20)
  {
    if (value & 0x20)
      d5::mode(GPIO_OUTPUT);
    else
    {
      d5::mode(GPIO_INPUT);
      // If made input, cancel TIM12 as D5 is also TIM12_CH2
      user_io_tim12_active_ = 0;
    }
  }
  if (user_io_mask_ & 0x40)
  {
    if (value & 0x40)
      d6::mode(GPIO_OUTPUT);
    else
    {
      d6::mode(GPIO_INPUT);
      // If made input, cancel TIM9 as D6 is also TIM9_CH1
      user_io_tim9_active_ = 0;
    }
  }
  if (user_io_mask_ & 0x80)
  {
    if (value & 0x80)
      d7::mode(GPIO_OUTPUT);
    else
    {
      d7::mode(GPIO_INPUT);
      // If made input, cancel TIM9 as D7 is also TIM9_CH2
      user_io_tim9_active_ = 0;
    }
  }
}

/******************************************************************************
 * USER USART
 */

/** @brief Start the USART */
inline void user_io_usart_init()
{
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  d3::mode(GPIO_ALTERNATE | GPIO_AF_USART3);
  d4::mode(GPIO_ALTERNATE | GPIO_AF_USART3);
  
  // Use dynamixel defs of baud
  uint32_t baud = 0;
  if (registers.usart3_baud == 1)
    baud = 1000000;
  else if (registers.usart3_baud == 3)
    baud = 500000;
  else if (registers.usart3_baud == 16)
    baud = 115200;
  else if (registers.usart3_baud == 34)
    baud = 57600;
  else if (registers.usart3_baud == 103)
    baud = 19200;
  else if (registers.usart3_baud == 207)
    baud = 9600;
  else if (registers.usart3_baud == 250)
    baud = 2250000;
  else if (registers.usart3_baud == 251)
    baud = 2500000;
  else if (registers.usart3_baud == 252)
    baud = 3000000;
  else
  {
    // Unsupported baud, set default
    baud = 9600;
    registers.usart3_baud = 207;
  }

  usart3.init(baud, 8);
  user_io_usart3_active_ = 1;
}

/** @brief Write a buffer of data to the USART3 */
inline void user_io_usart_write(uint8_t * data, uint8_t len)
{
  if (user_io_usart3_active_ == 0)
  {
    // Need to configure usart3
    user_io_usart_init();
  }

  uint16_t d[len];
  for (int i = 0; i < len; ++i)
    d[i] = data[i];

  usart3.write(d, len); 
}

/** @brief Read a buffer of data from the USART3 */
inline uint8_t user_io_usart_read(uint8_t * data, uint8_t max_len)
{
  if (user_io_usart3_active_ == 0)
    return 0;

  if (max_len == 0)
    max_len = 255;

  int len = 0;
  while (max_len--)
  {
    int16_t d = usart3.read();
    if (d == -1)
      return len;
    data[len++] = d;
  }
  return len;
}

/******************************************************************************
 * USER TIMER 12
 */

inline void user_io_tim12_init(uint16_t mode)
{
  // Turn on clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
  if (mode == 0)
  {
    // TODO disable
  }
  else if (mode == 1)  // Count on external clock
  {
    d5::mode(GPIO_ALTERNATE | GPIO_AF_TIM12);
    TIM12->CR1 &= (uint16_t) ~TIM_CR1_CEN;
    TIM12->ARR = 65535;  // Max range
    TIM12->SMCR |= TIM_TS_TI2FP2;  // Use filtered TI2
    TIM12->SMCR |= TIM_SlaveMode_External1;  // Use external clock mode 1 (count rising edges)
    // TODO Filtering with IC2F
    TIM12->CR1 |= TIM_CR1_CEN;
  }
  user_io_tim12_active_ = mode;
  registers.tim12_mode = mode;
}

inline uint16_t user_io_tim12_get_count()
{
  if (user_io_tim12_active_ > 0)
    return TIM12->CNT;
  else
    return 0;
}


/******************************************************************************
 * Feedback
 */

inline void user_io_update()
{
  //registers.digital_in = ?? TODO
  registers.tim12_count = user_io_tim12_get_count();
}



#endif  // _ETHERBOTIX_USER_IO_HPP_
