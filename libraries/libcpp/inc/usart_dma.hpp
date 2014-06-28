/*
 * Copyright (c) 2013-2014, Unbounded Robotics
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
 * Driver for a standard 8N1 or 9N1 serial port, with DMA read/write.
 */

#ifndef _STM32_CPP_USART_DMA_HPP_
#define	_STM32_CPP_USART_DMA_HPP_

#include <dma.hpp>
#include <stm32f4xx_usart.h>
#include <gpio.hpp>
#include <rcc.hpp>

#define USART_FLAG_TXE      ((uint16_t)0x0080)
#define USART_FLAG_TC       ((uint16_t)0x0040)
#define USART_FLAG_RXNE     ((uint16_t)0x0020)
#define USART_FLAG_IDLE     ((uint16_t)0x0010)
#define USART_FLAG_ORE      ((uint16_t)0x0008)
#define USART_FLAG_NE       ((uint16_t)0x0004)
#define USART_FLAG_FE       ((uint16_t)0x0002)
#define USART_FLAG_PE       ((uint16_t)0x0001)

template <unsigned int USARTx, typename DMAr, typename DMAw>
class UsartDMA
{
public:

  void init(uint32_t baud_rate, uint32_t word_length, uint32_t priority = DMA_Priority_High)
  {
    USART_DeInit(reinterpret_cast<USART_TypeDef*>(USARTx));
    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = baud_rate;
    USART_InitStructure.USART_WordLength = word_length;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(reinterpret_cast<USART_TypeDef*>(USARTx), &USART_InitStructure);

    write_dma.init(priority);
    read_dma.init(priority);

    reinterpret_cast<USART_TypeDef*>(USARTx)->SR &= ~USART_FLAG_TC;

    USART_DMACmd(reinterpret_cast<USART_TypeDef*>(USARTx), USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
    USART_Cmd(reinterpret_cast<USART_TypeDef*>(USARTx), ENABLE);
  }

  void write(uint16_t * data, uint8_t len)
  {
    /* Need to clear previous TC, if any */
    reinterpret_cast<USART_TypeDef*>(USARTx)->SR &= ~USART_FLAG_TC;

    /* Write the data, trigger the USART DMA. */
    write_dma.write(data, len);
  }

  /**
   *  \brief Updates the status of whether the write is done.
   *  \returns True if write is complete.
   */
  bool done()
  {
    return write_dma.done() && ((reinterpret_cast<USART_TypeDef*>(USARTx)->SR & USART_FLAG_TC) > 0);
  }

  int16_t read()
  {
    return (int16_t) read_dma.read();
  }

protected:
  DMAw write_dma;
  DMAr read_dma;
};



/**
 * Similar to above, except it adds an enable pin and functions.
 * This also requires the external code to enabled NVIC for proper USART IRQ
 * and to call usartIrqHandler() from appropriate USART IRQ handler:
 *
 * Code Example for USART1:
 *
 *  UsartDMAWithEnable<USART1_BASE, gpio_typedef, READ_DMA, WRITE_DMA> usart;
 *
 *  extern "C"
 *  {
 *    void USART1_IRQHandler()
 *    {
 *      usart.usartIrqHandler();
 *    }
 *  }
 *
 *  int main ()
 *  {
 *      ...
 *      NVIC_SetPriority(USART1_IRQn, 1);
 *      NVIC_EnableIRQ(USART1_IRQn);
 *      ...
 *  }
 *
 */
template<unsigned int USARTx, typename ENABLE_GPIO, typename DMAr, typename DMAw>
class UsartDMAWithEnable : public UsartDMA<USARTx, DMAr, DMAw>
{
public:
  void gpioInit()
  {
    ENABLE_GPIO::low();
    ENABLE_GPIO::mode(GPIO_OUTPUT);
  }

  void init(uint32_t baud_rate, uint32_t word_length, uint32_t priority = DMA_Priority_High)
  {
    gpioInit();
    UsartDMA<USARTx, DMAr, DMAw>::init(baud_rate, word_length, priority);
    /* Need to clear previous TC, if any */
    reinterpret_cast<USART_TypeDef*>(USARTx)->SR &= ~USART_FLAG_TC;
    /* Enable TC interrupt (to allow quick turn-around of RS485 write enable signal) */
    USART_ITConfig(reinterpret_cast<USART_TypeDef*>(USARTx), USART_IT_TC, ENABLE);
  }

  inline void setTX()
  {
    ENABLE_GPIO::high();
  }

  inline void setRX()
  {
    ENABLE_GPIO::low();
  }

  /**
   * \brief Should be called from USARTx_IRQHandler()
   *
   *  This will automatically call setRX() mode.
   */
  inline void usartIrqHandler()
  {
    this->setRX();
    // clear TC flag, or interrupt will re-occur
    reinterpret_cast<USART_TypeDef*>(USARTx)->SR &= ~USART_FLAG_TC;
    write_complete_ = true;
  }

  inline void write(uint16_t * data, uint8_t len)
  {
    setTX();
    write_complete_ = false;
    UsartDMA<USARTx, DMAr, DMAw>::write(data, len);
  }

  /**
   *  \brief Returns the status of whether the write is done.
   *  \returns True if write is complete.
   */
  inline bool done()
  {
    return write_complete_;
  }

protected:
  volatile bool write_complete_;
};

#endif /* _STM32_CPP_USART_DMA_HPP_ */
