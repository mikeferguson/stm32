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

#ifndef _STM32_CPP_USART_H_
#define	_STM32_CPP_USART_H_

#include "rcc.hpp"

#define USART_FLAG_TXE      ((uint16_t)0x0080)
#define USART_FLAG_TC       ((uint16_t)0x0040)
#define USART_FLAG_RXNE     ((uint16_t)0x0020)
#define USART_FLAG_IDLE     ((uint16_t)0x0010)
#define USART_FLAG_ORE      ((uint16_t)0x0008)
#define USART_FLAG_NE       ((uint16_t)0x0004)
#define USART_FLAG_FE       ((uint16_t)0x0002)
#define USART_FLAG_PE       ((uint16_t)0x0001)

/**
 *  \brief Simple driver for a standard 8N1 serial port. Serial write is
 *         blocking, reads are handled via an IRQ in the background.
 *  \tparam USARTx The base address of the USART, for isntance, USART1_BASE.
 *  \tparam SIZE The size of the read buffer to create.
 *
 *  Limitations:
 *   - no support for oversampling by 8.
 *   - mode is always 8N1, no parity.
 *
 *  Example:
 *  \code
 *  // define a usart
 *  Usart<USART1_BASE, 32> usart1;
 *
 *  // create an IRQ handler
 *  void USART1_IRQHandler(void)
 *  {
 *    usart1.irq();
 *  }
 *
 *  // in main():
 *  RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // APB2 also has USART6
 *  RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // ABP1 also has USART3, UART4/5
 *  usart1.init(115200);
 *  NVIC_SetPriority(USART1_IRQn, 1);
 *  NVIC_EnableIRQ(USART1_IRQn);
 *  \endcode
 */
template <unsigned int USARTx, unsigned int SIZE>
class Usart
{
  uint8_t head;
  uint8_t tail;
  uint16_t buffer[SIZE];

public:
  Usart() : head(0), tail(0) {}

  /**
   *  \brief Initialize the serial port.
   *  \param baud_rate The baud rate in bps to use.
   */
  static void init(uint32_t baud_rate)
  {
    /* setup for 1 stop bit */
    reinterpret_cast<USART_TypeDef*>(USARTx)->CR2 &= ~((uint32_t)USART_CR2_STOP);
    
    /* setup 8N1, no parity */
    reinterpret_cast<USART_TypeDef*>(USARTx)->CR1 &= ~((uint32_t) (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS));
    reinterpret_cast<USART_TypeDef*>(USARTx)->CR1 |= USART_CR1_RE | USART_CR1_TE;

    /* set baud rate */
    if(USARTx == USART1_BASE || USARTx == USART6_BASE)
      reinterpret_cast<USART_TypeDef*>(USARTx)->BRR = RccImpl::get_pclk2() / baud_rate;
    else
      reinterpret_cast<USART_TypeDef*>(USARTx)->BRR = RccImpl::get_pclk1() / baud_rate;

    /* enable rx interrupt and go... */
    reinterpret_cast<USART_TypeDef*>(USARTx)->CR1 |= USART_CR1_RXNEIE;
    reinterpret_cast<USART_TypeDef*>(USARTx)->CR1 |= USART_CR1_UE;
  }

  /**
   *  \brief Write a byte of data to the serial port. This blocks as byte is written.
   *  \param data The data to write.
   */
  static void write(uint16_t data)
  {
    while ((reinterpret_cast<USART_TypeDef*>(USARTx)->SR & USART_FLAG_TXE) == 0);
    reinterpret_cast<USART_TypeDef*>(USARTx)->DR = data;
    while ((reinterpret_cast<USART_TypeDef*>(USARTx)->SR & USART_FLAG_TC) == 0);
  }

  /**
   *  \brief Read a byte from the IRQ buffer.
   *  \returns The byte on the head of the buffer, -1 if buffer is empty.
   */
  int16_t read()
  {
    int16_t data = -1;
    if(head != tail){
      data = buffer[tail];
      tail = (tail+1)%SIZE;
    }
    return data;
  }

  /**
   *  \brief Callback for IRQ processing. This moves the value of the data register
   *         into our internal buffer.
   *
   *  This should be called from the IRQ:
   *  \code
   *  void USART1_IRQHandler(void)
   *  {
   *    usart1.irq();
   *  }
   *  \endcode
   */
  void irq()
  {
    buffer[head] = (reinterpret_cast<USART_TypeDef*>(USARTx)->DR & (uint16_t)0x01ff);
    head = (head+1)%SIZE;
  }
};

/* Similar to above, except it adds an enable pin and functions to toggle it:
 *  Usart<USART1_BASE, 32, gpio_typedef> usart1;
 */
/**
 *  \brief Simple driver for a standard 8N1 serial port with a write enable.
 *         Serial write is blocking, reads are handled via an IRQ in the
 *         background.
 *  \tparam USARTx The base address of the USART, for isntance, USART1_BASE.
 *  \tparam SIZE The size of the read buffer to create.
 *  \tparam ENABLE The Gpio definition for the enable pin.
 */
template<unsigned int USARTx, unsigned int SIZE, typename ENABLE>
class UsartWithEnable : public Usart<USARTx, SIZE>
{
public:
  /** \brief Set the enable pin to high */
  void setTX()
  {
    ENABLE::high();
  }

  /** \brief Set the enable pin to low */
  void setRX()
  {
    ENABLE::low();
  }
};

#endif /* _STM32_CPP_USART_H_ */
