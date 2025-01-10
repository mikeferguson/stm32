/*
 * Copyright (c) 2019-2023, Michael E. Ferguson
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

#ifndef DAC121_HPP
#define DAC121_HPP

/**
 * @brief Hardware interface for DAC121
 */
template<unsigned int SPIx,
         typename CS>
class DAC121
{
public:
  bool init()
  {
    static_assert(SPIx == SPI1_BASE |
                  SPIx == SPI2_BASE |
                  SPIx == SPI3_BASE, "DAC121: Unknown SPI");

    // CS is active LOW
    CS::mode(GPIO_OUTPUT);
    CS::high();

    /* Enable SPI before configuring it */
    if (SPIx == SPI1_BASE)
    {
      RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    }
    else if (SPIx == SPI2_BASE)
    {
      RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    }
    else if (SPIx == SPI3_BASE)
    {
      RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    }

    /* Configure SPI
     *  Defaults used: MSB first, No CRC, Bi-directional
     */
    SPI_TypeDef* SPI = reinterpret_cast<SPI_TypeDef*>(SPIx);
    SPI->CR1 = 0;               // Disable SPI, clear all settings
    SPI->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI;  // Master mode
    SPI->CR1 |= SPI_CR1_SSM;    // Software slave management (we toggle NSS)
    SPI->CR1 |= SPI_CR1_DFF;    // 16-bit data size
    SPI->CR1 &= ~SPI_CR1_CPOL;  // CPOL = 0: clock is low when idle
    SPI->CR1 |= SPI_CR1_CPHA;   // CPHA = 1: sample is taking on falling edge of clk
    /*  Max SCLK = 30MHz, configure for 10.5MHz */
    if (SPIx == SPI1_BASE)
    {
      /* ABP2 is 84MHz, set prescaler to 010b = /8 */
      SPI->CR1 &= ~SPI_CR1_BR;
      SPI->CR1 |= SPI_CR1_BR_1;
    }
    else
    {
      /* APB1 is 42MHz, set prescaler to 001b = /4 */
      SPI->CR1 &= ~SPI_CR1_BR;
      SPI->CR1 |= SPI_CR1_BR_0;
    }
    SPI->CR1 |= SPI_CR1_SPE;    // Enable SPI again

    return true;
  }

  /**
   * @brief Send a command
   */
  void setOutput(uint16_t output)
  {
    // We only toggle the CS (or SYNC as datasheet calls it) high for a brief
    // moment between transfers. This allows us to forget about the SPI call
    // completing (and also lowers power draw according to datasheet p.??)
    SPI_TypeDef* SPI = reinterpret_cast<SPI_TypeDef*>(SPIx);

    // Read previous data to avoid overrun error
    uint16_t garbage = SPI->DR;

    //while ((SPI->SR & SPI_I2S_FLAG_RXNE) == RESET);
    CS::high();
    delay_ns(20);  // Assumes 5V supply

    // CS to first SCLK edge = 10ns minimum
    CS::low();
    delay_ns(10);

    // Send 12-bit command
    SPI->DR = (output & 0xFFF);
  }

};

#endif  // DAC121_HPP
