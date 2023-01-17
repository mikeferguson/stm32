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

#ifndef ADS8684_HPP
#define ADS8684_HPP

#include "assert.h"
#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"  // for FLAGS only
#include "dma.hpp"

/* Command Register Map */
#define ADS8684_NO_OP         0x0000
#define ADS8684_STDBY         0x8200
#define ADS8684_PWR_DN        0x8300
#define ADS8684_RESET         0x8500
#define ADS8684_AUTOR_RST     0xA000
#define ADS8684_MAN_CH0       0xC000
#define ADS8684_MAN_CH1       0xC400
#define ADS8684_MAN_CH2       0xC800
#define ADS8684_MAN_CH3       0xCC00
#define ADS8684_MAN_AUX       0xE000

/* Program Register Map */
#define ADS8684_AUTO_SEQ_EN   0x01
#define ADS8684_CH_POWER_DN   0x02
#define ADS8684_FEATURE_SEL   0x03
#define ADS8684_CH0_RANGE     0x05
#define ADS8684_CH1_RANGE     0x06
#define ADS8684_CH2_RANGE     0x07
#define ADS8684_CH3_RANGE     0x85

#define ADS8684_PGM_WRITE     0x01

/* Range Selection Values */
#define ADS8684_RANGE_10v24   0x00  // +/- 10.24v
#define ADS8684_RANGE_5v12    0x01  // +/- 5.12v
#define ADS8684_RANGE_2v56    0x02  // +/- 2.56v
#define ADS8684_RANGE_p10v24  0x05  // 0 to 10.24v
#define ADS8684_RANGE_P5v12   0x06  // 0 to 5.12v

/**
 * @brief Hardware interface for ADS8684.
 * @tparam SPIx the SPI interface to use for comms.
 * @tparam CS the SPI chip select GPIO to use.
 * @tparam RST the GPIO for reset and power down.
 */
template<unsigned int SPIx,
         typename CS,
         typename RST,
         typename DMAr,
         typename DMAw>
class ADS8684
{
  static constexpr int NUM_CHANNELS = 4;

public:

  bool init(uint32_t priority = DMA_Priority_High)
  {
    static_assert(SPIx == SPI1_BASE |
                  SPIx == SPI2_BASE |
                  SPIx == SPI3_BASE, "ADS8684: Unknown SPI");

    /* CS is active LOW */
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

    /* Enable ADC */
    RST::mode(GPIO_OUTPUT);
    RST::high();

    write_dma.init(priority);
    read_dma.init(priority);

    /* Configure SPI
     *  Defaults used: MSB first, No CRC, Bi-directional
     */
    SPI_TypeDef* SPI = reinterpret_cast<SPI_TypeDef*>(SPIx);
    SPI->CR1 = 0;              // Disable SPI, clear all settings
    SPI->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI;  // Master mode
    SPI->CR1 |= SPI_CR1_SSM;   // Software slave management (we toggle NSS)
    SPI->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
    SPI->CR1 &= ~SPI_CR1_DFF;  // 8-bit data size
    reconfigureSPI();          // This sets bus polarity/edge AND enables SPI

    /* Enable all channels by default */
    channel_power_ = 0x00;
    prev_channel_ = 0x00;  // This is the sample that will come back this cycle
    next_channel_ = 0x00;  // This is what we have requested

    /* For debugging */
    cycles_ = 0;
    errors_ = 0;

    return true;
  }

  /**
   * @brief When SPI bus is shared, this will reset Polarity/Edge
   */
  void reconfigureSPI()
  {
    SPI_TypeDef* SPI = reinterpret_cast<SPI_TypeDef*>(SPIx);
    SPI->CR1 &= ~SPI_CR1_SPE;   // Disable before changing clock polarity/edge
    SPI->CR1 &= ~SPI_CR1_CPOL;  // CPOL = 0: clock is low when idle
    SPI->CR1 |= SPI_CR1_CPHA;   // CPHA = 1: sample is taking on falling edge of clk
    /*  Max SCLK = 17MHz (59ns), configure for 10.5MHz */
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
  }

  /**
   * @brief Power off/on individual channels.
   */
  void setChannelPower(uint8_t channel, bool enabled)
  {
    // Determine new value for channel power down register
    if (enabled)
    {
      channel_power_ &= ~(1 << channel);
    }
    else
    {
      channel_power_ |= (1 << channel);
    }

    uint8_t data[3];
    data[0] = (ADS8684_CH_POWER_DN << 1) + ADS8684_PGM_WRITE;
    data[1] = channel_power_;
    data[2] = 0;  // don't care, this is where we get data back

    // CS to first SCLK edge = 10ns minimum
    CS::low();
    delay_ns(10);

    // Use the DMA or die
    write_dma.write(data, 3);
    while (!write_dma.done());
    SPI_TypeDef* SPI = reinterpret_cast<SPI_TypeDef*>(SPIx);
    while ((SPI->SR & SPI_I2S_FLAG_BSY) != RESET);
    uint8_t data_back[3];
    data_back[0] = read_dma.read();
    data_back[1] = read_dma.read();
    data_back[2] = read_dma.read();
    while (!read_dma.isEmpty());

    // Confirm that data[1] == data_back[2]
    if (data[1] != data_back[2])
      ++errors_;
  
    // TODO: make sure that above commands ensure 10ns between SLCK edge and CS rising edge
    CS::high();
  }

  /**
   * @brief Set the range of a channel.
   * @param channel The channel ID, 0-3
   * @param range One of the ADS8684_RANGE_X values.
   */
  void setChannelRange(uint8_t channel, uint8_t range)
  {
    uint8_t data[3];
    data[0] = ((ADS8684_CH0_RANGE + channel) << 1) + ADS8684_PGM_WRITE;
    data[1] = range;
    data[2] = 0;  // don't care, this is where we get data back

    // CS to first SCLK edge = 10ns minimum
    CS::low();
    delay_ns(10);

    // Use the DMA or die
    write_dma.write(data, 3);
    while (!write_dma.done());
    SPI_TypeDef* SPI = reinterpret_cast<SPI_TypeDef*>(SPIx);
    while ((SPI->SR & SPI_I2S_FLAG_BSY) != RESET);
    uint8_t data_back[3];
    data_back[0] = read_dma.read();
    data_back[1] = read_dma.read();
    data_back[2] = read_dma.read();
    while (!read_dma.isEmpty());

    // Confirm that data[1] == data_back[2]
    if (data[1] != data_back[2])
      ++errors_;
  
    // TODO: make sure that above commands ensure 10ns between SLCK edge and CS rising edge
    CS::high();
  }

  /**
   * @brief Get the (latest) value of a channel
   */
  uint16_t getChannel(uint8_t channel)
  {
    return raw_values_[channel];
  }

  /**
   * @brief Setup the next read -- uses DMA
   */
  void update()
  {
    // Reset CS here, since DMA interrupt happens before transmission is done
    CS::high();

    // Get previous data
    int32_t prev_data[4];
    for (int i = 0; i < 4; ++i)
    {
      prev_data[i] = read_dma.read();
    }

    // Clean up from previous update
    while (!write_dma.done());
    while (!read_dma.isEmpty());

    // Copy input data to proper channel
    if (prev_data[2] != -1 && prev_data[3] != -1)
    {
      raw_values_[prev_channel_] = (prev_data[2] << 8) + prev_data[3];
    }
    prev_channel_ = next_channel_;

    // Select next (powered up) channel
    next_channel_ = (next_channel_ + 1) % NUM_CHANNELS;
    while (channel_power_ & (1 << next_channel_))
    {
      next_channel_ = (next_channel_ + 1) % NUM_CHANNELS;
    }

    // CS to first SCLK edge = 10ns minimum
    CS::low();
    delay_ns(10);


    // Shift out command with DMA (since it is 32 bits)
    uint16_t command = ADS8684_MAN_CH0 + (next_channel_ << 10);
    uint8_t data[4];
    data[0] = (command >> 8);
    data[1] = (command & 0xff);
    data[2] = 0;
    data[3] = 0;
    write_dma.write(data, 4);
    ++cycles_;
  }

  inline void irqHandler()
  {
    // Update DMA here (to clear interrupt flag)
    write_dma.done();
  }

protected:
  uint16_t raw_values_[NUM_CHANNELS];

  DMAw write_dma;
  DMAr read_dma;

  uint8_t channel_power_;
  uint8_t prev_channel_, next_channel_;

  uint32_t cycles_;
  uint32_t errors_;
};

#endif  // ADS8684_HPP
