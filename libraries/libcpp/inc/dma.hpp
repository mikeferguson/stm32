/*
 * Copyright (c) 2013-2014, Unbounded Robotics
 * Copyright (c) 2013, Michael E. Ferguson
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
 * Drivers for standard DMA read/write
 */

#ifndef _STM32_CPP_DMA_HPP_
#define	_STM32_CPP_DMA_HPP_

#include "stm32f4xx_dma.h"

/**
 *  \brief Generic templated class for reading to memory to a device.
 *  \tparam T The type of data being transmitted, usually uint8_t or uint16_t.
 *  \tparam DMA The DMA stream base, for instance DMA1_Stream5_BASE.
 *  \tparam STREAM_FLAG The stream flag, for instance DMA_FLAG_TCIF5.
 *  \tparam CHANNEL The channel, fo rinstance DMA_Channel_4.
 *  \tparam PERIPH_ADDR The address of the peripherial to read from.
 *  \tparam BUFFER_SIZE Size of internal buffer to read into.
 *
 *  DMA, STREAM_FLAG, CHANNEL should all be something defined in stm32f4xx_dma.h,
 *  these are all register mappings (for instance, "4" is not a valid channel
 *  option, but DMA_Channel_4 is).
 */
template <typename T, uint32_t DMA, uint32_t STREAM_FLAG, uint32_t CHANNEL, uint32_t PERIPH_ADDR, int BUFFER_SIZE>
class PeriphReadDMA
{
public:
  PeriphReadDMA() : head_(0), tail_(0) {}

  void init(uint32_t priority = DMA_Priority_High)
  {
    DMA_Cmd(reinterpret_cast<DMA_Stream_TypeDef*>(DMA), DISABLE);
    DMA_InitTypeDef DMA_InitStructure;
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel = CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = PERIPH_ADDR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &buffer_[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    if (sizeof(T) == 1)
    {
      DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
      DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    }
    else if (sizeof(T) == 2)
    {
      DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
      DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    }
    else if (sizeof(T) == 4)
    {
      DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
      DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    }
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = priority;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = 0;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(reinterpret_cast<DMA_Stream_TypeDef*>(DMA), &DMA_InitStructure);
    DMA_Cmd(reinterpret_cast<DMA_Stream_TypeDef*>(DMA), ENABLE);
  }

  /**
   *  \brief Read next byte from the buffer, if any exists.
   *  \returns -1 if nothing in buffer.
   */
  T read()
  {
    /* Check the status of the dma */
    head_ = (BUFFER_SIZE - (uint16_t)(reinterpret_cast<DMA_Stream_TypeDef*>(DMA)->NDTR))%BUFFER_SIZE;

    /* Get data to return */
    T data = (T) -1;
    if(head_ != tail_){
      data = buffer_[tail_];
      tail_ = (tail_+1)%BUFFER_SIZE;
    }

    return data;
  }

  /** 
   * \brief Returns true if there is curretly no data in the read buffer
   */
  bool isEmpty()
  {
    uint32_t head = (BUFFER_SIZE - (uint16_t)(reinterpret_cast<DMA_Stream_TypeDef*>(DMA)->NDTR))%BUFFER_SIZE;
    return (head == tail_);
  }

  /** 
   * \brief Clear data in buffer (resetting buffer)
   */
  void clearReadBuffer()
  {
    reinterpret_cast<DMA_Stream_TypeDef*>(DMA)->CR &= ~(uint32_t)DMA_SxCR_EN;  // Disable    
    head_ = 0;
    tail_ = 0;
    reinterpret_cast<DMA_Stream_TypeDef*>(DMA)->CR |= (uint32_t)DMA_SxCR_EN;   // Enable
  }

private:
  T buffer_[BUFFER_SIZE];
  uint32_t head_;
  uint32_t tail_;
};

/**
 *  \brief Generic templated class for writing a buffer from memory to a device.
 *  \tparam T The type of data being transmitted, usually uint8_t or uint16_t.
 *  \tparam DMA The DMA stream base, for instance DMA1_Stream5_BASE.
 *  \tparam STREAM_FLAG The stream flag, for instance DMA_FLAG_TCIF5.
 *  \tparam CHANNEL The channel, fo rinstance DMA_Channel_4.
 *  \tparam PERIPH_ADDR The address of the peripherial to write to.
 *  \tparam BUFFER_SIZE Size of internal buffer to write from.
 *
 *  DMA, STREAM_FLAG, CHANNEL should all be something defined in stm32f4xx_dma.h,
 *  these are all register mappings (for instance, "4" is not a valid channel
 *  option, but DMA_Channel_4 is).
 */
template <typename T, uint32_t DMA, uint32_t STREAM_FLAG, uint32_t CHANNEL, uint32_t PERIPH_ADDR, uint32_t BUFFER_SIZE>
class PeriphWriteDMA
{
public:
  /**
   *  \brief Initialize the DMA.
   *  \param priority The priority to assign to the DMA, should be selected from
   *         the DMA_Priority_X values found in stm32f4xx_dma.h
   */
  void init(uint32_t priority = DMA_Priority_High)
  {
    writing_ = false;
    DMA_Cmd(reinterpret_cast<DMA_Stream_TypeDef*>(DMA), DISABLE);
    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_Channel = CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = PERIPH_ADDR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &buffer_[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    if (sizeof(T) == 1)
    {
      DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
      DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    }
    else if (sizeof(T) == 2)
    {
      DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
      DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    }
    else if (sizeof(T) == 4)
    {
      DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
      DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    }
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = priority;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = 0;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(reinterpret_cast<DMA_Stream_TypeDef*>(DMA), &DMA_InitStructure);
  }

  /**
   *  \brief Start write of data stored in internal buffer to a peripherial, using this DMA.
   *  \param len Length of data to transfer to dma.
   */
  void startWrite(uint8_t len)
  {
    writing_ = true;
    reinterpret_cast<DMA_Stream_TypeDef*>(DMA)->CR &= ~(uint32_t)DMA_SxCR_EN;  // Disable
    reinterpret_cast<DMA_Stream_TypeDef*>(DMA)->NDTR = len;                    // Set length
    DMA_ClearFlag(reinterpret_cast<DMA_Stream_TypeDef*>(DMA), STREAM_FLAG);
    reinterpret_cast<DMA_Stream_TypeDef*>(DMA)->CR |= (uint32_t)DMA_SxCR_EN;   // Enable
  }

  /**
   *  \brief Write a buffer to a peripherial, using this DMA.
   *  \param data Pointer to buffer to write.
   *  \param len Length of buffer.
   */
  bool write(T * data, uint8_t len)
  {
    writing_ = true;
    for (int i = 0; i < len; ++i)
      buffer_[i] = data[i];
    startWrite(len);
    return writing_;
  }

  /**
   *  \brief Updates the status of whether the DMA has transferred all data.
   *  \returns True if DMA is write is complete.
   */
  bool done()
  {
    if (writing_)
    {
      if (DMA_GetFlagStatus(reinterpret_cast<DMA_Stream_TypeDef*>(DMA), STREAM_FLAG))
      {
        DMA_ClearFlag(reinterpret_cast<DMA_Stream_TypeDef*>(DMA), STREAM_FLAG);
        writing_ = false;
      }
    }
    return !writing_;
  }

  /**
   * \brief returns number of T objects buffer can hold (as opposed to bytes) 
   */
  inline uint32_t bufferSize()
  {
    return BUFFER_SIZE;
  }

  /** 
   * \brief returns a pointer to internal buffer that data is sent from
   */
  T* getBuffer()
  {
    return buffer_;
  }

private:
  T buffer_[BUFFER_SIZE];
  bool writing_;
};

#endif  // _STM32_CPP_DMA_HPP_
