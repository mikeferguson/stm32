/*
 * Copyright (c) 2013-2014, Unbounded Robotics Inc.
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

#include <stm32f4xx.h>
#include <stm32f4xx_spi.h>
#include <delay.hpp>

/**
 *  \brief Driver for ST LIS3DH & L3GD20 on same SPI bus, each with own CS.
 *  \tparam SPIx The base address of the SPI device, for instance, SPI1_BASE.
 *  \tparam GryoCs The gyro chip select IO pin.
 *  \tparam AccelCs The accelerometer chip select IO pin.
 *
 *  Example:
 *  \code
 *  typedef Gpio<GPIOA_BASE,5> imu_sck;  // SPI1
 *  typedef Gpio<GPIOA_BASE,6> imu_miso;
 *  typedef Gpio<GPIOB_BASE,5> imu_mosi;
 *  typedef Gpio<GPIOA_BASE,12> gyro_cs;
 *  typedef Gpio<GPIOC_BASE,9> accel_cs;
 *
 *  IMU<SPI1_BASE, gyro_cs, accel_cs> imu;
 *
 *  int main(void)
 *  {
 *       // Setup IMU
 *       RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
 *       imu_sck::mode(GPIO_ALTERNATE | GPIO_AF_SPI1);
 *       imu_miso::mode(GPIO_ALTERNATE | GPIO_AF_SPI1);
 *       imu_mosi::mode(GPIO_ALTERNATE | GPIO_AF_SPI1);
 *       imu.init();
 *
 *       // Enable the gyro and accelerometer
 *       imu.startGyro();
 *       imu.startAccel();
 *
 *       // Use IMU
 *       uint16_t x, y, z;
 *       imu.readAccel(&x, &y, &z);
 *  }
 *  \endcode
 */
template<unsigned int SPIx, typename GyroCs, typename AccelCs>
class IMU
{
  /* Bit 0 (MSB) is Read/Write select. */
  const static uint16_t WRITE = (0<<7);
  const static uint16_t READ = (1<<7);
  /* Bit 1 is MS, we are always going to increment. */
  const static uint16_t MS = (1<<6);

  /*
   * These register defs are used by both accel/gyro
   */
  const static uint8_t WHO_AM_I = 0x0F;
  const static uint8_t CTRL_REG1 = 0x20;
  const static uint8_t CTRL_REG2 = 0x21;
  const static uint8_t CTRL_REG3 = 0x22;
  /* Bit 6 is Big/Little Endian selection (default is 0=LSB@loweraddress)
   * Bit 5:4 is FS1-FS0 = Full scale section, default is 00 = +/-2G or 250dps
   */
  const static uint8_t CTRL_REG4 = 0x23;
  const static uint8_t CTRL_REG5 = 0x24;
  const static uint8_t CTRL_REG6 = 0x25;
  const static uint8_t OUT_X_L = 0x28;
  const static uint8_t OUT_Y_L = 0x2A;
  const static uint8_t OUT_Z_L = 0x2C;

  /*
   * Accelerometer defs
   */
  const static uint8_t ACCEL_ADC1_L = 0x08;
  const static uint8_t ACCEL_ADC2_L = 0x0A;
  const static uint8_t ACCEL_ADC3_L = 0x0C;
  /* Bit 7 is ADC enable, needs to be written to 1.
   * Bit 6 is Temperature sensor enable, needs to be written to 1 to enable.
   */
  const static uint8_t ACCEL_TMP_CFG_REG = 0x1f;

  const static uint8_t CTRL_REG4_FULL_SCALE_2G = 0 << 4;
  const static uint8_t CTRL_REG4_FULL_SCALE_4G = 1 << 4;
  const static uint8_t CTRL_REG4_FULL_SCALE_8G = 2 << 4;
  const static uint8_t CTRL_REG4_FULL_SCALE_16G = 3 << 4;

  /*
   * Gyro defs
   */
  const static uint8_t GYRO_TEMP = 0x26;
  const static uint8_t GYRO_STATUS = 0x27;

  const static uint8_t CTRL_REG4_FULL_SCALE_250DPS = 0 << 4;
  const static uint8_t CTRL_REG4_FULL_SCALE_500DPS = 1 << 4;
  const static uint8_t CTRL_REG4_FULL_SCALE_2000DPS = 2 << 4;


public:
  void init(void)
  {
    /* Setup I/O */
    GyroCs::mode(GPIO_OUTPUT);
    GyroCs::high();
    AccelCs::mode(GPIO_OUTPUT);
    AccelCs::high();

    /* Setup SPIx as follows:
     *  CPOL = 1 : clock is high when idle
     *  CPHA = 1 : sample is taken on trailing (rising) edge of clk
     */
    SPI_InitTypeDef SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    /*  Max SCLK = 10MHz, 168MHz/128 ~ 1.3MHz */
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(reinterpret_cast<SPI_TypeDef*>(SPIx), &SPI_InitStructure);

    SPI_Cmd(reinterpret_cast<SPI_TypeDef*>(SPIx), ENABLE);
  }

  /**
   *  \brief Read from device (chip-select should be managed outside this function).
   *  \param addr The address in the register table to read from.
   *  \param data Buffer to fill with data
   *  \param size The size of register to read, in bytes.
   */
  void read(const uint8_t addr, uint8_t * data, const uint8_t size)
  {
    /* enforce minimum time for CS setup time (datasheet says 6ns minimum). */
    delay_ns(10);

    uint16_t d = READ | MS | (addr & 0x3f);

    /* Send address to read from. */
    SPI_I2S_SendData(reinterpret_cast<SPI_TypeDef*>(SPIx), d);
    while (SPI_I2S_GetFlagStatus(reinterpret_cast<SPI_TypeDef*>(SPIx), SPI_I2S_FLAG_RXNE) == RESET);
    d = reinterpret_cast<SPI_TypeDef*>(SPIx)->DR;

    for (d = 0; d < size; ++d)
    {
      /* Force a read */
      SPI_I2S_SendData(reinterpret_cast<SPI_TypeDef*>(SPIx), 0);
      while (SPI_I2S_GetFlagStatus(reinterpret_cast<SPI_TypeDef*>(SPIx), SPI_I2S_FLAG_RXNE) == RESET);
      data[d] = (reinterpret_cast<SPI_TypeDef*>(SPIx)->DR)&0xff;
    }
  }

  /**
   *  \brief Write to device (chip-select should be managed outside this function).
   *  \param addr The address in the register table to read from.
   *  \param data The data to write.
   *  \param size The number of bytes to write.
   */
  void write(const uint8_t addr, const uint8_t * data, const uint8_t size)
  {
    /* enforce minimum time for CS setup time (datasheet says 6ns minimum). */
    delay_ns(10);
  
    uint16_t d = WRITE | MS | (addr & 0x3f);

    /* Send address to read from */
    SPI_I2S_SendData(reinterpret_cast<SPI_TypeDef*>(SPIx), d);
    while (SPI_I2S_GetFlagStatus(reinterpret_cast<SPI_TypeDef*>(SPIx), SPI_I2S_FLAG_RXNE) == RESET);
    d = reinterpret_cast<SPI_TypeDef*>(SPIx)->DR;

    for (d = 0; d < size; ++d)
    {
      SPI_I2S_SendData(reinterpret_cast<SPI_TypeDef*>(SPIx), data[d]);
      while (SPI_I2S_GetFlagStatus(reinterpret_cast<SPI_TypeDef*>(SPIx), SPI_I2S_FLAG_RXNE) == RESET);
      volatile uint16_t t = reinterpret_cast<SPI_TypeDef*>(SPIx)->DR;
    }
  }

  /** \brief Turn on ADCs on the Accelerometer. */
  void enableAccelADC()
  {
    AccelCs::low();
    uint8_t ADC_ON = (1<<7);
    write(ACCEL_TMP_CFG_REG, &ADC_ON, 1);
    AccelCs::high();
  }

  void readAccel(uint16_t * x, uint16_t * y, uint16_t * z)
  {
    uint16_t b[3];
    AccelCs::low();
    read(OUT_X_L, reinterpret_cast<uint8_t*>(&b[0]), 6);
    AccelCs::high();
    *x = b[0];
    *y = b[1];
    *z = b[2];
  }

  /**
   *  \brief Read the accelerometer ADC
   *  \param channel The channel to read, starting at 1.
   *  \returns a 10-bit read, or -1 if unable to read.
   */
  uint16_t readAccelADC(const uint8_t channel)
  {
    if (channel < 1 || channel > 3) return 0;

    uint16_t b;
    AccelCs::low();
    read(ACCEL_ADC1_L + 2 * channel, reinterpret_cast<uint8_t*>(&b), 2);
    AccelCs::high();
    return b;
  }

  void readGyro(uint8_t * temp, uint8_t * sr, uint16_t * x, uint16_t * y, uint16_t * z)
  {
    uint16_t b[4];
    GyroCs::low();
    read(GYRO_TEMP, reinterpret_cast<uint8_t*>(&b[0]), 8);
    GyroCs::high();
    *temp = (b[0]&0xff);
    *sr = (b[0]>>8);
    *x = b[1];
    *y = b[2];
    *z = b[3];
  }

  void startAccel()
  {
    uint8_t POWER_ON_NORMAL = 0x7f;
    AccelCs::low();
    write(CTRL_REG1, &POWER_ON_NORMAL, 1);
    AccelCs::high();
  }

  void startGyro()
  {
    uint8_t POWER_ON_760HZBW_100HZCUTOFF = 0xff;
    GyroCs::low();
    write(CTRL_REG1, &POWER_ON_760HZBW_100HZCUTOFF, 1);
    GyroCs::high();
    delay_ns(10);
    // 70mdps/digit = 2000dps max
    uint8_t FS2000DPS_LE_CONTINUOUS = 0x30;
    GyroCs::low();
    write(CTRL_REG4, &FS2000DPS_LE_CONTINUOUS, 1);
    GyroCs::high();
  }

  /** \brief Check WHO_AM_I register, make sure value is correct. */
  bool isAccelOK()
  {
    uint8_t who_am_i;
    AccelCs::low();
    read(WHO_AM_I, &who_am_i, 1);
    AccelCs::high();
    return who_am_i == 51;
  }

  /** \brief Check WHO_AM_I register, make sure value is correct. */
  bool isGyroOK()
  {
    uint8_t who_am_i;
    GyroCs::low();
    read(WHO_AM_I, &who_am_i, 1);
    GyroCs::high();
    return who_am_i == 212;
  }

};
