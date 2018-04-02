/*
 * Copyright (c) 2014-2018, Michael E. Ferguson
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

#ifndef _STM32_CPP_MINI_IMU9_H_
#define _STM32_CPP_MINI_IMU9_H_

#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dma.h"
#include "delay.hpp"

/* LSM6DS33 provides Gyro & Accel on v5 */
enum LSM6DS33_CONFIG
{
  LSM6DS33_DEVICE_ID =          0xD6,
  LSM6DS33_WHO_AM_I =           0x69,  // This is return value, not address

  LSM6DS33_CTRL1_XL =           0x10,
  LSM6DS33_CTRL2_G =            0x11,
  LSM6DS33_CTRL3_C =            0x12,
  LSM6DS33_CTRL4_C =            0x13,
  LSM6DS33_CTRL5_C =            0x14,
  LSM6DS33_CTRL6_C =            0x15,
  LSM6DS33_CTRL7_G =            0x16,
  LSM6DS33_CTRL8_XL =           0x17,
  LSM6DS33_CTRL9_XL =           0x18,
  LSM6DS33_CTRL10_C =           0x19,

  LSM6DS33_OUT_TEMP_L =         0x20,
  LSM6DS33_OUT_TEMP_H =         0x21,
  LSM6DS33_OUTX_L_G =           0x22,
  LSM6DS33_OUTX_H_G =           0x23,
  LSM6DS33_OUTY_L_G =           0x24,
  LSM6DS33_OUTY_H_G =           0x25,
  LSM6DS33_OUTZ_L_G =           0x26,
  LSM6DS33_OUTZ_H_G =           0x27,
  LSM6DS33_OUTX_L_XL =          0x28,
  LSM6DS33_OUTX_H_XL =          0x29,
  LSM6DS33_OUTY_L_XL =          0x2A,
  LSM6DS33_OUTY_H_XL =          0x2B,
  LSM6DS33_OUTZ_L_XL =          0x2C,
  LSM6DS33_OUTZ_H_XL =          0x2D,
};

/* LIS3MDL provides Mag on v5 */
enum LIS3MDL_CONFIG
{
  LIS3MDL_DEVICE_ID =           0x3C,
  LIS3MDL_WHO_AM_I =            0x4D,  // This is return value, not address

  LIS3MDL_CTRL_REG1 =           0x20,
  LIS3MDL_CTRL_REG2 =           0x21,
  LIS3MDL_CTRL_REG3 =           0x22,
  LIS3MDL_CTRL_REG4 =           0x23,
  LIS3MDL_CTRL_REG5 =           0x24,

  LIS3MDL_OUT_X_L =             0x28,
  LIS3MDL_OUT_X_H =             0x29,
  LIS3MDL_OUT_Y_L =             0x2A,
  LIS3MDL_OUT_Y_H =             0x2B,
  LIS3MDL_OUT_Z_L =             0x2C,
  LIS3MDL_OUT_Z_H =             0x2D,
  LIS3MDL_TEMP_OUT_L =          0x2E,
  LIS3MDL_TEMP_OUT_H =          0x2F,
};

/* L3GD20H provides Gyro on v3 */
enum L3GD20H_CONFIG
{
  L3GD20H_DEVICE_ID =            0xD6,
  L3GD20H_WHO_AM_I =             0xD7,  // This is return value, not address

  L3GD20H_CTRL_REG1 =            0x20,
  L3GD20H_CTRL_REG2 =            0x21,
  L3GD20H_CTRL_REG3 =            0x22,
  L3GD20H_CTRL_REG4 =            0x23,
  L3GD20H_CTRL_REG5 =            0x24,
  L3GD20H_REFERENCE =            0x25,

  L3GD20H_TEMP_OUT  =            0x26,
  L3GD20H_STATUS_REG =           0x27,
  L3GD20H_OUTX_L_G =             0x28,
  L3GD20H_OUTX_H_G =             0x29,
  L3GD20H_OUTY_L_G =             0x2A,
  L3GD20H_OUTY_H_G =             0x2B,
  L3GD20H_OUTZ_L_G =             0x2C,
  L3GD20H_OUTZ_H_G =             0x2D,
};

/* LSM303D provides Accel/Mag on v3 */
enum LSM303D_CONFIG
{
  LSM303D_DEVICE_ID =           0x3A,
  LSM303D_WHO_AM_I =            0x49,  // This is return value, not address

  LSM303D_TEMP_OUT_L =          0x05,
  LSM303D_TEMP_OUT_H =          0x06,
  LSM303D_STATUS_M =            0x07,
  LSM303D_OUTX_L_M =            0x08,
  LSM303D_OUTX_H_M =            0x09,
  LSM303D_OUTY_L_M =            0x0A,
  LSM303D_OUTY_H_M =            0x0B,
  LSM303D_OUTZ_L_M =            0x0C,
  LSM303D_OUTZ_H_M =            0x0D,

  LSM303D_CTRL_REG0 =           0x1F,
  LSM303D_CTRL_REG1 =           0x20,
  LSM303D_CTRL_REG2 =           0x21,
  LSM303D_CTRL_REG3 =           0x22,
  LSM303D_CTRL_REG4 =           0x23,
  LSM303D_CTRL_REG5 =           0x24,
  LSM303D_CTRL_REG6 =           0x25,
  LSM303D_CTRL_REG7 =           0x26,

  LSM303D_STATUS_A =            0x27,
  LSM303D_OUTX_L_A =            0x28,
  LSM303D_OUTX_H_A =            0x29,
  LSM303D_OUTY_L_A =            0x2A,
  LSM303D_OUTY_H_A =            0x2B,
  LSM303D_OUTZ_L_A =            0x2C,
  LSM303D_OUTZ_H_A =            0x2D,
};

/* L3GD20 provides Gyro on v2 */
enum L3GD20_CONFIG
{
  L3GD20_DEVICE_ID =            0xD6,
  L3GD20_WHO_AM_I =             0xD4,  // This is return value, not address

  L3GD20_CTRL_REG1 =            0x20,
  L3GD20_CTRL_REG2 =            0x21,
  L3GD20_CTRL_REG3 =            0x22,
  L3GD20_CTRL_REG4 =            0x23,
  L3GD20_CTRL_REG5 =            0x24,
  L3GD20_REFERENCE =            0x25,

  L3GD20_TEMP_OUT  =            0x26,
  L3GD20_STATUS_REG =           0x27,
  L3GD20_OUTX_L_G =             0x28,
  L3GD20_OUTX_H_G =             0x29,
  L3GD20_OUTY_L_G =             0x2A,
  L3GD20_OUTY_H_G =             0x2B,
  L3GD20_OUTZ_L_G =             0x2C,
  L3GD20_OUTZ_H_G =             0x2D,
};

/* LSM303DHLC provides Accel & Mag on v2 */
enum LSM303DHLC_CONFIG
{
  LSM303DHLC_DEVICE_ID =        0x32,
  LSM303DHLC_WHO_AM_I =         0x49,  // This is return value, not address

  LSM303DHLC_TEMP_OUT_L =       0x05,
  LSM303DHLC_TEMP_OUT_H =       0x06,

  LSM303DHLC_STATUS_A =         0x27,
  LSM303DHLC_OUTX_L_A =         0x28,
  LSM303DHLC_OUTX_H_A =         0x29,
  LSM303DHLC_OUTY_L_A =         0x2A,
  LSM303DHLC_OUTY_H_A =         0x2B,
  LSM303DHLC_OUTZ_L_A =         0x2C,
  LSM303DHLC_OUTZ_H_A =         0x2D,

  LSM303DHLC_CTRL_REG1_A =      0x20,
  LSM303DHLC_CTRL_REG2_A =      0x21,
  LSM303DHLC_CTRL_REG3_A =      0x22,
  LSM303DHLC_CTRL_REG4_A =      0x23,
  LSM303DHLC_CTRL_REG5_A =      0x24,
  LSM303DHLC_CTRL_REG6_A =      0x25,

  // Magnetometer has it's own address and table
  LSM303DHLC_DEVICE_ID_M =      0x3C,

  LSM303DHLC_CRA_REG_M =        0x00,
  LSM303DHLC_CRB_REG_M =        0x01,
  LSM303DHLC_MR_REG_M =         0x02,
  // NOTE: Order is different than anything else!
  LSM303DHLC_OUTX_H_M =         0x03,
  LSM303DHLC_OUTX_L_M =         0x04,
  LSM303DHLC_OUTZ_H_M =         0x05,
  LSM303DHLC_OUTZ_L_M =         0x06,
  LSM303DHLC_OUTY_H_M =         0x07,
  LSM303DHLC_OUTY_L_M =         0x08,
};

/*
 * This is currently tuned for 168mhz STM32F4 and 100khz I2C,
 * if used on a slower processor or a faster bus speed, it will
 * probably be excessively long for the timeout.
 */
#define IMU_FLAG_TIMEOUT        ((uint32_t)0x80*6)

#define VERSION_2_L3GD20_LSM303DLHC   2
#define VERSION_3_L3GD20H_LSM303D     3
#define VERSION_5_LSM6DS33_LIS3MDL    5

/**
 *  \brief Driver for Pololu MiniIMU-9, supports the following versions:
 *           v2 with ST L3GD20 and LSM303DLHC
 *           v3 with ST L3GD20H and LSM303D (untested)
 *           v5 with ST LSM6DS33 and LIS3MDL
 *  \tparam I2C The I2C device, for instance I2C1.
 *  \tparam DMA The stream device, for instance DMA1_Stream3_BASE.
 *  \tparam STREAM The stream number (3 for DMA1_Stream3_BASE).
 *  \tparam CHANNEL The channel number (0-7).
 *  \tparam SCL The GPIO used for SCL.
 *  \tparam SDA The GPIO used for SDA.
 *
 * Settings selected by the driver (closely match popular UM7):
 *  - Accelerometer is +/- 8g
 *  - Gyro is full scale, +/-2000degrees/s, or 70millidegrees/digit
 *  - Magnetometer is +/-12 gauss (except v2, which maxes out at 8.1)
 *
 * Example:
 * \code
 * // Example definitions for using I2C2, DMA1, Stream 3, Channel 7:
 * typedef Gpio<GPIOB_BASE,11> SDA;
 * typedef Gpio<GPIOB_BASE,10> SCL;
 * MiniImu9<I2C2_BASE,
 *          DMA1_Stream3_BASE,
 *          3, // DMA STREAM
 *          7, // DMA_CHANNEL
 *          SCL
 *          SDA> imu;
 *
 * // In the beginning of your main, initialize the IMU with the I2C speed.
 * imu.init(100000);
 *
 * // At periodic intervals, call update with the current millisecond clock.
 * imu.update(time_in_ms);
 * \endcode
 */
template<int I2C, int DMA, int STREAM, int CHANNEL, typename SCL, typename SDA>
class MiniImu9
{
  /** \brief Timeouts, in milliseconds */
  enum
  {
    GYRO_READ_TIMEOUT  = 6,
    GYRO_DELAY_TIME    = 2,
    ACCEL_READ_TIMEOUT = 4,
    ACCEL_DELAY_TIME   = 2,
    MAG_READ_TIMEOUT   = 4,
    MAG_DELAY_TIME     = 2
  };

  /** \brief Possible states for the IMU updates */
  enum imu_state_t
  {
    IMU_IDLE          = 0,  // initital state

    IMU_DELAY_GYRO    = 1,  // waiting a bit before starting gyro read
    IMU_READING_GYRO  = 2,  // waiting for gyro read to complete
    IMU_RESTART_GYRO  = 3,  // tries to intialize gyro, will return to
                            //   this state if there is a gyro read error

    IMU_DELAY_ACCEL   = 4,  // waiting a bit before starting accelerometer read
    IMU_READING_ACCEL = 5,  // waiting for accelerometer to read to complete
    IMU_RESTART_ACCEL = 6,  // tries to initialize accel, will return to this
                            //   state if there is accel read error

    IMU_DELAY_MAG     = 7,  // waiting a bit before starting magnetometer read
    IMU_READING_MAG   = 8,  // waiting for magnetometer read to return
    IMU_RESTART_MAG   = 9   // tries to initialize magnetometer, will return
                            //   to this state if there is a read error
  };

  /** \brief Types of i2c errors */
  enum i2c_error_t
  {
    I2C_ERROR_START_BIT         = 1,  // error occurred while sending i2c start
    I2C_ERROR_DEVICE_ADDR_WRITE = 2,  // error occurred while sending i2c device address
    I2C_ERROR_REG_ADDR_WRITE    = 3,  // error occurred while writing register address to device
    I2C_ERROR_DATA_WRITE        = 4,  // error occurred while writing data to device
    I2C_ERROR_TIMEOUT           = 5   // general timeout error from mainloop of
                                      //   device (most likely occurring during DMA)
  };

  /** \brief Types of IMU operations that can cause an error */
  enum imu_error_op_t
  {
    IMU_ERROR_WRITE_REG  = 1,  // error occurred while writing byte to register
    IMU_ERROR_WRITE_ADDR = 2,  // error occurred while writing register address
    IMU_ERROR_READ_REG   = 3,  // error occurred while reading register data
    IMU_ERROR_READ_DMA   = 4   // error occurred while reading register data DMA
  };

public:
  /** \brief Data read from accelerometer. */
  struct accel_data_t
  {
    int16_t x;
    int16_t y;
    int16_t z;
  };

  /** \brief Data read from gyro. */
  struct gyro_data_t
  {
    int8_t temp;
    uint8_t status;
    int16_t x;
    int16_t y;
    int16_t z;
  };

  /** \brief Data read from magnetometer. */
  struct mag_data_t
  {
    int16_t x;
    int16_t y;
    int16_t z;
  };

  /** \brief Accelerometer data read by the DMA */
  accel_data_t accel_data;

  /** \brief Gyro data read by the DMA */
  gyro_data_t gyro_data;

  /** \brief Magnetometer data read by the DMA */
  mag_data_t mag_data;

  /**
   *  \brief Initialize the I2C and DMA.
   *  \param clock_speed The clock rate for the I2C bus.
   */
  void init(uint32_t clock_speed)
  {
    version_ = 0;
    clock_speed_ = clock_speed;
    use_mag_ = false;  // do not use magnetometer by default

    I2C_TypeDef* const I2Cx = (I2C_TypeDef*) I2C;

    state_ = IMU_IDLE;

    /* Enable the proper I2C */
    if (I2C == I2C1_BASE)
    {
      RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
      SCL::mode(GPIO_ALTERNATE_OD_2MHz | GPIO_AF_I2C1);
      SDA::mode(GPIO_ALTERNATE_OD_2MHz | GPIO_AF_I2C1);
    }
    else if(I2C == I2C2_BASE)
    {
      RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
      SCL::mode(GPIO_ALTERNATE_OD_2MHz | GPIO_AF_I2C2);
      SDA::mode(GPIO_ALTERNATE_OD_2MHz | GPIO_AF_I2C2);
    }
    // Technically, the MINI-IMU9 has pull up resistors --
    // but they are connected to VIN, not VDD, so they aren't
    // pulling anything up
    SCL::pullup();
    SDA::pullup();

    /* Enable the proper DMA */
    if (DMA < DMA2_BASE)
      RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    else
      RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    /* Setup I2C */
    I2C_DeInit(I2Cx);
    I2C_InitTypeDef I2C_InitStructure;
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_ClockSpeed = clock_speed_;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2Cx, &I2C_InitStructure);
    I2C_Cmd(I2Cx, ENABLE);
  }

  /**
   *  \brief Should we read from the magnetometer.
   *  \param read_mag Should we read the magnetometer.
   */
  void use_magnetometer(bool read_mag)
  {
    use_mag_ = read_mag;
  }

  /**
   *  \brief Returns the version of the IMU (v2, 3, or 5)
   *         Will return 0 if no IMU is present
   */
  uint8_t get_version()
  {
    return version_;
  }

  /**
   *  \brief Handle timeout, starting of next DMA transfer.
   *  \param clock System clock time in milliseconds, used for timeouts.
   *  \returns True if new data has been transferred in this time step.
   */
  bool update(uint64_t clock)
  {
    DMA_Stream_TypeDef* const DMAy_Streamx = (DMA_Stream_TypeDef*) DMA;

    uint32_t flag_tc;
    switch(STREAM)
    {
      case 0 :flag_tc = DMA_FLAG_TCIF0; break;
      case 1 :flag_tc = DMA_FLAG_TCIF1; break;
      case 2 :flag_tc = DMA_FLAG_TCIF2; break;
      case 3 :flag_tc = DMA_FLAG_TCIF3; break;
      case 4 :flag_tc = DMA_FLAG_TCIF4; break;
      case 5 :flag_tc = DMA_FLAG_TCIF5; break;
      case 6 :flag_tc = DMA_FLAG_TCIF6; break;
      case 7 :flag_tc = DMA_FLAG_TCIF7; break;
    }

    if (state_ == IMU_DELAY_ACCEL)
    {
      /* Wait a bit before starting accelerometer read */
      if (time_diff(clock, timer_) > ACCEL_DELAY_TIME)
      {
        if (start_accel_read())
        {
          state_ = IMU_READING_ACCEL;
          timer_ = clock;
        }
        else  /* Failed, try to restart */
          state_ = IMU_RESTART_ACCEL;
      }
    }
    else if (state_ == IMU_READING_ACCEL)
    {
      if (DMA_GetFlagStatus(DMAy_Streamx, flag_tc))
      {
        imu_finish_read();
        /* Read successful */
        accel_data = accel_buffer_;
        ++num_accel_updates_;
        /* Setup next cycle */
        timer_ = clock;
        state_ = IMU_DELAY_GYRO;
        return true;
      }
      else if (time_diff(clock, timer_) > ACCEL_READ_TIMEOUT)
      {
        state_ = IMU_RESTART_ACCEL;
        return imu_timeout_callback(I2C_ERROR_TIMEOUT, IMU_ERROR_READ_DMA);
      }
    }
    else if (state_ == IMU_RESTART_ACCEL)
    {
      configure_accelerometer();
      state_ = IMU_DELAY_GYRO;
      timer_ = clock;
    }
    else if (state_ == IMU_DELAY_GYRO)
    {
      /* Wait a bit before starting gyro read */
      if (time_diff(clock, timer_) > GYRO_DELAY_TIME)
      {
        if (start_gyro_read())
        {
          state_ = IMU_READING_GYRO;
          timer_ = clock;
        }
        else  /* Failed, try to restart */
          state_ = IMU_RESTART_GYRO;
      }
    }
    else if (state_ == IMU_READING_GYRO)
    {
      if (DMA_GetFlagStatus(DMAy_Streamx, flag_tc))
      {
        imu_finish_read();
        /* Read successful */
        gyro_data = gyro_buffer_;
        ++num_gyro_updates_;
        /* Setup next cycle */
        timer_ = clock;
        if (use_mag_)
          state_ = IMU_DELAY_MAG;
        else
          state_ = IMU_DELAY_ACCEL;
        return true;
      }
      else if (time_diff(clock, timer_) > GYRO_READ_TIMEOUT)
      {
        state_ = IMU_RESTART_GYRO;
        return imu_timeout_callback(I2C_ERROR_TIMEOUT, IMU_ERROR_READ_DMA);
      }
    }
    else if (state_ == IMU_RESTART_GYRO)
    {
      configure_gyro();
      state_ = IMU_DELAY_ACCEL;
      timer_ = clock;
    }
    else if (state_ == IMU_DELAY_MAG)
    {
      /* Wait a bit before starting magnetometer read */
      if (time_diff(clock, timer_) > MAG_DELAY_TIME)
      {
        if (start_mag_read())
        {
          state_ = IMU_READING_MAG;
          timer_ = clock;
        }
        else  /* Failed, try to restart */
          state_ = IMU_RESTART_MAG;
      }
    }
    else if (state_ == IMU_READING_MAG)
    {
      if (DMA_GetFlagStatus(DMAy_Streamx, flag_tc))
      {
        imu_finish_read();
        /* Read successful */
        if (version_ == VERSION_2_L3GD20_LSM303DLHC)
        {
          // Order is different than anything else!
          mag_data.x = mag_buffer_[1] + (mag_buffer_[0]<<8);
          mag_data.z = mag_buffer_[3] + (mag_buffer_[2]<<8);
          mag_data.y = mag_buffer_[5] + (mag_buffer_[4]<<8);
        }
        else
        {
          mag_data.x = mag_buffer_[0] + (mag_buffer_[1]<<8);
          mag_data.y = mag_buffer_[2] + (mag_buffer_[3]<<8);
          mag_data.z = mag_buffer_[4] + (mag_buffer_[5]<<8);
        }
        ++num_mag_updates_;
        /* Setup next cycle */
        timer_ = clock;
        state_ = IMU_DELAY_ACCEL;
        return true;
      }
      else if (time_diff(clock, timer_) > MAG_READ_TIMEOUT)
      {
        state_ = IMU_RESTART_MAG;
        return imu_timeout_callback(I2C_ERROR_TIMEOUT, IMU_ERROR_READ_DMA);
      }
    }
    else if (state_ == IMU_RESTART_MAG)
    {
      configure_magnetometer();
      state_ = IMU_DELAY_ACCEL;
      timer_ = clock;
    }
    else
    {
      configure_accelerometer();
      configure_gyro();
      configure_magnetometer();
      state_ = IMU_DELAY_GYRO;
      timer_ = clock;
    }
    return false;
  }

private:
  /**
   *  \brief Handle a timeout event.
   *  \param error_i2c I2C error type.
   *  \param error_op What part of IMU operation the error occured in
   *  \returns false, always.
   */
  bool imu_timeout_callback(i2c_error_t error_i2c, imu_error_op_t error_op)
  {
    I2C_TypeDef* const I2Cx = (I2C_TypeDef*) I2C;
    DMA_Stream_TypeDef* const DMAy_Streamx = (DMA_Stream_TypeDef*) DMA;

    I2C_GenerateSTOP(I2Cx, DISABLE);

    /* Disable DMA, I2C */
    DMA_Cmd(DMAy_Streamx, DISABLE);
    I2C_DMACmd(I2Cx,DISABLE);
    DMA_ClearFlag(DMAy_Streamx, DMA_FLAG_TCIF7);

    /* Track errors */
    last_i2c_error_ = error_i2c;
    last_imu_error_ = error_op;
    ++num_timeouts_;

    /* Pull SCL low for 100us to force release SDA */
    SCL::mode(GPIO_OUTPUT_2MHz);
    SCL::low();
    delay_us(100);
    if (I2C == I2C1_BASE)
      SCL::mode(GPIO_ALTERNATE_OD_2MHz | GPIO_AF_I2C1);
    else if(I2C == I2C2_BASE)
      SCL::mode(GPIO_ALTERNATE_OD_2MHz | GPIO_AF_I2C2);

    /* Deinit and reinit the I2C */
    I2C_DeInit(I2Cx);
    I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = clock_speed_;
    I2C_Init(I2Cx, &I2C_InitStructure);
    I2C_Cmd(I2Cx, ENABLE);

    return false;
  }

  /**
   *  \brief Writes a single byte to register of I2C device.
   *  \param i2c_addr Address of device.
   *  \param reg_addr Address of register to write.
   *  \param data The byte of data to write.
   */
  bool imu_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data)
  {
    I2C_TypeDef* const I2Cx = (I2C_TypeDef*) I2C;

    /* Generate start bit */
    I2C_GenerateSTART(I2Cx, ENABLE);
    uint32_t timeout = IMU_FLAG_TIMEOUT;
    while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB))
    {
      if ((timeout--) == 0)
        return imu_timeout_callback(I2C_ERROR_START_BIT, IMU_ERROR_WRITE_REG);
    }

    /* Send device address */
    I2C_Send7bitAddress(I2Cx, i2c_addr, I2C_Direction_Transmitter);
    timeout = IMU_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
      if ((timeout--) == 0)
        return imu_timeout_callback(I2C_ERROR_DEVICE_ADDR_WRITE, IMU_ERROR_WRITE_REG);
    }

    /* Send register address */
    I2C_SendData(I2Cx, reg_addr);
    timeout = IMU_FLAG_TIMEOUT;
    while ((!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BTF)))
    {
      if ((timeout--) == 0)
        return imu_timeout_callback(I2C_ERROR_REG_ADDR_WRITE, IMU_ERROR_WRITE_REG);
    }

    /* Send data */
    I2C_SendData(I2Cx, data);
    timeout = IMU_FLAG_TIMEOUT;
    while ((!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BTF)))
    {
      if ((timeout--) == 0)
        return imu_timeout_callback(I2C_ERROR_DATA_WRITE, IMU_ERROR_WRITE_REG);
    }

    /* Send stop bit */
    I2C_GenerateSTOP(I2Cx, ENABLE);

    return true;
  }

  /**
   *  \brief Start a DMA based read.
   *  \param i2c_addr Address of the device to read from.
   *  \param reg_addr The register to start read at.
   *  \param buffer Memory location for DMA to transfer bytes to.
   *  \param num_bytes The number of bytes for the DMA to transfer.
   */
  bool imu_start_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t* buffer, uint8_t num_bytes)
  {
    I2C_TypeDef* const I2Cx = (I2C_TypeDef*) I2C;
    DMA_Stream_TypeDef* const DMAy_Streamx = (DMA_Stream_TypeDef*) DMA;

    uint32_t dma_channel = (CHANNEL & 7) << 25;

    /* Config DMA to move data from IC2 peripheral to memory */
    DMA_InitTypeDef DMA_InitStructure;
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel = dma_channel;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(I2Cx->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = num_bytes;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_DeInit(DMAy_Streamx);
    DMA_Init(DMAy_Streamx, &DMA_InitStructure);

    /* Generate start */
    I2C_GenerateSTART(I2Cx, ENABLE);
    uint32_t timeout = IMU_FLAG_TIMEOUT;
    while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB))
    {
      if ((timeout--) == 0)
        return imu_timeout_callback(I2C_ERROR_START_BIT, IMU_ERROR_WRITE_ADDR);
    }

    /* Write the i2c device address. */
    I2C_Send7bitAddress(I2Cx, i2c_addr, I2C_Direction_Transmitter);
    timeout = IMU_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
      if ((timeout--) == 0)
        return imu_timeout_callback(I2C_ERROR_DEVICE_ADDR_WRITE, IMU_ERROR_WRITE_ADDR);
    }

    /* Write register address to read from */
    I2C_SendData(I2Cx, reg_addr);
    timeout = IMU_FLAG_TIMEOUT;
    while ((!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BTF)))
    {
      if ((timeout--) == 0)
        return imu_timeout_callback(I2C_ERROR_REG_ADDR_WRITE, IMU_ERROR_WRITE_ADDR);
    }

    /* Start Second transfer to read data from selected register */
    I2C_GenerateSTART(I2Cx, ENABLE);
    timeout = IMU_FLAG_TIMEOUT;
    while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB))
    {
      if ((timeout--) == 0)
       return imu_timeout_callback(I2C_ERROR_START_BIT, IMU_ERROR_READ_REG);
    }

    /* Send I2C address of device and start I2C read transfer */
    I2C_Send7bitAddress(I2Cx, i2c_addr, I2C_Direction_Receiver);
    timeout = IMU_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
      if ((timeout--) == 0)
        return imu_timeout_callback(I2C_ERROR_DEVICE_ADDR_WRITE, IMU_ERROR_READ_REG);
    }

    /* Enable I2C DMA request */
    I2C_DMACmd(I2Cx,ENABLE);
    /* Enable DMA NACK */
    I2C_DMALastTransferCmd(I2Cx, ENABLE);
    /* Enable DMA RX Channel */
    DMA_Cmd(DMAy_Streamx, ENABLE);
    return true;
  }

  /**
   *  \brief DMA is done, shut things down
   */
  bool imu_finish_read(void)
  {
    I2C_TypeDef* const I2Cx = (I2C_TypeDef*) I2C;
    DMA_Stream_TypeDef* const DMAy_Streamx = (DMA_Stream_TypeDef*) DMA;

    /* Send STOP */
    I2C_GenerateSTOP(I2Cx, ENABLE);

    /* Disable DMA RX Channel */
    DMA_Cmd(DMAy_Streamx, DISABLE);

    /* Disable I2C DMA request */
    I2C_DMACmd(I2Cx,DISABLE);

    /* Clear DMA RX Transfer Complete Flag */
    uint32_t flag_tc;
    switch(STREAM)
    {
      case 0 :flag_tc = DMA_FLAG_TCIF0; break;
      case 1 :flag_tc = DMA_FLAG_TCIF1; break;
      case 2 :flag_tc = DMA_FLAG_TCIF2; break;
      case 3 :flag_tc = DMA_FLAG_TCIF3; break;
      case 4 :flag_tc = DMA_FLAG_TCIF4; break;
      case 5 :flag_tc = DMA_FLAG_TCIF5; break;
      case 6 :flag_tc = DMA_FLAG_TCIF6; break;
      case 7 :flag_tc = DMA_FLAG_TCIF7; break;
    }
    DMA_ClearFlag(DMAy_Streamx, flag_tc);

    return true;
  }

  /** \brief Get time differnece between two unsigned time values. */
  static int32_t time_diff(uint32_t t1, uint32_t t2)
  {
    /* This should handle wrap-around */
    return int32_t(t1-t2);
  }

  /** \brief Configure the accelerometer to +/-8g. */
  bool configure_accelerometer(void)
  {
    if (version_ == VERSION_5_LSM6DS33_LIS3MDL)
    {
      /* Enable Accelerometer via Control Register 1
       *  bits 7:4 = output data rate = 0110b (high performance (416hz))
       *  bit  3:2 = full scale = 11b = +/-8g
       *  bits 1:0 = anti-aliasing filter = 00b = 400hz
       */
      return imu_write(LSM6DS33_DEVICE_ID, LSM6DS33_CTRL1_XL, 0x6C);
    }
    else if (version_ == VERSION_3_L3GD20H_LSM303D)
    {
      /* Enable Accelerometer via Control Register 1
       *  bits 7:4 = data rate = 1000b (Normal / low-power mode (400hz))
       *  bit  3   = block data update = 1b
       *  bits 2:0 = Z/Y/X on = 111b
       */
      if (!imu_write(LSM303D_DEVICE_ID, LSM303D_CTRL_REG1, 0x8F))
        return false;

      /* Configure full scale via Control Register 2
       *   bits 5:3 = full scale = 011b = +/- 8g
       */
      return imu_write(LSM303D_DEVICE_ID, LSM303D_CTRL_REG2, 0x0C);
    }
    else if (version_ == VERSION_2_L3GD20_LSM303DLHC)
    {
      /* Enable Accelerometer via Control Register 1A
       *  bits 7:4 = data rate = 0111b (Normal / low-power mode (400hz))
       *  bit  3   = low power mode enabled = 0b
       *  bits 2:0 = Z/Y/X on = 111b
       */
      if (!imu_write(LSM303DHLC_DEVICE_ID, LSM303DHLC_CTRL_REG1_A, 0x77))
        return false;

      /* Configure data format via Control Register 4A
       *  bit  7   = block data update = 1b
       *  bits 5:4 = full scale selection = 10b = +/- 8g
       */
      return imu_write(LSM303DHLC_DEVICE_ID, LSM303DHLC_CTRL_REG4_A, 0x90);
    }
    // Unsupported version
    return false;
  }

  /** \brief Start reading from the accelerometer */
  bool start_accel_read()
  {
    if (version_ == VERSION_5_LSM6DS33_LIS3MDL)
    {
      return imu_start_read(LSM6DS33_DEVICE_ID, LSM6DS33_OUTX_L_XL, (uint8_t *) &accel_buffer_, sizeof(accel_buffer_));
    }
    else if (version_ == VERSION_3_L3GD20H_LSM303D)
    {
      /* Register address bits 6:0 are address, bit 7 is auto-increment (1 = auto_increment) */
      return imu_start_read(LSM303D_DEVICE_ID, LSM303D_OUTX_L_A | 0x80, (uint8_t *) &accel_buffer_, sizeof(accel_buffer_));
    }
    else if (version_ == VERSION_2_L3GD20_LSM303DLHC)
    {
      /* Register address bits 6:0 are address, bit 7 is auto-increment (1 = auto_increment) */
      return imu_start_read(LSM303DHLC_DEVICE_ID, LSM303DHLC_OUTX_L_A | 0x80, (uint8_t *) &accel_buffer_, sizeof(accel_buffer_));
    }
    // Unsupported version
    return false;
  }

  /** \brief Configure the gyro to +/- 2000dps. */
  bool configure_gyro(void)
  {
    if (version_ == VERSION_5_LSM6DS33_LIS3MDL)
    {
      /* Enable Gyro via Control Register 2
       *  bits 7:4 = output data rate = 0110b (416hz update rate)
       *  bits 3:2 = full scale = 11b = 2000dps
       */
      if (!imu_write(LSM6DS33_DEVICE_ID, LSM6DS33_CTRL2_G, 0x6C))
        return false;

      /* Configure Control Register 3
       *  bit  6   = block update (default: 0)
       *  bit  2   = register address updated automatically (default: 1)
       */
      if (!imu_write(LSM6DS33_DEVICE_ID, LSM6DS33_CTRL3_C, 0x44))
        return false;

      return true;
    }
    else if (version_ == VERSION_3_L3GD20H_LSM303D)
    {
      /* Enable Gyro via Control Register 1
       *  bits 7:6 = data rate = 10b (400hz update rate)
       *  bits 5:4 = bandwidth = 11b (110hz cutoff)
       *  bit  3   = power on = 1b
       *  bits 2:0 = Z/Y/X on = 111b
       */
      if (!imu_write(L3GD20H_DEVICE_ID, L3GD20H_CTRL_REG1, 0xBF))
        return false;

      /* Change Full Scale Selection
       *  bit  7   = block update (default: 0)
       *  bit  6   = lsb (0)
       *  bit  5:4 = full scale = 11b (2000dps)
       *                              (=70mdps/digit)
       */
      return imu_write(L3GD20H_DEVICE_ID, L3GD20H_CTRL_REG4, 0xE0);
    }
    else if (version_ == VERSION_2_L3GD20_LSM303DLHC)
    {
      /* Enable Gyro via Control Register 1
       *  bits 7:6 = data rate = 10b (380hz update rate)
       *  bits 5:4 = bandwidth = 11b (100hz cutoff)
       *  bit  3   = power on = 1b
       *  bits 2:0 = Z/Y/X on = 111b
       */
      if (!imu_write(L3GD20_DEVICE_ID, L3GD20_CTRL_REG1, 0xBF))
        return false;

      /* Change Full Scale Selection
       *  bit  7   = block update (default: 0)
       *  bit  6   = lsb (0)
       *  bit  5:4 = full scale = 11b (2000dps)
       *                              (=70mdps/digit)
       */
      return imu_write(L3GD20_DEVICE_ID, L3GD20_CTRL_REG4, 0xE0);
    }
    // Unsupported version
    return false;
  }

  /** \brief Start reading from the gyro */
  bool start_gyro_read()
  {
    if (version_ == VERSION_5_LSM6DS33_LIS3MDL)
    {
      return imu_start_read(LSM6DS33_DEVICE_ID, LSM6DS33_OUT_TEMP_L, (uint8_t *) &gyro_buffer_, sizeof(gyro_buffer_));
    }
    else if (version_ == VERSION_3_L3GD20H_LSM303D)
    {
      /* Register address bits 6:0 are address, bit 7 is auto-increment (1 = auto_increment) */
      return imu_start_read(L3GD20H_DEVICE_ID, L3GD20H_TEMP_OUT | 0x80, (uint8_t *) &gyro_buffer_, sizeof(gyro_buffer_));
    }
    else if (version_ == VERSION_2_L3GD20_LSM303DLHC)
    {
      /* Register address bits 6:0 are address, bit 7 is auto-increment (1 = auto_increment) */
      return imu_start_read(L3GD20_DEVICE_ID, L3GD20_TEMP_OUT | 0x80, (uint8_t *) &gyro_buffer_, sizeof(gyro_buffer_));
    }
    // Unsupported version
    return false;
  }

  /** \brief Configure the magnetometer to +/-12 Gauss. */
  bool configure_magnetometer(void)
  {
    if (version_ == VERSION_5_LSM6DS33_LIS3MDL)
    {
      /* Configure Performance
       *  bits 6:5 = 11b = Ultra High Performance X/Y
       *  bits 4:2 = 110b = 40hz output rate
       */
      if (!imu_write(LIS3MDL_DEVICE_ID, LIS3MDL_CTRL_REG1, 0x78))
        return false;

      /* Configure Scale
       *  bits 6:5 = 10b = +/-12 gauss
       *                 = 2281LSB/Gauss
       */
      if (!imu_write(LIS3MDL_DEVICE_ID, LIS3MDL_CTRL_REG2, 0x40))
        return false;

      /* Configure continuous conversion
       *  bit 1:0  = 00b = continuous conversion
       */
      if (!imu_write(LIS3MDL_DEVICE_ID, LIS3MDL_CTRL_REG3, 0x00))
        return false;

      /* Configure Z mode
       *  bit 3:2  = 11b = Ultra High Performance Z
       */
      if (!imu_write(LIS3MDL_DEVICE_ID, LIS3MDL_CTRL_REG4, 0x0C))
        return false;

      /* Configure BDU
       *  bit 6   = 1b = enable BDU
       */
      return imu_write(LIS3MDL_DEVICE_ID, LIS3MDL_CTRL_REG5, 0x40);
    }
    else if (version_ == VERSION_3_L3GD20H_LSM303D)
    {
      /* Configure Magnetometer Update Rate
       *  bit 4:2  = 011b = 25Hz
       */
      if (!imu_write(LSM303D_DEVICE_ID, LSM303D_CTRL_REG5, 0x0C))
        return false;

      /* Configure Magnetometer Scale
       *  bit 6:5  = 11b = +/- 12 Gauss
       *                 = 0.479 mgauss/lsb
       */
      return imu_write(LSM303D_DEVICE_ID, LSM303D_CTRL_REG6, 0x60);
    }
    else if (version_ == VERSION_2_L3GD20_LSM303DLHC)
    {
      /* Configure Magnetometer Update Rate
       *  bit 4:2  = 100b = 15Hz (default)
       */
      if (!imu_write(LSM303DHLC_DEVICE_ID_M, LSM303DHLC_CRA_REG_M, 0x10))
        return false;

      /* Configure Magnetometer Scale
       *  bit 7:5  = 111b = +/- 8.1 Gauss
       *  LSB/Gauss = 230 for X/Y, 205 for Z
       */
      if (!imu_write(LSM303DHLC_DEVICE_ID_M, LSM303DHLC_CRB_REG_M, 0xE0))
        return false;

      /* Configure continuous conversion
       *  bit 1:0  = 00b
       */
      return imu_write(LSM303DHLC_DEVICE_ID_M, LSM303DHLC_MR_REG_M, 0x00);
    }

    // Unsupported version
    return false;
  }

  /** \brief Start reading from the magnetometer */
  bool start_mag_read()
  {
    if (version_ == VERSION_5_LSM6DS33_LIS3MDL)
    {
      /* Register address bits 6:0 are address, bit 7 is auto-increment (1 = auto_increment) */
      return imu_start_read(LIS3MDL_DEVICE_ID, LIS3MDL_OUT_X_L | 0x80, (uint8_t *) &mag_buffer_, sizeof(mag_buffer_));
    }
    else if (version_ == VERSION_3_L3GD20H_LSM303D)
    {
      /* Register address bits 6:0 are address, bit 7 is auto-increment (1 = auto_increment) */
      return imu_start_read(LSM303D_DEVICE_ID, LSM303D_OUTX_L_M | 0x80, (uint8_t *) &mag_buffer_, sizeof(mag_buffer_));
    }
    else if (version_ == VERSION_2_L3GD20_LSM303DLHC)
    {
      /* Register address bits 6:0 are address, bit 7 is auto-increment (1 = auto_increment) */
      return imu_start_read(LSM303DHLC_DEVICE_ID_M, LSM303DHLC_OUTX_H_M | 0x80, (uint8_t *) &mag_buffer_, sizeof(mag_buffer_));
    }
    // Unsupported version
    return false;
  }

  /** \brief State of the updates */
  imu_state_t state_;

  /** \brief Internal timer used for timeouts */
  uint32_t timer_;

  /** \brief Private copy of accelerometer data for DMA read */
  accel_data_t accel_buffer_;
  /** \brief Private copy of gyro data for DMA read */
  gyro_data_t gyro_buffer_;
  /** \brief Private copy of mag data for DMA read, due to LSM303DHLC where
             the magnetometer is H/L byte, whereas everything else is L/H.. ugh */
  uint8_t mag_buffer_[6];

  /* internal logging */
  imu_error_op_t last_imu_error_;
  i2c_error_t    last_i2c_error_;
  uint32_t       num_timeouts_;
  uint32_t       num_gyro_updates_;
  uint32_t       num_accel_updates_;
  uint32_t       num_mag_updates_;

  /** \brief Store clock speed for timeout handler */
  uint32_t clock_speed_;

  /** \brief Should we read the magnetometer */
  bool use_mag_;

  /** \brief What verison IMU do we have? */
  uint8_t version_;
};

#endif  // _STM32_CPP_MINI_IMU9_H_
