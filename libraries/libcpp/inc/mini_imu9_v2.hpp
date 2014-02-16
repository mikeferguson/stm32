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

#ifndef _STM32_CPP_MINI_IMU9_V2_H_
#define	_STM32_CPP_MINI_IMU9_V2_H_

/* 
 * stm32_cpp: a C++ stm32 library
 * Driver for Pololu MiniIMU-9 v2, with ST L3GD20 and LSM303DLHC.
 *
 * Usage:
 *
 * typedef Gpio<GPIOB_BASE,11> imu_sda;
 * typedef Gpio<GPIOB_BASE,10> imu_scl;
 * MiniImu9v2<I2C2_BASE,
 *            DMA1_Stream3_BASE,
 *            3, // DMA STREAM
 *            7, // DMA_CHANNEL
 *            imu_scl
 *            imu_sda> imu;
 * // in setup
 * imu.init(100000);  // I2C speed
 * // at periodic intervals
 * imu.update(clock);  // should be passed 1khz clock
 */

#endif  // _STM32_CPP_MINI_IMU9_V2_H_
